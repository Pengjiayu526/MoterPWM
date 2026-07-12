/**
 * @file  motor.c
 * @brief 双轮电机驱动实现 — 通过改变 PWM 波占空比控制转速, GPIO 控制方向
 *
 * 硬件参数 (由 SysConfig 生成):
 * - TIMG7 定时器, PWM 时钟 = 1.28 MHz (32MHz / 25)
 * - PWM 周期 = 128 个时钟周期 → PWM 频率 = 10000 Hz
 * - 右轮: TIMG7 通道0 (CCP0, PA17), 方向 AIN1(PB16) / AIN2(PA12)
 * - 左轮: TIMG7 通道1 (CCP1, PA24), 方向 BIN1(PB15) / BIN2(PB4)
 *
 * 速度控制原理:
 *   PWM 占空比 = 比较值 / 周期
 *   比较值 0   →   0% 占空比 → 电机停转
 *   比较值 128 → 100% 占空比 → 电机全速
 *
 * 方向控制 (H 桥驱动, IN1/IN2 真值表):
 *   IN1=0, IN2=1 → 前进
 *   IN1=1, IN2=0 → 后退
 *   IN1=0, IN2=0 → 滑行/停止
 *   IN1=1, IN2=1 → 刹车
 */

#include "motor.h"
#include "ti_msp_dl_config.h"

/*===========================================================================
 * 硬件映射
 *===========================================================================*/

/** @brief PWM 定时器实例 (TIMG7) */
#define MOTOR_PWM           PWM_0_INST

/** @brief 右轮 CC 索引 (通道0) */
#define MOTOR_CC_RIGHT      DL_TIMER_CC_0_INDEX

/** @brief 左轮 CC 索引 (通道1) */
#define MOTOR_CC_LEFT       DL_TIMER_CC_1_INDEX

/*===========================================================================
 * 内部函数 — 前向声明
 *===========================================================================*/

static void Motor_SetDirectionRight(uint8_t in1, uint8_t in2);
static void Motor_SetDirectionLeft(uint8_t in1, uint8_t in2);

/*===========================================================================
 * 内部函数
 *===========================================================================*/

/**
 * @brief 将速度百分比 (0~100) 转换为 PWM 比较值。
 * @note  比较值与占空比成反比: 比较值=0→100%, 比较值=127→0%
 *        计数器 LOAD = period-1 = 127, 比较值必须 ≤127, 写128永不匹配→全速
 * @param speed 速度百分比, 超出 100 截断为 100
 * @return 对应的 PWM 比较寄存器值 (0 ~ 127)
 */
static uint32_t Motor_SpeedToCompare(uint8_t speed)
{
    if (speed > MOTOR_SPEED_MAX) {
        speed = MOTOR_SPEED_MAX;
    }
    /* cmp = (period-1) * (1 - speed/100), 有效范围 0~127 */
    return (MOTOR_PWM_PERIOD - 1)
           - ((uint32_t)speed * (MOTOR_PWM_PERIOD - 1)) / MOTOR_SPEED_MAX;
}

/*===========================================================================
 * 公有函数
 *===========================================================================*/

 //电机驱动板的所有线都要连接到主控板上去
void Motor_Init(void)
{
    /*
     * 先将两路比较值设为最大值 (period-1=127), 确保定时器启动时
     * 占空比 ≈ 0%, 电机保持停止, 等待后续 Motor_SetSpeed() 设定速度。
     */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_RIGHT);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_LEFT);

    /* 启动定时器计数, PWM 波形开始输出 */
    DL_TimerG_startCounter(MOTOR_PWM);
}

void Motor_SetSpeed(int8_t leftSpeed, int8_t rightSpeed)
{
    uint8_t  leftAbs  = (leftSpeed  < 0) ? (uint8_t)(-leftSpeed)  : (uint8_t)leftSpeed;
    uint8_t  rightAbs = (rightSpeed < 0) ? (uint8_t)(-rightSpeed) : (uint8_t)rightSpeed;
    uint32_t leftCompare  = Motor_SpeedToCompare(leftAbs);
    uint32_t rightCompare = Motor_SpeedToCompare(rightAbs);

    /* 根据正负号自动设置右轮方向 */
    if (rightSpeed > 0) {
        Motor_SetDirectionRight(0, 1);   /* IN1=0, IN2=1 → 前进 */
    } else if (rightSpeed < 0) {
        Motor_SetDirectionRight(1, 0);   /* IN1=1, IN2=0 → 后退 */
    } else {
        Motor_SetDirectionRight(0, 0);   /* IN1=0, IN2=0 → 滑行 */
    }

    /* 根据正负号自动设置左轮方向 */
    if (leftSpeed > 0) {
        Motor_SetDirectionLeft(0, 1);    /* IN1=0, IN2=1 → 前进 */
    } else if (leftSpeed < 0) {
        Motor_SetDirectionLeft(1, 0);    /* IN1=1, IN2=0 → 后退 */
    } else {
        Motor_SetDirectionLeft(0, 0);    /* IN1=0, IN2=0 → 滑行 */
    }

    /*
     * 更新两路 PWM 的比较值以改变占空比:
     * - 比较值 = 0:   输出始终为低, 电机两端电压为 0, 停止
     * - 比较值 = 128: 输出始终为高, 电机两端电压 = 电源电压, 全速
     * - 中间值:       输出方波, 占空比 = 比较值/128, 电机转速与占空比成正比
     */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM, rightCompare, MOTOR_CC_RIGHT);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM, leftCompare,  MOTOR_CC_LEFT);
}

void Motor_Stop(void)
{
    /*
     * 比较值 = period-1 = 127 → 占空比 ≈ 0% → 电机停止
     * 注意: 不能写 128, 因为计数器只到 127, 写 128 永不匹配反而 100% 全速
     */
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_RIGHT);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM,
        MOTOR_PWM_PERIOD - 1, MOTOR_CC_LEFT);
}

/*===========================================================================
 * 方向控制函数
 *===========================================================================*/

/**
 * @brief 设置单个 IN1/IN2 引脚电平 (右轮 A 通道)。
 * @param in1  IN1 目标电平 (0 或 1)
 * @param in2  IN2 目标电平 (0 或 1)
 */
static void Motor_SetDirectionRight(uint8_t in1, uint8_t in2)
{
    /* AIN1 = PB16, 端口 GPIOB */
    if (in1) {
        DL_GPIO_setPins(GPIOB, AIN_AIN1_PIN);
    } else {
        DL_GPIO_clearPins(GPIOB, AIN_AIN1_PIN);
    }

    /* AIN2 = PA12, 端口 GPIOA */
    if (in2) {
        DL_GPIO_setPins(GPIOA, AIN_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIOA, AIN_AIN2_PIN);
    }
}

/**
 * @brief 设置单个 IN1/IN2 引脚电平 (左轮 B 通道)。
 * @param in1  IN1 目标电平 (0 或 1)
 * @param in2  IN2 目标电平 (0 或 1)
 */
static void Motor_SetDirectionLeft(uint8_t in1, uint8_t in2)
{
    /* BIN1 = PB15, 端口 GPIOB */
    if (in1) {
        DL_GPIO_setPins(GPIOB, BIN_BIN1_PIN);
    } else {
        DL_GPIO_clearPins(GPIOB, BIN_BIN1_PIN);
    }

    /* BIN2 = PB4, 端口 GPIOB */
    if (in2) {
        DL_GPIO_setPins(GPIOB, BIN_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIOB, BIN_BIN2_PIN);
    }
}

/**
 * @brief 将方向枚举转换为 IN1/IN2 电平值。
 *
 * H 桥真值表:
 *   MOTOR_DIR_FORWARD  → IN1=0, IN2=1
 *   MOTOR_DIR_BACKWARD → IN1=1, IN2=0
 *   MOTOR_DIR_BRAKE    → IN1=1, IN2=1
 *   MOTOR_DIR_COAST    → IN1=0, IN2=0
 */
static void Motor_DirToInPins(Motor_Direction dir, uint8_t *in1, uint8_t *in2)
{
    switch (dir) {
    case MOTOR_DIR_FORWARD:
        *in1 = 0; *in2 = 1;
        break;
    case MOTOR_DIR_BACKWARD:
        *in1 = 1; *in2 = 0;
        break;
    case MOTOR_DIR_BRAKE:
        *in1 = 1; *in2 = 1;
        break;
    case MOTOR_DIR_COAST:
    default:
        *in1 = 0; *in2 = 0;
        break;
    }
}

void Motor_SetDirection(Motor_Select motor, Motor_Direction dir)
{
    uint8_t in1, in2;

    Motor_DirToInPins(dir, &in1, &in2);

    if (motor == MOTOR_RIGHT || motor == MOTOR_BOTH) {
        Motor_SetDirectionRight(in1, in2);
    }

    if (motor == MOTOR_LEFT || motor == MOTOR_BOTH) {
        Motor_SetDirectionLeft(in1, in2);
    }
}

void Motor_Forward(void)
{
    Motor_SetDirection(MOTOR_BOTH, MOTOR_DIR_FORWARD);
}

void Motor_Backward(void)
{
    Motor_SetDirection(MOTOR_BOTH, MOTOR_DIR_BACKWARD);
}

void Motor_Brake(void)
{
    Motor_SetDirection(MOTOR_BOTH, MOTOR_DIR_BRAKE);
}

void Motor_Coast(void)
{
    Motor_SetDirection(MOTOR_BOTH, MOTOR_DIR_COAST);
}
