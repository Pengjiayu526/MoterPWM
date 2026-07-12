/**
 * @file  motor.h
 * @brief 双轮电机驱动模块 — 通过 PWM 占空比控制转速, GPIO 控制方向
 *
 * 硬件连接:
 * - 通道0 (CCP0 / PA17) → 右轮 PWM (A通道)
 * - 通道1 (CCP1 / PA24) → 左轮 PWM (B通道)
 * - AIN1 (PB16) → 右轮 IN1,  AIN2 (PA12) → 右轮 IN2
 * - BIN1 (PB15) → 左轮 IN1,  BIN2 (PB4)  → 左轮 IN2
 *
 * 方向控制 (TB6612 / L298N 等 H桥驱动):
 * - IN1=0, IN2=1 → 前进
 * - IN1=1, IN2=0 → 后退
 * - IN1=0, IN2=0 → 滑行/停止
 * - IN1=1, IN2=1 → 刹车
 *
 * 使用示例:
 * @code
 *   SYSCFG_DL_init();
 *   Motor_Init();
 *   Motor_SetSpeed(50, 50);       // 两轮 50% 前进
 *   Motor_SetSpeed(-40, -40);     // 两轮 40% 后退
 *   Motor_SetSpeed(30, 80);       // 左转 (左轮慢, 右轮快, 前进)
 *   Motor_SetSpeed(-30, 80);      // 原地左转 (左轮后退, 右轮前进)
 *   Motor_SetSpeed(0, 0);         // 停止
 * @endcode
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

/*===========================================================================
 * 常量定义
 *===========================================================================*/

/** @brief PWM 周期 (定时器重装载值) — 占空比分辨率 0 ~ 128 */
#define MOTOR_PWM_PERIOD    128U

/** @brief 速度百分比上限 */
#define MOTOR_SPEED_MAX     100

/*===========================================================================
 * 方向枚举定义
 *===========================================================================*/

/** @brief 电机运转方向 */
typedef enum {
    MOTOR_DIR_FORWARD  = 0,  /**< IN1=0, IN2=1 → 前进 */
    MOTOR_DIR_BACKWARD,      /**< IN1=1, IN2=0 → 后退 */
    MOTOR_DIR_BRAKE,         /**< IN1=1, IN2=1 → 刹车 */
    MOTOR_DIR_COAST          /**< IN1=0, IN2=0 → 滑行/停止 */
} Motor_Direction;

/** @brief 电机选择 (用于单独控制某个电机方向) */
typedef enum {
    MOTOR_LEFT  = 0,         /**< 左轮 (B通道) */
    MOTOR_RIGHT,             /**< 右轮 (A通道) */
    MOTOR_BOTH               /**< 左右两轮 */
} Motor_Select;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化电机 PWM 并启动输出。
 * @note  必须在 SYSCFG_DL_init() 之后调用。
 * @note  方向引脚由 SYSCFG_DL_GPIO_init() 初始化, 默认为前进方向。
 */
void Motor_Init(void);

/**
 * @brief 同时设置左右两轮的速度与方向 (通过正负号控制正反转)。
 * @param leftSpeed  左轮速度 (-100 ~ 100)
 *                   正数 = 前进, 负数 = 后退, 0 = 停止
 * @param rightSpeed 右轮速度 (-100 ~ 100)
 *                   正数 = 前进, 负数 = 后退, 0 = 停止
 *
 * 通过改变 PWM 波占空比来控制电机转速, 同时根据正负号自动切换 IN1/IN2 方向:
 * - 占空比越高, 电机两端等效平均电压越高, 转速越快
 * - 传入 0 则对应通道输出 0% 占空比, 电机停转
 *
 * @note  本函数会自动设置方向引脚, 无需再单独调用方向控制函数。
 */
void Motor_SetSpeed(int8_t leftSpeed, int8_t rightSpeed);

/**
 * @brief 停止两个电机 (等价于 Motor_SetSpeed(0, 0))。
 * @note  不改变当前方向设定。
 */
void Motor_Stop(void);

/*===========================================================================
 * 方向控制函数
 *===========================================================================*/

/**
 * @brief 单独设置指定电机的运转方向。
 * @param motor 目标电机 (MOTOR_LEFT / MOTOR_RIGHT / MOTOR_BOTH)
 * @param dir   运转方向
 *
 * 通过控制 IN1/IN2 引脚电平实现 H 桥方向切换。
 * AIN1/AIN2 对应右轮, BIN1/BIN2 对应左轮。
 */
void Motor_SetDirection(Motor_Select motor, Motor_Direction dir);

/**
 * @brief 设置两轮方向为前进 (IN1=0, IN2=1)。
 */
void Motor_Forward(void);

/**
 * @brief 设置两轮方向为后退 (IN1=1, IN2=0)。
 */
void Motor_Backward(void);

/**
 * @brief 两轮刹车 (IN1=1, IN2=1) — 电机绕组短接, 产生制动阻力。
 */
void Motor_Brake(void);

/**
 * @brief 两轮滑行/停止 (IN1=0, IN2=0) — 电机自由转动, 无制动力。
 */
void Motor_Coast(void);

#endif /* MOTOR_H_ */
