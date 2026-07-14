/**
 * @file  speed_control.c
 * @brief 左右轮独立 PI 速度闭环控制器实现
 *
 * 算法: 位置式 PI + 条件积分抗饱和
 *
 *   error = target_abs_pps - measured_pps
 *   P = Kp × error
 *   I = I + Ki × error × Ts   (条件更新)
 *   output = clamp(P + I, 0, 100)
 *   signed_pwm = direction × output
 *
 * 安全特性:
 *   - 目标为 0 时立即清零积分并输出 PWM=0
 *   - 方向切换时清零积分并空一个控制周期
 *   - 输出 PWM 始终限制在 -100 ~ 100
 *   - 目标速度限制在 ±1800 pps
 *   - 关中断保护目标速度的原子写入
 */

#include "speed_control.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

/*===========================================================================
 * 默认 PI 参数 (保守初值，左右轮相同)
 *===========================================================================*/

#define DEFAULT_KP             0.030f
#define DEFAULT_KI             0.05f
#define DEFAULT_INTEGRAL_MIN  -60.0f
#define DEFAULT_INTEGRAL_MAX   60.0f
#define DEFAULT_OUTPUT_MIN     0.0f
#define DEFAULT_OUTPUT_MAX   100.0f

/*===========================================================================
 * 全局控制器实例 (静态，仅本文件可见)
 *===========================================================================*/

static SpeedPI_Controller g_leftController;
static SpeedPI_Controller g_rightController;

/*===========================================================================
 * 内部工具函数
 *===========================================================================*/

/**
 * @brief 浮点限幅。
 */
static inline float clamp_float(float val, float min, float max)
{
    if (val < min) { return min; }
    if (val > max) { return max; }
    return val;
}

/**
 * @brief int32_t 限幅 (消除 INT32_MIN 绝对值溢出风险)。
 */
static inline int32_t clamp_int32(int32_t val, int32_t min, int32_t max)
{
    if (val < min) { return min; }
    if (val > max) { return max; }
    return val;
}

/**
 * @brief 安全的 int32_t 绝对值 (避免 INT32_MIN 溢出)。
 * @note  目标速度范围远小于 INT32_MAX，但为鲁棒性仍做保护。
 */
static inline int32_t safe_abs_int32(int32_t val)
{
    if (val < 0) {
        return (val == INT32_MIN) ? INT32_MAX : (-val);
    }
    return val;
}

/*===========================================================================
 * 单轮 PI 更新 (内部函数)
 *===========================================================================*/

/**
 * @brief 对单个电机执行一次位置式 PI 更新。
 * @param ctrl        控制器实例指针
 * @param measuredPPS 实测速度 (pps, 无符号绝对值)
 * @return 有符号 PWM 输出 (-100 ~ 100)
 *
 * 处理顺序:
 *   1. 目标为 0 → 清积分, PWM=0
 *   2. 获取方向和绝对值目标
 *   3. 方向改变 → 清积分, 空一个周期输出 0
 *   4. 计算误差 → P 项 → 积分候选 → 抗饱和条件判断 → 更新积分
 *   5. 输出限幅 → 赋符号
 */
static int8_t SpeedPI_UpdateSingle(SpeedPI_Controller *ctrl, uint32_t measuredPPS)
{
    int32_t target;
    int8_t  direction;
    int32_t target_abs;
    float   error;
    float   p_term;
    float   integral_candidate;
    float   output_candidate;
    float   output;
    bool    allow_integral;

    target = ctrl->target_pps;

    /*-----------------------------------------------------------------
     * 7.1: 目标为 0 — 立即停止，清零积分
     *-----------------------------------------------------------------*/
    if (target == 0) {
        ctrl->integral       = 0.0f;
        ctrl->output_pwm     = 0;
        ctrl->last_direction = 0;
        return 0;
    }

    /*-----------------------------------------------------------------
     * 7.2: 获取方向和绝对值目标
     *-----------------------------------------------------------------*/
    direction  = (target > 0) ? 1 : -1;
    target_abs = safe_abs_int32(target);

    /*-----------------------------------------------------------------
     * 7.3: 方向改变 — 清零积分，空一个控制周期 (输出 0)
     *-----------------------------------------------------------------*/
    if (direction != ctrl->last_direction) {
        ctrl->integral       = 0.0f;
        ctrl->last_direction = direction;
        ctrl->output_pwm     = 0;
        return 0;
    }

    /*-----------------------------------------------------------------
     * 7.4: 计算误差
     *-----------------------------------------------------------------*/
    error = (float)target_abs - (float)measuredPPS;

    /*-----------------------------------------------------------------
     * 7.5: 比例项
     *-----------------------------------------------------------------*/
    p_term = ctrl->kp * error;

    /*-----------------------------------------------------------------
     * 7.6: 积分候选 (先做积分限幅)
     *-----------------------------------------------------------------*/
    integral_candidate = ctrl->integral
                       + ctrl->ki * error * SPEED_CONTROL_DT_S;
    integral_candidate = clamp_float(integral_candidate,
                                     ctrl->integral_min,
                                     ctrl->integral_max);

    /*-----------------------------------------------------------------
     * 7.7: 抗积分饱和 — 条件积分
     *
     * 允许积分更新的条件:
     *   1. 输出在 (min, max) 区间内
     *   2. 输出已 ≥ max 但误差 < 0 (需要减小输出，允许积分下降)
     *   3. 输出已 ≤ min 但误差 > 0 (需要增大输出，允许积分上升)
     *-----------------------------------------------------------------*/
    output_candidate = p_term + integral_candidate;

    allow_integral = false;

    if ((output_candidate > ctrl->output_min) &&
        (output_candidate < ctrl->output_max)) {
        /* 输出未饱和，正常积分 */
        allow_integral = true;
    } else if ((output_candidate >= ctrl->output_max) &&
               (error < 0.0f)) {
        /* 输出正饱和但误差已转负，允许积分减小以退出饱和 */
        allow_integral = true;
    } else if ((output_candidate <= ctrl->output_min) &&
               (error > 0.0f)) {
        /* 输出负饱和但误差已转正，允许积分恢复 */
        allow_integral = true;
    }

    if (allow_integral) {
        ctrl->integral = integral_candidate;
    }
    /* 否则保持上一次的积分值不变 */

    /*-----------------------------------------------------------------
     * 计算最终输出并限幅
     *-----------------------------------------------------------------*/
    output = p_term + ctrl->integral;
    output = clamp_float(output, ctrl->output_min, ctrl->output_max);

    /*-----------------------------------------------------------------
     * 转换成有符号 PWM (四舍五入)
     *-----------------------------------------------------------------*/
    ctrl->output_pwm = (int8_t)((float)direction * (output + 0.5f));

    return ctrl->output_pwm;
}

/*===========================================================================
 * 公有函数
 *===========================================================================*/

void SpeedControl_Init(void)
{
    /* 左轮控制器默认参数 */
    g_leftController.kp             = DEFAULT_KP;
    g_leftController.ki             = DEFAULT_KI;
    g_leftController.integral       = 0.0f;
    g_leftController.integral_min   = DEFAULT_INTEGRAL_MIN;
    g_leftController.integral_max   = DEFAULT_INTEGRAL_MAX;
    g_leftController.output_min     = DEFAULT_OUTPUT_MIN;
    g_leftController.output_max     = DEFAULT_OUTPUT_MAX;
    g_leftController.target_pps     = 0;
    g_leftController.last_direction = 0;
    g_leftController.output_pwm     = 0;

    /* 右轮控制器默认参数 */
    g_rightController.kp             = DEFAULT_KP;
    g_rightController.ki             = DEFAULT_KI;
    g_rightController.integral       = 0.0f;
    g_rightController.integral_min   = DEFAULT_INTEGRAL_MIN;
    g_rightController.integral_max   = DEFAULT_INTEGRAL_MAX;
    g_rightController.output_min     = DEFAULT_OUTPUT_MIN;
    g_rightController.output_max     = DEFAULT_OUTPUT_MAX;
    g_rightController.target_pps     = 0;
    g_rightController.last_direction = 0;
    g_rightController.output_pwm     = 0;
}

void SpeedControl_SetTargetPPS(int32_t leftTargetPPS, int32_t rightTargetPPS)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    g_leftController.target_pps  = clamp_int32(leftTargetPPS,
                                               -SPEED_TARGET_MAX_PPS,
                                                SPEED_TARGET_MAX_PPS);
    g_rightController.target_pps = clamp_int32(rightTargetPPS,
                                               -SPEED_TARGET_MAX_PPS,
                                                SPEED_TARGET_MAX_PPS);

    if (primask == 0U) {
        __enable_irq();
    }
}

void SpeedControl_SetLeftTargetPPS(int32_t targetPPS)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_leftController.target_pps = clamp_int32(targetPPS,
                                              -SPEED_TARGET_MAX_PPS,
                                               SPEED_TARGET_MAX_PPS);
    if (primask == 0U) {
        __enable_irq();
    }
}

void SpeedControl_SetRightTargetPPS(int32_t targetPPS)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_rightController.target_pps = clamp_int32(targetPPS,
                                               -SPEED_TARGET_MAX_PPS,
                                                SPEED_TARGET_MAX_PPS);
    if (primask == 0U) {
        __enable_irq();
    }
}

void SpeedControl_Reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    g_leftController.integral       = 0.0f;
    g_leftController.output_pwm     = 0;
    g_leftController.last_direction = 0;

    g_rightController.integral       = 0.0f;
    g_rightController.output_pwm     = 0;
    g_rightController.last_direction = 0;

    if (primask == 0U) {
        __enable_irq();
    }
}

void SpeedControl_ResetLeft(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_leftController.integral       = 0.0f;
    g_leftController.output_pwm     = 0;
    g_leftController.last_direction = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}

void SpeedControl_ResetRight(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_rightController.integral       = 0.0f;
    g_rightController.output_pwm     = 0;
    g_rightController.last_direction = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}

void SpeedControl_Update(uint32_t measuredLeftPPS, uint32_t measuredRightPPS)
{
    int8_t leftPWM;
    int8_t rightPWM;

    leftPWM  = SpeedPI_UpdateSingle(&g_leftController,  measuredLeftPPS);
    rightPWM = SpeedPI_UpdateSingle(&g_rightController, measuredRightPPS);

    Motor_SetSpeed(leftPWM, rightPWM);
}

int32_t SpeedControl_GetLeftTargetPPS(void)
{
    return g_leftController.target_pps;
}

int32_t SpeedControl_GetRightTargetPPS(void)
{
    return g_rightController.target_pps;
}

int8_t SpeedControl_GetLeftPWM(void)
{
    return g_leftController.output_pwm;
}

int8_t SpeedControl_GetRightPWM(void)
{
    return g_rightController.output_pwm;
}

void SpeedControl_SetLeftPI(float kp, float ki)
{
    g_leftController.kp = kp;
    g_leftController.ki = ki;
}

void SpeedControl_SetRightPI(float kp, float ki)
{
    g_rightController.kp = kp;
    g_rightController.ki = ki;
}
