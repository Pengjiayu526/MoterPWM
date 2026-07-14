/**
 * @file  speed_control.h
 * @brief 左右轮独立 PI 速度闭环控制器 — 位置式 PI + 抗积分饱和
 *
 * 控制周期: 20 ms (与编码器测速周期一致)
 * 输出范围: -100 ~ 100 (直接传给 Motor_SetSpeed)
 * 目标范围: ±1800 pps
 *
 * 使用示例:
 * @code
 *   SpeedControl_Init();
 *   SpeedControl_SetTargetPPS(800, 800);   // 两轮 800 pps 前进
 *   SpeedControl_SetTargetPPS(-600, 600);  // 原地右转
 *   SpeedControl_SetTargetPPS(0, 0);       // 停止
 * @endcode
 */

#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 常量定义
 *===========================================================================*/

/** @brief 目标速度上限 (pps)，防止设置超出电机能力的目标 */
#define SPEED_TARGET_MAX_PPS    1800

/** @brief 控制周期 (秒)，与编码器测速定时器 ENCODER_SAMPLE_MS 一致 */
#define SPEED_CONTROL_DT_S      0.02f

/*===========================================================================
 * PI 控制器结构体
 *===========================================================================*/

typedef struct
{
    float kp;                /**< 比例系数 */
    float ki;                /**< 积分系数 (按秒定义) */

    float integral;          /**< 积分项 (已限幅) */
    float integral_min;      /**< 积分项下限 */
    float integral_max;      /**< 积分项上限 */

    float output_min;        /**< PWM 输出幅值下限 (0) */
    float output_max;        /**< PWM 输出幅值上限 (100) */

    int32_t target_pps;      /**< 有符号目标速度 (pps)，正=前进，负=后退 */

    int8_t  last_direction;  /**< 上一次目标方向: 1=前进, -1=后退, 0=停止 */
    int8_t  output_pwm;      /**< 最近一次 PI 输出的 PWM 值 (-100 ~ 100) */

} SpeedPI_Controller;

/*===========================================================================
 * 公开接口
 *===========================================================================*/

/**
 * @brief 初始化左右轮 PI 控制器为默认参数。
 * @note  默认 Kp=0.020, Ki=0.080, 积分限幅 ±60, 输出限幅 0~100。
 */
void SpeedControl_Init(void);

/**
 * @brief 同时设置左右轮目标速度 (线程安全，关中断保护)。
 * @param leftTargetPPS  左轮目标速度 (pps)，-1800 ~ +1800
 * @param rightTargetPPS 右轮目标速度 (pps)，-1800 ~ +1800
 */
void SpeedControl_SetTargetPPS(int32_t leftTargetPPS, int32_t rightTargetPPS);

/**
 * @brief 单独设置左轮目标速度。
 */
void SpeedControl_SetLeftTargetPPS(int32_t targetPPS);

/**
 * @brief 单独设置右轮目标速度。
 */
void SpeedControl_SetRightTargetPPS(int32_t targetPPS);

/**
 * @brief 复位左右轮控制器 (积分清零、PWM 归零、方向清零)。
 */
void SpeedControl_Reset(void);

/**
 * @brief 仅复位左轮控制器。
 */
void SpeedControl_ResetLeft(void);

/**
 * @brief 仅复位右轮控制器。
 */
void SpeedControl_ResetRight(void);

/**
 * @brief 执行一次 PI 更新 (应在 20ms 定时器 ISR 中调用)。
 * @param measuredLeftPPS  左轮实测速度 (pps)
 * @param measuredRightPPS 右轮实测速度 (pps)
 *
 * @note  本函数内部调用 Motor_SetSpeed()，主循环不应再直接调用。
 * @note  不在本函数中进行串口打印或延时。
 */
void SpeedControl_Update(uint32_t measuredLeftPPS, uint32_t measuredRightPPS);

/**
 * @brief 获取左轮当前目标速度 (pps)。
 */
int32_t SpeedControl_GetLeftTargetPPS(void);

/**
 * @brief 获取右轮当前目标速度 (pps)。
 */
int32_t SpeedControl_GetRightTargetPPS(void);

/**
 * @brief 获取左轮最近一次 PI 输出的 PWM 值。
 */
int8_t SpeedControl_GetLeftPWM(void);

/**
 * @brief 获取右轮最近一次 PI 输出的 PWM 值。
 */
int8_t SpeedControl_GetRightPWM(void);

/**
 * @brief 单独设置左轮 PI 参数。
 */
void SpeedControl_SetLeftPI(float kp, float ki);

/**
 * @brief 单独设置右轮 PI 参数。
 */
void SpeedControl_SetRightPI(float kp, float ki);

#endif /* SPEED_CONTROL_H_ */
