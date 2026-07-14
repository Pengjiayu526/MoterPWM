/**
 * @file  line_follow.h
 * @brief 10路灰度传感器循迹模块 — PD控制器 + 位置EMA滤波
 *
 * 控制层次:
 *   灰度传感器 → 线位置与循迹PD → 左右目标pps → 左右轮速度PI → PWM
 *
 * 更新周期: 10 ms (100 Hz), 由 LINE_TIMER 定时器驱动
 * 速度PI周期: 20 ms (50 Hz), 由 TIMER_TICK 定时器驱动
 *
 * 第一版使用 PD (Ki=0)，底层电机已有速度PI。
 * dt 已吸收到 Kd 中 — 更新周期变化后必须重新调 Kd。
 *
 * 使用示例:
 * @code
 *   LineFollow_Config cfg = {
 *       .kp = 100.0f, .ki = 0.0f, .kd = 40.0f,
 *       .position_alpha = 0.4f, .derivative_alpha = 0.3f,
 *       .base_pps = 500, .max_turn_pps = 400, .max_target_pps = 1800
 *   };
 *   LineFollow_Init(&cfg);
 *   LineFollow_TimerInit();
 *   __enable_irq();
 *   LineFollow_Start();
 *
 *   while (1) {
 *       if (g_line_update_flag) {
 *           g_line_update_flag = 0;
 *           LineFollow_Update();
 *       }
 *       // 200ms 串口遥测 ...
 *   }
 * @endcode
 */

#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdint.h>

/*===========================================================================
 * 类型定义
 *===========================================================================*/

/** @brief 循迹状态枚举 */
typedef enum
{
    LINE_STATE_NORMAL   = 0,  /**< 正常循迹 (1~5 通道检测到黑线) */
    LINE_STATE_LOST,           /**< 丢线 (全白, 0 通道)            */
    LINE_STATE_ALL_BLACK,      /**< 全黑 (10 通道全部检测到黑线)   */
    LINE_STATE_WIDE            /**< 宽线 (6~9 通道检测到黑线)      */
} LineFollow_State;

/** @brief 循迹PD控制器配置参数 */
typedef struct
{
    float kp;                  /**< 比例系数 (单位: pps / 位置单位) */
    float ki;                  /**< 积分系数 (第一版设为 0)          */
    float kd;                  /**< 微分系数 (dt已吸收, 10ms周期)    */

    float position_alpha;      /**< 位置EMA滤波系数 (0.3~0.7)        */
    float derivative_alpha;    /**< 微分项EMA滤波系数 (0.2~0.6)      */

    int32_t base_pps;          /**< 基础速度 (pps), 推荐 400~600     */
    int32_t max_turn_pps;      /**< 转向分量限幅 (pps), 初始 400     */
    int32_t max_target_pps;    /**< 单轮最大目标速度 (pps), 推荐1800 */

} LineFollow_Config;

/*===========================================================================
 * 10ms 定时器标志 (ISR与主循环共享, 必须 volatile)
 *===========================================================================*/

/** @brief 10ms 循迹更新标志, 由 LINE_TIMER ISR 置1, 主循环消费后清0 */
extern volatile uint8_t g_line_update_flag;

/*===========================================================================
 * 公开接口 — 生命周期
 *===========================================================================*/

/**
 * @brief 使用指定配置初始化循迹模块内部状态。
 * @param config 配置参数指针 (内部会复制一份, 可传栈上临时变量)
 */
void LineFollow_Init(const LineFollow_Config *config);

/**
 * @brief 初始化 10ms 循迹定时器 (LINE_TIMER)。
 * @note  必须在 SYSCFG_DL_init() 之后, __enable_irq() 之前调用。
 * @note  清除中断标志和 NVIC pending, 使能 NVIC。
 */
void LineFollow_TimerInit(void);

/** @brief 启用循迹控制 (设置内部使能标志)。 */
void LineFollow_Start(void);

/** @brief 停止循迹控制, 清控制器状态, 设目标速度为 0。 */
void LineFollow_Stop(void);

/** @brief 复位控制器内部状态 (积分/微分/滤波器), 保持配置和启停状态不变。 */
void LineFollow_Reset(void);

/**
 * @brief 执行一次循迹更新 (应在主循环中每 10ms 调用一次)。
 *
 * 流程:
 *   1. 读取 10 路灰度位图
 *   2. 统计有效通道数, 判断丢线/全黑/宽线
 *   3. 计算加权位置 → EMA 滤波 → 误差 → PD 输出
 *   4. 限幅 → 转换左右目标 pps → 调用 SpeedControl_SetTargetPPS()
 *
 * @note  不在本函数内发送串口或延时。
 * @note  不直接调用 Motor_SetSpeed()。
 */
void LineFollow_Update(void);

/*===========================================================================
 * 公开接口 — 参数设置
 *===========================================================================*/

void LineFollow_SetBasePPS(int32_t basePPS);
void LineFollow_SetPD(float kp, float kd);
void LineFollow_SetPID(float kp, float ki, float kd);
void LineFollow_SetMaxTurnPPS(int32_t maxTurnPPS);
void LineFollow_SetPositionAlpha(float alpha);
void LineFollow_SetDerivativeAlpha(float alpha);

/*===========================================================================
 * 公开接口 — 状态查询 (用于低频串口调试)
 *===========================================================================*/

uint16_t        LineFollow_GetSensorBitmap(void);
uint8_t         LineFollow_GetActiveCount(void);
float           LineFollow_GetRawPosition(void);
float           LineFollow_GetFilteredPosition(void);
float           LineFollow_GetError(void);
float           LineFollow_GetOutput(void);
float           LineFollow_GetIntegral(void);
int32_t         LineFollow_GetTurnPPS(void);
int32_t         LineFollow_GetLeftTargetPPS(void);
int32_t         LineFollow_GetRightTargetPPS(void);
uint8_t         LineFollow_IsEnabled(void);
LineFollow_State LineFollow_GetState(void);

#endif /* LINE_FOLLOW_H_ */
