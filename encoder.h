/**
 * @file  encoder.h
 * @brief 双路编码器速度测量模块
 *
 * 仅测速，不判断方向 — 每个编码器只用 A 通道的上升沿计脉冲。
 * GPIO 和定时器的外设初始化由 SysConfig 负责，
 * 本模块只做 NVIC 使能 + 中断处理 + 速度读取 API。
 *
 * 架构 (参考 hw_encoder / mid_timer 模式):
 *   GPIO ISR  →  实时修改 temp_count (volatile)
 *   Timer ISR →  每 20ms: speed = temp_count × 50 → temp_count = 0
 *   API       →  返回 speed (非 volatile, 主循环安全读取)
 *
 * 使用示例:
 * @code
 *   SYSCFG_DL_init();
 *   Motor_Init();
 *   Encoder_Init();
 *
 *   Motor_SetSpeed(50, 50);
 *   uint32_t leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
 *   uint32_t rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
 * @endcode
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

/*===========================================================================
 * 硬件映射 — 引用 SysConfig 生成的宏 (ti_msp_dl_config.h)
 *
 * 左轮 A_Encode: AA = PA0 (上升沿中断), AB = PA1 (仅输入)
 * 右轮 B_Encode: BA = PA8 (上升沿中断), BB = PA9 (仅输入)
 * 速度刷新定时器: TIMER_TICK (TIMA0), 周期 20ms
 *===========================================================================*/

/** @brief 速度采样周期 (ms) — 对应 TIMER_TICK 的 20ms */
#define ENCODER_SAMPLE_PERIOD_MS    20U

/** @brief 速度换算系数: 1000ms / 20ms = 50 */
#define ENCODER_SPEED_FACTOR        (1000U / ENCODER_SAMPLE_PERIOD_MS)

/*===========================================================================
 * 编码器参数 — 13 线霍尔编码器, 减速比 1:28
 *===========================================================================*/

/** @brief 编码器线数 (电机轴每转脉冲数) */
#define ENCODER_PULSES_PER_REV      13U

/** @brief 减速比 (电机轴 : 输出轴 = 28 : 1) */
#define ENCODER_GEAR_RATIO          28U

/** @brief 输出轴每转脉冲数 = 13 × 28 = 364 */
#define ENCODER_OUTPUT_PPR          (ENCODER_PULSES_PER_REV * ENCODER_GEAR_RATIO)

/*===========================================================================
 * 枚举定义
 *===========================================================================*/

/** @brief 编码器选择 */
typedef enum {
    ENCODER_LEFT  = 0,          /**< 左轮编码器 (A_Encode) */
    ENCODER_RIGHT = 1           /**< 右轮编码器 (B_Encode) */
} Encoder_Select;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化编码器模块 — 使能 NVIC 中断。
 *
 * GPIO 和定时器外设已由 SYSCFG_DL_init() 完成配置,
 * 本函数仅清除挂起标志并使能 NVIC 中断线。
 *
 * @note 必须在 SYSCFG_DL_init() 之后调用。
 */
void Encoder_Init(void);

/**
 * @brief 获取指定编码器的当前速度。
 * @param encoder 编码器选择 (ENCODER_LEFT 或 ENCODER_RIGHT)
 * @return 当前速度 (单位: 脉冲/秒), 每 20ms 刷新一次
 *
 * 32-bit 读取在 Cortex-M0+ 上是原子的, 无需关中断。
 */
uint32_t Encoder_GetSpeed(Encoder_Select encoder);

/**
 * @brief 复位指定编码器的脉冲计数和速度归零。
 * @param encoder 编码器选择 (ENCODER_LEFT 或 ENCODER_RIGHT)
 */
void Encoder_Reset(Encoder_Select encoder);

/*===========================================================================
 * RPM 换算 — 基于编码器参数的内联函数
 *
 * 电机轴 RPM = pps × 60 / 13
 * 输出轴 RPM = pps × 60 / 364  (减速后)
 *===========================================================================*/

/**
 * @brief 获取电机轴转速 (减速前)。
 * @return 电机轴转速 (RPM)
 */
static inline uint32_t Encoder_GetMotorRPM(Encoder_Select encoder)
{
    return (Encoder_GetSpeed(encoder) * 60U) / ENCODER_PULSES_PER_REV;
}

/**
 * @brief 获取输出轴转速 (减速后, 1:28)。
 * @return 输出轴转速 (RPM)
 */
static inline uint32_t Encoder_GetOutputRPM(Encoder_Select encoder)
{
    return (Encoder_GetSpeed(encoder) * 60U) / ENCODER_OUTPUT_PPR;
}

#endif /* ENCODER_H_ */
