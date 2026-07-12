/**
 * @file  encoder.c
 * @brief 双路编码器速度测量 — GPIO 中断计脉冲 + 定时器周期性算速度
 *
 * 架构 (参考 hw_encoder / mid_timer 模式):
 * - GPIO ISR (GROUP1_IRQHandler)          → 实时累加 temp_count (A通道上升沿)
 * - Timer ISR (TIMER_TICK_INST_IRQHandler) → 每 20ms: speed = temp_count × 50,
 *                                             temp_count = 0
 * - API (Encoder_GetSpeed)                → 主循环安全读取 speed
 *
 * 仅测速，不判断方向 — 只用 A 通道上升沿计数。
 *
 * GPIO 和定时器外设初始化由 SysConfig 完成:
 * - A_Encode (AA=PA0, AB=PA1): SYSCFG_DL_GPIO_init()
 * - B_Encode (BA=PA8, BB=PA9): SYSCFG_DL_GPIO_init()
 * - TIMER_TICK (TIMA0, 20ms):  SYSCFG_DL_TIMER_TICK_init()
 * 本模块仅负责 NVIC 使能和中断处理。
 */

#include "encoder.h"
#include "ti_msp_dl_config.h"

/*===========================================================================
 * 编码器运行时数据 (参考 ENCODER_RES 结构)
 *
 * temp_count: volatile, GPIO ISR 中实时累加
 * speed:      非 volatile, Timer ISR 中定期快照, 主循环安全读取
 *===========================================================================*/

typedef struct {
    volatile int32_t  temp_count;   /**< 实时脉冲计数 (GPIO ISR 写入) */
    uint32_t          speed;        /**< 当前速度 pulses/sec (Timer ISR 更新) */
} Encoder_Data;

static Encoder_Data g_leftEncoder;   /**< 左轮编码器 (A_Encode) */
static Encoder_Data g_rightEncoder;  /**< 右轮编码器 (B_Encode) */

/*===========================================================================
 * 公有函数
 *===========================================================================*/

void Encoder_Init(void)
{
    /*
     * GPIO 和定时器外设已由 SYSCFG_DL_init() 配置完毕。
     * 此处参照 hw_encoder / mid_timer 的模式，仅使能 NVIC 中断线。
     */

    /* GPIOA 中断 — 编码器脉冲 (PA0, PA8) */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /* TIMA0 中断 — 速度刷新 (20ms 周期) */
    NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
}

uint32_t Encoder_GetSpeed(Encoder_Select encoder)
{
    /* 32-bit 读取在 Cortex-M0+ 上是原子的, 无需关中断 */
    if (encoder == ENCODER_LEFT) {
        return g_leftEncoder.speed;
    } else {
        return g_rightEncoder.speed;
    }
}

void Encoder_Reset(Encoder_Select encoder)
{
    if (encoder == ENCODER_LEFT) {
        g_leftEncoder.temp_count = 0;
        g_leftEncoder.speed = 0;
    } else {
        g_rightEncoder.temp_count = 0;
        g_rightEncoder.speed = 0;
    }
}

/*===========================================================================
 * 中断处理
 *===========================================================================*/

/**
 * @brief GPIOA 中断 — 编码器脉冲计数
 *
 * PA0 (A_Encode AA) 上升沿 → 左轮 temp_count++
 * PA8 (B_Encode BA) 上升沿 → 右轮 temp_count++
 *
 * 参照 hw_encoder.c 中 GROUP1_IRQHandler 的处理流程:
 * 读中断状态 → 判断触发源 → 累加计数 → 清除中断标志
 */
void GROUP1_IRQHandler(void)
{
    uint32_t pending;

    pending = DL_GPIO_getEnabledInterruptStatus(A_Encode_PORT,
        A_Encode_AA_PIN | B_Encode_BA_PIN);

    /* 左轮编码器脉冲 (PA0 上升沿) */
    if (pending & A_Encode_AA_PIN) {
        g_leftEncoder.temp_count++;
    }

    /* 右轮编码器脉冲 (PA8 上升沿) */
    if (pending & B_Encode_BA_PIN) {
        g_rightEncoder.temp_count++;
    }

    /* 清除已处理的中断标志 */
    DL_GPIO_clearInterruptStatus(A_Encode_PORT,
        A_Encode_AA_PIN | B_Encode_BA_PIN);
}

/**
 * @brief TIMA0 中断 — 每 20ms 刷新一次速度
 *
 * 参照 hw_timer.c 中 TIMER_TICK_INST_IRQHandler 的处理流程:
 * 检查 ZERO 中断 → 快照 temp_count → 换算速度 → 清零 temp_count
 */
void TIMER_TICK_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_TICK_INST) == DL_TIMER_IIDX_ZERO) {

        /* 左轮: 快照脉冲数 → 换算速度 → 清零 */
        g_leftEncoder.speed = (uint32_t)g_leftEncoder.temp_count
                            * ENCODER_SPEED_FACTOR;
        g_leftEncoder.temp_count = 0;

        /* 右轮: 快照脉冲数 → 换算速度 → 清零 */
        g_rightEncoder.speed = (uint32_t)g_rightEncoder.temp_count
                             * ENCODER_SPEED_FACTOR;
        g_rightEncoder.temp_count = 0;
    }
}
