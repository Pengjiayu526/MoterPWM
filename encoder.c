/**
 * @file  encoder.c
 * @brief 双路编码器速度测量 — 参照 11_PID_car 架构 + 三级排查调试
 *
 * GPIO ISR: DL_GPIO_getPendingInterrupt → switch/case IIDX
 * Timer ISR: DL_Timer_getPendingInterrupt → switch/case ZERO
 */

#include "encoder.h"
#include "speed_control.h"
#include "ti_msp_dl_config.h"

/*===========================================================================
 * 调试计数器 — 只增不减，用于三级排查
 *===========================================================================*/

volatile uint32_t g_enc_edge_left  = 0;   /**< PA12 边沿触发次数 */
volatile uint32_t g_enc_edge_right = 0;   /**< PA8 边沿触发次数 */
volatile uint32_t g_timer_ticks    = 0;   /**< TIMA0 ZERO 中断次数 */
volatile uint32_t g_group1_entry   = 0;   /**< GROUP1 ISR 入口次数 */
volatile uint32_t g_gpioa_entry    = 0;   /**< GPIOA 中断源命中次数 */
volatile uint32_t g_gpio_pending   = 0;   /**< 最近一次 GPIO pending 值 */

/*===========================================================================
 * 运行时数据
 *===========================================================================*/

/** @brief 20ms 窗口内的脉冲计数 (GPIO ISR 累加, Timer ISR 清零) */
static volatile uint32_t g_speed_counter_left  = 0;
static volatile uint32_t g_speed_counter_right = 0;

/** @brief 速度快照 pps (Timer ISR 更新, 主循环通过 Encoder_GetSpeed 读取) */
static volatile uint32_t g_speed_left  = 0;
static volatile uint32_t g_speed_right = 0;

/*===========================================================================
 * 公有函数
 *===========================================================================*/

void Encoder_Init(void)
{
    uint32_t encoderPins =
        A_Encode_AA_PIN |
        B_Encode_BA_PIN;

    /*
     * SysConfig 已经使能了引脚中断，
     * 这里再次显式使能，排除生成配置未更新的问题。
     */
    DL_GPIO_disableInterrupt(GPIOA, encoderPins);
    DL_GPIO_clearInterruptStatus(GPIOA, encoderPins);
    DL_GPIO_enableInterrupt(GPIOA, encoderPins);

    /*
     * 左右编码器都在 GPIOA，使能 GROUP1 / GPIOA_INT_IRQn。
     */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);

    /* TIMA0 定时器中断 — 速度刷新 */
    NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
}

uint32_t Encoder_GetSpeed(Encoder_Select encoder)
{
    if (encoder == ENCODER_LEFT) {
        return g_speed_left;
    } else {
        return g_speed_right;
    }
}

void Encoder_Reset(Encoder_Select encoder)
{
    if (encoder == ENCODER_LEFT) {
        g_speed_counter_left = 0;
        g_speed_left         = 0;
    } else {
        g_speed_counter_right = 0;
        g_speed_right         = 0;
    }
}

/*===========================================================================
 * GPIOA 中断
 *
 * 三层排查信息:
 *   g_group1_entry  — GROUP1 ISR 是否被调用
 *   g_gpioa_entry   — GPIOA 是否确认为中断源
 *   g_gpio_pending  — GPIOA MIS 寄存器原始值 (看哪些引脚有中断)
 *   g_enc_edge_*    — 左右编码器各自触发次数
 *===========================================================================*/

void GROUP1_IRQHandler(void)
{
    uint32_t groupPending;
    uint32_t gpioPending;
    uint32_t encoderPins =
        A_Encode_AA_PIN |
        B_Encode_BA_PIN;

    g_group1_entry++;

    /* 读取 GROUP1 中当前最高优先级的中断源 */
    groupPending =
        DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

    /* 确认是 GPIOA 触发了 GROUP1 */
    if (groupPending == DL_INTERRUPT_GROUP1_IIDX_GPIOA)
    {
        g_gpioa_entry++;

        gpioPending =
            DL_GPIO_getEnabledInterruptStatus(
                GPIOA,
                encoderPins);

        g_gpio_pending = gpioPending;

        /* 左编码器 AA = PA12 */
        if ((gpioPending & A_Encode_AA_PIN) != 0U)
        {
            g_enc_edge_left++;
            g_speed_counter_left++;
        }

        /* 右编码器 BA = PA8 */
        if ((gpioPending & B_Encode_BA_PIN) != 0U)
        {
            g_enc_edge_right++;
            g_speed_counter_right++;
        }

        /* 清除已处理的中断标志 */
        DL_GPIO_clearInterruptStatus(
            GPIOA,
            gpioPending & encoderPins);
    }
}

/*===========================================================================
 * TIMA0 中断 — 每 20ms 快照 speed_counter → 换算 speed → 清零 counter
 *===========================================================================*/

void TIMER_TICK_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_TICK_INST))
    {
    case DL_TIMER_IIDX_ZERO:
        g_timer_ticks++;

        /* pps = counter × (1000ms / 20ms) = counter × 50 */
        g_speed_left  = g_speed_counter_left  * (1000U / ENCODER_SAMPLE_MS);
        g_speed_right = g_speed_counter_right * (1000U / ENCODER_SAMPLE_MS);

        /* 清零窗口计数器, 开始下一个 20ms 周期 */
        g_speed_counter_left  = 0;
        g_speed_counter_right = 0;

        /*
         * PI 速度闭环更新 — 必须在速度计算之后调用。
         * 该函数内部调用 Motor_SetSpeed()，主循环不应再直接调用。
         */
        SpeedControl_Update(g_speed_right, g_speed_left);
        break;

    default:
        break;
    }
}
