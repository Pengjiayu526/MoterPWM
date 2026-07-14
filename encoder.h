/**
 * @file  encoder.h
 * @brief 双路编码器速度测量 — 参考 11_PID_car 架构
 *        13线霍尔编码器, 减速比 1:28
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

/*===========================================================================
 * 编码器参数
 *===========================================================================*/

#define ENCODER_PULSES_PER_REV      13U     /**< 编码器线数 */
#define ENCODER_GEAR_RATIO          28U     /**< 减速比 1:28 */
#define ENCODER_OUTPUT_PPR          364U    /**< 输出轴每转脉冲数 13×28 */
#define ENCODER_SAMPLE_MS           20U     /**< 速度刷新周期 ms */

/* pps → RPM */
#define ENCODER_PPS_TO_OUTPUT_RPM(pps)  (((pps) * 60U) / ENCODER_OUTPUT_PPR)

/*===========================================================================
 * 调试变量 (extern, 主循环可直接读取用于三级排查)
 *===========================================================================*/

extern volatile uint32_t g_enc_edge_left;    /**< 左编码器边沿触发次数 */
extern volatile uint32_t g_enc_edge_right;   /**< 右编码器边沿触发次数 */
extern volatile uint32_t g_timer_ticks;      /**< TIMA0 ZERO 中断次数 */
extern volatile uint32_t g_group1_entry;     /**< GROUP1_IRQHandler 入口次数 */
extern volatile uint32_t g_gpioa_entry;      /**< GPIOA 中断源命中次数 */
extern volatile uint32_t g_gpio_pending;     /**< 最近一次 GPIO pending 寄存器值 */

/*===========================================================================
 * 接口
 *===========================================================================*/

typedef enum {
    ENCODER_LEFT  = 0,
    ENCODER_RIGHT = 1
} Encoder_Select;

void     Encoder_Init(void);
uint32_t Encoder_GetSpeed(Encoder_Select encoder);
void     Encoder_Reset(Encoder_Select encoder);

#endif /* ENCODER_H_ */
