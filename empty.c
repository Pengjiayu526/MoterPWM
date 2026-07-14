/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * 10路灰度循迹 + 左右轮独立 PI 速度闭环
 *
 * 架构:
 *   LineFollow ISR (LINE_TIMER, 10ms) → 置 g_line_update_flag
 *   main() 循环:
 *     - 消费 g_line_update_flag → LineFollow_Update() (10ms)
 *       → 灰度读数 → 位置PD → 左右目标pps → SpeedControl_SetTargetPPS()
 *     - 串口遥测 (200ms)
 *   Timer ISR (TIMER_TICK, 20ms) → 编码器测速 → SpeedControl_Update() → Motor_SetSpeed()
 *
 * 控制层次:
 *   灰度传感器 → 线位置与循迹PD → 左右目标pps → 左右轮速度PI → PWM
 *
 * 注意:
 *   - 循迹模块禁止直接调用 Motor_SetSpeed()
 *   - 不在 ISR 中打印串口
 *   - 不在 ISR 中延时
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "encoder.h"
#include "usart.h"
#include "grayscale.h"
#include "speed_control.h"
#include "line_follow.h"
#include <stdio.h>

/*===========================================================================
 * 串口遥测周期: 200ms
 *
 * 循迹周期为 10ms, 每 20 个循迹周期打印一次。
 * 可修改 TELEMETRY_INTERVAL 调整遥测频率。
 *===========================================================================*/

#define TELEMETRY_INTERVAL  20U   /* 20 × 10ms = 200ms */

/*===========================================================================
 * 串口重定向 (printf 底层支持)
 *===========================================================================*/

int fputc(int ch, FILE *stream)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

int fputs(const char *restrict s, FILE *restrict stream)
{
    uint16_t char_len = 0;
    while (*s != 0)
    {
        while (DL_UART_isBusy(UART_0_INST) == true);
        DL_UART_Main_transmitData(UART_0_INST, *s++);
        char_len++;
    }
    return char_len;
}

int puts(const char *_ptr)
{
    return 0;
}

/*===========================================================================
 * 主函数
 *===========================================================================*/

int main(void)
{
    /*
     * 循迹配置参数
     *
     * 修改位置 (调参时改变以下数值):
     *   .kp              — 比例系数 (推荐 80~150)
     *   .ki              — 积分系数 (第一版保持 0)
     *   .kd              — 微分系数 (推荐 10~60, dt=10ms已吸收)
     *   .position_alpha  — 位置低通滤波 (0.3~0.7, 越小越平滑)
     *   .derivative_alpha — 微分低通滤波 (0.2~0.6)
     *   .base_pps        — 基础速度 pps (推荐 400~600)
     *   .max_turn_pps    — 转向分量限幅 (初始 400)
     *   .max_target_pps  — 单轮最大目标速度 (1800)
     */
    LineFollow_Config lineConfig = {
        .kp               = 100.0f,
        .ki               = 0.0f,
        .kd               = 40.0f,
        .position_alpha   = 0.4f,
        .derivative_alpha = 0.3f,
        .base_pps         = 800,
        .max_turn_pps     = 400,
        .max_target_pps   = 1800
    };

    uint32_t telemetryTick = 0U;
    char     txBuf[256];

    /*-----------------------------------------------------------------
     * 初始化顺序: 先关全局中断, 逐一初始化, 最后开中断
     *-----------------------------------------------------------------*/
    __disable_irq();

    SYSCFG_DL_init();       /* SysConfig 生成的时钟/GPIO/外设初始化 */

    Motor_Init();           /* PWM 输出初始化, 电机默认停止 */
    USART_Init();           /* 串口 (UART0) 中断初始化 */

    SpeedControl_Init();    /* 左右轮速度 PI 控制器初始化 */
    Encoder_Init();         /* 编码器 GPIO + TIMER_TICK (20ms) 中断初始化 */

    /*
     * 灰度循迹模块初始化
     * 必须在 SYSCFG_DL_init() 之后, __enable_irq() 之前。
     */
    LineFollow_Init(&lineConfig);
    LineFollow_TimerInit(); /* LINE_TIMER (10ms) 中断初始化 */

    /*
     * 全部初始化完成后使能全局中断。
     * 此后 TIMER_TICK (20ms) 和 LINE_TIMER (10ms) 都开始产生中断。
     */
    __enable_irq();

    /*
     * 启动循迹
     *
     * 调参期间可以暂时注释掉下面这行, 通过静态验证确认传感器读数正确
     * (参见指导文档第19节) 后再启用。
     */
    LineFollow_Start();

    /*-----------------------------------------------------------------
     * 主循环
     *-----------------------------------------------------------------*/
    while (1)
    {
        /*-------------------------------------------------------------
         * 10ms 循迹更新
         *
         * 由 LINE_TIMER ISR 触发 g_line_update_flag。
         * 不在 ISR 中处理是为了:
         *   1. 保持 ISR 极短 (只置标志)
         *   2. 浮点运算和串口调用在主线上下文中更安全
         *   3. 避免阻塞编码器中断
         *-------------------------------------------------------------*/
        if (g_line_update_flag != 0U)
        {
            g_line_update_flag = 0U;
            LineFollow_Update();
        }

        /*-------------------------------------------------------------
         * 200ms 串口遥测
         *
         * 每 20 个 10ms 周期 (即 200ms) 打印一次状态。
         * 使用 telemetryTick 计数器而非 delay_ms, 避免阻塞主循环。
         *
         * 遥测格式:
         *   GRAY BITMAP | ACTIVE | POS(milli) | ERR(milli) | TURN PPS |
         *   TARGET L/R | SPEED L/R | PWM L/R | STATE
         *
         * 注意: 全部使用整数格式, 浮点值 ×1000 转 milli 单位,
         *        避免 %f 增加代码体积和栈压力。
         *-------------------------------------------------------------*/
        telemetryTick++;
        if (telemetryTick >= TELEMETRY_INTERVAL)
        {
            telemetryTick = 0U;

            /* 浮点转整数 (毫单位), 避免 %f */
            int32_t posMilli   = (int32_t)(LineFollow_GetFilteredPosition() * 1000.0f);
            int32_t errorMilli = (int32_t)(LineFollow_GetError() * 1000.0f);

            snprintf(
                txBuf,
                sizeof(txBuf),
                "GRAY=0x%03X ACTIVE=%u POS=%ld ERR=%ld TURN=%ld "
                "TARGET=%ld/%ld SPEED=%lu/%lu PWM=%d/%d STATE=%u\r\n",
                (unsigned)LineFollow_GetSensorBitmap(),
                (unsigned)LineFollow_GetActiveCount(),
                (long)posMilli,
                (long)errorMilli,
                (long)LineFollow_GetTurnPPS(),
                (long)LineFollow_GetLeftTargetPPS(),
                (long)LineFollow_GetRightTargetPPS(),
                (unsigned long)Encoder_GetSpeed(ENCODER_LEFT),
                (unsigned long)Encoder_GetSpeed(ENCODER_RIGHT),
                (int)SpeedControl_GetLeftPWM(),
                (int)SpeedControl_GetRightPWM(),
                (unsigned)LineFollow_GetState());

            USART_SendString(txBuf);
        }
    }
}
