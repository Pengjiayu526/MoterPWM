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
#include "icm42688.h"
#include "IMU.h"
#include "I2C_communication.h"

float ypr[3];          // 上传yaw pitch roll的值
/*
 * IMU 调度标志与计数器
 * ---------------------------------------------------------------
 * g_imu_update_flag : 20 ms 定时器 ISR 置 1，主循环清 0。
 *                     使用 volatile —— ISR 和主循环共享。
 * g_imu_timer_count : 定时器 ISR 每 20 ms 递增一次（只增不减）。
 * g_imu_missed_count : 上一次标志未被主循环处理时再次触发 ISR 的次数。
 * g_imu_update_count : 主循环实际完成姿态更新的次数（static，非 volatile）。
 */
volatile uint8_t  g_imu_update_flag  = 0U;
volatile uint32_t g_imu_timer_count  = 0U;
volatile uint32_t g_imu_missed_count = 0U;

static uint32_t g_imu_update_count  = 0U;
static uint32_t g_imu_success_count = 0U;
static uint32_t g_imu_error_count   = 0U;
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

/*
 * 陀螺仪定时器中断初始化
 * 使能 TIMER_0 的 NVIC 中断
 */
void TimeA1_Init(void)
{
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}

/*
陀螺仪定时器中断：周期20ms ， 中断内只设标志位，读取姿态角在主函数实现
*/
void TIMER_0_INST_IRQHandler(void)
{
	switch(DL_TimerG_getPendingInterrupt(TIMER_0_INST))
	{
		case DL_TIMER_IIDX_ZERO:
			g_imu_timer_count++;

			/*
			 * 如果上一次任务标志尚未被主循环清除，
			 * 说明主循环没有在一个 20 ms 周期内及时处理。
			 */
			if (g_imu_update_flag != 0U)
			{
				g_imu_missed_count++;
			}

			/* ISR 只发布任务，不执行任何 I²C 或姿态计算 */
			g_imu_update_flag = 1U;
			break;

		default:
			break;
	}
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
    uint8_t printDivider = 0U;
    /*-----------------------------------------------------------------
     * 初始化顺序: 先关全局中断, 逐一初始化, 最后开中断
     *-----------------------------------------------------------------*/
    __disable_irq();

    SYSCFG_DL_init();       /* SysConfig 生成的时钟/GPIO/外设初始化 */

    Motor_Init();           /* PWM 输出初始化, 电机默认停止 */
    USART_Init();           /* 串口 (UART0) 中断初始化 */

    SpeedControl_Init();    /* 左右轮速度 PI 控制器初始化 */
    Encoder_Init();         /* 编码器 GPIO + TIMER_TICK (20ms) 中断初始化 */

    IMU_init();
	TimeA1_Init();
    
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

        if (g_imu_update_flag != 0U)
		{
			/*
			 * 必须先清标志，再执行阻塞任务。
			 * 原因：如果 IMU 任务执行期间下一次 20 ms 中断到达，
			 * ISR 会再次把标志置 1。先清标志可以保留这个新请求。
			 */
			g_imu_update_flag = 0U;

			if (IMU_getYawPitchRoll(ypr) == 0)
			{
				g_imu_update_count++;
				g_imu_success_count++;
			}
			else
			{
				/*
				 * 失败时ypr保持上一帧值。
				 */
				g_imu_error_count++;
			}

			/* 5 × 20 ms = 100 ms，约 10 Hz 打印 */
			printDivider++;

			if (printDivider >= 5U)
			{
				printDivider = 0U;

				printf(
					"%.2f-%.2f-%.2f "
					"STATE=%u READY=%u "
					"OFFSET=%.4f/%.4f/%.4f "
					"TIMER=%lu UPDATE=%lu OK=%lu ERR=%lu MISS=%lu I2C=%d\r\n",
					ypr[0],
					ypr[1],
					ypr[2],
					(unsigned)IMU_GetState(),
					(unsigned)IMU_IsReady(),
					IMU_GetGyroOffsetX(),
					IMU_GetGyroOffsetY(),
					IMU_GetGyroOffsetZ(),
					(unsigned long)g_imu_timer_count,
					(unsigned long)g_imu_update_count,
					(unsigned long)g_imu_success_count,
					(unsigned long)g_imu_error_count,
					(unsigned long)g_imu_missed_count,
					(int)I2C_GetLastResult());
			}
		}
    }
}
