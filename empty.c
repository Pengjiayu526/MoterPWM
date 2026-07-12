/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "delay.h"
#include "encoder.h"
#include "usart.h"

int main(void)
{
    uint32_t leftSpeed, rightSpeed;
    uint32_t leftRPM, rightRPM;

    SYSCFG_DL_init();       /* 系统及外设初始化 (GPIO, PWM, 时钟, 编码器, 定时器, 串口) */
    Motor_Init();           /* 启动 PWM 输出 */
    Encoder_Init();         /* 使能编码器中断 */
    USART_Init();           /* 使能串口中断 + printf 重定向 */

    printf("\r\n==== Motor + Encoder Demo (13-line, 1:28) ====\r\n\r\n");

    while (1) {
        /* 两轮 50% 前进 */
        Motor_SetSpeed(50, 50);
        delay_ms(2000);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("FWD 50%%  | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);

        /* 加速到 80% */
        Motor_SetSpeed(80, 80);
        delay_ms(2000);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("FWD 80%%  | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);

        /* 左转: 左轮慢(30%), 右轮快(80%) */
        Motor_SetSpeed(30, 80);
        delay_ms(1000);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("L-TURN   | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);

        /* 停止 1 秒 */
        Motor_SetSpeed(0, 0);
        delay_ms(1000);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("STOP     | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);

        /* 后退 40% */
        Motor_SetSpeed(-40, -40);
        delay_ms(1500);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("REV 40%%  | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);

        /* 刹车 */
        Motor_Brake();
        delay_ms(500);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("BRAKE    | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);

        /* 原地右转: 左轮前进, 右轮后退 */
        Motor_SetSpeed(40, -40);
        delay_ms(800);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("R-SPIN   | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);

        /* 停止 */
        Motor_SetSpeed(0, 0);
        delay_ms(1000);
        leftSpeed  = Encoder_GetSpeed(ENCODER_LEFT);
        rightSpeed = Encoder_GetSpeed(ENCODER_RIGHT);
        leftRPM    = Encoder_GetOutputRPM(ENCODER_LEFT);
        rightRPM   = Encoder_GetOutputRPM(ENCODER_RIGHT);
        printf("STOP     | L:%4lu pps %4lu RPM | R:%4lu pps %4lu RPM\r\n\n",
               leftSpeed, leftRPM, rightSpeed, rightRPM);
    }
}
