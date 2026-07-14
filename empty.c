/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * 左右轮独立 PI 速度闭环测试程序
 *
 * 架构:
 *   main() → 设置目标速度 + 串口打印 (200ms 周期)
 *   Timer ISR (20ms) → 编码器测速 → SpeedControl_Update() → Motor_SetSpeed()
 *
 * 注意:
 *   - 主循环不应再直接调用 Motor_SetSpeed()，PWM 全部由 PI 控制器在 ISR 中更新
 *   - 串口打印周期建议 200~500ms，不要在 20ms ISR 中打印
 */

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "delay.h"
#include "encoder.h"
#include "usart.h"
#include "speed_control.h"
#include <stdio.h>

//重定向fputc函数
int fputc(int ch, FILE *stream)
{
    while( DL_UART_isBusy(UART_0_INST) == true );
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

//重定向fputs函数
int fputs(const char* restrict s, FILE* restrict stream) {

    uint16_t char_len=0;
    while(*s!=0)
    {
        while( DL_UART_isBusy(UART_0_INST) == true );
        DL_UART_Main_transmitData(UART_0_INST, *s++);
        char_len++;
    }
    return char_len;
}
int puts(const char* _ptr)
{
 return 0;
}

int main(void)
{
    

    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    USART_Init();
    SpeedControl_Init();

    __enable_irq();

    
   
   //SpeedControl_SetTargetPPS(800, 800);
  while (1)
    {
        
        Grayscale_Print();

        delay_ms(200);
      
    }
}
