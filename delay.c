/**
  ******************************************************************************
  * @file    delay.c
  * @brief   精确延时 — 基于 CPU 周期计数（delay_cycles），不依赖 SysTick
  *          适配 MSPM0G350X + TI DriverLib
  ******************************************************************************
  */

#include "delay.h"

/**
 * @brief  毫秒级延时
 * @param  ms  延时毫秒数
 * @note   CPUCLK_FREQ / 1000 × ms = 每毫秒周期数 × ms，直接交给 delay_cycles
 */
void delay_ms(uint32_t ms)
{
    uint32_t cycles = (CPUCLK_FREQ / 1000) * ms;
    delay_cycles(cycles);
}

/**
 * @brief  微秒级延时
 * @param  us  延时微秒数
 * @note   微秒数量级较小，需要确保 (CPUCLK_FREQ / 1000000) 不丢失精度
 */
void delay_us(uint32_t us)
{
    uint32_t cycles = (CPUCLK_FREQ / 1000000) * us;
    delay_cycles(cycles);
}
