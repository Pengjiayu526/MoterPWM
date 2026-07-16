/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG7
#define PWM_0_INST_IRQHandler                                   TIMG7_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG7_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              1280000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_17
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM39)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM39_PF_TIMG7_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_24
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM54)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM54_PF_TIMG7_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_TICK */
#define TIMER_TICK_INST                                                  (TIMA0)
#define TIMER_TICK_INST_IRQHandler                              TIMA0_IRQHandler
#define TIMER_TICK_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define TIMER_TICK_INST_LOAD_VALUE                                       (7999U)
/* Defines for LINE_TIMER */
#define LINE_TIMER_INST                                                  (TIMG6)
#define LINE_TIMER_INST_IRQHandler                              TIMG6_IRQHandler
#define LINE_TIMER_INST_INT_IRQN                                (TIMG6_INT_IRQn)
#define LINE_TIMER_INST_LOAD_VALUE                                       (3999U)
/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (799U)




/* Defines for I2C_1 */
#define I2C_1_INST                                                          I2C1
#define I2C_1_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_1_INST_INT_IRQN                                        I2C1_INT_IRQn
#define I2C_1_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_1_SDA_PORT                                                GPIOB
#define GPIO_I2C_1_SDA_PIN                                         DL_GPIO_PIN_3
#define GPIO_I2C_1_IOMUX_SDA                                     (IOMUX_PINCM16)
#define GPIO_I2C_1_IOMUX_SDA_FUNC                      IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_1_SCL_PORT                                                GPIOB
#define GPIO_I2C_1_SCL_PIN                                         DL_GPIO_PIN_2
#define GPIO_I2C_1_IOMUX_SCL                                     (IOMUX_PINCM15)
#define GPIO_I2C_1_IOMUX_SCL_FUNC                      IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_PIN_22_PIN                                          (DL_GPIO_PIN_22)
#define LED_PIN_22_IOMUX                                         (IOMUX_PINCM50)
/* Defines for AIN1: GPIOB.16 with pinCMx 33 on package pin 4 */
#define AIN_AIN1_PORT                                                    (GPIOB)
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_16)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM33)
/* Defines for AIN2: GPIOA.0 with pinCMx 1 on package pin 33 */
#define AIN_AIN2_PORT                                                    (GPIOA)
#define AIN_AIN2_PIN                                             (DL_GPIO_PIN_0)
#define AIN_AIN2_IOMUX                                            (IOMUX_PINCM1)
/* Defines for BIN1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define BIN_BIN1_PORT                                                    (GPIOB)
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_15)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM32)
/* Defines for BIN2: GPIOA.1 with pinCMx 2 on package pin 34 */
#define BIN_BIN2_PORT                                                    (GPIOA)
#define BIN_BIN2_PIN                                             (DL_GPIO_PIN_1)
#define BIN_BIN2_IOMUX                                            (IOMUX_PINCM2)
/* Port definition for Pin Group A_Encode */
#define A_Encode_PORT                                                    (GPIOA)

/* Defines for AA: GPIOA.12 with pinCMx 34 on package pin 5 */
// groups represented: ["B_Encode","A_Encode"]
// pins affected: ["BA","AA"]
#define GPIO_MULTIPLE_GPIOA_INT_IRQN                            (GPIOA_INT_IRQn)
#define GPIO_MULTIPLE_GPIOA_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define A_Encode_AA_IIDX                                    (DL_GPIO_IIDX_DIO12)
#define A_Encode_AA_PIN                                         (DL_GPIO_PIN_12)
#define A_Encode_AA_IOMUX                                        (IOMUX_PINCM34)
/* Defines for AB: GPIOA.13 with pinCMx 35 on package pin 6 */
#define A_Encode_AB_PIN                                         (DL_GPIO_PIN_13)
#define A_Encode_AB_IOMUX                                        (IOMUX_PINCM35)
/* Port definition for Pin Group B_Encode */
#define B_Encode_PORT                                                    (GPIOA)

/* Defines for BA: GPIOA.9 with pinCMx 20 on package pin 55 */
#define B_Encode_BA_IIDX                                     (DL_GPIO_IIDX_DIO9)
#define B_Encode_BA_PIN                                          (DL_GPIO_PIN_9)
#define B_Encode_BA_IOMUX                                        (IOMUX_PINCM20)
/* Defines for BB: GPIOA.8 with pinCMx 19 on package pin 54 */
#define B_Encode_BB_PIN                                          (DL_GPIO_PIN_8)
#define B_Encode_BB_IOMUX                                        (IOMUX_PINCM19)
/* Defines for PIN_1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define Grays_PIN_1_PORT                                                 (GPIOA)
#define Grays_PIN_1_PIN                                         (DL_GPIO_PIN_25)
#define Grays_PIN_1_IOMUX                                        (IOMUX_PINCM55)
/* Defines for PIN_2: GPIOA.22 with pinCMx 47 on package pin 18 */
#define Grays_PIN_2_PORT                                                 (GPIOA)
#define Grays_PIN_2_PIN                                         (DL_GPIO_PIN_22)
#define Grays_PIN_2_IOMUX                                        (IOMUX_PINCM47)
/* Defines for PIN_3: GPIOB.25 with pinCMx 56 on package pin 27 */
#define Grays_PIN_3_PORT                                                 (GPIOB)
#define Grays_PIN_3_PIN                                         (DL_GPIO_PIN_25)
#define Grays_PIN_3_IOMUX                                        (IOMUX_PINCM56)
/* Defines for PIN_4: GPIOB.24 with pinCMx 52 on package pin 23 */
#define Grays_PIN_4_PORT                                                 (GPIOB)
#define Grays_PIN_4_PIN                                         (DL_GPIO_PIN_24)
#define Grays_PIN_4_IOMUX                                        (IOMUX_PINCM52)
/* Defines for PIN_5: GPIOB.20 with pinCMx 48 on package pin 19 */
#define Grays_PIN_5_PORT                                                 (GPIOB)
#define Grays_PIN_5_PIN                                         (DL_GPIO_PIN_20)
#define Grays_PIN_5_IOMUX                                        (IOMUX_PINCM48)
/* Defines for PIN_6: GPIOA.14 with pinCMx 36 on package pin 7 */
#define Grays_PIN_6_PORT                                                 (GPIOA)
#define Grays_PIN_6_PIN                                         (DL_GPIO_PIN_14)
#define Grays_PIN_6_IOMUX                                        (IOMUX_PINCM36)
/* Defines for PIN_7: GPIOB.19 with pinCMx 45 on package pin 16 */
#define Grays_PIN_7_PORT                                                 (GPIOB)
#define Grays_PIN_7_PIN                                         (DL_GPIO_PIN_19)
#define Grays_PIN_7_IOMUX                                        (IOMUX_PINCM45)
/* Defines for PIN_8: GPIOA.27 with pinCMx 60 on package pin 31 */
#define Grays_PIN_8_PORT                                                 (GPIOA)
#define Grays_PIN_8_PIN                                         (DL_GPIO_PIN_27)
#define Grays_PIN_8_IOMUX                                        (IOMUX_PINCM60)
/* Defines for PIN_9: GPIOA.30 with pinCMx 5 on package pin 37 */
#define Grays_PIN_9_PORT                                                 (GPIOA)
#define Grays_PIN_9_PIN                                         (DL_GPIO_PIN_30)
#define Grays_PIN_9_IOMUX                                         (IOMUX_PINCM5)
/* Defines for PIN_10: GPIOB.0 with pinCMx 12 on package pin 47 */
#define Grays_PIN_10_PORT                                                (GPIOB)
#define Grays_PIN_10_PIN                                         (DL_GPIO_PIN_0)
#define Grays_PIN_10_IOMUX                                       (IOMUX_PINCM12)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_TICK_init(void);
void SYSCFG_DL_LINE_TIMER_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_UART_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
