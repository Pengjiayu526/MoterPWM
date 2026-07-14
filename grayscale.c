#include "grayscale.h"
#include "usart.h"

/* ================================================================
 *  十路灰度传感器引脚查找表
 *  10-way grayscale sensor pin lookup table
 *
 *  引脚分布在 GPIOA 和 GPIOB 两个端口:
 *  CH0 -> PA25, CH1 -> PA22, CH2 -> PB25
 *  CH3 -> PB24, CH4 -> PB20, CH5 -> PA14
 *  CH6 -> PB19, CH7 -> PA27, CH8 -> PA30
 *  CH9 -> PB0
 * ================================================================ */

typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
} GrayscalePin_t;

static const GrayscalePin_t gGrayscalePins[GRAYSCALE_CHANNEL_COUNT] = {
    { Grays_PIN_1_PORT,  Grays_PIN_1_PIN  },  /* CH0: PA25 */
    { Grays_PIN_2_PORT,  Grays_PIN_2_PIN  },  /* CH1: PA22 */
    { Grays_PIN_3_PORT,  Grays_PIN_3_PIN  },  /* CH2: PB25 */
    { Grays_PIN_4_PORT,  Grays_PIN_4_PIN  },  /* CH3: PB24 */
    { Grays_PIN_5_PORT,  Grays_PIN_5_PIN  },  /* CH4: PB20 */
    { Grays_PIN_6_PORT,  Grays_PIN_6_PIN  },  /* CH5: PA14 */
    { Grays_PIN_7_PORT,  Grays_PIN_7_PIN  },  /* CH6: PB19 */
    { Grays_PIN_8_PORT,  Grays_PIN_8_PIN  },  /* CH7: PA27 */
    { Grays_PIN_9_PORT,  Grays_PIN_9_PIN  },  /* CH8: PA30 */
    { Grays_PIN_10_PORT, Grays_PIN_10_PIN },  /* CH9: PB0  */
};

/**
 * @brief 读取单个灰度传感器通道的电平
 *        Read the level of a single grayscale sensor channel
 * @param channel 通道号 0~11
 * @return 0=低电平(黑/暗), 1=高电平(白/亮)
 *         0=low (dark), 1=high (bright)
 */
uint8_t Grayscale_ReadChannel(uint8_t channel)
{
    if (channel >= GRAYSCALE_CHANNEL_COUNT)
    {
        return 0;
    }

    /* DL_GPIO_readPins 返回该引脚所在位的值: 高电平返回非零, 低电平返回0
       DL_GPIO_readPins returns the pin bit value: non-zero = high, 0 = low */
    if (DL_GPIO_readPins(gGrayscalePins[channel].port,
                         gGrayscalePins[channel].pin) &
        gGrayscalePins[channel].pin)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 读取全部12路灰度传感器, 结果存入16位变量
 *        Read all 12 grayscale channels, return a 16-bit bitmap
 * @return bit0~bit11 对应通道0~11 (1=高电平, 0=低电平)
 *         bit0~bit11 correspond to channel 0~11 (1=high, 0=low)
 */
uint16_t Grayscale_ReadAll(void)
{
    uint16_t result = 0;
    uint8_t  i;

    for (i = 0; i < GRAYSCALE_CHANNEL_COUNT; i++)
    {
        if (DL_GPIO_readPins(gGrayscalePins[i].port,
                             gGrayscalePins[i].pin) &
            gGrayscalePins[i].pin)
        {
            result |= (1 << i);
        }
    }

    return result;
}

/**
 * @brief 通过串口打印全部12路灰度传感器的高低电平
 *        Print all 12 grayscale channel levels via UART
 *
 * 输出格式: "Grayscale: CH00:1 CH01:0 CH02:1 ... CH11:0\r\n"
 * Output format: high=1, low=0
 */
void Grayscale_Print(void)
{
    uint8_t i;
    uint8_t level;
    char    buf[8];

    USART_SendString("Grayscale: ");

    for (i = 0; i < GRAYSCALE_CHANNEL_COUNT; i++)
    {
        level = Grayscale_ReadChannel(i);

        /* 组装 "CHxx:x" 字符串  Build "CHxx:x" string */
        buf[0] = 'C';
        buf[1] = 'H';
        buf[2] = '0' + (i / 10);
        buf[3] = '0' + (i % 10);
        buf[4] = ':';
        buf[5] = '0' + level;
        buf[6] = '\0';
        USART_SendString(buf);

        /* 通道间加空格, 行末加回车换行
           Space between channels, CR+LF at end of line */
        if (i < GRAYSCALE_CHANNEL_COUNT - 1)
        {
            USART_SendData(' ');
        }
    }

    USART_SendString("\r\n");
}
