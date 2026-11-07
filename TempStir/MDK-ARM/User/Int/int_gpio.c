/*
 * int_gpio.c
 * 通用 GPIO 板级封装。所有引脚/有效电平引用 int_bsp_pins.h，不重复定义。
 * 电气属性（上拉/开漏/推挽）由 Core/Src/gpio.c 的 MX_GPIO_Init 负责，
 * 本文件只做读/写动作。
 */
#include "int_gpio.h"

uint8_t Int_GpioReadKeys(void)
{
    uint8_t keys = 0U;

    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == KEY_PRESSED_STATE)
    {
        keys |= 0x01U;
    }
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == KEY_PRESSED_STATE)
    {
        keys |= 0x02U;
    }
    if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == KEY_PRESSED_STATE)
    {
        keys |= 0x04U;
    }
    if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == KEY_PRESSED_STATE)
    {
        keys |= 0x08U;
    }

    return keys;
}

uint8_t Int_GpioIsHeatFault(void)
{
    return (HAL_GPIO_ReadPin(HEAT_FAULT_N_GPIO_Port, HEAT_FAULT_N_Pin) == HEAT_FAULT_ACTIVE_STATE) ? 1U : 0U;
}

void Int_GpioFaultReset(void)
{
    /* 拉低（ASSERT）触发锁存复位，保持数毫秒后释放（RELEASE=写高）。 */
    HAL_GPIO_WritePin(FAULT_RST_N_GPIO_Port, FAULT_RST_N_Pin, FAULT_RST_ASSERT_STATE);
    HAL_Delay(5);
    HAL_GPIO_WritePin(FAULT_RST_N_GPIO_Port, FAULT_RST_N_Pin, FAULT_RST_RELEASE_STATE);
}
