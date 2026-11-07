/*
 * int_gpio.h
 * BSP 层通用 GPIO 封装：按键输入、硬件故障锁存读/复位。
 * 引脚与有效电平一律来自 int_bsp_pins.h，禁止在本文件硬编码端口/引脚/电平。
 */
#ifndef INT_GPIO_H
#define INT_GPIO_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "int_bsp_pins.h"

/* 读取 4 个按键，返回位图：bit0=KEY1, bit1=KEY2, bit2=KEY3, bit3=KEY4。
 * 按下（低有效，KEY_PRESSED_STATE）记为 1。 */
uint8_t Int_GpioReadKeys(void);

/* 读取加热故障锁存：返回 1 表示有故障（HEAT_FAULT_N 低有效）。 */
uint8_t Int_GpioIsHeatFault(void);

/* 复位硬件故障锁存：FAULT_RST_N 拉低脉冲后释放（写高由外部上拉建立高电平）。 */
void Int_GpioFaultReset(void);

#endif /* INT_GPIO_H */
