/*
 * int_spi.h
 * BSP 层 SPI2 封装：面向 LSM6DSM 等挂接在 SPI2 上的器件，提供寄存器读/写，
 * 片选（CS，PB12）由本封装控制。SPI2 句柄 hspi2 由 Core 层 MX_SPI2_Init 初始化。
 */
#ifndef INT_SPI_H
#define INT_SPI_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "spi.h"

/* 初始化 SPI2 封装：确保 CS 释放到无效电平（MX_SPI2_Init 已先被 Core 调用）。 */
void Int_SpiInit(void);

/* 通过 SPI2 读/写单寄存器（地址 7bit）；CS 在本函数内自动拉低/释放。 */
uint8_t Int_SpiReadReg(uint8_t reg);
void Int_SpiWriteReg(uint8_t reg, uint8_t val);

/* 连续读/写多个寄存器（利用器件地址自增，要求 CTRL3_C.IF_INC=1，LSM6DSM 默认即开）。
 * 用于 6 轴一次突发读取（0x20~0x2D 共 14 字节），避免逐字节读的额外 CS 开销。
 * len 上限 23（内部栈缓冲 24 字节）；成功返回 1。 */
uint8_t Int_SpiReadBuf(uint8_t reg, uint8_t *out, uint16_t len);
uint8_t Int_SpiWriteBuf(uint8_t reg, const uint8_t *in, uint16_t len);

/* LSM6DSM WHO_AM_I 寄存器（用于验证 SPI 链路是否连通）。 */
#define LSM6DSM_WHO_AM_I_REG  0x0FU
#define LSM6DSM_WHO_AM_I_VAL  0x6AU

#endif /* INT_SPI_H */
