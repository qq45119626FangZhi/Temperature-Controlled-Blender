/*
 * int_bsp_i2c1.h
* I2C1 阻塞式访问与 FreeRTOS 互斥保护接口，当前由 OLED 使用。
 */
#ifndef BSP_I2C1_H
#define BSP_I2C1_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* 创建 I2C1 互斥量；应在创建相关任务之前调用一次。 */
void Bsp_I2c1Init(void);

/*
 * 将 PB6/PB7 临时切换为开漏 GPIO，输出最多 9 个恢复时钟和 STOP，随后
 * 通过 MX_I2C1_Init() 恢复外设。仅允许在启动阶段、总线无并发访问时调用。
 */
HAL_StatusTypeDef Bsp_I2c1RecoverBus(void);

/*
 * 以下接口均为阻塞式 HAL 封装，设备地址沿用 HAL 约定（7 bit 地址左移一位）。
 * 调度器运行后自动串行化访问；不得从中断服务函数调用。
 */
HAL_StatusTypeDef Bsp_I2c1MasterTransmit(uint16_t devAddress, const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c1MemWrite(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c1MemRead(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c1IsDeviceReady(uint16_t devAddress, uint32_t trials, uint32_t timeout);

#endif /* BSP_I2C1_H */
