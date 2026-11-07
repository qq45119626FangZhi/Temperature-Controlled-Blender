/*
 * int_bsp_i2c2.h
 * I2C2 阻塞式访问与 FreeRTOS 互斥保护接口，供低温漂外部时钟（RTC）
 * 与参数存储（EEPROM）共用。
 */
#ifndef BSP_I2C2_H
#define BSP_I2C2_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* 创建 I2C2 互斥量；应在创建相关任务之前调用一次。 */
void Bsp_I2c2Init(void);

/*
 * 将 PB10/PB11 临时切换为开漏 GPIO，输出最多 9 个恢复时钟和 STOP，随后
 * 通过 MX_I2C2_Init() 恢复外设。仅允许在启动阶段、总线无并发访问时调用。
 */
HAL_StatusTypeDef Bsp_I2c2RecoverBus(void);

/*
 * 以下接口均为阻塞式 HAL 封装，设备地址沿用 HAL 约定（7 bit 地址左移一位）。
 * RTC 与 EEPROM 分处不同从机地址，调度器运行后由互斥量串行化访问；
 * 不得从中断服务函数调用。
 */
HAL_StatusTypeDef Bsp_I2c2MasterTransmit(uint16_t devAddress, const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c2MemWrite(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,const uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c2MemRead(uint16_t devAddress, uint16_t memAddress, uint16_t memAddSize,uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef Bsp_I2c2IsDeviceReady(uint16_t devAddress, uint32_t trials, uint32_t timeout);

#endif /* BSP_I2C2_H */
