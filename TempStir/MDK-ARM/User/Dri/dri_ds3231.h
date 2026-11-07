/*
 * dri_ds3231.h
 * DS3231 高精度 RTC 驱动（I2C2，与 M24C02 共用总线，由 bsp_i2c2 串行化）。
 *
 * 器件：DS3231MZ+TRL，7 bit 地址 0x68；INT/SQW 接 PC12（DS_INT）。
 * 掉电走时由纽扣电池维持，MCU 复位/重新烧录不影响时间。
 *
 * 约定：
 *  - 全部为阻塞式 I2C 轮询，不得在中断中调用。
 *  - 时间单位：year 为 2000~2099 的完整年份，其余为 1~31 / 0~23 / 0~59。
 *  - OSF（振荡器停止标志，状态寄存器 0x0F bit7）：掉电/首次上电后为 1，
 *    表示时间不可信；写时间后自动清除，也可用 DS3231_ClearOsf() 单独清。
 */
#ifndef DRI_DS3231_H
#define DRI_DS3231_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* DS3231 的 7 bit 地址为 0x68；HAL 接口要求左移一位后的地址。 */
#define DS3231_I2C_ADDR_7BIT   0x68U
#define DS3231_I2C_ADDR        (DS3231_I2C_ADDR_7BIT << 1)   /* = 0xD0 */

/* 寄存器地址 */
#define DS3231_REG_SECONDS     0x00U
#define DS3231_REG_MINUTES     0x01U
#define DS3231_REG_HOURS       0x02U
#define DS3231_REG_DAY         0x03U
#define DS3231_REG_DATE        0x04U
#define DS3231_REG_MONTH       0x05U
#define DS3231_REG_YEAR        0x06U
#define DS3231_REG_CONTROL     0x0EU   /* bit7 EOSC / bit6 BBSQW / bit2 INTCN / bit4-3 RS */
#define DS3231_REG_STATUS      0x0FU   /* bit7 OSF / bit3 EN32kHz / bit0 A1F */
#define DS3231_REG_AGING       0x10U
#define DS3231_REG_TEMP_MSB    0x11U

/* 状态寄存器位 */
#define DS3231_STATUS_OSF      0x80U
/* 控制寄存器位（1Hz 方波：INTCN=0、RS=00、EOSC=0 允许电池供电时也走时） */
#define DS3231_CTRL_SQW_1HZ    0x00U

typedef struct
{
    uint16_t year;     /* 2000 ~ 2099 */
    uint8_t  month;    /* 1 ~ 12 */
    uint8_t  day;      /* 1 ~ 31 */
    uint8_t  hour;     /* 0 ~ 23 */
    uint8_t  minute;   /* 0 ~ 59 */
    uint8_t  second;   /* 0 ~ 59 */
    uint8_t  weekday;  /* 1 ~ 7（1=周日，DS3231 定义） */
} DS3231_Time_t;

/* 在 I2C2 上轮询器件 ACK；就绪返回 1。 */
uint8_t DS3231_IsReady(void);

/* 初始化：探测器件并读一次时间验证通讯；成功返回 1。 */
uint8_t DS3231_Init(void);

/* 读当前时间；成功返回 1，失败返回 0（结构体内容不变）。 */
uint8_t DS3231_GetTime(DS3231_Time_t *out);

/* 设置时间（同时清 OSF 标志）；成功返回 1。 */
uint8_t DS3231_SetTime(const DS3231_Time_t *in);

/* 读状态寄存器 0x0F；成功返回 1。 */
uint8_t DS3231_ReadStatus(uint8_t *status);

/* 清 OSF 标志（保留状态寄存器其他位）；成功返回 1。 */
uint8_t DS3231_ClearOsf(void);

/* 时间是否可信：OSF=0 且状态可读（1=可信）。 */
uint8_t DS3231_IsTimeValid(void);

/* 使能 INT/SQW 引脚 1Hz 方波输出（写控制寄存器 0x0E）；成功返回 1。 */
uint8_t DS3231_Enable1HzSqw(void);

/* 关闭方波输出并把 INT/SQW 交还给闹钟逻辑（INTCN=1，无闹钟使能即不再翻转）。 */
uint8_t DS3231_DisableSqw(void);

#endif /* DRI_DS3231_H */
