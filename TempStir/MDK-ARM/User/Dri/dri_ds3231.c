/*
 * dri_ds3231.c
 * DS3231 RTC 驱动实现：7 字节连续读写（秒/分/时/周/日/月/年），BCD 与二进制互转。
 *
 * 说明：
 *  - 只经 bsp_i2c2 访问总线（I2C2 与 M24C02 共用，互斥由 BSP 层处理）。
 *  - 仅支持 24 小时制写入；读取时兼容芯片处于 12 小时制的情况。
 *  - 掉电走时依赖纽扣电池，本驱动不负责电池检测。
 */
#include "dri_ds3231.h"
#include "int_bsp_i2c2.h"

#define DS3231_TIMEOUT_MS     50U
#define DS3231_READY_TRIALS   10U

/* BCD -> 二进制 */
static uint8_t DS3231_Bcd2Bin(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10U) + (bcd & 0x0FU));
}

/* 二进制 -> BCD */
static uint8_t DS3231_Bin2Bcd(uint8_t bin)
{
    return (uint8_t)(((bin / 10U) << 4) | (bin % 10U));
}

/* 单寄存器读写（状态/控制寄存器用）。 */
static uint8_t DS3231_ReadReg(uint8_t reg, uint8_t *val)
{
    if (val == NULL)
    {
        return 0U;
    }

    if (Bsp_I2c2MemRead(DS3231_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1U, DS3231_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t DS3231_WriteReg(uint8_t reg, uint8_t val)
{
    if (Bsp_I2c2MemWrite(DS3231_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, DS3231_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

uint8_t DS3231_IsReady(void)
{
    if (Bsp_I2c2IsDeviceReady(DS3231_I2C_ADDR, DS3231_READY_TRIALS, DS3231_TIMEOUT_MS) == HAL_OK)
    {
        return 1U;
    }
    return 0U;
}

uint8_t DS3231_GetTime(DS3231_Time_t *out)
{
    uint8_t buf[7];
    uint8_t hourRaw;
    uint8_t hour;

    if (out == NULL)
    {
        return 0U;
    }

    if (Bsp_I2c2MemRead(DS3231_I2C_ADDR, DS3231_REG_SECONDS, I2C_MEMADD_SIZE_8BIT,
                        buf, 7U, DS3231_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    hourRaw = buf[DS3231_REG_HOURS];

    if ((hourRaw & 0x40U) != 0U)
    {
        /* 12 小时制：bit5 = PM，低 5 位为 BCD 小时 */
        hour = DS3231_Bcd2Bin((uint8_t)(hourRaw & 0x1FU));
        if ((hourRaw & 0x20U) != 0U)
        {
            if (hour < 12U)
            {
                hour = (uint8_t)(hour + 12U);
            }
        }
        else if (hour == 12U)
        {
            hour = 0U;   /* 12 AM -> 0 点 */
        }
    }
    else
    {
        hour = DS3231_Bcd2Bin((uint8_t)(hourRaw & 0x3FU));
    }

    out->second  = DS3231_Bcd2Bin((uint8_t)(buf[DS3231_REG_SECONDS] & 0x7FU));
    out->minute  = DS3231_Bcd2Bin((uint8_t)(buf[DS3231_REG_MINUTES] & 0x7FU));
    out->hour    = hour;
    out->weekday = (uint8_t)(buf[DS3231_REG_DAY] & 0x07U);
    out->day     = DS3231_Bcd2Bin((uint8_t)(buf[DS3231_REG_DATE] & 0x3FU));
    out->month   = DS3231_Bcd2Bin((uint8_t)(buf[DS3231_REG_MONTH] & 0x1FU));
    out->year    = (uint16_t)(2000U + DS3231_Bcd2Bin(buf[DS3231_REG_YEAR]));

    if (out->weekday == 0U)
    {
        out->weekday = 1U;
    }

    return 1U;
}

uint8_t DS3231_SetTime(const DS3231_Time_t *in)
{
    uint8_t buf[7];
    uint8_t weekday;

    if (in == NULL)
    {
        return 0U;
    }

    weekday = (in->weekday == 0U) ? 1U : (uint8_t)(in->weekday & 0x07U);

    buf[DS3231_REG_SECONDS] = DS3231_Bin2Bcd((uint8_t)(in->second % 60U));
    buf[DS3231_REG_MINUTES] = DS3231_Bin2Bcd((uint8_t)(in->minute % 60U));
    buf[DS3231_REG_HOURS]   = DS3231_Bin2Bcd((uint8_t)(in->hour % 24U));   /* bit6=0 -> 24 小时制 */
    buf[DS3231_REG_DAY]     = weekday;
    buf[DS3231_REG_DATE]    = DS3231_Bin2Bcd((uint8_t)(in->day % 32U));
    buf[DS3231_REG_MONTH]   = DS3231_Bin2Bcd((uint8_t)(in->month % 13U));
    buf[DS3231_REG_YEAR]    = DS3231_Bin2Bcd((uint8_t)((in->year >= 2000U) ? (in->year - 2000U) : 0U));

    if (Bsp_I2c2MemWrite(DS3231_I2C_ADDR, DS3231_REG_SECONDS, I2C_MEMADD_SIZE_8BIT,
                         buf, 7U, DS3231_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    /* 写时间即视为“已校准”：清 OSF。清标志失败不影响时间已写入，
     * 调用方可用 DS3231_IsTimeValid() 复核。 */
    (void)DS3231_ClearOsf();

    return 1U;
}

uint8_t DS3231_ReadStatus(uint8_t *status)
{
    return DS3231_ReadReg(DS3231_REG_STATUS, status);
}

uint8_t DS3231_ClearOsf(void)
{
    uint8_t status;

    if (DS3231_ReadReg(DS3231_REG_STATUS, &status) == 0U)
    {
        return 0U;
    }

    /* EN32kHz/BSY/A2F/A1F 由器件维护，写回时保持原值，只清 bit7。 */
    return DS3231_WriteReg(DS3231_REG_STATUS, (uint8_t)(status & (uint8_t)(~DS3231_STATUS_OSF)));
}

uint8_t DS3231_IsTimeValid(void)
{
    uint8_t status;

    if (DS3231_ReadStatus(&status) == 0U)
    {
        return 0U;
    }

    return ((status & DS3231_STATUS_OSF) == 0U) ? 1U : 0U;
}

uint8_t DS3231_Enable1HzSqw(void)
{
    /* INTCN=0 且 RS=00 -> INT/SQW 输出 1Hz 方波；EOSC=0 允许电池供电时继续走时。 */
    return DS3231_WriteReg(DS3231_REG_CONTROL, DS3231_CTRL_SQW_1HZ);
}

uint8_t DS3231_DisableSqw(void)
{
    uint8_t ctrl;

    if (DS3231_ReadReg(DS3231_REG_CONTROL, &ctrl) == 0U)
    {
        return 0U;
    }

    /* INTCN=1：INT/SQW 改由闹钟驱动（当前未使能任何闹钟，引脚保持不翻转）。 */
    return DS3231_WriteReg(DS3231_REG_CONTROL, (uint8_t)(ctrl | 0x04U));
}

uint8_t DS3231_Init(void)
{
    DS3231_Time_t t;

    if (DS3231_IsReady() == 0U)
    {
        return 0U;
    }

    /* 读一次时间，确认寄存器可访问（不校验 OSF，避免把正常走时的芯片误判为未初始化）。 */
    return DS3231_GetTime(&t);
}
