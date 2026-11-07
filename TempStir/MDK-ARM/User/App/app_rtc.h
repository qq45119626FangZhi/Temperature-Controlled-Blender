/*
 * app_rtc.h
 * 实时时钟（DS3231，I2C2）App 层封装：初始化、有效性判定、读写时间、1Hz 秒节拍。
 *
 * DS3231 掉电由纽扣电池维持走时，MCU 复位或重新烧录不影响时钟；
 * 本模块不做网络同步，校时入口为 App_RtcSetTime / App_RtcSetTimeFromString。
 *
 * 上电流程（App_RtcInit）：
 *   1) 探测器件；
 *   2) 读状态寄存器 OSF 判定时间是否可信；OSF=1（首次上电/换电池）时用
 *      __DATE__/__TIME__ 的编译时刻写入一次，并自动清 OSF，保证走时可用；
 *   3) 使能 INT/SQW 1Hz 方波，作为 DS_INT(PC12) 的秒节拍。
 */
#ifndef APP_RTC_H
#define APP_RTC_H

#include <stdint.h>
#include "dri_ds3231.h"

/* 初始化：探测 DS3231、按需写入编译时刻、使能 1Hz 方波。
 * 须在 vTaskStartScheduler() 之前调用。
 * 注意 App_StoreInit() 也会创建 I2C2 互斥量，二者顺序无关。 */
void App_RtcInit(void);

/* 1 = 器件在线且通讯正常。 */
uint8_t App_RtcIsReady(void);

/* 1 = 时间可信（OSF=0 且通讯正常）。0 表示掉电后尚未校准。 */
uint8_t App_RtcIsValid(void);

/* 1 = 已成功使能 1Hz 方波输出（DS_INT 秒节拍可用）。 */
uint8_t App_RtcIsSqwEnabled(void);

/* 1 = 本次上电检测到 OSF=1 且已用编译时刻重新初始化。
 * 说明掉电期间时间已丢失，最常见原因是纽扣电池未装/失效，可作为硬件判据。 */
uint8_t App_RtcWasAutoInit(void);

/* 读当前时间：1=成功。 */
uint8_t App_RtcGetTime(DS3231_Time_t *out);

/* 设置时间：1=成功（成功后 OSF 被清除）。 */
uint8_t App_RtcSetTime(const DS3231_Time_t *in);

/* 设置完整日期时间（星期按公历自动推算）：1=成功。
 * 供面板校时页使用；参数越界返回 0，RTC 不变。 */
uint8_t App_RtcSetDateTime(uint16_t year, uint8_t month, uint8_t day,
                           uint8_t hour, uint8_t minute, uint8_t second);

/* 当月天数（含闰年判断）：month 越界返回 31。 */
uint8_t App_RtcDaysInMonth(uint16_t year, uint8_t month);

/* 字符串校时，格式 "YYYY-MM-DD HH:MM:SS"（空格分隔，24 小时制）：1=成功。
 * 供串口命令使用；非法格式/越界返回 0，RTC 不变。 */
uint8_t App_RtcSetTimeFromString(const char *str);

#endif /* APP_RTC_H */
