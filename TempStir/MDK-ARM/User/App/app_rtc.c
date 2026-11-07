/*
 * app_rtc.c
 * DS3231 的 App 层薄封装：在线/有效性标志、编译时刻兜底校时、1Hz 秒节拍、
 * 字符串校时（供串口命令使用）。
 */
#include "app_rtc.h"
#include <stdio.h>   /* sscanf：解析校时字符串 */

static uint8_t app_rtc_ready = 0U;   /* 1 = 器件在线 */
static uint8_t app_rtc_valid = 0U;   /* 1 = OSF 已清，时间可信 */
static uint8_t app_rtc_sqw   = 0U;   /* 1 = 1Hz 方波已使能 */
static uint8_t app_rtc_autoInit = 0U; /* 1 = 本次上电因 OSF=1 用编译时刻初始化过 */

/* 当月天数（含闰年判断）。 */
uint8_t App_RtcDaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t dim[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    uint8_t days;

    if ((month < 1U) || (month > 12U))
    {
        return 31U;
    }

    days = dim[month - 1U];

    if ((month == 2U) &&
        (((year % 4U) == 0U) && (((year % 100U) != 0U) || ((year % 400U) == 0U))))
    {
        days = 29U;
    }

    return days;
}

/* 星期推算（Sakamoto 算法）：返回 1=周日 … 7=周六，与 DS3231 定义一致。 */
static uint8_t App_Rtc_Weekday(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t t[12] = {0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U};
    uint16_t y = year;
    uint32_t sum;

    if ((month < 1U) || (month > 12U) || (day < 1U) || (day > 31U))
    {
        return 1U;
    }

    if (month < 3U)
    {
        y--;
    }

    sum = (uint32_t)y + (uint32_t)(y / 4U) - (uint32_t)(y / 100U) + (uint32_t)(y / 400U)
        + (uint32_t)t[month - 1U] + (uint32_t)day;

    return (uint8_t)((sum % 7U) + 1U);
}

/* 用编译时刻初始化时钟（__DATE__ = "Mmm dd yyyy"，__TIME__ = "hh:mm:ss"）。
 * 仅在 OSF=1（时间已丢失）时调用，保证上电即有可用的走时基准。 */
static uint8_t App_Rtc_SetBuildTime(void)
{
    static const char months[12][4] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    const char *d = __DATE__;
    const char *t = __TIME__;
    DS3231_Time_t tm;
    uint8_t i;
    uint8_t mon = 0U;

    for (i = 0U; i < 12U; i++)
    {
        if ((d[0] == months[i][0]) && (d[1] == months[i][1]) && (d[2] == months[i][2]))
        {
            mon = (uint8_t)(i + 1U);
            break;
        }
    }
    if (mon == 0U)
    {
        return 0U;
    }

    if ((d[4] < '0') || (d[4] > '9'))
    {
        /* 个位日期形如 "  3"：第 4 个字符为空格，当作 0 十位处理。 */
        if (d[4] != ' ')
        {
            return 0U;
        }
    }

    tm.month  = mon;
    tm.day    = (uint8_t)(((d[4] == ' ') ? 0U : (uint8_t)(d[4] - '0')) * 10U + (uint8_t)(d[5] - '0'));
    tm.year   = (uint16_t)((d[7] - '0') * 1000 + (d[8] - '0') * 100 + (d[9] - '0') * 10 + (d[10] - '0'));
    tm.hour   = (uint8_t)((t[0] - '0') * 10 + (t[1] - '0'));
    tm.minute = (uint8_t)((t[3] - '0') * 10 + (t[4] - '0'));
    tm.second = (uint8_t)((t[6] - '0') * 10 + (t[7] - '0'));

    if ((tm.day < 1U) || (tm.day > 31U) || (tm.year < 2000U) || (tm.year > 2099U))
    {
        return 0U;
    }

    tm.weekday = App_Rtc_Weekday(tm.year, tm.month, tm.day);
    return DS3231_SetTime(&tm);
}

void App_RtcInit(void)
{
    uint8_t status = 0U;

    app_rtc_ready = DS3231_Init();
    app_rtc_valid = 0U;
    app_rtc_sqw   = 0U;
    app_rtc_autoInit = 0U;

    if (app_rtc_ready == 0U)
    {
        return;
    }

    /* 先读状态寄存器单独判断：读失败（总线抖动）时绝不写时间，
     * 避免把仍在正常走时的时钟误覆盖成编译时刻。 */
    if (DS3231_ReadStatus(&status) == 0U)
    {
        return;
    }

    if ((status & DS3231_STATUS_OSF) != 0U)
    {
        /* OSF=1：掉电期间时间已丢失（首次上电或换电池），用编译时刻兜底。 */
        app_rtc_valid = App_Rtc_SetBuildTime();
        app_rtc_autoInit = app_rtc_valid;
    }
    else
    {
        app_rtc_valid = 1U;
    }

    /* INT/SQW -> PC12(DS_INT)：1Hz 秒节拍，供 UI 任务挂接使用。 */
    if (app_rtc_valid != 0U)
    {
        app_rtc_sqw = DS3231_Enable1HzSqw();
    }
}

uint8_t App_RtcIsReady(void)
{
    return app_rtc_ready;
}

uint8_t App_RtcIsValid(void)
{
    return app_rtc_valid;
}

uint8_t App_RtcIsSqwEnabled(void)
{
    return app_rtc_sqw;
}

uint8_t App_RtcWasAutoInit(void)
{
    return app_rtc_autoInit;
}

uint8_t App_RtcGetTime(DS3231_Time_t *out)
{
    if (app_rtc_ready == 0U)
    {
        return 0U;
    }
    return DS3231_GetTime(out);
}

uint8_t App_RtcSetTime(const DS3231_Time_t *in)
{
    if (app_rtc_ready == 0U)
    {
        return 0U;
    }

    if (DS3231_SetTime(in) == 0U)
    {
        return 0U;
    }

    app_rtc_valid = 1U;   /* SetTime 内部已清 OSF */
    return 1U;
}

uint8_t App_RtcSetTimeFromString(const char *str)
{
    DS3231_Time_t tm;
    int y = 0;
    int mo = 0;
    int d = 0;
    int h = 0;
    int mi = 0;
    int s = 0;

    if ((app_rtc_ready == 0U) || (str == NULL))
    {
        return 0U;
    }

    if (sscanf(str, "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) != 6)
    {
        return 0U;
    }

    if ((y < 2000) || (y > 2099) || (mo < 1) || (mo > 12) || (d < 1) || (d > 31) ||
        (h < 0) || (h > 23) || (mi < 0) || (mi > 59) || (s < 0) || (s > 59))
    {
        return 0U;
    }

    tm.year    = (uint16_t)y;
    tm.month   = (uint8_t)mo;
    tm.day     = (uint8_t)d;
    tm.hour    = (uint8_t)h;
    tm.minute  = (uint8_t)mi;
    tm.second  = (uint8_t)s;
    tm.weekday = App_Rtc_Weekday(tm.year, tm.month, tm.day);

    return App_RtcSetTime(&tm);
}

uint8_t App_RtcSetDateTime(uint16_t year, uint8_t month, uint8_t day,
                           uint8_t hour, uint8_t minute, uint8_t second)
{
    DS3231_Time_t tm;

    if ((year < 2000U) || (year > 2099U))
    {
        return 0U;
    }
    if ((month < 1U) || (month > 12U))
    {
        return 0U;
    }
    if ((day < 1U) || (day > App_RtcDaysInMonth(year, month)))
    {
        return 0U;
    }
    if ((hour > 23U) || (minute > 59U) || (second > 59U))
    {
        return 0U;
    }

    tm.year    = year;
    tm.month   = month;
    tm.day     = day;
    tm.hour    = hour;
    tm.minute  = minute;
    tm.second  = second;
    tm.weekday = App_Rtc_Weekday(year, month, day);

    return App_RtcSetTime(&tm);
}
