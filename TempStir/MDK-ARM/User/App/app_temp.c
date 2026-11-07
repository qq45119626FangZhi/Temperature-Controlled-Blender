/*
 * app_temp.c
 * 温度采集：从 int_adc 读原始值 -> 电压 -> Steinhart-Hart 温度，做滑动平均滤波，
 * 并据电压范围判定 NTC 断线/短路。NTC 物理参数（B 值/阻值/拓扑）为 TODO 占位。
 */
#include "app_temp.h"
#include <math.h>        /* log()：Steinhart-Hart 温度换算 */

static uint16_t s_buf[APP_TEMP_SRC_COUNT][APP_TEMP_FILTER_LEN];
static uint8_t  s_idx[APP_TEMP_SRC_COUNT];
static uint8_t  s_filled[APP_TEMP_SRC_COUNT];
static int16_t  s_offsetC10[APP_TEMP_SRC_COUNT];   /* 校准偏移（0.1℃） */

/* 阈值上穿边沿检测状态（见 app_temp.h 说明） */
static int16_t s_markC10 = APP_TEMP_LIQUID_MARK_C10;  /* 提示阈值 0.1℃ */
static uint8_t s_markValid = 0U;   /* 0=尚未拿到有效采样，下次只建立基准 */
static uint8_t s_markAbove = 0U;   /* 1=上次采样 ≥ 阈值 */
static uint8_t s_markEvent = 0U;   /* 1=有一笔未取走的上穿事件 */

/* 诊断：最近原始值 + 长时间不变（冻结）检测 */
#define APP_TEMP_STUCK_WINDOW_MS  30000U
#define APP_TEMP_STUCK_DELTA      3U
static uint16_t s_lastRaw[APP_TEMP_SRC_COUNT];
static uint16_t s_stuckRef[APP_TEMP_SRC_COUNT];
static uint32_t s_stuckTick[APP_TEMP_SRC_COUNT];
static uint8_t  s_stuck[APP_TEMP_SRC_COUNT];

/* P8：NTC 曲线参数（现场可标定；默认取头文件占位常量，上电由 EEPROM 下发覆盖）。 */
static float s_ntcR25 = APP_TEMP_NTC_R25;
static float s_ntcB   = APP_TEMP_NTC_B;
static float s_rup    = APP_TEMP_R_UP;

static const IntAdcChannel_t s_ch[APP_TEMP_SRC_COUNT] = {
    INT_ADC_CH_BOARD_TEMP,
    INT_ADC_CH_LIQUID_TEMP
};

void App_TempInit(void)
{
    AppTempSrc_t s;
    for (s = APP_TEMP_SRC_BOARD; s < APP_TEMP_SRC_COUNT; s++)
    {
        uint8_t i;
        for (i = 0U; i < APP_TEMP_FILTER_LEN; i++)
        {
            s_buf[s][i] = 0U;
        }
        s_idx[s]   = 0U;
        s_filled[s] = 0U;
        s_offsetC10[s] = 0;
    }

    /* 边沿检测复位：上电后第一次采样只建立基准，不会立刻鸣叫 */
    s_markValid = 0U;
    s_markAbove = 0U;
    s_markEvent = 0U;

    /* 诊断状态复位 */
    for (s = APP_TEMP_SRC_BOARD; s < APP_TEMP_SRC_COUNT; s++)
    {
        s_lastRaw[s]   = 0U;
        s_stuckRef[s]  = 0U;
        s_stuckTick[s] = 0U;
        s_stuck[s]     = 0U;
    }
}

/* 记录原始值并做“长时间不动”判定（不改变采样结果，只做诊断）。 */
static void App_TempTrackRaw(AppTempSrc_t src, uint16_t raw)
{
    uint32_t now = HAL_GetTick();
    uint16_t diff;

    s_lastRaw[src] = raw;

    diff = (raw > s_stuckRef[src]) ? (uint16_t)(raw - s_stuckRef[src])
                                   : (uint16_t)(s_stuckRef[src] - raw);
    if (diff >= APP_TEMP_STUCK_DELTA)
    {
        s_stuckRef[src]  = raw;
        s_stuckTick[src] = now;
        s_stuck[src]     = 0U;
    }
    else if ((uint32_t)(now - s_stuckTick[src]) >= APP_TEMP_STUCK_WINDOW_MS)
    {
        s_stuck[src] = 1U;
    }
    else
    {
        /* 变化很小但还没到判定窗口：保持原状态 */
    }
}

uint16_t App_TempGetLastRaw(AppTempSrc_t src)
{
    if (src >= APP_TEMP_SRC_COUNT)
    {
        return 0U;
    }
    return s_lastRaw[src];
}

uint8_t App_TempIsRawStuck(AppTempSrc_t src)
{
    if (src >= APP_TEMP_SRC_COUNT)
    {
        return 0U;
    }
    return s_stuck[src];
}

float App_TempAdcToVoltage(uint16_t raw)
{
    return ((float)raw) * APP_TEMP_VREF / APP_TEMP_ADC_MAX;
}

float App_TempVoltageToCelsius(float vout)
{
    /* 通用 NTC 上拉模型：Rntc = R_up * Vout / (Vref - Vout) */
    float denom = APP_TEMP_VREF - vout;
    float rntc;
    float invT;
    float tKelvin;

    if (denom <= 0.001f)
    {
        return -273.15f;   /* 开路：温度无意义 */
    }

    rntc = s_rup * vout / denom;
    if (rntc <= 0.0f)
    {
        return -273.15f;
    }

    /* Steinhart-Hart（简化 1/T = 1/T25 + (1/B) * ln(R/R25)） */
    invT = (1.0f / (25.0f + 273.15f))                     /* 1/T0，T0=25℃=298.15K */
          + (1.0f / s_ntcB) * (float)log((double)(rntc / s_ntcR25));
    tKelvin = 1.0f / invT;
    return tKelvin - 273.15f;
}

int16_t App_TempReadCelsius(AppTempSrc_t src)
{
    uint16_t raw = 0U;
    uint32_t sum = 0U;
    uint8_t  i;
    float    v;
    float    c;
    int32_t  t10;

    if (src >= APP_TEMP_SRC_COUNT)
    {
        return INT16_MIN;
    }

    if (Int_AdcReadSingle(s_ch[src], &raw) != 0)
    {
        return INT16_MIN;
    }

    /* 滑动平均 */
    s_buf[src][s_idx[src]] = raw;
    s_idx[src] = (s_idx[src] + 1U) % APP_TEMP_FILTER_LEN;
    if (s_filled[src] < APP_TEMP_FILTER_LEN)
    {
        s_filled[src]++;
    }
    for (i = 0U; i < s_filled[src]; i++)
    {
        sum += s_buf[src][i];
    }
    raw = (uint16_t)(sum / s_filled[src]);

    App_TempTrackRaw(src, raw);   /* 诊断：记录原始值 + 冻结检测 */

    v = App_TempAdcToVoltage(raw);
    c = App_TempVoltageToCelsius(v);

    /* 0.1℃ 定点输出后再叠加校准偏移，避免偏移被浮点环节吃掉。 */
    t10 = (int32_t)(c * 10.0f) + (int32_t)s_offsetC10[src];
    if (t10 > 32767) {
        t10 = 32767;
    } else if (t10 < -32767) {
        t10 = -32767;
    }
    return (int16_t)t10;
}

void App_TempSetOffsetC10(AppTempSrc_t src, int16_t offsetC10)
{
    if (src < APP_TEMP_SRC_COUNT)
    {
        s_offsetC10[src] = offsetC10;
    }
}

int16_t App_TempGetOffsetC10(AppTempSrc_t src)
{
    if (src >= APP_TEMP_SRC_COUNT)
    {
        return 0;
    }
    return s_offsetC10[src];
}

void App_TempSetNtcParams(uint16_t r25, uint16_t b, uint16_t rup)
{
    /* 合法性：R25/上拉 100Ω~100kΩ、B 500~10000K；非法值保持原参数不变，避免除零。 */
    if ((r25 >= 100U) && (r25 <= 100000U))
    {
        s_ntcR25 = (float)r25;
    }
    if ((b >= 500U) && (b <= 10000U))
    {
        s_ntcB = (float)b;
    }
    if ((rup >= 100U) && (rup <= 100000U))
    {
        s_rup = (float)rup;
    }
}

void App_TempGetNtcParams(uint16_t *r25, uint16_t *b, uint16_t *rup)
{
    if (r25 != NULL)
    {
        *r25 = (uint16_t)s_ntcR25;
    }
    if (b != NULL)
    {
        *b = (uint16_t)s_ntcB;
    }
    if (rup != NULL)
    {
        *rup = (uint16_t)s_rup;
    }
}

void App_TempSetMarkC10(int16_t markC10)
{
    s_markC10 = markC10;
    /* 阈值改变：重新建立基准，避免用旧的“高于/低于”状态误判一次边沿 */
    s_markValid = 0U;
    s_markAbove = 0U;
    s_markEvent = 0U;
}

int16_t App_TempGetMarkC10(void)
{
    return s_markC10;
}

uint8_t App_TempMarkStep(int16_t tempC10)
{
    uint8_t ev;

    /* 无效采样（NTC 断线/短路）：不改变状态，避免噪声造成误鸣 */
    if (tempC10 == INT16_MIN)
    {
        return 0U;
    }

    if (s_markValid == 0U)
    {
        /* 首次有效采样：只建立基准（上电时已高于阈值也不鸣叫） */
        s_markValid = 1U;
        s_markAbove = (tempC10 >= s_markC10) ? 1U : 0U;
        return 0U;
    }

    if ((s_markAbove == 0U) && (tempC10 >= s_markC10))
    {
        /* 上穿阈值：产生一次事件 */
        s_markAbove = 1U;
        s_markEvent = 1U;
    }
    else if ((s_markAbove != 0U) && (tempC10 < s_markC10))
    {
        /* 下穿阈值：仅复位状态，不产生事件（下降不提示） */
        s_markAbove = 0U;
    }
    else
    {
        /* 同侧：保持 */
    }

    ev = s_markEvent;
    s_markEvent = 0U;
    return ev;
}

uint8_t App_TempIsBroken(AppTempSrc_t src)
{
    uint16_t raw = 0U;
    if (src >= APP_TEMP_SRC_COUNT)
    {
        return 1U;
    }
    if (Int_AdcReadSingle(s_ch[src], &raw) != 0)
    {
        return 1U;
    }
    if ((raw < APP_TEMP_BROKEN_LOW) || (raw > APP_TEMP_BROKEN_HIGH))
    {
        return 1U;
    }
    return 0U;
}
