/*
 * app_temp.h
 * 温度采集（App 层）：NTC ADC -> 电压 -> 温度（Steinhart-Hart），含滑动平均滤波
 * 与断线/短路判定。NTC 参数来自决策 6 的分压网络，具体 B 值/阻值为 TODO 占位。
 */
#ifndef APP_TEMP_H
#define APP_TEMP_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "int_adc.h"

/* NTC 参数（决策 6：分压 R14/R15 10k、R16/R17 1k + NTC 规格；以下为 TODO 占位值，
 * 待用户提供真实 B 值/阻值或原理图拓扑后替换）。
 * 采用通用“NTC 对地 + 上拉电阻到 VCC”模型：Vout = Vref*Rntc/(Rntc+R_up)。 */
#define APP_TEMP_NTC_R25        10000.0f   /* 25℃ 标称阻值 (Ω)，TODO 确认 */
#define APP_TEMP_NTC_B          3950.0f    /* B 值，TODO 确认 */
#define APP_TEMP_R_UP           10000.0f    /* 上拉电阻 (Ω)，TODO 确认拓扑 */
#define APP_TEMP_VREF           3.3f
#define APP_TEMP_ADC_MAX        4095.0f
#define APP_TEMP_FILTER_LEN     8           /* 滑动平均窗口 */
#define APP_TEMP_BROKEN_LOW     100         /* 电压 < 此(raw) 视为短路/断线 */
#define APP_TEMP_BROKEN_HIGH    4000        /* 电压 > 此(raw) 视为开路/断线 */

typedef enum
{
    APP_TEMP_SRC_BOARD = 0,    /* 板载 NTC：PB0 / ADC1_IN8 */
    APP_TEMP_SRC_LIQUID,       /* 液体 NTC：PB1 / ADC1_IN9 */
    APP_TEMP_SRC_COUNT
} AppTempSrc_t;

void App_TempInit(void);
float App_TempAdcToVoltage(uint16_t raw);          /* raw -> 分压电压(V) */
float App_TempVoltageToCelsius(float vout);        /* 电压 -> 温度(℃)，参数 TODO */
int16_t App_TempReadCelsius(AppTempSrc_t src);     /* 滤波后温度，单位 0.1℃，失败返回 INT16_MIN */
uint8_t App_TempIsBroken(AppTempSrc_t src);        /* 断线/短路判定 1=异常 */

/* 校准偏移（0.1℃）：直接加到读取结果上，用于实测标定（参数存 EEPROM）。
 * App_TempInit 会清零，需在初始化之后由 App 层按存储值下发。 */
void App_TempSetOffsetC10(AppTempSrc_t src, int16_t offsetC10);
int16_t App_TempGetOffsetC10(AppTempSrc_t src);

/* P8：现场标定 NTC 曲线参数（Ω / K）。传入 0 或明显非法值则忽略并保持当前值；
 * 上电由 App_Main_ApplyStoreParams() 从 EEPROM 下发。 */
void App_TempSetNtcParams(uint16_t r25, uint16_t b, uint16_t rup);
void App_TempGetNtcParams(uint16_t *r25, uint16_t *b, uint16_t *rup);

/* ---- 阈值“上穿”事件（边沿检测，用于液温提示音） ----
 * 语义（以液体 30.0℃ 为例）：
 *   - 由「< 阈值」上穿到「≥ 阈值」→ 产生一次事件（调用方据此停电机 + 长鸣 1s）；
 *   - 由「≥ 阈值」回落到「< 阈值」→ 只复位状态，不产生事件（下降不提示、不动电机）；
 *   - 首次采样（含上电时已高于阈值）只建立基准，不产生事件，避免误报。
 * 由任务按固定周期把已读到的温度喂进来（不重复读 ADC）。 */
#define APP_TEMP_LIQUID_MARK_C10   300   /* 液体温度提示阈值 30.0℃ */

/* 上穿阈值时是否同时停止电机。
 * 默认 0 = 只长鸣 1s，不动电机 —— 因为"停电机"会通过加热运行联锁关断加热，
 * 属于用一个新功能打断已验收的加热功能（违反模块隔离原则）。
 * 确需"到 30℃ 停机"时把此宏置 1 即可，其它逻辑不变。 */
#define APP_TEMP_MARK_STOP_MOTOR   0

void App_TempSetMarkC10(int16_t markC10);      /* 修改阈值（同时复位边沿状态） */
int16_t App_TempGetMarkC10(void);
uint8_t App_TempMarkStep(int16_t tempC10);     /* 喂一次采样：1=刚发生上穿事件 */

/* ---- 诊断：现场判定 NTC 接线/失效 ----
 * GetLastRaw 返回最近一次滤波后的 ADC 原始值（读失败/尚未采样为 0）。
 * 参考：室温 25℃ 时 10k/10k 分压约 2047；接近 0 或 4095 说明短路/开路。
 * IsRawStuck：1=该路原始值连续 ≥30s 变化不足 3 LSB（传感器未接、分压被固定
 * 电压钳住、或通道没真正切换）——用于判断"温度不变"到底是没加热还是采样失效。 */
uint16_t App_TempGetLastRaw(AppTempSrc_t src);
uint8_t  App_TempIsRawStuck(AppTempSrc_t src);

#endif /* APP_TEMP_H */
