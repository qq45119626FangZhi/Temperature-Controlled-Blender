/*
 * app_heat.h
 * 加热控制（App 层）：驱动 TIM2_CH1 (PA0) PWM 输出，并监测加热电流（HEAT_I_ADC）
 * 与硬件故障锁存（HEAT_FAULT_N）。PID 闭环在 P3 实现，本模块只提供执行与保护接口。
 */
#ifndef APP_HEAT_H
#define APP_HEAT_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define APP_HEAT_PWM_MAX       100U   /* 占空比上限(%) */
#define APP_HEAT_OC_THRESHOLD  3500   /* 过流判定阈值(raw, INA199 放大)，TODO 按规格整定 */

void App_HeatInit(void);
void App_HeatSetPwmPercent(uint8_t pct);    /* 0~100 -> TIM2_CH1 */
uint8_t App_HeatGetPwmPercent(void);
uint8_t App_HeatIsFault(void);              /* HEAT_FAULT_N 低有效 -> 1=故障 */
int16_t App_HeatReadCurrentRaw(void);       /* HEAT_I_ADC 原始值 */
uint8_t App_HeatIsOverCurrent(void);        /* 过流判定 */

/* P8：加热过流判定阈值（ADC raw，100~4095）。上电由 App_Main_ApplyStoreParams()
 * 从 EEPROM 下发；非法值忽略并保持当前阈值。 */
void App_HeatSetOverCurrentThreshold(uint16_t raw);
uint16_t App_HeatGetOverCurrentThreshold(void);
void App_HeatEmergencyOff(void);            /* 立即关加热（保护） */

/* ---- P3 温控闭环 ---- */
void App_HeatSetTargetTemp(int16_t targetC10);  /* 设定目标温度(0.1℃)：液体温度闭环 */
int16_t App_HeatGetTargetTemp(void);

/* ---- P4：铝板限温（与液体目标温度分开设定） ----
 * 板载 NTC(APP_TEMP_SRC_BOARD) 贴在铝板上。板温达到该上限时强制关断加热，
 * 防止液体目标很高时把铝板烧过头。0 表示不限制。 */
void App_HeatSetPlateLimitTemp(int16_t limitC10);

/* 铝板上限的合法下限（0.1℃）：0 = 不限制，其余必须 ≥ 35.0℃。
 * 低于该值的设定（例如 30℃）在室温下就会立刻锁存，且板温永远降不到
 * 「上限 - 2.0℃」以下 → 加热被永久关断，属于无法解除的非法配置。 */
#define APP_HEAT_PLATE_LIMIT_MIN_C10   350

/* 规范化铝板上限：0 或 ≥ 35.0℃，非法值（1~34.9℃）返回 0（按不限制处理）。 */
int16_t App_HeatSanitizePlateLimit(int16_t limitC10);

/* 清除铝板限温锁存（供 0x0007 bit2「清故障」使用；板温仍超高会在下一
 * 个控制周期重新锁存，属正常保护行为）。 */
void App_HeatClearPlateLimit(void);

int16_t App_HeatGetPlateLimitTemp(void);
uint8_t App_HeatIsPlateLimited(void);       /* 1=当前被铝板限温锁存关断（通信故障位用） */

/* ---- P7：液温到温“完成”事件 ----
 * 目标液温 >0 且液温首次达到目标时：关闭加热并锁存（电机不受影响），
 * 同时置一次性事件供 UI 蜂鸣提示（1s 长鸣）；修改目标温度或调用
 * App_HeatClearDone() 解除。 */
uint8_t App_HeatTakeDoneEvent(void);        /* 取走一次性完成事件：1=有新事件 */
uint8_t App_HeatIsDone(void);               /* 1=已到温/加热结束锁存中 */
void    App_HeatClearDone(void);            /* 清完成锁存与事件 */
/* ---- P10：铝板目标加热（可选模块，默认关闭） ----
 * 需求定稿（2026-11-05）：面板语义是「Liquid = 加热目标（以液体设定为准）」，
 * 「Plate = 上限温度 / 实际测量温度」。因此本模式默认关闭（=0），
 * 加热目标只取液体设定温度，铝板设定只作保护上限。
 * 保留实现供特殊工况选用（置 1 才生效，条件：液体目标=0 且铝板设定>0）。
 * 安全：硬限幅自动放宽到「铝板设定 + 2.0℃」，板温 NTC 无效仍立即断电。 */
#define APP_HEAT_PLATE_TARGET_DEFAULT     0
#define APP_HEAT_PLATE_TARGET_MARGIN_C10  20   /* 板温目标的硬限幅余量 2.0℃ */
void App_HeatSetPlateAsTarget(uint8_t enable);
uint8_t App_HeatGetPlateAsTarget(void);

/* 清除"板温失控（高占空比加热但板温不升）"锁存；修改铝板设定或重新启停电机时自动清除。 */
void App_HeatClearPlateStuck(void);

/* ---- A3：液体加热「升温异常」判定（仅告警，不停机、不关加热） ----
 * 独立新增模块，默认启用；置 APP_HEAT_NORISE_ENABLE=0 则完全不参与，行为与旧版一致。
 * 判据（每个 600ms 控制周期评估，只在正常加热路径上累计）：
 *   - 目标液温 > 0、加热占空比 ≥ APP_HEAT_NORISE_PCT_MIN、电机运行中（有加热许可）；
 *   - 以窗口起点液温为基准，连续 APP_HEAT_NORISE_WIN_MS 内温升 < APP_HEAT_NORISE_DELTA_C10
 *     → 判定「升温异常」，置锁存 + 一次性事件（供 UI 蜂鸣/上位机读取）。
 * 动作：仅告警（状态字 bit12 APP_MB_ST_HEAT_NO_RISE + 串口一行），
 *       绝不调用 App_HeatEmergencyOff()，也不置故障字，避免影响既有加热流程。 */
#define APP_HEAT_NORISE_ENABLE      1U        /* 1 = 启用判定 */
#define APP_HEAT_NORISE_WIN_MS      120000U   /* 观察窗口 120s */
#define APP_HEAT_NORISE_DELTA_C10   20        /* 窗口内升温 < 2.0℃ 视为异常 */
#define APP_HEAT_NORISE_PCT_MIN     50U       /* 仅高占空比(≥50%)时判定 */

uint8_t App_HeatTakeNoRiseEvent(void);      /* 取走一次性告警事件：1=有新事件 */
uint8_t App_HeatIsNoRise(void);             /* 1=已判定升温异常（锁存） */
void    App_HeatClearNoRise(void);          /* 清锁存与事件 */

/* ---- 运行联锁：只有电机启动后才允许加热 ----
 * 默认 permit=0（上电/停机都不加热），由控制任务按“电机目标转速≠0”置位。
 * permit 0→1：开始新一轮加热判定（清到温锁存、清 PID）；1→0：立即关加热。 */
void App_HeatSetRunPermit(uint8_t permit);
uint8_t App_HeatGetRunPermit(void);

void App_HeatControlStep(void);                 /* 单步 PID：读液体温度 -> 加热 PWM，周期调用 */
void App_HeatPidSetGains(float kp, float ki, float kd);  /* 实测整定 */

#endif /* APP_HEAT_H */
