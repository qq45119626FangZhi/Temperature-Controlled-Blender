/*
 * @file    app_fan.h
 * @brief   风扇控制策略层（铝板温度回差启停）。
 *
 * 分层（2026-11-06 重构）：引脚写电平在 Dri/dri_fan.c（Fan_PinInit / Fan_PinSet），
 * 本层只做"何时开/关"的判定，对外 API 与重构前一致。
 */
#ifndef APP_FAN_H
#define APP_FAN_H

#include <stdint.h>

/* 按铝板温度自动启停的判据（温度回差 3℃ + 时间回差 5s 双重防抖）
 *   - 开：板温 ≥ 35.0℃
 *   - 关：板温 ≤ 32.0℃
 *   - 中间 32.0~35.0℃ 为回差带，保持当前状态不动。
 * 原值 30.0℃ 单一门限太低（板子自热就可能越过），会在门限附近长期"开-关-开"。 */
#define APP_FAN_PLATE_ON_C10    350     /* 开阈值 35.0℃（≥ 开启） */
#define APP_FAN_PLATE_OFF_C10   320     /* 关阈值 32.0℃（≤ 关闭） */
#define APP_FAN_SWITCH_GAP_MS   5000U   /* 两次启停之间最小间隔 5s，防临界温度频繁切换 */

/* 初始化风扇策略：确保默认停止（MOTOR_ON 拉低）。
 * 建议在 vTaskStartScheduler() 之前调用。 */
void Fan_Init(void);

/* 强制启动风扇（策略层封装，内部调 Dri 层引脚）。 */
void Fan_Start(void);

/* 强制停止风扇（策略层封装，内部调 Dri 层引脚）。 */
void Fan_Stop(void);

/* 按铝板温度自动控制：≥ 35.0℃ 开，≤ 32.0℃ 关（回差 3℃），两次切换之间至少间隔 5s。
 * plateC10 为板载 NTC 温度（单位 0.1℃），INT16_MIN 表示传感器异常（保持现状）。
 * 需周期性调用（建议 200ms）。 */
void Fan_AutoByPlateTemp(int16_t plateC10);

/* 当前风扇状态：1=转动，0=停止。 */
uint8_t Fan_IsOn(void);

#endif /* APP_FAN_H */
