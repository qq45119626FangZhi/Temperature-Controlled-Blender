/*
 * @file    app_fault.h
 * @brief   P7：故障监控与可靠性（故障字聚合 + 任务心跳 + IWDG 看门狗）。
 *
 * 设计（对应《整体规划.md》13.1）：
 *  - 故障监控任务 FAULT（50ms）：周期采集加热硬件故障/过流/NTC/铝板限温/
 *    EEPROM/RTC 等状态，聚合成与 0x0006 一致的故障字，供通信与 UI 共用；
 *  - 心跳：CTRL / UI / FAULT 三个关键任务每轮循环上报一次；
 *  - 看门狗守护任务 WDG（100ms，最低优先级）：所有心跳都在窗口内才喂狗，
 *    任一任务卡死即停止喂狗，由 IWDG 复位整机。
 *
 * 注意：FreeModbus 的 COMM 任务阻塞在协议栈事件上，周期不可控，因此
 * 不参与心跳判定（否则正常空闲也会被判成卡死）。
 */
#ifndef APP_FAULT_H
#define APP_FAULT_H

#include <stdint.h>

/* 1 = 启用独立看门狗（约 1.6s 超时）。
 * 单步调试 / 长时间断点会触发复位，此时可临时改 0 重新编译。 */
#define APP_IWDG_ENABLE     1U

/* 看门狗超时（ms）：IWDG 设定值，仅用于提示与文档一致 */
#define APP_IWDG_TIMEOUT_MS 1600U

/* 说明：IWDG 自检注入代码（模拟 CTRL 任务卡死）已于 2026-11-05 验证通过后删除，
 * 验证证据见启动日志 `[P7] WDG: CTRL heartbeat lost (534 ms) -> stop feeding`
 * 与复位后 `[P7] reset cause: IWDG=1`。此后固件不再有注入逻辑，正常运行即喂狗。 */

/* 心跳源：参与看门狗判定的关键任务 */
typedef enum
{
    APP_HB_CTRL = 0,     /* 控制环：200ms 一轮 */
    APP_HB_UI,           /* UI：40ms 一轮 */
    APP_HB_FAULT,        /* 故障监控：50ms 一轮 */
    APP_HB_IMU,          /* P9：六轴采集：10ms 一轮（器件缺省时 100ms 一轮，仍心跳） */
    APP_HB_COUNT
} App_HbSource_t;

/* 初始化：清心跳、启动 IWDG（须在 vTaskStartScheduler 之前调用）。
 * 工程未引入 HAL IWDG 模块，这里直接操作 IWDG 寄存器，避免改 Core/.ioc。 */
void App_FaultInit(void);

/* 故障监控任务体（50ms 周期）。 */
void App_FaultTask(void *argument);

/* 看门狗守护任务体（100ms 周期，最低优先级）。 */
void App_FaultWatchdogTask(void *argument);

/* 任务心跳上报：在各关键任务每轮循环里调用一次。 */
void App_FaultHeartbeat(App_HbSource_t src);

/* 聚合故障字（位定义与 app_modbus.h 的 APP_MB_FLT_* 一致）。 */
uint16_t App_FaultGetBits(void);

#endif /* APP_FAULT_H */
