/*
 * @file    app_ble.h
 * @brief   蓝牙通道业务层：精简 Modbus RTU 从机（0x03/0x06/0x10 + 异常响应）。
 *
 * 分层（2026-11-06 重构）：
 *   - 串口/AT 握手/空闲分帧/模块上报解析在 Dri/dri_ble.c（Ble_* 接口）；
 *   - 本层只做 Modbus 协议：地址过滤、CRC、功能码、异常码，寄存器读写走 app_comm。
 *
 * 帧边界与帧长约束见 dri_ble.h（空闲 >= 10ms 一帧；BLE 单包 <= 20 字节，
 * 主站一次读寄存器建议 <= 7 个）。
 */
#ifndef APP_BLE_H
#define APP_BLE_H

#include <stdint.h>

/* 初始化：转调 Dri 层 USART2 初始化并挂接单字节中断接收。 */
void App_BleInit(void);

/* 上电握手：等待模块自检 -> 识别波特率 -> 打印一次名称/连接状态/速率。
 * 返回 1=模块已响应 AT，0=无应答。由 BLE 任务调用，失败每 5s 重试。 */
uint8_t App_BleBringUp(void);

/* 周期处理（任务上下文，5ms 一次）：取一帧 -> Modbus 解析。 */
void App_BlePoll(void);

/* 蓝牙是否已连接（由模块上报的 CONNECT OK / DISCONNECT 维护，在 Dri 层）。 */
uint8_t App_BleIsOnline(void);

#endif /* APP_BLE_H */
