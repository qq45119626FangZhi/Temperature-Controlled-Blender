/*
 * @file    dri_ble.h
 * @brief   蓝牙透传模块 ECB01CV2 驱动（USART2 PA2/PA3，8N1，本板 9600）。
 *
 * 硬件对应（Doc/SCH_主控_2_2026-08-27new.pdf）：
 *   PA2(USART2_TX) -> BL-RXD -> ECB01C pin3 RXD
 *   PA3(USART2_RX) <- BL-TXD <- ECB01C pin2 TXD
 *
 * 本层职责（只做模块与串口，不认识 Modbus）：
 *   1) USART2 初始化、单字节中断接收、错误自愈重挂；
 *   2) 上电 AT 握手 + 波特率自动识别 + 名称查询（只读，不改名）；
 *   3) 总线空闲分帧（空闲 >= 10ms 视为一帧结束）；
 *   4) 消化模块自身上报（CONNECT OK / DISCONNECT）与 MCU 自己的 AT 回显；
 *   5) 其余整帧经 Ble_TakeFrame() 交给 App 层（App/app_ble.c）。
 */
#ifndef DRI_BLE_H
#define DRI_BLE_H

#include <stdint.h>
#include "usart.h"

#define DRI_BLE_RX_BUF_SIZE    96U
#define DRI_BLE_TX_BUF_SIZE    96U

/* 目标蓝牙名称（暂时不启用，保留以便换模块后使用）。
 * 实测本板模块 ECB01CV2-SOP8：AT+NAME=<新名> 会回 OK，但改名后 AT+RST 再查询
 * 仍是出厂名 BT_<MAC>，即该版本不支持改用户自定义名（名字由 MAC 派生）。
 * 为避免继续消耗手册规定的 10 次写入额度，默认关闭改名，直接使用出厂名。 */
#define DRI_BLE_NAME_TARGET   "BT_JBJ_FANGZHI"

/* 1=启用启动改名流程；0=不改名（当前取 0，使用模块出厂名 BT_<MAC>） */
#define DRI_BLE_NAME_PROVISION  0

/* 初始化：USART2 按默认速率重配并挂接单字节中断接收（实际速率由握手流程锁定）。 */
void Ble_Init(void);

/* 上电握手：等待模块自检 -> 识别波特率 -> 按需改名 -> 串口打印一次名称/连接状态/速率。
 * 返回 1=模块已响应 AT，0=无应答。由上层任务调用，失败可每 5s 重试。 */
uint8_t Ble_BringUp(void);

/* 周期处理（任务上下文，建议 5ms）：接收自愈 + 空闲分帧 + 吞掉模块上报/AT 回显。 */
void Ble_Poll(void);

/* 取走一帧业务数据（长度 0 表示当前没有帧）。 */
uint16_t Ble_TakeFrame(uint8_t *out, uint16_t maxLen);

/* 发送原始字节 / 字符串（阻塞式，供上层组 Modbus 响应帧）。 */
void Ble_Send(const uint8_t *data, uint16_t len);
void Ble_SendString(const char *text);

/* USART2 单字节接收完成 / 接收错误（由 app_comm 的 HAL 回调转发）。 */
void Ble_RxCpltCallback(UART_HandleTypeDef *huart);
void Ble_RxErrorCallback(UART_HandleTypeDef *huart);

/* 模块上报的连接状态（CONNECT OK / DISCONNECT）。 */
uint8_t Ble_IsOnline(void);

/* 当前锁定的波特率（AT 识别结果）。 */
uint32_t Ble_GetBaud(void);

#endif /* DRI_BLE_H */
