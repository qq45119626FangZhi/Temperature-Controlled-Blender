/*
 * @file    app_comm.h
 * @brief   P6：通信通道管理、写命令仲裁与统一写入口。
 *
 * 三条通道（RS485/Modbus、蓝牙、CAN）共用同一张保持寄存器表（见 app_modbus.h）：
 *   - 读：各通道自行取 App_ModbusReadReg()；
 *   - 写：一律经 App_CommWriteReg()，内部完成 面板仲裁 -> 范围校验 -> 保护联锁 -> 下发。
 *
 * 面板仲裁：面板按键活动后 APP_COMM_PANEL_HOLD_MS（8s）内，通信写命令被拒绝
 * （对应 Modbus 异常码 06 从机忙），避免上位机与现场按键互相覆盖。
 *
 * UART 接收统一在 app_comm.c 的 HAL_UART_RxCpltCallback() 分发：
 *   UART4(PC10/PC11, RS485) -> FreeModbus RTU；USART2(PA2/PA3, 蓝牙) -> app_ble。
 */
#ifndef APP_COMM_H
#define APP_COMM_H

#include <stdint.h>

/* 通道使能位（存 EEPROM commEn） */
#define APP_COMM_CH_RS485     0x01U
#define APP_COMM_CH_BT        0x02U
#define APP_COMM_CH_CAN       0x04U   /* 本期未实现 */
#define APP_COMM_CH_DEFAULT   (APP_COMM_CH_RS485 | APP_COMM_CH_BT)

/* 面板占用判据：最近一次按键后 8s 内拒绝通信写入 */
#define APP_COMM_PANEL_HOLD_MS  8000U

/* 写命令结果（由调用方映射成 Modbus 异常码） */
typedef enum
{
    APP_COMM_OK = 0,      /* 成功 */
    APP_COMM_ERR_ADDR,    /* 地址非法/只读      -> 02 非法数据地址 */
    APP_COMM_ERR_VALUE,   /* 数值非法/保护拒绝  -> 04 从机设备故障 */
    APP_COMM_ERR_BUSY     /* 面板占用           -> 06 从机忙 */
} App_CommResult_t;

/* 初始化：按 EEPROM 参数（波特率/从机地址/通道使能）配置并启动各通道。
 * 必须在 vTaskStartScheduler() 之前调用。 */
void App_CommInit(void);

/* Modbus RTU 轮询任务体（内部阻塞等协议事件）。 */
void App_CommTask(void *argument);

/* 蓝牙通道任务体（AT/透传解析，5ms 周期）。 */
void App_BleTask(void *argument);

/* 面板按键活动：每次检测到按键动作调用一次，开启 8s 占用窗口。 */
void App_CommNotifyPanelKey(void);

/* 1=面板处于占用窗口内（通信写命令应被拒绝）。 */
uint8_t App_CommIsPanelBusy(void);

/* 统一写入口：校验 + 仲裁 + 下发。addr 为寄存器地址，value 为 16bit 原值。 */
App_CommResult_t App_CommWriteReg(uint16_t addr, uint16_t value);

/* 故障码位图（0x0006 寄存器内容），同时供启停命令做联锁判断。 */
uint16_t App_CommGetFaultBits(void);

/* 当前配置查询 */
uint8_t  App_CommGetChannels(void);
uint8_t  App_CommGetSlaveAddr(void);
uint32_t App_CommGetBaud(void);
uint16_t App_CommGetBaudCode(void);

#endif /* APP_COMM_H */
