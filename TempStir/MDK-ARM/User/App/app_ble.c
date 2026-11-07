/*
 * @file    app_ble.c
 * @brief   蓝牙通道业务层：精简 Modbus RTU 从机（0x03/0x06/0x10 + 异常响应）。
 *
 * 分层（2026-11-06 重构）：
 *   - 串口、AT 握手、空闲分帧、模块上报（CONNECT OK/DISCONNECT）全部在
 *     Dri/dri_ble.c；本文件通过 Ble_Poll() + Ble_TakeFrame() 取整帧；
 *   - 本文件只做 Modbus 业务：地址过滤、CRC 校验、功能码分发、异常码；
 *   - 寄存器读写走 App_CommWriteReg()（面板仲裁/范围校验/保护联锁与 RS485 一致），
 *     读取走 App_ModbusSyncRegs() + App_ModbusReadReg()。
 */
#include "app_ble.h"
#include "dri_ble.h"
#include "app_modbus.h"
#include "app_comm.h"

#include <string.h>

/* Modbus 异常码 */
#define APP_BLE_EX_ILLEGAL_FUNC    0x01U
#define APP_BLE_EX_ILLEGAL_ADDR    0x02U
#define APP_BLE_EX_ILLEGAL_VALUE   0x04U   /* 数值非法/保护拒绝 */
#define APP_BLE_EX_SLAVE_BUSY      0x06U   /* 面板占用 */

/* ---------------- 内部工具 ---------------- */

static uint16_t App_BleCrc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static void App_BleSendException(uint8_t addr, uint8_t func, uint8_t code)
{
    uint8_t  frame[5];
    uint16_t crc;

    frame[0] = addr;
    frame[1] = (uint8_t)(func | 0x80U);
    frame[2] = code;
    crc = App_BleCrc16(frame, 3U);
    frame[3] = (uint8_t)(crc & 0xFFU);
    frame[4] = (uint8_t)(crc >> 8);

    Ble_Send(frame, 5U);
}

/* 把 App_Comm 的结果映射成异常码 */
static uint8_t App_BleMapResult(App_CommResult_t result)
{
    switch (result)
    {
        case APP_COMM_OK:
            return 0U;
        case APP_COMM_ERR_ADDR:
            return APP_BLE_EX_ILLEGAL_ADDR;
        case APP_COMM_ERR_BUSY:
            return APP_BLE_EX_SLAVE_BUSY;
        default:
            return APP_BLE_EX_ILLEGAL_VALUE;
    }
}

/* ---------------- 各功能码 ---------------- */

static void App_BleHandleRead03(const uint8_t *buf, uint16_t len, uint8_t addr)
{
    uint16_t start;
    uint16_t count;
    uint16_t i;
    uint16_t pos = 0U;
    uint16_t crc;
    uint8_t  frame[DRI_BLE_TX_BUF_SIZE];
    uint8_t  respAddr;

    if (len != 8U)
    {
        App_BleSendException(addr, 0x03U, APP_BLE_EX_ILLEGAL_VALUE);
        return;
    }

    start = (uint16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    count = (uint16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);

    if ((count == 0U) || (count > 32U) ||
        (((uint32_t)start + (uint32_t)count) > (uint32_t)APP_MB_REG_HOLDING_NREGS))
    {
        App_BleSendException(addr, 0x03U, APP_BLE_EX_ILLEGAL_ADDR);
        return;
    }

    /* 广播地址不回应，读请求统一用本机地址回应 */
    respAddr = (addr == 0U) ? App_CommGetSlaveAddr() : addr;

    App_ModbusSyncRegs();

    frame[pos++] = respAddr;
    frame[pos++] = 0x03U;
    frame[pos++] = (uint8_t)(count * 2U);
    for (i = 0U; i < count; i++)
    {
        uint16_t value = App_ModbusReadReg((USHORT)(start + i));

        frame[pos++] = (uint8_t)(value >> 8);
        frame[pos++] = (uint8_t)(value & 0xFFU);
    }

    crc = App_BleCrc16(frame, pos);
    frame[pos++] = (uint8_t)(crc & 0xFFU);
    frame[pos++] = (uint8_t)(crc >> 8);
    Ble_Send(frame, pos);
}

static void App_BleHandleWrite06(const uint8_t *buf, uint16_t len, uint8_t addr)
{
    uint16_t start;
    uint16_t value;
    uint16_t crc;
    uint8_t  err;
    uint8_t  frame[8];

    if (len != 8U)
    {
        App_BleSendException(addr, 0x06U, APP_BLE_EX_ILLEGAL_VALUE);
        return;
    }

    start = (uint16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    value = (uint16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);

    err = App_BleMapResult(App_CommWriteReg(start, value));
    if (err != 0U)
    {
        App_BleSendException(addr, 0x06U, err);
        return;
    }

    if (addr == 0U)
    {
        return;   /* 广播写：不回应 */
    }

    /* 正常响应：原样回显 */
    memcpy(frame, buf, 6U);
    crc = App_BleCrc16(frame, 6U);
    frame[6] = (uint8_t)(crc & 0xFFU);
    frame[7] = (uint8_t)(crc >> 8);
    Ble_Send(frame, 8U);
}

static void App_BleHandleWrite10(const uint8_t *buf, uint16_t len, uint8_t addr)
{
    uint16_t start;
    uint16_t count;
    uint8_t  byteCount;
    uint16_t i;
    uint16_t crc;
    uint8_t  frame[8];

    if (len < 9U)
    {
        App_BleSendException(addr, 0x10U, APP_BLE_EX_ILLEGAL_VALUE);
        return;
    }

    start     = (uint16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    count     = (uint16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
    byteCount = buf[6];

    if ((count == 0U) || (count > 32U) || (byteCount != (uint8_t)(count * 2U)) ||
        (len != (uint16_t)(9U + byteCount)) ||
        (((uint32_t)start + (uint32_t)count) > (uint32_t)APP_MB_REG_HOLDING_NREGS))
    {
        App_BleSendException(addr, 0x10U, APP_BLE_EX_ILLEGAL_ADDR);
        return;
    }

    for (i = 0U; i < count; i++)
    {
        uint16_t value = (uint16_t)(((uint16_t)buf[7U + (i * 2U)] << 8) |
                                    (uint16_t)buf[8U + (i * 2U)]);
        uint8_t  err = App_BleMapResult(App_CommWriteReg((uint16_t)(start + i), value));

        if (err != 0U)
        {
            App_BleSendException(addr, 0x10U, err);
            return;
        }
    }

    if (addr == 0U)
    {
        return;   /* 广播写：不回应 */
    }

    frame[0] = addr;
    frame[1] = 0x10U;
    frame[2] = (uint8_t)(start >> 8);
    frame[3] = (uint8_t)(start & 0xFFU);
    frame[4] = (uint8_t)(count >> 8);
    frame[5] = (uint8_t)(count & 0xFFU);
    crc = App_BleCrc16(frame, 6U);
    frame[6] = (uint8_t)(crc & 0xFFU);
    frame[7] = (uint8_t)(crc >> 8);
    Ble_Send(frame, 8U);
}

/* ---------------- 帧分发（只处理 Modbus；模块文本已由 Dri 层消化） ---------------- */

static void App_BleHandleFrame(const uint8_t *buf, uint16_t len)
{
    uint8_t  addr;
    uint8_t  func;
    uint16_t crcCalc;
    uint16_t crcRecv;

    if (len < 4U)
    {
        return;
    }

    addr = buf[0];
    if ((addr != App_CommGetSlaveAddr()) && (addr != 0U))
    {
        return;   /* 不是发给本机的帧 */
    }

    crcRecv = (uint16_t)((uint16_t)buf[len - 2U] | ((uint16_t)buf[len - 1U] << 8));
    crcCalc = App_BleCrc16(buf, (uint16_t)(len - 2U));
    if (crcRecv != crcCalc)
    {
        return;   /* CRC 错：静默丢弃 */
    }

    func = buf[1];

    switch (func)
    {
        case 0x03U:
            App_BleHandleRead03(buf, len, addr);
            break;

        case 0x06U:
            App_BleHandleWrite06(buf, len, addr);
            break;

        case 0x10U:
            App_BleHandleWrite10(buf, len, addr);
            break;

        default:
            App_BleSendException(addr, func, APP_BLE_EX_ILLEGAL_FUNC);
            break;
    }
}

/* ---------------- 对外接口 ---------------- */

void App_BleInit(void)
{
    Ble_Init();
}

uint8_t App_BleBringUp(void)
{
    return Ble_BringUp();
}

void App_BlePoll(void)
{
    uint8_t  frame[DRI_BLE_RX_BUF_SIZE];
    uint16_t len;

    Ble_Poll();
    len = Ble_TakeFrame(frame, (uint16_t)sizeof(frame));
    if (len == 0U)
    {
        return;
    }

    App_BleHandleFrame(frame, len);
}

uint8_t App_BleIsOnline(void)
{
    return Ble_IsOnline();
}
