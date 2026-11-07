/*
 * FreeModbus UART4(RS485) 端口适配。
 * 中断回调只保存一个字节并驱动协议栈接收状态机；发送沿用协议栈逐字节回调。
 * 硬件对应（SCH_主控_2_2026-08-27new.pdf）：
 *   PC10(UART4_TX) = MCU_485_TX，PC11(UART4_RX) = MCU_485_RX，经隔离后到 MAX13487E。
 *   USART2(PA2/PA3) 才是蓝牙 ECB01C（app_bt.c），两个通道互不抢占。
 */
#include "mb.h"
#include "mbport.h"
#include "app_freemodbus_port.h"
#include "usart.h"

#define MB_SERIAL_TX_TIMEOUT_MS     20U

static UCHAR s_rxByte = 0U;
static UCHAR s_lastRxByte = 0U;
static volatile BOOL s_txEnabled = FALSE;
static volatile BOOL s_rxEnabled = FALSE;
static volatile ULONG s_rxIrqCount = 0U;
static volatile ULONG s_rxDropCount = 0U;
static volatile ULONG s_txCount = 0U;
static volatile ULONG s_txFailCount = 0U;
static volatile UCHAR s_lastTxByte = 0U;

BOOL xMBPortSerialInit(UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits,
                       eMBParity eParity, UCHAR ucStopBits)
{
    (void)ucPort;
    (void)ucDataBits;
    (void)eParity;
    (void)ucStopBits;

    /* UART4 在 CubeMX 里按蓝牙配的是 115200，Modbus 要 ulBaudRate（默认 9600）：
     * 这里按协议栈给的速率重初始化一次（不改 Core 的生成代码）。 */
    huart4.Init.BaudRate = ulBaudRate;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        return FALSE;
    }

    return TRUE;
}

void vMBPortClose(void)
{
    xMBPortSerialClose();
    xMBPortTimersClose();
}

void xMBPortSerialClose(void)
{
    s_rxEnabled = FALSE;
    s_txEnabled = FALSE;
    (void)HAL_UART_AbortReceive_IT(&huart4);
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    s_rxEnabled = xRxEnable;
    s_txEnabled = xTxEnable;

    if (xRxEnable != FALSE)
    {
        (void)HAL_UART_Receive_IT(&huart4, &s_rxByte, 1U);
    }
    else
    {
        (void)HAL_UART_AbortReceive_IT(&huart4);
    }

    if (xTxEnable != FALSE)
    {
        while (s_txEnabled != FALSE)
        {
            if (pxMBFrameCBTransmitterEmpty == NULL)
            {
                break;
            }
            (void)pxMBFrameCBTransmitterEmpty();
        }
    }
}

BOOL xMBPortSerialGetByte(CHAR *pucByte)
{
    if (pucByte == NULL)
    {
        return FALSE;
    }

    *pucByte = (CHAR)s_lastRxByte;
    return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    UCHAR txByte = (UCHAR)ucByte;
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&huart4, &txByte, 1U, MB_SERIAL_TX_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        s_txCount++;
        s_lastTxByte = txByte;
        return TRUE;
    }

    s_txFailCount++;
    return FALSE;
}

void App_FreeModbusSerialRxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    s_lastRxByte = s_rxByte;
    if ((s_rxEnabled != FALSE) && (pxMBFrameCBByteReceived != NULL))
    {
        s_rxIrqCount++;
        (void)pxMBFrameCBByteReceived();
        (void)HAL_UART_Receive_IT(&huart4, &s_rxByte, 1U);
    }
    else
    {
        s_rxDropCount++;
    }
}

/* 接收错误（ORE/NE/FE 等）恢复：清标志后重新挂接收，避免通道失聪。 */
void App_FreeModbusSerialRxErrorCallback(void)
{
    if (s_rxEnabled != FALSE)
    {
        (void)HAL_UART_AbortReceive_IT(&huart4);
        (void)HAL_UART_Receive_IT(&huart4, &s_rxByte, 1U);
    }
}

ULONG App_FreeModbusSerialGetRxCount(void)
{
    return s_rxIrqCount;
}

ULONG App_FreeModbusSerialGetRxDropCount(void)
{
    return s_rxDropCount;
}

ULONG App_FreeModbusSerialGetTxCount(void)
{
    return s_txCount;
}

ULONG App_FreeModbusSerialGetTxFailCount(void)
{
    return s_txFailCount;
}

UCHAR App_FreeModbusSerialGetLastTxByte(void)
{
    return s_lastTxByte;
}
