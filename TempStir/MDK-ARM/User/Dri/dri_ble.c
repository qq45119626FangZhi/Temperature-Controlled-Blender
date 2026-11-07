/*
 * @file    dri_ble.c
 * @brief   蓝牙透传模块驱动实现：USART2 + AT 握手 + 空闲分帧 + 发送。
 *
 * 帧边界：按"总线空闲 >= 10ms"判一帧结束（9600 下 t3.5 约 3.6ms）。
 * 模块上报与 AT 回显在本层消化，业务帧通过 Ble_TakeFrame() 上抛。
 */
#include "dri_ble.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdio.h>

#define DRI_BLE_IDLE_MS        10U
#define DRI_BLE_TX_TIMEOUT_MS  50U
#define DRI_BLE_AT_WAIT_MS     400U   /* AT 命令应答等待（查询） */
#define DRI_BLE_AT_DETECT_MS   300U   /* 波特率识别时单档等待 */
#define DRI_BLE_BOOT_WAIT_MS   1500U  /* 上电等模块自检 */
#define DRI_BLE_BAUD_DEFAULT   9600U  /* 本板模块出厂速率（实测） */
#define DRI_BLE_BAUD_NUM       8U     /* 手册 CFG1/2/3 的全部 8 档 */

static uint8_t  s_rxByte;
static uint8_t  s_rxBuf[DRI_BLE_RX_BUF_SIZE];
static uint16_t s_rxLen;
static volatile uint32_t s_lastRxTick;
static volatile uint8_t  s_rxPending;
static uint8_t  s_online;
static uint8_t  s_baudLocked;
static uint32_t s_baud = DRI_BLE_BAUD_DEFAULT;

/* 分帧结果：最近一帧业务数据，等待上层取走 */
static uint8_t  s_frameBuf[DRI_BLE_RX_BUF_SIZE];
static uint16_t s_frameLen;
static uint8_t  s_frameReady;

/* ---------------- 内部工具 ---------------- */

static void Ble_FlushRx(void)
{
    s_rxLen = 0U;
    s_rxPending = 0U;
}

static uint8_t Ble_Contains(const uint8_t *buf, uint16_t len, const char *needle)
{
    uint16_t nlen = (uint16_t)strlen(needle);
    uint16_t i;

    if ((nlen == 0U) || (len < nlen))
    {
        return 0U;
    }

    for (i = 0U; (uint16_t)(i + nlen) <= len; i++)
    {
        if (memcmp(&buf[i], needle, nlen) == 0)
        {
            return 1U;
        }
    }

    return 0U;
}

/* 接收缓冲中是否包含指定 ASCII 片段（AT 应答判定用） */
static uint8_t Ble_RxContains(const char *needle)
{
    char text[DRI_BLE_RX_BUF_SIZE + 1U];
    uint16_t len = (s_rxLen > DRI_BLE_RX_BUF_SIZE) ? DRI_BLE_RX_BUF_SIZE : s_rxLen;

    memcpy(text, s_rxBuf, len);
    text[len] = '\0';

    return (strstr(text, needle) != NULL) ? 1U : 0U;
}

/* 从接收缓冲的文本中取出 key 之后、行尾之前的值（如 "NAME=" 后面到 \r\n 之间）。
 * 找到且非空返回 1，否则 out 置空返回 0。 */
static uint8_t Ble_CopyValue(const char *key, char *out, uint16_t outSize)
{
    char     text[DRI_BLE_RX_BUF_SIZE + 1U];
    char    *pos;
    uint16_t i = 0U;
    uint16_t len = (s_rxLen > DRI_BLE_RX_BUF_SIZE) ? DRI_BLE_RX_BUF_SIZE : s_rxLen;

    if (outSize == 0U)
    {
        return 0U;
    }

    out[0] = '\0';
    memcpy(text, s_rxBuf, len);
    text[len] = '\0';

    pos = strstr(text, key);
    if (pos == NULL)
    {
        return 0U;
    }

    pos += strlen(key);
    while ((pos[i] != '\0') && (pos[i] != '\r') && (pos[i] != '\n') &&
           (i < (uint16_t)(outSize - 1U)))
    {
        out[i] = pos[i];
        i++;
    }
    out[i] = '\0';

    return (i != 0U) ? 1U : 0U;
}

/* 在线标志变化时才打日志（连接/断开各一条），避免刷屏 */
static void Ble_SetOnline(uint8_t online)
{
    if (s_online == online)
    {
        return;
    }

    s_online = online;
    printf("[P6] BLE %s\r\n", (online != 0U) ? "connected" : "disconnected");
}

/* ---------------- 对外接口：收发 ---------------- */

void Ble_Send(const uint8_t *data, uint16_t len)
{
    if ((data != 0) && (len > 0U))
    {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)data, len, DRI_BLE_TX_TIMEOUT_MS);
    }
}

void Ble_SendString(const char *text)
{
    if (text != 0)
    {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text),
                                DRI_BLE_TX_TIMEOUT_MS);
    }
}

void Ble_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;

    if (s_rxLen < DRI_BLE_RX_BUF_SIZE)
    {
        s_rxBuf[s_rxLen++] = s_rxByte;
    }
    else
    {
        /* 缓冲满：丢弃本字节（异常帧，等空闲后整体清掉） */
    }

    s_lastRxTick = (uint32_t)xTaskGetTickCount();
    s_rxPending = 1U;

    (void)HAL_UART_Receive_IT(&huart2, &s_rxByte, 1U);   /* 重新挂接收 */
}

void Ble_RxErrorCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
    (void)HAL_UART_Receive_IT(&huart2, &s_rxByte, 1U);
}

void Ble_Poll(void)
{
    uint16_t len;

    /* 自愈：接收若因溢出/噪声错误被 HAL 中止（RxState 回到 READY），这里重新挂上，
     * 否则通道会永久失聪（表现为"模块明明回了却收不到"）。 */
    if (huart2.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_IT(&huart2, &s_rxByte, 1U);
    }

    if (s_rxPending == 0U)
    {
        return;
    }

    /* 总线空闲 >= 10ms 视为一帧结束 */
    if ((uint32_t)(xTaskGetTickCount() - s_lastRxTick) < pdMS_TO_TICKS(DRI_BLE_IDLE_MS))
    {
        return;
    }

    len = s_rxLen;
    s_rxLen = 0U;
    s_rxPending = 0U;          /* 先清标志：处理期间新到的字节进入新帧 */

    if (len > DRI_BLE_RX_BUF_SIZE)
    {
        len = DRI_BLE_RX_BUF_SIZE;
    }

    /* 先整块搬到工作缓冲，避免中断继续写入覆盖正在判定的内容 */
    memcpy(s_frameBuf, s_rxBuf, len);
    s_frameLen = len;
    s_frameReady = 0U;

    /* 模块主动上报的连接状态（手册第 10 页）：以 C/D 开头，必须先于业务解析判掉。
     * 先判 DISCONNECT，避免 "CONNECT OK" 子串误匹配。 */
    if (Ble_Contains(s_frameBuf, len, "DISCONNECT") != 0U)
    {
        Ble_SetOnline(0U);
        s_frameLen = 0U;
        return;
    }

    if (Ble_Contains(s_frameBuf, len, "CONNECT OK") != 0U)
    {
        Ble_SetOnline(1U);
        s_frameLen = 0U;
        return;
    }

    /* MCU 自己发 AT 命令时模块会先回显，这类回显不是业务数据 */
    if ((len >= 2U) && (s_frameBuf[0] == (uint8_t)'A') && (s_frameBuf[1] == (uint8_t)'T'))
    {
        s_frameLen = 0U;
        return;
    }

    s_frameReady = 1U;         /* 其余整帧交给上层（Modbus 业务） */
}

uint16_t Ble_TakeFrame(uint8_t *out, uint16_t maxLen)
{
    uint16_t len;

    if ((s_frameReady == 0U) || (out == 0) || (maxLen == 0U) || (s_frameLen == 0U))
    {
        return 0U;
    }

    len = (s_frameLen > maxLen) ? maxLen : s_frameLen;
    memcpy(out, s_frameBuf, len);

    s_frameReady = 0U;
    s_frameLen = 0U;

    return len;
}

uint8_t Ble_IsOnline(void)
{
    return s_online;
}

uint32_t Ble_GetBaud(void)
{
    return s_baud;
}

/* ---------------- 初始化与 AT 握手 ---------------- */

void Ble_Init(void)
{
    s_rxLen = 0U;
    s_rxPending = 0U;
    s_frameLen = 0U;
    s_frameReady = 0U;
    s_online = 0U;

    /* 先按默认速率初始化；实际速率由 Ble_BringUp() 里的 AT 识别流程锁定。 */
    huart2.Init.BaudRate = DRI_BLE_BAUD_DEFAULT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        printf("[P6] BLE USART2 init FAIL\r\n");
    }

    if (HAL_UART_Receive_IT(&huart2, &s_rxByte, 1U) != HAL_OK)
    {
        printf("[P6] BLE USART2 rx arm FAIL\r\n");
    }
}

/* 切换波特率并重新挂接收 */
static void Ble_ApplyBaud(uint32_t baud)
{
    huart2.Init.BaudRate = baud;
    (void)HAL_UART_Init(&huart2);
    (void)HAL_UART_Receive_IT(&huart2, &s_rxByte, 1U);
}

/* 在当前速率下发一次 AT，收到 OK 返回 1 */
static uint8_t Ble_PingAt(void)
{
    Ble_FlushRx();
    Ble_SendString("AT\r\n");
    vTaskDelay(pdMS_TO_TICKS(DRI_BLE_AT_DETECT_MS));

    return Ble_RxContains("OK");
}

/* 依次尝试各档速率（默认优先，其次 9600），命中即锁定并打印。全部失败返回 0。 */
static uint8_t Ble_DetectBaud(void)
{
    static const uint32_t order[DRI_BLE_BAUD_NUM] =
    {
        115200U, 9600U, 57600U, 38400U, 19200U, 14400U, 4800U, 2400U
    };
    uint8_t i;
    uint8_t tries;

    /* 已经锁定过：直接用当前速率确认一次，不通再重新全扫 */
    if ((s_baudLocked != 0U) && (Ble_PingAt() != 0U))
    {
        return 1U;
    }
    s_baudLocked = 0U;

    for (tries = 0U; tries < 2U; tries++)
    {
        for (i = 0U; i < DRI_BLE_BAUD_NUM; i++)
        {
            Ble_ApplyBaud(order[i]);

            if (Ble_PingAt() != 0U)
            {
                s_baud = order[i];
                s_baudLocked = 1U;
                printf("[P6] BLE baud=%lu (AT ok)\r\n", (unsigned long)s_baud);
                return 1U;
            }
        }
    }

    Ble_ApplyBaud(DRI_BLE_BAUD_DEFAULT);
    return 0U;
}

/* 上电只打印一次：模块名称 + 当前连接状态 + 锁定速率。
 * 名字只查询（AT+NAME?）不写入，不消耗模块的 10 次改名额度。 */
static void Ble_ReportOnce(void)
{
    static uint8_t s_reported = 0U;
    char name[24];

    if (s_reported != 0U)
    {
        return;
    }
    s_reported = 1U;

    Ble_FlushRx();
    Ble_SendString("AT+NAME?\r\n");
    vTaskDelay(pdMS_TO_TICKS(DRI_BLE_AT_WAIT_MS));

    if ((Ble_CopyValue("NAME=", name, sizeof(name)) == 0U) &&
        (Ble_CopyValue("NAME:", name, sizeof(name)) == 0U))
    {
        name[0] = '?';
        name[1] = '\0';
    }

    /* 查询窗口内模块可能已上报连接状态，先更新在线标志，避免被 flush 丢掉 */
    if (Ble_RxContains("CONNECT OK") != 0U)
    {
        s_online = 1U;
    }
    else if (Ble_RxContains("DISCONNECT") != 0U)
    {
        s_online = 0U;
    }

    Ble_FlushRx();

    printf("[P6] BLE name=%s link=%s baud=%lu\r\n", name,
           (s_online != 0U) ? "connected" : "disconnected",
           (unsigned long)s_baud);
}

uint8_t Ble_BringUp(void)
{
    static uint8_t s_bootWaitDone = 0U;

    /* 首次调用先等模块上电自检（约 1~2s），后续重试不再重复等待 */
    if (s_bootWaitDone == 0U)
    {
        s_bootWaitDone = 1U;
        vTaskDelay(pdMS_TO_TICKS(DRI_BLE_BOOT_WAIT_MS));
    }

    if (Ble_DetectBaud() == 0U)
    {
        printf("[P6] BLE no AT reply\r\n");
        Ble_FlushRx();
        return 0U;
    }

#if (DRI_BLE_NAME_PROVISION != 0)
    /* 仅当明确启用改名时才写名字：先查询，已等于目标名则不写，省下 10 次写额度 */
    Ble_FlushRx();
    Ble_SendString("AT+NAME?\r\n");
    vTaskDelay(pdMS_TO_TICKS(DRI_BLE_AT_WAIT_MS));

    if (Ble_RxContains(DRI_BLE_NAME_TARGET) == 0U)
    {
        Ble_FlushRx();
        Ble_SendString("AT+NAME=" DRI_BLE_NAME_TARGET "\r\n");
        vTaskDelay(pdMS_TO_TICKS(600U));

        if (Ble_RxContains("OK") != 0U)
        {
            /* 复位让新名字进入广播，复位后 UART 需重新挂接收 */
            Ble_FlushRx();
            Ble_SendString("AT+RST\r\n");
            vTaskDelay(pdMS_TO_TICKS(1200U));
            Ble_ApplyBaud(s_baud);
            printf("[P6] BLE renamed -> %s\r\n", DRI_BLE_NAME_TARGET);
        }
        else
        {
            printf("[P6] BLE rename FAIL\r\n");
        }
    }

    Ble_FlushRx();
#endif

    Ble_ReportOnce();   /* 名称/连接状态/速率：全程只打印这一次 */

    return 1U;
}
