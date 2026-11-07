/*
 * @file    app_fan.c
 * @brief   风扇控制策略：按铝板温度自动启停（回差 3℃ + 最小间隔 5s）。
 *
 * 分层（2026-11-06 重构）：
 *   - 引脚操作下沉到 Dri/dri_fan.c（Fan_PinInit / Fan_PinSet）；
 *   - 本文件只保留"何时开、何时关"的温度策略与状态记忆，
 *     对外 API 与重构前完全一致（Fan_Init / Fan_Start / Fan_Stop /
 *     Fan_AutoByPlateTemp / Fan_IsOn），调用方无需改动。
 *
 * 自动控制（P4）：铝板温度 ≥ 35.0℃ 开风扇，≤ 32.0℃ 关风扇（回差 3℃ 防抖）；
 * 另外两次启停之间强制间隔 5s，双保险，避免温度抖动导致风扇频繁启停。
 */

#include "app_fan.h"
#include "dri_fan.h"        /* 风扇引脚：Fan_PinInit / Fan_PinSet */
#include "FreeRTOS.h"
#include "task.h"       /* xTaskGetTickCount：启停间隔计时 */

static uint8_t  s_fanOn = 0U;            /* 1=当前判定为开启 */
static uint8_t  s_switchValid = 0U;      /* 1=已记录过一次切换时刻 */
static uint32_t s_lastSwitchTick = 0U;   /* 上次切换时的 tick */

/* 初始化：默认停止。 */
void Fan_Init(void)
{
    Fan_Stop();
    s_fanOn = 0U;
    s_switchValid = 0U;
    s_lastSwitchTick = 0U;
}

/* 启动风扇：MOTOR_ON 置高电平。 */
void Fan_Start(void)
{
    Fan_PinSet(1U);
}

/* 停止风扇：MOTOR_ON 置低电平。 */
void Fan_Stop(void)
{
    Fan_PinSet(0U);
}

uint8_t Fan_IsOn(void)
{
    return s_fanOn;
}

/* 按铝板温度自动启停：回差 3℃（开 35.0 / 关 32.0）+ 5s 最小切换间隔。 */
void Fan_AutoByPlateTemp(int16_t plateC10)
{
    uint32_t now;
    uint32_t gap;

    /* 传感器异常：保持当前状态，不动作。 */
    if (plateC10 == INT16_MIN)
    {
        return;
    }

    now = (uint32_t)xTaskGetTickCount();

    if (s_switchValid == 0U)
    {
        /* 首次调用：允许立即动作，并开始计时。 */
        s_switchValid = 1U;
        s_lastSwitchTick = now;
        gap = APP_FAN_SWITCH_GAP_MS;
    }
    else
    {
        gap = now - s_lastSwitchTick;
    }

    if ((plateC10 >= APP_FAN_PLATE_ON_C10) && (s_fanOn == 0U))
    {
        if (gap >= APP_FAN_SWITCH_GAP_MS)
        {
            Fan_Start();
            s_fanOn = 1U;
            s_lastSwitchTick = now;
        }
    }
    else if ((plateC10 <= APP_FAN_PLATE_OFF_C10) && (s_fanOn != 0U))
    {
        if (gap >= APP_FAN_SWITCH_GAP_MS)
        {
            Fan_Stop();
            s_fanOn = 0U;
            s_lastSwitchTick = now;
        }
    }
    else
    {
        /* 处于回差带（32.0~35.0℃）内：保持当前状态 */
    }
}
