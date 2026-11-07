/*
 * menu_ui.h
 * P4 UI：4 按键 + SSD1315 OLED 菜单。
 *
 * 按键约定（支持长按连发）：
 *   KEY1(PC6) 切换焦点项：速度 -> 铝板温度 -> 液体温度 -> 时间 -> 速度
 *   KEY1 长按（≥600ms）  进入/退出 RTC 校时页（不依赖串口 RX）
 *   KEY2(PC7) 当前项参数 +   （短按 1 步，长按约 600ms 后每 80ms 连发）
 *   KEY3(PC8) 当前项参数 -
 *   KEY4(PC9) 电机运行 / 停止
 *
 * 校时页（长按 KEY1 进入，独立页面）：
 *   KEY1 循环切换 年/月/日/时/分/秒，KEY2/KEY3 增减（年 2000~2099、月 1~12、
 *   日按当月实际天数、时/分/秒 24/60 循环），KEY4 写入 RTC 并退出；
 *   星期由 app_rtc 按公历自动推算。日期与时间都必须可改——
 *   只改时分秒会让日期永久停在固件编译日。
 *
 * 显示（128x64，5x7 字体，行高 10px，共 6 行），每行格式为“设定值/实际值”：
 *   TempStir V1.0
 *   >Speed  100/80 rpm    设定目标转速 / 编码器实测转速（均为电机轴 RPM，每分钟转数）
 *    Plate   60/25 C      设定上限温度 / 铝板实测温度
 *    Liquid  45/23 C      设定目标温度 / 液体实测温度
 *   >Time   60/45 s       设定时间 / 剩余时间（秒）
 *   Motor  Running        运行状态：Running / Idle
 *
 * 定时：电机启动时刻开始计时（xTaskGetTickCount），剩余时间归零时自动停机。
 * 设定时间单位为秒，0 表示不限时。
 *
 * 注意：SSD1315 驱动要求“显示权收敛到单个任务”，本模块的 Menu_UiTask 是
 * 唯一操作 OLED 的任务；Menu_UiInit 必须在 vTaskStartScheduler() 之前调用。
 */
#ifndef MENU_UI_H
#define MENU_UI_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* 速度：电机轴 RPM，0 起，越大越快，上限 500（超速限制已移除，仅剩占空比硬上限保护）
 * 与编码器反馈同量纲，PID 才能真正收敛到设定值。 */
#define MENU_UI_SPEED_MIN    0
#define MENU_UI_SPEED_MAX    500
#define MENU_UI_SPEED_STEP   5

/* 铝板温度：整数 ℃，达到该值强制关加热（0 = 不限制） */
#define MENU_UI_PLATE_MIN    0
#define MENU_UI_PLATE_MAX    120
#define MENU_UI_PLATE_STEP   1

/* 液体温度：整数 ℃，加热 PID 的目标值（0 = 不加热） */
#define MENU_UI_LIQUID_MIN   0
#define MENU_UI_LIQUID_MAX   100
#define MENU_UI_LIQUID_STEP  1

/* 定时：整数秒，电机启动后倒计时，归零自动停机；0 = 不限时 */
#define MENU_UI_TIME_MIN     0
#define MENU_UI_TIME_MAX     999
#define MENU_UI_TIME_STEP    1

/* 初值：上电电机停止、速度 0、液体目标 0（不加热）、不限时；
 * 铝板上限默认 40℃（P7 需求：板子最热 40℃，防止空烧/干烧把铝板烧过头）。 */
#define MENU_UI_SPEED_DEFAULT   0
#define MENU_UI_PLATE_DEFAULT   40
#define MENU_UI_LIQUID_DEFAULT  0
#define MENU_UI_TIME_DEFAULT    0

/* 初始化：初始化 OLED、载入菜单初值并同步到全局状态。
 * 必须且只能在 vTaskStartScheduler() 之前调用一次。 */
void Menu_UiInit(void);

/* UI 任务体：按键扫描 + 消抖/长按 + 定时倒计时 + 显示刷新（周期 40ms）。 */
void Menu_UiTask(void *argument);

/* ---------------- P6 通信桥（供 app_comm / app_modbus 调用） ----------------
 * 上位机或蓝牙写参数时走这些接口，与面板按键共用同一份设定、同一套下发路径，
 * 保证「屏幕显示 / EEPROM / 执行器」三者一致；内部会自动请求参数落盘。 */
void    Menu_UiCommSetSpeed(int32_t rpm);
void    Menu_UiCommSetPlateLimit(int32_t c);
void    Menu_UiCommSetLiquid(int32_t c);
void    Menu_UiCommSetTime(uint32_t sec);
void    Menu_UiCommSetRunning(uint8_t run);   /* 1=启动 0=停止（等价面板 KEY4） */
void    Menu_UiCommReloadParams(void);        /* 恢复默认参数后重新载入面板 */

/* 面板焦点项（上位机 KEY1 的等价操作）：0=速度 1=铝板 2=液体 3=时间。
 * 写入后 OLED 上的 ">" 箭头立即移动到同一行，与面板 KEY1 完全一致。 */
void    Menu_UiCommSetFocus(uint8_t idx);
uint8_t Menu_UiGetFocus(void);

/* 设定值/运行状态读取（Modbus 寄存器快照用） */
int32_t  Menu_UiGetSpeed(void);
int32_t  Menu_UiGetPlate(void);
int32_t  Menu_UiGetLiquid(void);
uint32_t Menu_UiGetTime(void);
uint32_t Menu_UiGetRemainSec(void);
uint8_t  Menu_UiIsRunning(void);

#endif /* MENU_UI_H */
