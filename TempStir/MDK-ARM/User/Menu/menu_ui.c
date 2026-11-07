/*
 * menu_ui.c
 * P4 UI 实现：菜单状态机 + 定时停机 + OLED 显示刷新。
 *
 * 设计约束：
 *  - 只由 Menu_UiTask 操作 OLED（ssd1315 的帧缓冲无内部锁）。
 *  - 按键的采样/消抖/长按/连发由 BSP 模块 int_key 负责（Int_KeyScan 每 40ms
 *    产出 press/click/longOn/repeat/release 事件）；本文件只把这些事件翻译成
 *    业务语义（切焦点/加减/切页面/启停），不再直接读引脚、不维护消抖计数。
 *  - 参数改动立即下发到 app_motor / app_heat，并同步写回 app_main_state。
 *  - 定时使用 FreeRTOS 的 xTaskGetTickCount() 计时，到时自动停机。
 *  - 实际转速取自 app_main_state.cur_speed（由 CTRL 任务每 200ms 写入），
 *    避免 UI 另起编码器采样、干扰 PID 的采样基线。
 */
#include "menu_ui.h"
#include "int_key.h"      /* BSP：4 路按键事件扫描（消抖/长按/连发） */
#include "app_heat.h"
#include "app_fault.h"    /* P7：看门狗心跳 */
#include "dri_buzzer.h"   /* P7：到温完成提示音 */
#include "app_comm.h"     /* P6：按键活动通知（通信写入仲裁） */
#include "app_main.h"
#include "app_motor.h"
#include "app_temp.h"
#include "app_store.h"
#include "app_rtc.h"
#include "app_imu.h"      /* P9：IMU 页（第 7 页） */
#include "dri_ssd1315.h"
#include "usart.h"   /* huart1：串口校时命令的轮询接收 */
#include <stdio.h>
#include <string.h>

/* UI 任务周期(ms)：40ms 一次按键扫描 + 页面刷新（与 int_key 的延时参数配套） */
#define MENU_UI_PERIOD_MS 40U
/* 无按键时也周期性重绘：40ms * 12 = 480ms */
#define MENU_UI_REFRESH_DIV 12U

/* 行高：5x7 字体 + 3px 行距，6 行 × 10px = 60px */
#define MENU_UI_LINE_H 10U

/* 按键位定义：直接沿用 int_key 的事件位图，避免两套编号 */
#define MENU_UI_KEY1_BIT INT_KEY1_BIT
#define MENU_UI_KEY2_BIT INT_KEY2_BIT
#define MENU_UI_KEY3_BIT INT_KEY3_BIT
#define MENU_UI_KEY4_BIT INT_KEY4_BIT

/* ---- 菜单状态 ---- */
static int32_t menu_ui_speed = MENU_UI_SPEED_DEFAULT;    /* 速度设定 0~500 */
static int32_t menu_ui_plate = MENU_UI_PLATE_DEFAULT;    /* 铝板温度设定(℃) */
static int32_t menu_ui_liquid = MENU_UI_LIQUID_DEFAULT;  /* 液体温度设定(℃) */
static uint32_t menu_ui_time = MENU_UI_TIME_DEFAULT;     /* 定时设定(秒)，0=不限时 */
static uint8_t menu_ui_running = 0U;                     /* 1=电机运行 */
static App_UiFocus_t menu_ui_focus = APP_UI_FOCUS_SPEED; /* 当前焦点项 */
static uint8_t menu_ui_oledOk = 0U;                      /* 1=OLED 初始化成功 */
static uint8_t menu_ui_dirty = 1U;                       /* 1=需要立即重绘 */

/* ---- 定时状态 ---- */
static uint32_t menu_ui_startTick = 0U; /* 电机启动时刻(tick) */
static uint32_t menu_ui_remainSec = 0U; /* 剩余秒数（显示用） */

/* ---- 校时模式（KEY1 长按进入/退出，不依赖串口 RX） ---- */
static uint8_t menu_ui_clkMode = 0U;   /* 1 = 正在校时 */
static uint8_t menu_ui_clkField = 0U;  /* 0=年 1=月 2=日 3=时 4=分 5=秒 */
static uint16_t menu_ui_clkY = 2000U;  /* 年 */
static uint8_t menu_ui_clkMo = 1U;     /* 月 */
static uint8_t menu_ui_clkD = 1U;      /* 日 */
static uint8_t menu_ui_clkH = 0U;
static uint8_t menu_ui_clkM = 0U;
static uint8_t menu_ui_clkS = 0U;
static uint8_t menu_ui_k1Long = 0U;    /* KEY1 长按已消费：按住期间屏蔽 KEY1 连发 */

/* ---- P8 标定页（长按 KEY4 进入 / 保存退出） ----
 * 只改"出厂标定参数"（编码器 PPR、减速比、NTC 曲线、加热过流阈值、温度偏移），
 * 不动速度/温度目标，也不改变电机与加热状态，与主界面完全隔离。 */
#define MENU_UI_CAL_ITEMS  8U
static uint8_t menu_ui_calMode = 0U;   /* 1 = 正在标定页 */
static uint8_t menu_ui_calIdx  = 0U;   /* 当前项 0~7 */
static int32_t menu_ui_calPpr  = 44;
static int32_t menu_ui_calGear = 3000;   /* 减速比×100 */
static int32_t menu_ui_calR25  = 10000;  /* Ω */
static int32_t menu_ui_calB    = 3950;   /* K */
static int32_t menu_ui_calRup  = 10000;  /* Ω */
static int32_t menu_ui_calOc   = 3500;   /* ADC raw */
static int32_t menu_ui_calOffL = 0;      /* 0.1℃ */
static int32_t menu_ui_calOffB = 0;      /* 0.1℃ */
static uint8_t menu_ui_k4Long  = 0U;     /* KEY4 长按已消费：按住期间屏蔽 KEY4 连发
                                          * （"按下成立/短按点击"已下沉到 int_key 的 armed/click） */

/* P9：第 7 页——IMU 六轴/姿态页（长按 KEY3 进入/退出，与校时页/标定页互斥）。
 * 注意：本页占用 KEY3 长按，会导致 KEY3 长按连发减值失效；
 * 现按用户要求暂时停用（入口不调用、按键长按归还给 KEY3 连发），代码保留备后续启用。 */
#define MENU_UI_IMU_PAGE_ENABLE 0U       /* 0 = 停用 IMU 页（KEY3 长按恢复连发减值）；1 = 启用 */

#if (MENU_UI_IMU_PAGE_ENABLE != 0U)
static uint8_t menu_ui_imuMode = 0U;     /* 1 = 正在显示 IMU 页 */
static uint8_t menu_ui_k3Long  = 0U;     /* KEY3 长按已消费：按住期间屏蔽 KEY3 连发 */

static void Menu_UiToggleImuMode(void);  /* P9：前向声明（HandleKeys 先于实现使用） */
static void Menu_UiRenderImu(void);
#endif

/* ---- DS_INT(1Hz 秒节拍) 自检：使能后 5s 无通知即判定中断链路不可用 ---- */
#define MENU_UI_SQW_TIMEOUT_MS 5000U
static uint32_t menu_ui_sqwTick = 0U;
static uint8_t  menu_ui_sqwWarned = 0U;

/* ---- 参数保存：防抖（静置 2s 落盘）由 app_store 统一实现，
 * 长按连发时参数每 80ms 变一次，逐次写会快速消耗 EEPROM 擦写寿命。 ---- */

/* 把菜单状态同步到全局状态（供串口日志/后续模块使用）。 */
static void Menu_UiSyncState(void)
{
    App_Main_StateLock();
    app_main_state.target_speed = menu_ui_speed;
    app_main_state.target_temp = menu_ui_liquid;    /* 液体目标温度(℃) */
    app_main_state.target_time = menu_ui_time;      /* 秒 */
    app_main_state.remain_time = menu_ui_remainSec; /* 秒 */
    app_main_state.ui_focus = menu_ui_focus;
    app_main_state.sys_state = (menu_ui_running != 0U) ? APP_SYS_RUNNING : APP_SYS_IDLE;
    App_Main_StateUnlock();
}

/* 把焦点项的设定值下发到执行器。 */
static void Menu_UiApply(void)
{
    switch (menu_ui_focus) {
        case APP_UI_FOCUS_SPEED:
            /* 仅在运行中实时改变目标；停止时只记录，等 KEY4 启动时使用。 */
            if (menu_ui_running != 0U) {
                App_MotorSetTargetRpm(menu_ui_speed);
            }
            break;

        case APP_UI_FOCUS_PLATE:
            /* 铝板上限温度：达到即强制关加热，防止把铝板烧过头。 */
            App_HeatSetPlateLimitTemp((int16_t)(menu_ui_plate * 10));
            break;

        case APP_UI_FOCUS_TEMP:
            /* 液体目标温度：加热 PID 的设定值（0 = 不加热）。 */
            App_HeatSetTargetTemp((int16_t)(menu_ui_liquid * 10));
            break;

        case APP_UI_FOCUS_TIME:
            /* 运行中修改定时：以新时长重新开始计时。 */
            if (menu_ui_running != 0U) {
                menu_ui_startTick = (uint32_t)xTaskGetTickCount();
                menu_ui_remainSec = menu_ui_time;
            }
            break;

        default:
            break;
    }
}

/* 电机启停：以目标 0 平滑停车（软启动会按 5%/周期下降），而非硬切 PWM。 */
static void Menu_UiToggleMotor(void)
{
    if (menu_ui_running != 0U) {
        menu_ui_running = 0U;
        menu_ui_remainSec = menu_ui_time;
        App_MotorSetTargetRpm(0);
        /* 停机后统计稳定，顺带把累计运行时间/启动次数落盘（静置 2s 后写入）。 */
        App_StoreRequestSave();
    } else {
        menu_ui_running = 1U;
        menu_ui_startTick = (uint32_t)xTaskGetTickCount(); /* 启动即开始计时 */
        menu_ui_remainSec = menu_ui_time;
        App_MotorSetTargetRpm(menu_ui_speed);
        App_StoreAddMotorStart();
    }
}

/* 倒计时：用 FreeRTOS tick 计时，归零自动停机。每 40ms 调用一次。 */
static void Menu_UiTickTimer(void)
{
    uint32_t now;
    uint32_t elapsedMs;
    uint32_t totalMs;

    if ((menu_ui_running == 0U) || (menu_ui_time == 0U)) {
        menu_ui_remainSec = menu_ui_time; /* 未运行或不限时：剩余 = 设定 */
        return;
    }

    now = (uint32_t)xTaskGetTickCount();
    elapsedMs = (now - menu_ui_startTick) * 1000U / (uint32_t)configTICK_RATE_HZ;
    totalMs = menu_ui_time * 1000U;

    if (elapsedMs >= totalMs) {
        /* 时间到：自动停机 */
        menu_ui_running = 0U;
        menu_ui_remainSec = 0U;
        App_MotorSetTargetRpm(0);
        Menu_UiSyncState();
        App_StoreRequestSave(); /* 到时停机：顺带把运行统计落盘 */
        menu_ui_dirty = 1U; /* 立即刷新，让屏幕反映停机 */
        return;
    }

    menu_ui_remainSec = (totalMs - elapsedMs + 999U) / 1000U; /* 向上取整到秒 */
}

/* 标记参数已改动：先同步到 app_store 的运行时镜像，再由它静置 2s 后统一落盘。 */
static void Menu_UiRequestSave(void)
{
    App_StoreSetUi(menu_ui_speed, menu_ui_plate, menu_ui_liquid, menu_ui_time);
    App_StoreRequestSave();
}

/* 静置时间到则写 EEPROM；未到或写失败时保持 pending，下个周期继续尝试。 */
static void Menu_UiFlushSave(void)
{
    (void)App_StoreFlushSave();
}

static int32_t Menu_UiClamp(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* 上电恢复上次保存的参数（取自 app_store 运行时镜像，其内容已包含默认值/V1 迁移结果）。 */
static void Menu_UiLoadParams(void)
{
    const App_Store_Params_t *p = App_StoreGet();

    menu_ui_speed  = Menu_UiClamp(p->speed, MENU_UI_SPEED_MIN, MENU_UI_SPEED_MAX);
    menu_ui_plate  = Menu_UiClamp(p->plateTemp, MENU_UI_PLATE_MIN, MENU_UI_PLATE_MAX);
    menu_ui_liquid = Menu_UiClamp(p->liquidTemp, MENU_UI_LIQUID_MIN, MENU_UI_LIQUID_MAX);

    /* 铝板上限合法性自愈：1~34℃ 这类配置在室温下就会锁存且永远无法回差解除，
     * 会把加热永久关断（旧固件允许写入 30℃ 就是这个坑）。恢复时按默认 40℃ 处理，
     * 并把修正值写回 EEPROM，避免每次上电重复告警。 */
    if ((menu_ui_plate > 0) && (menu_ui_plate < (APP_HEAT_PLATE_LIMIT_MIN_C10 / 10))) {
        printf("[P7] stored plate limit %dC invalid -> %dC\r\n",
               (int)menu_ui_plate, (int)APP_STORE_PLATE_DEFAULT_C);
        menu_ui_plate = APP_STORE_PLATE_DEFAULT_C;
        Menu_UiRequestSave();
    }

    if (p->runTimeSec <= (uint32_t)MENU_UI_TIME_MAX) {   /* 无符号量无需比较下界 */
        menu_ui_time = p->runTimeSec;
    }
}

/* ---------------- P6 通信桥 ----------------
 * 上位机/蓝牙写参数时走这里，和面板按键共用同一份设定与下发路径，
 * 保证「屏幕显示、EEPROM、执行器」三者始终一致。 */
void Menu_UiCommSetSpeed(int32_t rpm)
{
    menu_ui_speed = Menu_UiClamp(rpm, MENU_UI_SPEED_MIN, MENU_UI_SPEED_MAX);
    if (menu_ui_running != 0U) {
        App_MotorSetTargetRpm(menu_ui_speed);
    }
    Menu_UiRequestSave();
    Menu_UiSyncState();
    menu_ui_dirty = 1U;
}

void Menu_UiCommSetPlateLimit(int32_t c)
{
    int32_t v = Menu_UiClamp(c, MENU_UI_PLATE_MIN, MENU_UI_PLATE_MAX);

    /* 0=不限制；其余最小 35℃（低于该值会永久锁死加热，按不限制处理） */
    if ((v > 0) && (v < (APP_HEAT_PLATE_LIMIT_MIN_C10 / 10))) {
        v = 0;
    }
    menu_ui_plate = v;
    App_HeatSetPlateLimitTemp((int16_t)(menu_ui_plate * 10));
    Menu_UiRequestSave();
    Menu_UiSyncState();
    menu_ui_dirty = 1U;
}

void Menu_UiCommSetLiquid(int32_t c)
{
    menu_ui_liquid = Menu_UiClamp(c, MENU_UI_LIQUID_MIN, MENU_UI_LIQUID_MAX);
    App_HeatSetTargetTemp((int16_t)(menu_ui_liquid * 10));
    Menu_UiRequestSave();
    Menu_UiSyncState();
    menu_ui_dirty = 1U;
}

void Menu_UiCommSetTime(uint32_t sec)
{
    if (sec > (uint32_t)MENU_UI_TIME_MAX) {
        sec = (uint32_t)MENU_UI_TIME_MAX;
    }
    menu_ui_time = sec;

    /* 运行中改定时：以新时长重新开始计时（与面板行为一致） */
    if (menu_ui_running != 0U) {
        menu_ui_startTick = (uint32_t)xTaskGetTickCount();
    }
    menu_ui_remainSec = menu_ui_time;

    Menu_UiRequestSave();
    Menu_UiSyncState();
    menu_ui_dirty = 1U;
}

void Menu_UiCommSetRunning(uint8_t run)
{
    if (((run != 0U) && (menu_ui_running == 0U)) ||
        ((run == 0U) && (menu_ui_running != 0U))) {
        Menu_UiToggleMotor();     /* 复用面板 KEY4 的启停语义（含统计与落盘） */
    }
    Menu_UiSyncState();
    menu_ui_dirty = 1U;
}

void Menu_UiCommReloadParams(void)
{
    Menu_UiLoadParams();
    App_HeatSetTargetTemp((int16_t)(menu_ui_liquid * 10));
    App_HeatSetPlateLimitTemp((int16_t)(menu_ui_plate * 10));
    if (menu_ui_running != 0U) {
        App_MotorSetTargetRpm(menu_ui_speed);
    }
    Menu_UiSyncState();
    menu_ui_dirty = 1U;
}

/* 上位机切换焦点：与面板 KEY1 完全等价（箭头同步移动） */
void Menu_UiCommSetFocus(uint8_t idx)
{
    App_UiFocus_t f;

    switch (idx) {
        case 0U: f = APP_UI_FOCUS_SPEED; break;
        case 1U: f = APP_UI_FOCUS_PLATE; break;
        case 2U: f = APP_UI_FOCUS_TEMP;  break;
        case 3U: f = APP_UI_FOCUS_TIME;  break;
        default: return;
    }

    if (f != menu_ui_focus) {
        menu_ui_focus = f;
        Menu_UiSyncState();
    }
    menu_ui_dirty = 1U;      /* 下一帧刷新立即重画箭头 */
}

uint8_t Menu_UiGetFocus(void)
{
    switch (menu_ui_focus) {
        case APP_UI_FOCUS_SPEED: return 0U;
        case APP_UI_FOCUS_PLATE: return 1U;
        case APP_UI_FOCUS_TEMP:  return 2U;
        case APP_UI_FOCUS_TIME:  return 3U;
        default:                 return 0U;
    }
}

int32_t Menu_UiGetSpeed(void)
{
    return menu_ui_speed;
}

int32_t Menu_UiGetPlate(void)
{
    return menu_ui_plate;
}

int32_t Menu_UiGetLiquid(void)
{
    return menu_ui_liquid;
}

uint32_t Menu_UiGetTime(void)
{
    return menu_ui_time;
}

uint32_t Menu_UiGetRemainSec(void)
{
    return menu_ui_remainSec;
}

uint8_t Menu_UiIsRunning(void)
{
    return menu_ui_running;
}

/* 焦点循环：速度 -> 铝板温度 -> 液体温度 -> 时间 -> 速度 */
static void Menu_UiNextFocus(void)
{
    if (menu_ui_focus == APP_UI_FOCUS_SPEED) {
        menu_ui_focus = APP_UI_FOCUS_PLATE;
    } else if (menu_ui_focus == APP_UI_FOCUS_PLATE) {
        menu_ui_focus = APP_UI_FOCUS_TEMP;
    } else if (menu_ui_focus == APP_UI_FOCUS_TEMP) {
        menu_ui_focus = APP_UI_FOCUS_TIME;
    } else {
        menu_ui_focus = APP_UI_FOCUS_SPEED;
    }
}

/* 当前焦点项参数增减：dir = +1 加，-1 减。 */
static void Menu_UiAdjustFocus(int8_t dir)
{
    switch (menu_ui_focus) {
        case APP_UI_FOCUS_SPEED:
            menu_ui_speed += (int32_t)dir * MENU_UI_SPEED_STEP;
            if (menu_ui_speed > MENU_UI_SPEED_MAX) {
                menu_ui_speed = MENU_UI_SPEED_MAX;
            }
            if (menu_ui_speed < MENU_UI_SPEED_MIN) {
                menu_ui_speed = MENU_UI_SPEED_MIN;
            }
            break;

        case APP_UI_FOCUS_PLATE:
            /* 铝板上限取值域 = {0(不限制)} ∪ [35, 120]℃：
             * 从 0 加一档直接跳到 35，从 35 减一档直接回 0，
             * 中间 1~34℃ 无法选中（那样的限值会把加热永久锁死）。 */
            if ((menu_ui_plate == 0) && (dir > 0)) {
                menu_ui_plate = APP_HEAT_PLATE_LIMIT_MIN_C10 / 10;
            } else if ((menu_ui_plate <= (APP_HEAT_PLATE_LIMIT_MIN_C10 / 10)) && (dir < 0)) {
                menu_ui_plate = 0;
            } else {
                menu_ui_plate += (int32_t)dir * MENU_UI_PLATE_STEP;
            }
            if (menu_ui_plate > MENU_UI_PLATE_MAX) {
                menu_ui_plate = MENU_UI_PLATE_MAX;
            }
            if (menu_ui_plate < MENU_UI_PLATE_MIN) {
                menu_ui_plate = MENU_UI_PLATE_MIN;
            }
            break;

        case APP_UI_FOCUS_TEMP:
            menu_ui_liquid += (int32_t)dir * MENU_UI_LIQUID_STEP;
            if (menu_ui_liquid > MENU_UI_LIQUID_MAX) {
                menu_ui_liquid = MENU_UI_LIQUID_MAX;
            }
            if (menu_ui_liquid < MENU_UI_LIQUID_MIN) {
                menu_ui_liquid = MENU_UI_LIQUID_MIN;
            }
            break;

        case APP_UI_FOCUS_TIME:
            if (dir > 0) {
                if (menu_ui_time < MENU_UI_TIME_MAX) {
                    menu_ui_time += MENU_UI_TIME_STEP;
                }
            } else {
                if (menu_ui_time >= MENU_UI_TIME_STEP) {
                    menu_ui_time -= MENU_UI_TIME_STEP;
                } else {
                    menu_ui_time = MENU_UI_TIME_MIN;
                }
            }
            break;

        default:
            break;
    }

    Menu_UiApply();
    Menu_UiSyncState();
    Menu_UiRequestSave();   /* 参数变化：静置 2s 后保存到 EEPROM */
}

/* ---- 校时模式：不依赖串口 RX，长按 KEY1 进入/退出 ----
 * KEY1 短按切换字段（年/月/日/时/分/秒），KEY2/KEY3 增减（循环），
 * KEY4 写入 RTC 并退出；星期由 app_rtc 按公历自动推算。 */
static void Menu_UiToggleClockMode(void)
{
    DS3231_Time_t t;

    if (menu_ui_clkMode == 0U) {
        /* 进入：把 RTC 现值装入编辑器（不可用时从 2000-01-01 00:00:00 起调） */
        if (App_RtcGetTime(&t) != 0U) {
            menu_ui_clkY  = t.year;
            menu_ui_clkMo = t.month;
            menu_ui_clkD  = t.day;
            menu_ui_clkH  = t.hour;
            menu_ui_clkM  = t.minute;
            menu_ui_clkS  = t.second;
        } else {
            menu_ui_clkY  = 2000U;
            menu_ui_clkMo = 1U;
            menu_ui_clkD  = 1U;
            menu_ui_clkH  = 0U;
            menu_ui_clkM  = 0U;
            menu_ui_clkS  = 0U;
        }
        menu_ui_clkField = 0U;
        menu_ui_clkMode = 1U;
    } else {
        menu_ui_clkMode = 0U;
    }

    menu_ui_dirty = 1U;
}

/* 校时字段增减：年 2000~2099、月 1~12、日按当月天数、时/分/秒 24/60 循环。 */
static void Menu_UiAdjustClock(int8_t dir)
{
    uint8_t maxDay;

    switch (menu_ui_clkField) {
        case 0U:   /* 年 */
            menu_ui_clkY = (uint16_t)(2000U + (uint16_t)((menu_ui_clkY - 2000U + 100U + dir) % 100U));
            break;

        case 1U:   /* 月 */
            menu_ui_clkMo = (uint8_t)(((menu_ui_clkMo - 1U + 12U + dir) % 12U) + 1U);
            break;

        case 2U:   /* 日 */
            maxDay = App_RtcDaysInMonth(menu_ui_clkY, menu_ui_clkMo);
            menu_ui_clkD = (uint8_t)(((menu_ui_clkD - 1U + maxDay + dir) % maxDay) + 1U);
            break;

        case 3U:   /* 时 */
            menu_ui_clkH = (uint8_t)((menu_ui_clkH + 24 + dir) % 24);
            break;

        case 4U:   /* 分 */
            menu_ui_clkM = (uint8_t)((menu_ui_clkM + 60 + dir) % 60);
            break;

        default:   /* 秒 */
            menu_ui_clkS = (uint8_t)((menu_ui_clkS + 60 + dir) % 60);
            break;
    }

    /* 换月/换年后夹紧日：例如 01-31 切到 2 月应变为 28/29。 */
    maxDay = App_RtcDaysInMonth(menu_ui_clkY, menu_ui_clkMo);
    if (menu_ui_clkD > maxDay) {
        menu_ui_clkD = maxDay;
    }

    menu_ui_dirty = 1U;
}

/* 写入 RTC（日期+时间+自动星期）；成功后退出校时模式。 */
static void Menu_UiCommitClock(void)
{
    if (App_RtcSetDateTime(menu_ui_clkY, menu_ui_clkMo, menu_ui_clkD,
                           menu_ui_clkH, menu_ui_clkM, menu_ui_clkS) != 0U) {
        printf("[RTC] set %04u-%02u-%02u %02u:%02u:%02u OK\r\n",
               (unsigned)menu_ui_clkY, (unsigned)menu_ui_clkMo, (unsigned)menu_ui_clkD,
               (unsigned)menu_ui_clkH, (unsigned)menu_ui_clkM, (unsigned)menu_ui_clkS);
        menu_ui_clkMode = 0U;
    } else {
        printf("[RTC] set FAIL (DS3231)\r\n");   /* 留在校时模式，便于重试 */
    }

    menu_ui_dirty = 1U;
}

/* ================= P8：标定页 ================= */

/* 进入/退出标定页；进入时从 EEPROM 镜像装载现值（不改变任何运行状态）。 */
static void Menu_UiToggleCalibMode(void)
{
    const App_Store_Params_t *p;

    if (menu_ui_calMode == 0U) {
        if (menu_ui_clkMode != 0U) {
            return;   /* 校时页优先，避免两套按键语义叠加 */
        }

        p = App_StoreGet();
        if (p != NULL) {
            menu_ui_calPpr  = p->encoderPpr;
            menu_ui_calGear = p->gearRatioX100;
            menu_ui_calR25  = p->ntcR25;
            menu_ui_calB    = p->ntcB;
            menu_ui_calRup  = p->ntcRup;
            menu_ui_calOc   = p->heatOcRaw;
            menu_ui_calOffL = p->liquidOffsetC10;
            menu_ui_calOffB = p->boardOffsetC10;
        }
        menu_ui_calIdx  = 0U;
        menu_ui_calMode = 1U;
        printf("[P8] CALIB enter (K1 next, K2/K3 change, K4 save+exit)\r\n");
    } else {
        menu_ui_calMode = 0U;
        printf("[P8] CALIB exit without save\r\n");
    }

    menu_ui_dirty = 1U;
}

/* 当前标定项增减（范围/步长按硬件特性设定）。 */
static void Menu_UiAdjustCalib(int8_t dir)
{
    int32_t *v;
    int32_t step;
    int32_t lo;
    int32_t hi;

    switch (menu_ui_calIdx) {
        case 0U:  v = &menu_ui_calPpr;  step = 4;   lo = 4;     hi = 4096;   break;
        case 1U:  v = &menu_ui_calGear; step = 100; lo = 100;   hi = 10000;  break;
        case 2U:  v = &menu_ui_calR25;  step = 500; lo = 1000;  hi = 100000; break;
        case 3U:  v = &menu_ui_calB;    step = 50;  lo = 1000;  hi = 6000;   break;
        case 4U:  v = &menu_ui_calRup;  step = 500; lo = 500;   hi = 100000; break;
        case 5U:  v = &menu_ui_calOc;   step = 50;  lo = 500;   hi = 4000;   break;
        case 6U:  v = &menu_ui_calOffL; step = 1;   lo = -100;  hi = 100;    break;
        default:  v = &menu_ui_calOffB; step = 1;   lo = -100;  hi = 100;    break;
    }

    *v += (int32_t)dir * step;
    if (*v > hi) {
        *v = hi;
    }
    if (*v < lo) {
        *v = lo;
    }

    menu_ui_dirty = 1U;
}

/* 保存标定：写 EEPROM 镜像（静置 2s 落盘）+ 立即下发，无需重启。 */
static void Menu_UiCommitCalib(void)
{
    App_StoreSetCalib(menu_ui_calPpr, menu_ui_calGear, menu_ui_calOffL, menu_ui_calOffB);
    App_StoreSetNtc((uint16_t)menu_ui_calR25, (uint16_t)menu_ui_calB, (uint16_t)menu_ui_calRup);
    App_StoreSetHeatOc((uint16_t)menu_ui_calOc);
    App_StoreRequestSave();

    App_MotorSetEncoderPpr(menu_ui_calPpr);
    App_TempSetNtcParams((uint16_t)menu_ui_calR25, (uint16_t)menu_ui_calB, (uint16_t)menu_ui_calRup);
    App_TempSetOffsetC10(APP_TEMP_SRC_LIQUID, (int16_t)menu_ui_calOffL);
    App_TempSetOffsetC10(APP_TEMP_SRC_BOARD,  (int16_t)menu_ui_calOffB);
    App_HeatSetOverCurrentThreshold((uint16_t)menu_ui_calOc);

    printf("[P8] CALIB saved: ppr=%ld gearX100=%ld R25=%ld B=%ld Rup=%ld OC=%ld offL=%ld offB=%ld\r\n",
           (long)menu_ui_calPpr, (long)menu_ui_calGear,
           (long)menu_ui_calR25, (long)menu_ui_calB, (long)menu_ui_calRup,
           (long)menu_ui_calOc, (long)menu_ui_calOffL, (long)menu_ui_calOffB);

    menu_ui_calMode = 0U;
    menu_ui_dirty = 1U;
}

/* 标定页显示：标题 + 项名 + 值 + 操作提示 + 电机状态。 */
static void Menu_UiRenderCalib(void)
{
    static const char *const nameTxt[MENU_UI_CAL_ITEMS] = {
        "Encoder PPR", "Gear x100", "NTC R25 ohm", "NTC B (K)",
        "NTC Rup ohm", "Heat OC raw", "Liquid off", "Board off"
    };
    char line[40];
    char val[24];

    switch (menu_ui_calIdx) {
        case 0U:
            (void)snprintf(val, sizeof(val), "%ld ppr", (long)menu_ui_calPpr);
            break;
        case 1U:
            (void)snprintf(val, sizeof(val), "%ld = x%ld.%02ld",
                           (long)menu_ui_calGear, (long)(menu_ui_calGear / 100),
                           (long)(menu_ui_calGear % 100));
            break;
        case 2U:
            (void)snprintf(val, sizeof(val), "%ld ohm", (long)menu_ui_calR25);
            break;
        case 3U:
            (void)snprintf(val, sizeof(val), "%ld K", (long)menu_ui_calB);
            break;
        case 4U:
            (void)snprintf(val, sizeof(val), "%ld ohm", (long)menu_ui_calRup);
            break;
        case 5U:
            (void)snprintf(val, sizeof(val), "%ld raw", (long)menu_ui_calOc);
            break;
        case 6U:
            (void)snprintf(val, sizeof(val), "%ld (0.1C)", (long)menu_ui_calOffL);
            break;
        default:
            (void)snprintf(val, sizeof(val), "%ld (0.1C)", (long)menu_ui_calOffB);
            break;
    }

    SSD1315_ClearBuffer();

    (void)snprintf(line, sizeof(line), "CALIB %u/%u",
                   (unsigned)(menu_ui_calIdx + 1U), (unsigned)MENU_UI_CAL_ITEMS);
    SSD1315_DrawString(0U, 0U, line, SSD1315_COLOR_WHITE);
    SSD1315_DrawString(0U, MENU_UI_LINE_H, nameTxt[menu_ui_calIdx], SSD1315_COLOR_WHITE);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 2U, val, SSD1315_COLOR_WHITE);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 3U, "K1 next  K2+ K3-", SSD1315_COLOR_WHITE);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 4U, "K4 save+exit", SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "Motor  %s", (menu_ui_running != 0U) ? "Running" : "Idle");
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 5U, line, SSD1315_COLOR_WHITE);

    (void)SSD1315_Refresh();
}

/* 按键动作分发（edge 为本次产生的动作位图）。 */
static void Menu_UiHandleKeys(uint8_t edge)
{
    /* P6：有按键动作就刷新面板占用窗口，8s 内通信写命令被拒绝（异常码 06）。 */
    if (edge != 0U) {
        App_CommNotifyPanelKey();
    }

#if (MENU_UI_IMU_PAGE_ENABLE != 0U)
    /* P9：IMU 页只做显示，按键只保留 KEY4 退出（避免误改运行参数）。 */
    if (menu_ui_imuMode != 0U) {
        if ((edge & MENU_UI_KEY4_BIT) != 0U) {
            Menu_UiToggleImuMode();
        }
        return;
    }
#endif

    /* P8 标定页：按键语义整体替换（只改标定参数，不动运行参数）。 */
    if (menu_ui_calMode != 0U) {
        if ((edge & MENU_UI_KEY1_BIT) != 0U) {
            menu_ui_calIdx = (uint8_t)((menu_ui_calIdx + 1U) % MENU_UI_CAL_ITEMS);
            menu_ui_dirty = 1U;
        }
        if ((edge & MENU_UI_KEY2_BIT) != 0U) {
            Menu_UiAdjustCalib(1);
        }
        if ((edge & MENU_UI_KEY3_BIT) != 0U) {
            Menu_UiAdjustCalib(-1);
        }
        if ((edge & MENU_UI_KEY4_BIT) != 0U) {
            Menu_UiCommitCalib();
        }
        return;
    }

    /* 校时模式下按键语义整体替换，避免误改运行参数。 */
    if (menu_ui_clkMode != 0U) {
        if ((edge & MENU_UI_KEY1_BIT) != 0U) {
            menu_ui_clkField = (uint8_t)((menu_ui_clkField + 1U) % 6U);
            menu_ui_dirty = 1U;
        }
        if ((edge & MENU_UI_KEY2_BIT) != 0U) {
            Menu_UiAdjustClock(1);
        }
        if ((edge & MENU_UI_KEY3_BIT) != 0U) {
            Menu_UiAdjustClock(-1);
        }
        if ((edge & MENU_UI_KEY4_BIT) != 0U) {
            Menu_UiCommitClock();
        }
        return;
    }

    if ((edge & MENU_UI_KEY1_BIT) != 0U) {
        Menu_UiNextFocus();
        Menu_UiSyncState();
    }

    if ((edge & MENU_UI_KEY2_BIT) != 0U) {
        Menu_UiAdjustFocus(1);
    }

    if ((edge & MENU_UI_KEY3_BIT) != 0U) {
        Menu_UiAdjustFocus(-1);
    }

    if ((edge & MENU_UI_KEY4_BIT) != 0U) {
        Menu_UiToggleMotor();
        Menu_UiSyncState();
    }
}

/* 格式化字段：固定宽度并按是否处于编辑态加方括号。 */
static void Menu_UiFmtField(char *dst, uint16_t size, uint16_t value, uint8_t digits, uint8_t active)
{
    char tmp[8];

    if (digits == 4U) {
        (void)snprintf(tmp, sizeof(tmp), "%04u", (unsigned)value);
    } else {
        (void)snprintf(tmp, sizeof(tmp), "%02u", (unsigned)value);
    }

    if (active != 0U) {
        (void)snprintf(dst, size, "[%s]", tmp);
    } else {
        (void)snprintf(dst, size, "%s", tmp);
    }
}

/* 校时页：Date / Time 两行，[] 标出当前编辑字段。 */
static void Menu_UiRenderClock(void)
{
    char line[40];
    char y[10];
    char mo[6];
    char d[6];
    char h[6];
    char mi[6];
    char s[6];

    Menu_UiFmtField(y,  sizeof(y),  menu_ui_clkY,  4U, (menu_ui_clkField == 0U) ? 1U : 0U);
    Menu_UiFmtField(mo, sizeof(mo), menu_ui_clkMo, 2U, (menu_ui_clkField == 1U) ? 1U : 0U);
    Menu_UiFmtField(d,  sizeof(d),  menu_ui_clkD,  2U, (menu_ui_clkField == 2U) ? 1U : 0U);
    Menu_UiFmtField(h,  sizeof(h),  menu_ui_clkH,  2U, (menu_ui_clkField == 3U) ? 1U : 0U);
    Menu_UiFmtField(mi, sizeof(mi), menu_ui_clkM,  2U, (menu_ui_clkField == 4U) ? 1U : 0U);
    Menu_UiFmtField(s,  sizeof(s),  menu_ui_clkS,  2U, (menu_ui_clkField == 5U) ? 1U : 0U);

    SSD1315_ClearBuffer();

    SSD1315_DrawString(0U, 0U, "Set RTC", SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "Date %s-%s-%s", y, mo, d);
    SSD1315_DrawString(0U, MENU_UI_LINE_H, line, SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "Time %s:%s:%s", h, mi, s);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 2U, line, SSD1315_COLOR_WHITE);

    SSD1315_DrawString(0U, MENU_UI_LINE_H * 3U, "K1  next field", SSD1315_COLOR_WHITE);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 4U, "K2+ K3-  K4 save", SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "Motor  %s", (menu_ui_running != 0U) ? "Running" : "Idle");
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 5U, line, SSD1315_COLOR_WHITE);

    (void)SSD1315_Refresh();
}

#if (MENU_UI_IMU_PAGE_ENABLE != 0U)
/* P9：IMU 页进出（长按 KEY3）。与校时页/标定页互斥，避免两页叠加。 */
static void Menu_UiToggleImuMode(void)
{
    if ((menu_ui_calMode != 0U) || (menu_ui_clkMode != 0U)) {
        printf("[P9] IMU page: exit clock/calib page first\r\n");
        return;
    }

    menu_ui_imuMode = (menu_ui_imuMode == 0U) ? 1U : 0U;
    menu_ui_dirty = 1U;
    printf("[P9] IMU page %s\r\n", (menu_ui_imuMode != 0U) ? "on" : "off");
}

/* P9：IMU 页渲染（6 行）：
 *   IMU 6-axis STxxxx     状态位（bit0 有效 / bit6 已标定 / bit7 打滑 …）
 *   A  -13    0 1017      加速度 mg
 *   G    2   -1    0      角速度 0.1dps（已扣零偏）
 *   R   12 P   -4         横滚/俯仰 0.1°
 *   T20.3 V  12mg         片内温度 / 振动 RMS
 *   Rp2  120rpm S  3      斜坡状态/指令转速/打滑偏差 0.1%
 */
static void Menu_UiRenderImu(void)
{
    char line[24];
    App_ImuSample_t s;
    int16_t tAbs;

    SSD1315_ClearBuffer();

    if (App_ImuGetSample(&s) == 0U) {
        SSD1315_DrawString(0U, 0U, "IMU 6-axis", SSD1315_COLOR_WHITE);
        SSD1315_DrawString(0U, MENU_UI_LINE_H, "no data (absent)", SSD1315_COLOR_WHITE);
        SSD1315_DrawString(0U, MENU_UI_LINE_H * 4U, "K3 long: exit", SSD1315_COLOR_WHITE);
        (void)SSD1315_Refresh();
        return;
    }

    (void)snprintf(line, sizeof(line), "IMU 6-axis ST%04X", (unsigned)s.state);
    SSD1315_DrawString(0U, 0U, line, SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "A%5d%5d%5d", (int)s.accMg[0], (int)s.accMg[1], (int)s.accMg[2]);
    SSD1315_DrawString(0U, MENU_UI_LINE_H, line, SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "G%5d%5d%5d", (int)s.gyroDps10[0], (int)s.gyroDps10[1], (int)s.gyroDps10[2]);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 2U, line, SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "R%5d P%5d", (int)s.rollC10, (int)s.pitchC10);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 3U, line, SSD1315_COLOR_WHITE);

    tAbs = (s.tempC10 < 0) ? (int16_t)(-s.tempC10) : s.tempC10;
    (void)snprintf(line, sizeof(line), "T%s%2d.%d V%4umg", (s.tempC10 < 0) ? "-" : "",
                   (int)(tAbs / 10), (int)(tAbs % 10), (unsigned)s.vibRmsMg);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 4U, line, SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "Rp%u %4ldrpm S%3d", (unsigned)App_MotorGetRampState(),
                   (long)App_MotorGetCmdRpm(), (int)s.slipErrX10);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 5U, line, SSD1315_COLOR_WHITE);

    (void)SSD1315_Refresh();
}
#endif /* MENU_UI_IMU_PAGE_ENABLE */

/* 重绘一屏：清帧缓冲 -> 画 6 行 -> 提交（驱动只发送变化的页）。 */
static void Menu_UiRender(void)
{
    char line[40];
    char actSpeed[12];
    char actPlate[8];
    char actLiquid[8];
    char mark;
    int32_t curSpeed;
    int16_t tPlate;
    int16_t tLiquid;

    if (menu_ui_oledOk == 0U) {
        return;
    }

    /* P8 标定页使用独立页面。 */
    if (menu_ui_calMode != 0U) {
        Menu_UiRenderCalib();
        return;
    }

    /* 校时模式使用独立页面，跳过温度/转速采样。 */
    if (menu_ui_clkMode != 0U) {
        Menu_UiRenderClock();
        return;
    }

#if (MENU_UI_IMU_PAGE_ENABLE != 0U)
    /* P9：IMU 页（第 7 页，长按 KEY3 进入） */
    if (menu_ui_imuMode != 0U) {
        Menu_UiRenderImu();
        return;
    }
#endif

    /* 实际转速：CTRL 任务写入的全局状态；实际温度：直接读 NTC。 */
    App_Main_StateLock();
    curSpeed = app_main_state.cur_speed;
    App_Main_StateUnlock();

    tPlate = App_TempReadCelsius(APP_TEMP_SRC_BOARD);
    tLiquid = App_TempReadCelsius(APP_TEMP_SRC_LIQUID);

    if (tPlate == INT16_MIN) {
        (void)snprintf(actPlate, sizeof(actPlate), "--");
    } else {
        (void)snprintf(actPlate, sizeof(actPlate), "%ld", (long)(tPlate / 10));
    }

    if (tLiquid == INT16_MIN) {
        (void)snprintf(actLiquid, sizeof(actLiquid), "--");
    } else {
        (void)snprintf(actLiquid, sizeof(actLiquid), "%ld", (long)(tLiquid / 10));
    }

    (void)snprintf(actSpeed, sizeof(actSpeed), "%ld", (long)curSpeed);

    SSD1315_ClearBuffer();

    SSD1315_DrawString(0U, 0U, "TempStir V1.0", SSD1315_COLOR_WHITE);

    mark = (menu_ui_focus == APP_UI_FOCUS_SPEED) ? '>' : ' ';
    (void)snprintf(line, sizeof(line), "%cSpeed  %ld/%s rpm", mark, (long)menu_ui_speed, actSpeed);
    SSD1315_DrawString(0U, MENU_UI_LINE_H, line, SSD1315_COLOR_WHITE);

    mark = (menu_ui_focus == APP_UI_FOCUS_PLATE) ? '>' : ' ';
    (void)snprintf(line, sizeof(line), "%cPlate  %ld/%s C", mark, (long)menu_ui_plate, actPlate);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 2U, line, SSD1315_COLOR_WHITE);

    mark = (menu_ui_focus == APP_UI_FOCUS_TEMP) ? '>' : ' ';
    (void)snprintf(line, sizeof(line), "%cLiquid %ld/%s C", mark, (long)menu_ui_liquid, actLiquid);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 3U, line, SSD1315_COLOR_WHITE);

    mark = (menu_ui_focus == APP_UI_FOCUS_TIME) ? '>' : ' ';
    (void)snprintf(line, sizeof(line), "%cTime   %lu/%lu s", mark, (unsigned long)menu_ui_time, (unsigned long)menu_ui_remainSec);
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 4U, line, SSD1315_COLOR_WHITE);

    (void)snprintf(line, sizeof(line), "Motor  %s", (menu_ui_running != 0U) ? "Running" : "Idle");
    SSD1315_DrawString(0U, MENU_UI_LINE_H * 5U, line, SSD1315_COLOR_WHITE);

    (void)SSD1315_Refresh();
}

/* ---- 串口校时命令（USART1，轮询接收，不占用中断/回调） ----
 * 命令格式（回车或换行结尾）：
 *   time 2026-11-03 12:34:56      -> 设置 RTC 时间
 *   time                          -> 回读当前 RTC 时间
 * 说明：直接读 DR 寄存器取字符，不经过 HAL 状态机，避免与 printf 的
 * 阻塞发送互相干扰（TX/RX 通路独立）。 */
#define MENU_UI_CMD_BUF_LEN 32U
static char menu_ui_cmdBuf[MENU_UI_CMD_BUF_LEN];
static uint8_t menu_ui_cmdLen = 0U;

static void Menu_UiHandleCmd(const char *cmd)
{
    DS3231_Time_t t;

    if (strncmp(cmd, "time", 4U) != 0) {
        return;   /* 非本模块命令：静默忽略，避免干扰其他串口数据 */
    }

    if ((cmd[4] == '\0') || (cmd[4] == ' ')) {
        if (cmd[4] == '\0') {
            /* 只读回显 */
            if (App_RtcGetTime(&t) != 0U) {
                printf("[CMD] RTC %04u-%02u-%02u %02u:%02u:%02u\r\n",
                       (unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
                       (unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second);
            } else {
                printf("[CMD] RTC not ready\r\n");
            }
            return;
        }

        if (App_RtcSetTimeFromString(&cmd[5]) != 0U) {
            printf("[CMD] RTC time set OK\r\n");
        } else {
            printf("[CMD] RTC set FAIL (format: time YYYY-MM-DD HH:MM:SS)\r\n");
        }
    }
}

/* 每 40ms 调一次：把 USART1 收到的字符攒成一行，遇到回车/换行执行。 */
static void Menu_UiPollSerialCmd(void)
{
    uint8_t ch;

    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
        ch = (uint8_t)(huart1.Instance->DR & 0xFFU);   /* 读 DR 同时清 RXNE/ORE */

        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
            if (menu_ui_cmdLen > 0U) {
                menu_ui_cmdBuf[menu_ui_cmdLen] = '\0';
                Menu_UiHandleCmd(menu_ui_cmdBuf);
                menu_ui_cmdLen = 0U;
            }
        } else if (menu_ui_cmdLen < (MENU_UI_CMD_BUF_LEN - 1U)) {
            menu_ui_cmdBuf[menu_ui_cmdLen++] = (char)ch;
        } else {
            menu_ui_cmdLen = 0U;   /* 超长：丢弃整行 */
        }
    }
}

void Menu_UiInit(void)
{
    /* OLED 初始化（内部会探测器件、下发初始化序列并清屏）。失败不阻塞系统，
     * 仅置标志，UI 任务继续跑按键逻辑但不刷屏。 */
    menu_ui_oledOk = (SSD1315_Init() == HAL_OK) ? 1U : 0U;

    menu_ui_speed = MENU_UI_SPEED_DEFAULT;
    menu_ui_plate = MENU_UI_PLATE_DEFAULT;
    menu_ui_liquid = MENU_UI_LIQUID_DEFAULT;
    menu_ui_time = MENU_UI_TIME_DEFAULT;
    menu_ui_focus = APP_UI_FOCUS_SPEED;
    menu_ui_running = 0U; /* 上电默认停止，由 KEY4 启动 */
    menu_ui_startTick = 0U;
    menu_ui_remainSec = MENU_UI_TIME_DEFAULT;
    Int_KeyInit();          /* 按键事件模块：清消抖计数与长按状态 */
    menu_ui_dirty = 1U;
    menu_ui_clkMode = 0U;   /* 上电不进校时页 */
    menu_ui_clkField = 0U;
    menu_ui_clkY = 2000U;
    menu_ui_clkMo = 1U;
    menu_ui_clkD = 1U;
    menu_ui_clkH = 0U;
    menu_ui_clkM = 0U;
    menu_ui_clkS = 0U;
    menu_ui_k1Long = 0U;
    menu_ui_calMode = 0U;   /* 上电不进标定页 */
    menu_ui_calIdx  = 0U;
    menu_ui_k4Long  = 0U;
#if (MENU_UI_IMU_PAGE_ENABLE != 0U)
    menu_ui_imuMode = 0U;   /* 上电不进 IMU 页 */
    menu_ui_k3Long  = 0U;
#endif
    menu_ui_sqwTick = 0U;
    menu_ui_sqwWarned = 0U;

    /* 从 EEPROM 恢复上次保存的参数（无有效数据时保持上面的默认值）。 */
    Menu_UiLoadParams();
    menu_ui_remainSec = menu_ui_time; /* 未运行时剩余时间 = 设定时间 */

    /* 电机上电一律停止，需按 KEY4 启动；温度类设定按恢复值下发。 */
    App_MotorSetTargetRpm(0);
    App_HeatSetTargetTemp((int16_t)(menu_ui_liquid * 10));
    App_HeatSetPlateLimitTemp((int16_t)(menu_ui_plate * 10));

    Menu_UiSyncState();
    Menu_UiRender();
}

void Menu_UiTask(void *argument)
{
    uint8_t edge;
    uint32_t div = 0U;
    Int_KeyEvent_t keyEv;   /* int_key 每拍产出的按键事件（press/click/longOn/repeat/release） */

    (void)argument;

    /* DS_INT（DS3231 1Hz 秒节拍）挂到本任务：收到通知立即重绘，
     * 让显示刷新与真实秒对齐（错过也最多延后一个 40ms 周期）。 */
    App_Main_AttachNotifyTask(xTaskGetCurrentTaskHandle());
    menu_ui_sqwTick = (uint32_t)xTaskGetTickCount();   /* 自检起点 */

    for (;;) {
        /* 秒节拍通知：置脏标记（重绘走统一出口），并记录最近一次中断时刻。 */
        if (ulTaskNotifyTake(pdTRUE, 0) != 0U) {
            menu_ui_dirty = 1U;
            menu_ui_sqwTick = (uint32_t)xTaskGetTickCount();
            menu_ui_sqwWarned = 0U;
        }

        /* DS_INT 自检：1Hz 方波已使能却超过 5s 无中断 -> 判定中断链路不可用。
         * 只打印一次；刷新继续靠 40ms 轮询兜底，功能不受影响。 */
        if ((App_RtcIsSqwEnabled() != 0U) && (menu_ui_sqwWarned == 0U)) {
            if (((uint32_t)xTaskGetTickCount() - menu_ui_sqwTick) > pdMS_TO_TICKS(MENU_UI_SQW_TIMEOUT_MS)) {
                menu_ui_sqwWarned = 1U;
                printf("[P5] DS_INT no edge in 5s (sqw on), polling fallback\r\n");
            }
        }

        /* 串口校时命令（USART1 轮询；RX 未接线时自然无效，UI 校时不受影响）。 */
        Menu_UiPollSerialCmd();

        /* P7：液温达到目标 → 加热已在 app_heat 里关断，这里只负责提示：
         * 蜂鸣器长鸣 1s（软件定时器非阻塞，不卡 UI 刷新）。 */
        if (App_HeatTakeDoneEvent() != 0U) {
            Buzzer_ShortBeep(1000U);
        }

        /* P7：看门狗心跳（UI 任务 40ms 一轮） */
        App_FaultHeartbeat(APP_HB_UI);

        /* 1) 按键：采样/消抖/长按/连发全部由 int_key 模块完成，
         *    这里只把事件翻译成"业务动作位图" edge。 */
        Int_KeyScan(&keyEv);
        edge = 0U;

        /* 1a) 短按（按下即触发）：KEY1 切焦点、KEY2/KEY3 增减 */
        if ((keyEv.press & MENU_UI_KEY1_BIT) != 0U) {
            edge |= MENU_UI_KEY1_BIT;
        }
        if ((keyEv.press & MENU_UI_KEY2_BIT) != 0U) {
            edge |= MENU_UI_KEY2_BIT;
        }
        if ((keyEv.press & MENU_UI_KEY3_BIT) != 0U) {
            edge |= MENU_UI_KEY3_BIT;
        }

        /* 1b) 长按：KEY1 = 校时页、KEY4 = 标定页；长按成立后该键停止连发，松手恢复。 */
        if ((keyEv.longOn & MENU_UI_KEY1_BIT) != 0U) {
            menu_ui_k1Long = 1U;
            Menu_UiToggleClockMode();
        }
        if ((keyEv.longOn & MENU_UI_KEY4_BIT) != 0U) {
            menu_ui_k4Long = 1U;
            Menu_UiToggleCalibMode();
        }
        if ((keyEv.release & MENU_UI_KEY1_BIT) != 0U) {
            menu_ui_k1Long = 0U;
        }
        if ((keyEv.release & MENU_UI_KEY4_BIT) != 0U) {
            menu_ui_k4Long = 0U;
        }

        /* 1c) 连发：KEY2 恒连发；KEY1/KEY4 被页面占用时长按连发屏蔽 */
        if ((keyEv.repeat & MENU_UI_KEY2_BIT) != 0U) {
            edge |= MENU_UI_KEY2_BIT;
        }
        if (((keyEv.repeat & MENU_UI_KEY1_BIT) != 0U) && (menu_ui_k1Long == 0U)) {
            edge |= MENU_UI_KEY1_BIT;
        }
        if (((keyEv.repeat & MENU_UI_KEY4_BIT) != 0U) && (menu_ui_k4Long == 0U)) {
            edge |= MENU_UI_KEY4_BIT;
        }

#if (MENU_UI_IMU_PAGE_ENABLE != 0U)
        /* 1d) KEY3 长按 = IMU 页（当前停用，见 MENU_UI_IMU_PAGE_ENABLE 说明） */
        if ((keyEv.longOn & MENU_UI_KEY3_BIT) != 0U) {
            menu_ui_k3Long = 1U;
            Menu_UiToggleImuMode();
        }
        if ((keyEv.release & MENU_UI_KEY3_BIT) != 0U) {
            menu_ui_k3Long = 0U;
        }
        if (((keyEv.repeat & MENU_UI_KEY3_BIT) != 0U) && (menu_ui_k3Long == 0U)) {
            edge |= MENU_UI_KEY3_BIT;
        }
#else
        /* 1d) KEY3 长按不再占用页面切换 -> 正常连发减值（默认走这条） */
        if ((keyEv.repeat & MENU_UI_KEY3_BIT) != 0U) {
            edge |= MENU_UI_KEY3_BIT;
        }
#endif

        /* 1e) KEY4 短按（点击）= 启动/暂停（标定页保存、校时页确认）：松手瞬间生效 */
        if ((keyEv.click & MENU_UI_KEY4_BIT) != 0U) {
            edge |= MENU_UI_KEY4_BIT;
        }

        if (edge != 0U) {
            Menu_UiHandleKeys(edge);
            menu_ui_dirty = 1U;
        }

        /* 定时倒计时：归零则自动停机（内部会置 dirty 触发刷新）。 */
        Menu_UiTickTimer();

        /* 参数防抖落盘：停止调整 2s 后写 EEPROM。 */
        Menu_UiFlushSave();

        div++;
        if ((div >= MENU_UI_REFRESH_DIV) || (menu_ui_dirty != 0U)) {
            div = 0U;
            menu_ui_dirty = 0U;
            Menu_UiRender();
        }

        vTaskDelay(pdMS_TO_TICKS(MENU_UI_PERIOD_MS));
    }
}
