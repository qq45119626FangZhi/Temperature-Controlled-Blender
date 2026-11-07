/*
 * @file    app_fault.c
 * @brief   P7：故障监控任务 + 心跳 + IWDG 看门狗守护。
 *
 * 实现要点：
 *  - IWDG 直接写寄存器（LSI≈40kHz → PR=/64(625Hz) → RLR=1000 → 1.6s），
 *    不引入 HAL IWDG 模块，也不改 Core/Src/main.c 与 .ioc；
 *  - 心跳用 TICK 计数判定，任一关键任务超过 APP_HB_TIMEOUT_MS 未上报即停止喂狗；
 *  - 故障字聚合与原 app_comm 内部实现等价，改成单一来源后由通信/UI 共用。
 */
#include "app_fault.h"
#include "app_main.h"
#include "app_heat.h"
#include "app_motor.h"   /* P7：堵转故障位 */
#include "app_temp.h"
#include "app_store.h"
#include "app_rtc.h"
#include "app_imu.h"     /* P9：IMU 通信故障位 */
#include "app_modbus.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"      /* IWDG 寄存器定义（stm32f1xx.h 经 main.h 引入） */

#include <stdio.h>

#define APP_HB_TIMEOUT_MS    500U   /* 任一任务超过该时间未报心跳 -> 判定卡死 */
#define APP_HB_GRACE_MS      3000U  /* 上电宽限：期间即使个别任务未报到也先喂狗 */
#define APP_FAULT_PERIOD_MS  50U
#define APP_WDG_PERIOD_MS    100U

/* IWDG 键值（RM0008）：0xAAAA 重装、0x5555 允许写 PR/RLR、0xCCCC 启动 */
#define APP_IWDG_KEY_RELOAD  0x0000AAAAU
#define APP_IWDG_KEY_ENABLE  0x0000CCCCU
#define APP_IWDG_KEY_WRITE   0x00005555U

static volatile uint32_t s_hbTick[APP_HB_COUNT];
static volatile uint8_t  s_hbSeen[APP_HB_COUNT];
static uint32_t s_bootTick;
static uint16_t s_lastBits;
static uint8_t  s_wdgWarned;
static uint8_t  s_iwdgStarted;

static const char *const s_hbName[APP_HB_COUNT] = { "CTRL", "UI", "FAULT", "IMU" };

/* ---------------- IWDG ---------------- */

static void App_FaultIwdgStart(void)
{
#if (APP_IWDG_ENABLE != 0)
    /* 调试器暂停内核时冻结 IWDG：否则单步/烧录下载时会被看门狗复位，
     * 表现为 "Flash Download failed" 或单步乱跳。 */
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    IWDG->KR = APP_IWDG_KEY_ENABLE;      /* 启动 IWDG（LSI 由硬件自动使能） */
    IWDG->KR = APP_IWDG_KEY_WRITE;       /* 解锁 PR/RLR */
    IWDG->PR  = 0x03U;                   /* LSI/64：约 40kHz/64 = 625Hz */
    IWDG->RLR = 1000U;                   /* 1000/625 ≈ 1.6s 超时 */
    while (IWDG->SR != 0U)
    {
        /* 等待 PR/RLR 同步到 LSI 域 */
    }
    IWDG->KR = APP_IWDG_KEY_RELOAD;

    printf("[P7] IWDG on (timeout ~%u ms)\r\n", (unsigned)APP_IWDG_TIMEOUT_MS);
#else
    printf("[P7] IWDG disabled (APP_IWDG_ENABLE=0)\r\n");
#endif
}

static void App_FaultIwdgFeed(void)
{
#if (APP_IWDG_ENABLE != 0)
    IWDG->KR = APP_IWDG_KEY_RELOAD;
#endif
}

/* ---------------- 对外接口 ---------------- */

/* 打印本次上电的复位原因（在 App_FaultInit 里最早调用，便于串口第一屏就能看到）。
 * 其中 IWDG=1 就是“看门狗把整机复位了”的直接证据。
 * 读完立即清标志，保证下一次复位的原因不会被旧标志污染。 */
static void App_FaultPrintResetCause(void)
{
    printf("[P7] reset cause: IWDG=%u WWDG=%u SW=%u POR=%u PIN=%u LPWR=%u\r\n",
           (unsigned)((__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) ? 1U : 0U),
           (unsigned)((__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) ? 1U : 0U),
           (unsigned)((__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)   != RESET) ? 1U : 0U),
           (unsigned)((__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)   != RESET) ? 1U : 0U),
           (unsigned)((__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)   != RESET) ? 1U : 0U),
           (unsigned)((__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)  != RESET) ? 1U : 0U));

    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void App_FaultInit(void)
{
    uint8_t i;

    for (i = 0U; i < (uint8_t)APP_HB_COUNT; i++)
    {
        s_hbTick[i] = 0U;
        s_hbSeen[i] = 0U;
    }

    App_FaultPrintResetCause();   /* P7：先报告上次复位原因（IWDG=1 即看门狗复位） */

    s_bootTick   = 0U;          /* 调度器未启动，xTaskGetTickCount() 恒 0 */
    s_lastBits   = 0U;
    s_wdgWarned  = 0U;
    s_iwdgStarted = 0U;

    /* 注意：IWDG 不在初始化阶段启动 —— App_Main_Init 里有 OLED/EEPROM/RTC
     * 探测与 printf，耗时可能超过 1.6s 超时窗口。改为在守护任务第一次运行时
     * 启动（此时调度器已在跑、心跳机制就绪）。 */
    printf("[P7] fault monitor init\r\n");
}

void App_FaultHeartbeat(App_HbSource_t src)
{
    if ((uint8_t)src < (uint8_t)APP_HB_COUNT)
    {
        s_hbSeen[src] = 1U;
        s_hbTick[src] = (uint32_t)xTaskGetTickCount();
    }
}

uint16_t App_FaultGetBits(void)
{
    uint16_t bits = 0U;

    if (App_HeatIsFault() != 0U)
    {
        bits |= APP_MB_FLT_HEAT_HW;
    }
    if (App_HeatIsOverCurrent() != 0U)
    {
        bits |= APP_MB_FLT_HEAT_OC;
    }
    if (App_TempIsBroken(APP_TEMP_SRC_LIQUID) != 0U)
    {
        bits |= APP_MB_FLT_NTC_LIQUID;
    }
    if (App_TempIsBroken(APP_TEMP_SRC_BOARD) != 0U)
    {
        bits |= APP_MB_FLT_NTC_PLATE;
    }
    if (App_HeatIsPlateLimited() != 0U)
    {
        bits |= APP_MB_FLT_PLATE_LIMIT;
    }
    if (App_MotorIsStalled() != 0U)
    {
        bits |= APP_MB_FLT_MOTOR_STALL;   /* P7：堵转（电机类故障，只拦启动，不拦加热判定） */
    }
    if (App_StoreIsReady() == 0U)
    {
        bits |= APP_MB_FLT_EEPROM;
    }
    if (App_RtcWasAutoInit() != 0U)
    {
        bits |= APP_MB_FLT_RTC_LOST;
    }
    if (App_ImuIsFault() != 0U)
    {
        bits |= APP_MB_FLT_IMU;   /* P9：IMU 通信异常（只报警，不拦电机/加热） */
    }

    return bits;
}

/* 故障字变化时打印一次（中文名便于现场判断，不逐周期刷屏） */
static void App_FaultPrintChange(uint16_t bits)
{
    static const struct
    {
        uint16_t    bit;
        const char *name;
    } table[] =
    {
        { APP_MB_FLT_HEAT_HW,     "heat-hw"    },
        { APP_MB_FLT_HEAT_OC,     "heat-oc"    },
        { APP_MB_FLT_NTC_LIQUID,  "ntc-liquid" },
        { APP_MB_FLT_NTC_PLATE,   "ntc-plate"  },
        { APP_MB_FLT_PLATE_LIMIT, "plate-limit"},
        { APP_MB_FLT_MOTOR_STALL, "motor-stall"},
        { APP_MB_FLT_EEPROM,      "eeprom"     },
        { APP_MB_FLT_RTC_LOST,    "rtc-lost"   },
        { APP_MB_FLT_IMU,         "imu"        }
    };
    uint8_t i;
    uint8_t first = 1U;

    printf("[P7] fault=0x%04X ", (unsigned)bits);
    for (i = 0U; i < (uint8_t)(sizeof(table) / sizeof(table[0])); i++)
    {
        if ((bits & table[i].bit) != 0U)
        {
            printf("%s%s", (first != 0U) ? "" : ",", table[i].name);
            first = 0U;
        }
    }
    if (first != 0U)
    {
        printf("none");
    }
    printf("\r\n");
}

void App_FaultTask(void *argument)
{
    uint16_t bits;

    (void)argument;

    for (;;)
    {
        bits = App_FaultGetBits();

        /* 故障字变化才打印（上电先打一次基线） */
        if ((bits != s_lastBits) || (s_lastBits == 0xFFFFU))
        {
            App_FaultPrintChange(bits);
            s_lastBits = bits;
        }

        /* 加热类故障：立刻关加热（比 600ms 温控周期更快）。
         * 电机不受牵连（联锁只在 app_comm 的启动命令里按电机类故障判断）。 */
        if ((bits & (uint16_t)(APP_MB_FLT_HEAT_HW | APP_MB_FLT_HEAT_OC |
                               APP_MB_FLT_NTC_LIQUID | APP_MB_FLT_NTC_PLATE |
                               APP_MB_FLT_PLATE_LIMIT)) != 0U)
        {
            App_HeatEmergencyOff();
        }

        App_FaultHeartbeat(APP_HB_FAULT);
        vTaskDelay(pdMS_TO_TICKS(APP_FAULT_PERIOD_MS));
    }
}

void App_FaultWatchdogTask(void *argument)
{
    uint32_t now;
    uint32_t diff;
    uint8_t  i;
    uint8_t  ok;

    (void)argument;

    /* 记录启动时刻，用于上电宽限窗口 */
    s_bootTick = (uint32_t)xTaskGetTickCount();

    /* 看门狗在本任务里启动：先喂一次，保证从启动到首次判定之间有足够时间 */
    if (s_iwdgStarted == 0U)
    {
        s_iwdgStarted = 1U;
        App_FaultIwdgStart();
        App_FaultIwdgFeed();
    }

    for (;;)
    {
        now  = (uint32_t)xTaskGetTickCount();
        ok   = 1U;

        for (i = 0U; i < (uint8_t)APP_HB_COUNT; i++)
        {
#if (APP_IMU_TASK_ENABLE == 0)
            /* IMU 任务未创建（开关关闭）时不参与心跳判定，否则会被误判卡死 */
            if (i == (uint8_t)APP_HB_IMU)
            {
                continue;
            }
#endif
            if (s_hbSeen[i] == 0U)
            {
                /* 从未上报：已过宽限窗口 -> 视为卡死（任务未创建/未启动） */
                if ((uint32_t)(now - s_bootTick) > pdMS_TO_TICKS(APP_HB_GRACE_MS))
                {
                    ok = 0U;
                }
                continue;
            }

            diff = (uint32_t)(now - s_hbTick[i]);
            if (diff > pdMS_TO_TICKS(APP_HB_TIMEOUT_MS))
            {
                ok = 0U;
                if (s_wdgWarned == 0U)
                {
                    s_wdgWarned = 1U;
                    printf("[P7] WDG: %s heartbeat lost (%lu ms) -> stop feeding\r\n",
                           s_hbName[i], (unsigned long)diff);
                }
            }
        }

        if (ok != 0U)
        {
            s_wdgWarned = 0U;
            App_FaultIwdgFeed();
        }

        vTaskDelay(pdMS_TO_TICKS(APP_WDG_PERIOD_MS));
    }
}
