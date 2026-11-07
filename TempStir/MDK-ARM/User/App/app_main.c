/*
 * @file    app_main.c
 * @brief   业务应用层（MDK-ARM/User/App）：全局状态、互斥锁与测试任务。
 *
 * 代码组织约定：
 *  - 所有函数的“声明”统一放在 App_Main_Init() 之上；
 *  - App_Main_Init() 作为入口，依次完成状态初始化、任务创建、启动调度器；
 *  - 所有函数的“实现”统一放在 App_Main_Init() 之下。
 */

#include "app_main.h"
#include "semphr.h"        /* xSemaphoreCreateMutex / xSemaphoreTake / Give */
#include "stm32f1xx_hal.h" /* HAL_GetUIDw0/1/2：读取 STM32 96bit 唯一 ID */
#include "dri_buzzer.h"    /* 蜂鸣器模块：Buzzer_Init / ShortBeep / LongBeep */
#include "app_fan.h"       /* 风扇模块：Fan_Init / Fan_Start / Fan_Stop */
#include "int_gpio.h"      /* BSP GPIO 封装：Int_GpioReadKeys / Int_GpioIsHeatFault */
#include "int_adc.h"       /* BSP ADC 封装：Int_AdcInit / Int_AdcReadAll */
#include "int_spi.h"       /* BSP SPI2 封装：Int_SpiInit / Int_SpiReadReg (LSM6DSM WHO_AM_I) */
#include "int_tim.h"       /* BSP TIM 封装：Int_TimInit / Int_TimEncGetCount / Int_TimPwmSetPercent */
#include "app_temp.h"      /* 温度采集：App_TempInit / App_TempReadCelsius */
#include "app_heat.h"      /* 加热控制：App_HeatInit / App_HeatSetPwmPercent / App_HeatIsFault */
#include "app_motor.h"     /* 电机控制：App_MotorInit / App_MotorSetSpeedPercent / App_MotorCalcRpm */
#include "menu_ui.h"       /* P4 UI：Menu_UiInit / Menu_UiTask（按键 + OLED 菜单） */
#include "app_store.h"     /* P5 存储：App_StoreInit（M24C02 参数掉电保存） */
#include "app_rtc.h"       /* P5 时间：App_RtcInit / App_RtcGetTime（DS3231） */
#include "app_comm.h"      /* P6 通信：通道初始化与 COMM/BT 任务 */
#include "app_fault.h"     /* P7 故障监控/心跳/IWDG 守护任务 */
#include "app_imu.h"       /* P9 六轴采集：App_ImuInit / App_ImuTask（100Hz 轮询） */
#include "app_tune.h"      /* A2 实测统计：App_TuneInit / App_TuneStep（默认不打印） */
#include <stdio.h>

/* 全局状态实例（先零初始化，随后由 App_Main_StateInit 填充）。 */
App_State_t app_main_state = {0};

/* 全局状态互斥锁：保护 app_main_state 的读写。 */
static SemaphoreHandle_t app_main_state_mutex = NULL;

/* P9 倾斜保护：蜂鸣器状态。
 *   app_main_tiltAlarmOn：1 = 报警中（独立标志，保证解除时一定执行 Buzzer_Stop）
 *   app_main_tiltBeepMs ：重触发计时，累计到 2000ms 重新起鸣，形成“长鸣”
 * 两者必须分开：若只用计数（==0 判断），恰好计数器归零那一拍解除就会漏掉 Stop，
 * 蜂鸣器会继续响到 2.5s 定时器到期（2026-11-06 实测日志发现）。 */
static uint8_t  app_main_tiltAlarmOn = 0U;
static uint32_t app_main_tiltBeepMs  = 0U;

/* DS_INT（DS3231 1Hz 秒节拍）通知的接收任务句柄。
 * 由 UI 任务在进入循环前用 App_Main_AttachNotifyTask() 挂接；
 * 未挂接时 App_Main_NotifyFromISR() 会跳过通知。 */
static TaskHandle_t app_main_task_handle = NULL;

/* ==================== 函数声明（全部在 App_Main_Init 之上） ==================== */

static uint32_t App_Main_CalcDeviceId(void);

/* 专门创建所有业务任务（优先级默认 APP_TASK_PRIO_5）。 */
static void App_Main_CreateTasks(void);

static void App_Main_ControlLoopTask(void *argument);  /* P3：PID 闭环（转速/温控）+ 风扇自动控制 */

/* P5：启动时通过串口报告存储/RTC 状态与当前时间（验证 I2C2 两个从机）。 */
static void App_Main_PrintBootInfo(void);

/* P5：把 EEPROM 中的控制参数（PID 增益、校准系数）下发到各模块。 */
static void App_Main_ApplyStoreParams(void);

/* 说明：App_Main_Init / App_Main_StateInit / App_Main_StateLock /
 * App_Main_StateUnlock / App_Main_NotifyFromISR 已在 app_main.h 中声明。 */

/* ==================== 程序入口：App_Main_Init ==================== */

void App_Main_Init(void)
{
    printf("App_Main_Init\n");
    App_Main_StateInit();   /* 状态初始化：设备 ID（UID 计算）+ 默认值 + 互斥锁 */
    Buzzer_Init();          /* 蜂鸣器初始化（静音：占空比 0） */
    Fan_Init();             /* 风扇初始化：默认停止（MOTOR_ON 拉低） */
    Int_AdcInit();          /* BSP ADC 接管为单通道轮询模式并校准 */
    Int_SpiInit();          /* BSP SPI2 封装初始化（CS 释放） */
    App_ImuInit();          /* P9：LSM6DSM 上电自检 + 配置（须在 Int_SpiInit 之后） */
    Int_TimInit();          /* BSP TIM：启动 PWM 输出与编码器接口 */
    App_TempInit();         /* P2 温度采集初始化（滤波缓冲） */
    App_HeatInit();         /* P2 加热默认关 */
    App_MotorInit();        /* P2 电机默认停、编码器计数复位 */
    App_StoreInit();        /* P5 存储：I2C2 互斥量 + 探测 EEPROM（须在 Menu_UiInit 之前） */
    App_RtcInit();          /* P5 时间：探测 DS3231（与 EEPROM 共用 I2C2，顺序无关） */
    Menu_UiInit();          /* P4 UI：OLED 初始化 + 从 EEPROM 恢复参数（电机默认停止） */
    App_Main_ApplyStoreParams(); /* P5：下发 PID 增益与校准系数（须在参数加载之后） */
    App_TuneInit();         /* A2：实测统计模块（默认不打印，等上位机写 0x003C 开） */
    App_CommInit();         /* P6：按 EEPROM 参数启动 RS485/Modbus 与蓝牙通道 */
    App_FaultInit();        /* P7：故障监控/心跳初始化（IWDG 在守护任务首次运行时启动） */
    App_Main_PrintBootInfo(); /* P5：串口报告存储/RTC 状态，便于验收 */
    App_Main_CreateTasks(); /* 专门用于创建任务 */
    vTaskStartScheduler();  /* 启动调度器 */
}

/* ==================== 函数实现（全部在 App_Main_Init 之下） ==================== */

/* 由 STM32 96bit UID 计算 32bit 设备 ID（确定性、降低碰撞）。 */
static uint32_t App_Main_CalcDeviceId(void)
{
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    uint32_t id;

    /* XOR 三段 UID 后再做一次 xorshift 混洗。 */
    id = uid0 ^ uid1 ^ uid2;
    id ^= (id << 13);
    id ^= (id >> 17);
    id ^= (id << 5);
    return id;
}

/* P5 启动自检：报告 EEPROM / DS3231 是否在线，并打印 RTC 当前时间。
 * 调度器未启动时 printf 为阻塞式，可直接在 App_Main_Init 中调用。 */
static void App_Main_PrintBootInfo(void)
{
    const App_Store_Params_t *p;
    DS3231_Time_t t;

    if (App_StoreIsReady() != 0U) {
        printf("[P5] EEPROM=OK\r\n");
    } else {
        printf("[P5] EEPROM=NG (params cannot be saved)\r\n");
    }

    if ((App_RtcIsReady() != 0U) && (App_RtcGetTime(&t) != 0U)) {
        /* valid=0 表示 OSF 未能清除（时间不可信）；sqw=1 表示 1Hz 秒节拍已使能。 */
        printf("[P5] DS3231=OK %04u-%02u-%02u %02u:%02u:%02u valid=%u sqw=%u\r\n",
               (unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
               (unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second,
               (unsigned)App_RtcIsValid(), (unsigned)App_RtcIsSqwEnabled());
        printf("[P5] RTC set: hold KEY1 on panel, or serial \"time YYYY-MM-DD HH:MM:SS\"\r\n");

        /* OSF=1 说明掉电期间时间已丢失（多为纽扣电池未装/失效），已用编译时刻兜底。 */
        if (App_RtcWasAutoInit() != 0U) {
            printf("[P5] RTC time lost (OSF=1) -> init from build time, check battery\r\n");
        }
    } else {
        printf("[P5] DS3231=NG (time unavailable)\r\n");
    }

    /* 参数与统计：用于确认 EEPROM 里读到的确实是上次保存的值。 */
    p = App_StoreGet();
    printf("[P5] PARAM speed=%ld plate=%ld liquid=%ld time=%lu\r\n",
           (long)p->speed, (long)p->plateTemp, (long)p->liquidTemp,
           (unsigned long)p->runTimeSec);
    printf("[P5] CALIB mPID=%ld/%ld/%ld hPID=%ld/%ld/%ld PPR=%ld gearX100=%ld\r\n",
           (long)p->motorPidKp, (long)p->motorPidKi, (long)p->motorPidKd,
           (long)p->heatPidKp, (long)p->heatPidKi, (long)p->heatPidKd,
           (long)p->encoderPpr, (long)p->gearRatioX100);
    printf("[P8] NTC R25=%u B=%u Rup=%u | heatOC=%u | offL=%ld offB=%ld\r\n",
           (unsigned)p->ntcR25, (unsigned)p->ntcB, (unsigned)p->ntcRup,
           (unsigned)p->heatOcRaw, (long)p->liquidOffsetC10, (long)p->boardOffsetC10);
    printf("[P5] STAT boot=%lu run=%lu s heat=%lu s start=%lu\r\n",
           (unsigned long)p->bootCnt, (unsigned long)p->totalRunSec,
           (unsigned long)p->heatRunSec, (unsigned long)p->motorStartCnt);
}

/* P5：把存储中的控制参数下发到各执行/采集模块。 */
static void App_Main_ApplyStoreParams(void)
{
    const App_Store_Params_t *p = App_StoreGet();

    /* PID 增益：×1000 定点 -> float（2.0 存 2000）。 */
    App_MotorPidSetGains(APP_STORE_MILLI_TO_PID(p->motorPidKp),
                         APP_STORE_MILLI_TO_PID(p->motorPidKi),
                         APP_STORE_MILLI_TO_PID(p->motorPidKd));
    App_HeatPidSetGains(APP_STORE_MILLI_TO_PID(p->heatPidKp),
                        APP_STORE_MILLI_TO_PID(p->heatPidKi),
                        APP_STORE_MILLI_TO_PID(p->heatPidKd));

    /* 校准系数：编码器每转计数 + 两路温度偏移（0.1℃）。 */
    App_MotorSetEncoderPpr(p->encoderPpr);
    App_TempSetOffsetC10(APP_TEMP_SRC_LIQUID, (int16_t)p->liquidOffsetC10);
    App_TempSetOffsetC10(APP_TEMP_SRC_BOARD, (int16_t)p->boardOffsetC10);

    /* P8：NTC 曲线参数（R25/B/上拉）与加热过流阈值（现场标定值）。 */
    App_TempSetNtcParams(p->ntcR25, p->ntcB, p->ntcRup);
    App_HeatSetOverCurrentThreshold(p->heatOcRaw);

    /* A8（M5）：振动辅助堵转的运行期开关按 EEPROM（IMU 参数块）恢复。
     * 默认 0 = 关闭，堵转判定时间仍是 3s，行为与旧版一致。 */
    App_MotorSetStallAssistEnable(App_ImuGetParams()->slipAssistStallEn);
}

void App_Main_StateInit(void)
{
    /* 默认值（速度 / 温度均为整数）。 */
    app_main_state.device_id = App_Main_CalcDeviceId();
    app_main_state.cur_speed = 0;
    app_main_state.target_speed = 0;
    app_main_state.cur_temp = 0;
    app_main_state.target_temp = 0;
    app_main_state.target_time = 0U;
    app_main_state.remain_time = 0U;
    app_main_state.ui_focus = APP_UI_FOCUS_SPEED;
    app_main_state.sys_state = APP_SYS_IDLE;

    /* 互斥锁必须在调度器启动前创建（App_Main_Init 在 vTaskStartScheduler 之前调用）。 */
    if (app_main_state_mutex == NULL) {
        app_main_state_mutex = xSemaphoreCreateMutex();
    }
}

/**
 * @brief 加锁访问全局状态
 */
void App_Main_StateLock(void)
{
    if (app_main_state_mutex != NULL) {
        (void)xSemaphoreTake(app_main_state_mutex, portMAX_DELAY);
    }
}

/**
 * @brief 解锁全局状态
 */
void App_Main_StateUnlock(void)
{
    if (app_main_state_mutex != NULL) {
        (void)xSemaphoreGive(app_main_state_mutex);
    }
}

/* 专门用于创建任务：集中创建所有业务任务，优先级默认 5。 */
static void App_Main_CreateTasks(void)
{
    (void)xTaskCreate(App_Main_ControlLoopTask, "CTRL", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_5, NULL); /* P3：PID 闭环 */
    /* P4 UI：优先级低于 CTRL，保证 OLED 的 I2C 阻塞刷新不会拖慢 200ms 控制周期；
     * 栈用 4X(512 字) 供 snprintf 格式化使用。 */
    (void)xTaskCreate(Menu_UiTask, "UI", APP_TASK_STACK_4X, NULL, APP_TASK_PRIO_3, NULL);

    /* P6 通信：按 EEPROM 的通道使能位创建，未启用的通道不建任务（省 RAM）。
     * 优先级 3 与 UI 同级、低于 CTRL(5)，保证 200ms 控制周期不被串口阻塞拖慢。 */
    if ((App_CommGetChannels() & APP_COMM_CH_RS485) != 0U) {
        (void)xTaskCreate(App_CommTask, "COMM", APP_TASK_STACK_4X, NULL, APP_TASK_PRIO_3, NULL);
    }
    if ((App_CommGetChannels() & APP_COMM_CH_BT) != 0U) {
        (void)xTaskCreate(App_BleTask, "BLE", APP_TASK_STACK_4X, NULL, APP_TASK_PRIO_3, NULL);
    }

    /* P7 故障监控：优先级 4（高于 UI/COMM，低于 CTRL），50ms 周期。
     * 看门狗守护任务用最低优先级 1：心跳齐全才喂狗，任一关键任务卡死即复位。 */
    (void)xTaskCreate(App_FaultTask, "FAULT", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_4, NULL);
    (void)xTaskCreate(App_FaultWatchdogTask, "WDG", APP_TASK_STACK_DEFAULT, NULL, APP_TASK_PRIO_1, NULL);

    /* P9 六轴采集：优先级 2（低于 UI/FAULT/CTRL，只读传感器，不能抢占控制环）；
     * 栈 4X 供 printf 格式化使用；未识别到器件时任务自身空转，不产生周期日志。 */
#if (APP_IMU_TASK_ENABLE != 0U)
    (void)xTaskCreate(App_ImuTask, "IMU", APP_TASK_STACK_4X, NULL, APP_TASK_PRIO_2, NULL);
#endif
}

/* P3 闭环验证任务：电机转速 PID 闭环（安全）；温控闭环代码就绪但目标=0（不加热，
 * 避免液体空载干烧，待确认有液体后再设目标温度）。 */
static void App_Main_ControlLoopTask(void *argument)
{
    (void)argument;
    uint32_t seq = 0U;
    int16_t tLiquid;
    int16_t tPlate;
    int32_t rpm;

    /* PID 增益与校准系数已由 App_Main_ApplyStoreParams() 按 EEPROM 内容下发，
     * 目标转速/温度由 P4 UI 菜单管理，此处不再覆盖任何设定值。 */

    for (;;) {
        App_FaultHeartbeat(APP_HB_CTRL);   /* P7：看门狗心跳 */
        App_MotorControlStep();

        /* 加热联锁：电机启动（目标转速≠0）后才允许加热，电机停止立即关加热；
         * P7：判定堵转时电机已停机，这里同步撤掉加热许可（避免不搅拌还继续加热）。 */
        App_HeatSetRunPermit(((App_MotorGetTargetRpm() != 0) && (App_MotorIsStalled() == 0U)) ? 1U : 0U);

        /* P9-3（M5）：把 IMU 的振动结论喂给堵转判定（仅当
         * APP_MOTOR_STALL_ASSIST_ENABLE=1 时才影响判定时间，默认关）。 */
        App_MotorSetStallAssist(App_ImuHasVibration());

        /* ---------------- P9 倾斜保护（需求） ----------------
         * 倾角 > 门限（默认 10.0°，可用 0x0036 改）且持续 500ms：
         *   1) 电机在运转 -> 立即停机（目标转速置 0，与面板 KEY4 停机等价）；
         *   2) 蜂鸣器长鸣（非阻塞，每 2s 重新起鸣；放平后自动停）。
         * 判定与执行分离：判定在 app_imu，执行在这里，便于单独开关/回退。 */
#if (APP_IMU_TILT_PROTECT_ENABLE != 0)
        if (App_ImuIsTiltBlocked() != 0U) {
            if (App_MotorGetTargetRpm() != 0) {
                Menu_UiCommSetRunning(0U);      /* 面板/上位机显示同步为“停止” */
                App_MotorSetTargetRpm(0);       /* 目标 0 -> 斜坡减速到停 + 撤加热许可 */
                printf("[P9] TILT protect -> motor stopped (target set 0)\r\n");
            }
            if (app_main_tiltAlarmOn == 0U) {
                app_main_tiltAlarmOn = 1U;
                app_main_tiltBeepMs  = 0U;
                Buzzer_ShortBeep(2500U);        /* 首次立刻起鸣（非阻塞长音） */
            }
            app_main_tiltBeepMs += 200U;
            if (app_main_tiltBeepMs >= 2000U) {
                app_main_tiltBeepMs = 0U;
                Buzzer_ShortBeep(2500U);        /* 每 2s 续鸣 = 连续长鸣 */
            }
        } else if (app_main_tiltAlarmOn != 0U) {
            app_main_tiltAlarmOn = 0U;
            app_main_tiltBeepMs  = 0U;
            Buzzer_Stop();                      /* 放平/解除 -> 立刻停鸣（无条件执行） */
            printf("[P9] TILT cleared -> alarm off\r\n");
        }
#endif
        if ((seq % 3U) == 0U) {
            App_HeatControlStep();
        }

        /* A3：液体升温异常 —— 仅告警（判定与打印在 app_heat 内），这里补一次蜂鸣；
         * 不停机、不关加热、不置故障位，既有加热流程完全不受影响。 */
        if (App_HeatTakeNoRiseEvent() != 0U) {
            Buzzer_ShortBeep(1500U);
        }

        tLiquid = App_TempReadCelsius(APP_TEMP_SRC_LIQUID);

        /* 液温“上穿 30.0℃”动作（边沿触发）：
         *   默认只长鸣 1s（软件定时器非阻塞）；是否同时停电机由
         *   APP_TEMP_MARK_STOP_MOTOR 决定（默认 0，避免停电机→加热联锁→断热）。
         * 由 ≥30℃ 回落到 30℃ 以下：不动电机、不鸣叫。 */
        if (App_TempMarkStep(tLiquid) != 0U) {
#if (APP_TEMP_MARK_STOP_MOTOR != 0)
            Menu_UiCommSetRunning(0U);
            printf("[P9] liquid up-cross %d.%dC -> motor stop + beep 1s\r\n",
                   (int)(tLiquid / 10), (int)(tLiquid % 10));
#else
            printf("[P9] liquid up-cross %d.%dC -> beep 1s\r\n",
                   (int)(tLiquid / 10), (int)(tLiquid % 10));
#endif
            Buzzer_ShortBeep(1000U);
        }

        rpm     = App_MotorCalcRpm();

        /* 风扇：铝板温度 ≥ 35.0℃ 开启，≤ 32.0℃ 关闭（回差 3℃，见 app_fan.h），
         * 两次切换至少间隔 5s。原 30.0℃ 单一门限过低，板子自热越过门限会反复启停。 */
        tPlate = App_TempReadCelsius(APP_TEMP_SRC_BOARD);
        Fan_AutoByPlateTemp(tPlate);

        /* 说明：本轮排查用的 [NTC] 每 5s 诊断打印已移除（保留 app_temp 的
         * App_TempGetLastRaw/App_TempIsRawStuck 接口，供 app_heat 的板温失效
         * fail-safe 与板温失控保护使用，不再产生周期性日志）。 */

        /* P5 运行统计：累计电机运行/加热时间（不足 1s 的部分由 app_store 保留）。 */
        App_StoreStatsTick(200U,
                           (App_MotorGetTargetRpm() != 0) ? 1U : 0U,
                           (App_HeatGetPwmPercent() > 0U) ? 1U : 0U);

        /* 统计量变化时每 5 分钟批量落盘一次（1500 × 200ms），避免频繁擦写。 */
        if ((seq % 1500U) == 0U) {
            if (App_StoreStatsDirty() != 0U) {
                App_StoreRequestSave();
            }
        }

        App_Main_StateLock();
        app_main_state.cur_speed = rpm;   /* 供 UI 显示“实际转速”，避免 UI 另起编码器采样 */
        // printf("[P3] seq=%lu Tliq=%d.%d Tplate=%d.%d FAN=%u RPM=%ld CNT=%d MOT_PWM=%u HEAT_PWM=%u TgtRPM=%ld\r\n",
        //        (unsigned long)seq,
        //        (int)(tLiquid / 10), (int)(tLiquid % 10),
        //        (int)(tPlate / 10), (int)(tPlate % 10),
        //        (unsigned)Fan_IsOn(),
        //        (long)rpm, (int)(htim3.Instance->CNT & 0xFFFFU),
        //        (unsigned)App_MotorGetSpeedPercent(),
        //        (unsigned)App_HeatGetPwmPercent(),
        //        (long)App_MotorGetTargetRpm());
        App_Main_StateUnlock();

        /* 说明：P3 验收用的 [P3E] 周期评估打印已按需求移除（不需要实时打印）。
         * 转速性能统计本身仍在 App_MotorControlStep() -> App_MotorPerfStep() 内累计，
         * 需要标定 PPR/减速比时可通过 App_MotorPerfGet* 接口另行读取，控制逻辑不受影响。 */

        /* A2：实测整定统计（默认关闭；上位机写 0x003C bit0=1 后每 5s 打一行 [TUNE]） */
        App_TuneStep(200U);

        seq++;
        /* 必须保持 200ms：与 APP_MOTOR_PID_DT(0.2) / APP_HEAT_PID_DT(0.6) 及
         * P3 统计窗口（1s 判定稳定、10s 求平均）全部对齐。 */
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* 挂接/解除 DS_INT 秒节拍通知的接收任务。 */
void App_Main_AttachNotifyTask(TaskHandle_t task)
{
    app_main_task_handle = task;
}

/* 供 EXTI15_10（DS_INT，抢占优先级 6）中断调用：仅做任务通知。 */
void App_Main_NotifyFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (app_main_task_handle != NULL) {
        vTaskNotifyGiveFromISR(app_main_task_handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
