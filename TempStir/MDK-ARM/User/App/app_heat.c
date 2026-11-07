/*
 * app_heat.c
 * 加热执行器控制：封装 int_tim 的 PWM 设置，提供占空比控制、电流读取、
 * 故障/过流检测与紧急关断。不直接碰 TIM 寄存器，全部经 BSP 层。
 */
#include "app_heat.h"
#include "int_tim.h"
#include "int_gpio.h"
#include "int_adc.h"
#include "app_temp.h"     /* 温控闭环：读液体温度 */
#include "utils_pid.h"      /* 通用 PID */

#include <stdio.h>        /* 非法限温配置告警打印 */

/* 温控 PID 参数（决策 5：无经验初值，以下为占位，待 P3 实测整定）。 */
#define APP_HEAT_PID_KP   2.0f
#define APP_HEAT_PID_KI   0.05f
#define APP_HEAT_PID_KD   0.0f
#define APP_HEAT_PID_DT   0.6f    /* 控制周期(s)：200ms 循环 × seq%3 → 实际每 600ms 调一次 */

/* 铝板限温回差：触到上限后，必须降到「上限 - 2.0℃」才允许恢复加热。
 * 没有回差时板温贴着限值会以 600ms 控制节奏断续开关加热（PWM 虽已改 20kHz
 * 听不见，但频繁通断会对加热组件与电源造成无谓冲击）。 */
#define APP_HEAT_PLATE_LIMIT_HYST_C10   20

static uint8_t s_pct = 0U;
/* P8：加热过流判定阈值（ADC raw），可由面板/EEPROM 标定覆盖编译期默认值 */
static uint16_t s_ocThreshold = (uint16_t)APP_HEAT_OC_THRESHOLD;
static AppPid_t s_heatPid;   /* 液温闭环 PID（既有，不动） */
static AppPid_t s_platePid;  /* P10：板温目标 PID（独立实例，避免与液温积分互相污染） */
static uint8_t s_plateAsTarget = APP_HEAT_PLATE_TARGET_DEFAULT;
static uint16_t s_plateStuckMs = 0U;      /* 高占空比加热但板温不动的累计时间(ms) */
static uint8_t  s_plateStuckTripped = 0U; /* 1=已判定板温失控（板温不升）并锁存断电 */
static int16_t  s_targetC10 = 0;      /* 液体目标温度 0.1℃ */
static int16_t  s_plateLimitC10 = 0;  /* 铝板限温 0.1℃，0=不限制 */
static uint8_t  s_plateLimited = 0U;  /* 1=已因铝板限温关断，等待降温回差 */
static uint8_t  s_plateSensorBad = 0U;/* 1=板温 NTC 无效（采样失败/断线短路），已强制关加热 */

/* ---- P7：液温到温完成 ----
 * 只有当"曾低于目标（真的在加热）"之后温度达到目标，才判定完成，
 * 避免上电时液体本来就高于目标就误报。完成后锁存关加热（电机不受影响），
 * 直到修改目标温度或显式清除。 */
static uint8_t  s_heatingSeen = 0U;   /* 1=本次目标下发后曾低于目标（确实加热过） */
static uint8_t  s_doneLatched = 0U;   /* 1=已到温，加热结束并锁存 */
static uint8_t  s_doneEvent   = 0U;   /* 1=有一笔未取走的“完成”事件（供 UI 蜂鸣） */

/* 运行联锁：0=电机未启动，禁止加热；1=电机运行中，允许加热。 */
static uint8_t  s_runPermit = 0U;

/* ---- A3：液体加热「升温异常」判定（仅告警，不影响加热输出） ---- */
static uint32_t s_noRiseMs      = 0U;   /* 当前观察窗口已累计时间(ms)：120s 窗口，必须 32 位 */
static int16_t  s_noRiseBaseC10 = 0;    /* 窗口起点液温(0.1℃) */
static uint8_t  s_noRiseTripped = 0U;   /* 1=已判定升温异常（锁存） */
static uint8_t  s_noRiseEvent   = 0U;   /* 1=有一笔未取走的告警事件 */

void App_HeatInit(void)
{
    s_pct = 0U;
    s_targetC10 = 0;
    s_plateLimitC10 = 0;
    s_plateLimited = 0U;
    s_plateSensorBad = 0U;
    s_plateStuckMs = 0U;
    s_plateStuckTripped = 0U;
    s_heatingSeen = 0U;
    s_doneLatched = 0U;
    s_doneEvent   = 0U;
    s_runPermit   = 0U;
    s_noRiseMs      = 0U;
    s_noRiseBaseC10 = 0;
    s_noRiseTripped = 0U;
    s_noRiseEvent   = 0U;
    s_plateAsTarget = APP_HEAT_PLATE_TARGET_DEFAULT;
    App_PidInit(&s_heatPid, APP_HEAT_PID_KP, APP_HEAT_PID_KI, APP_HEAT_PID_KD,
                0.0f, 100.0f, 0.0f, 100.0f, APP_HEAT_PID_DT);
    App_PidInit(&s_platePid, APP_HEAT_PID_KP, APP_HEAT_PID_KI, APP_HEAT_PID_KD,
                0.0f, 100.0f, 0.0f, 100.0f, APP_HEAT_PID_DT);
    App_HeatEmergencyOff();   /* 上电默认关加热 */
}

void App_HeatSetPlateAsTarget(uint8_t enable)
{
    uint8_t e = (enable != 0U) ? 1U : 0U;

    if (e != s_plateAsTarget)
    {
        s_plateAsTarget = e;
        App_PidReset(&s_platePid);   /* 模式切换：清板温 PID 状态，避免残留积分 */
        App_HeatClearPlateStuck();
        printf("[P10] plate-as-target %s\r\n", (e != 0U) ? "on" : "off");
    }
}

void App_HeatClearPlateStuck(void)
{
    s_plateStuckMs      = 0U;
    s_plateStuckTripped = 0U;
}

uint8_t App_HeatGetPlateAsTarget(void)
{
    return s_plateAsTarget;
}

void App_HeatSetRunPermit(uint8_t permit)
{
    uint8_t p = (permit != 0U) ? 1U : 0U;

    if (p == s_runPermit)
    {
        return;
    }

    s_runPermit = p;
    App_HeatEmergencyOff();
    App_PidReset(&s_heatPid);   /* 停机期间不保留积分，避免重启瞬间冲功率 */

    if (p != 0U)
    {
        /* 电机启动：开始新一轮加热判定（清掉上一轮的到温锁存，允许重新加热） */
        App_HeatClearDone();
        App_HeatClearNoRise();       /* A3：重新启动电机 = 新一轮升温观察 */
        App_HeatClearPlateStuck();   /* 重新启停电机：给板温闭环一次重试机会 */
        printf("[P9] heat enabled (motor running)\r\n");
    }
    else
    {
        printf("[P9] heat disabled (motor stopped)\r\n");
    }
}

uint8_t App_HeatGetRunPermit(void)
{
    return s_runPermit;
}

void App_HeatSetPwmPercent(uint8_t pct)
{
    if (pct > APP_HEAT_PWM_MAX)
    {
        pct = APP_HEAT_PWM_MAX;
    }
    Int_TimPwmSetPercent(&htim2, TIM_CHANNEL_1, pct);
    s_pct = pct;
}

uint8_t App_HeatGetPwmPercent(void)
{
    return s_pct;
}

uint8_t App_HeatIsFault(void)
{
    return Int_GpioIsHeatFault();
}

int16_t App_HeatReadCurrentRaw(void)
{
    uint16_t raw = 0U;
    if (Int_AdcReadSingle(INT_ADC_CH_HEAT_CURRENT, &raw) != 0)
    {
        return -1;
    }
    return (int16_t)raw;
}

uint8_t App_HeatIsOverCurrent(void)
{
    uint16_t raw = 0U;
    if (Int_AdcReadSingle(INT_ADC_CH_HEAT_CURRENT, &raw) != 0)
    {
        return 0U;
    }
    return (raw > s_ocThreshold) ? 1U : 0U;
}

/* P8：过流阈值现场可标定（存 EEPROM）；非法值保持当前值。 */
void App_HeatSetOverCurrentThreshold(uint16_t raw)
{
    if ((raw >= 100U) && (raw <= 4095U))
    {
        s_ocThreshold = raw;
    }
}

uint16_t App_HeatGetOverCurrentThreshold(void)
{
    return s_ocThreshold;
}

void App_HeatEmergencyOff(void)
{
    Int_TimPwmSetPercent(&htim2, TIM_CHANNEL_1, 0U);
    s_pct = 0U;
}

void App_HeatSetTargetTemp(int16_t targetC10)
{
    if (targetC10 != s_targetC10)
    {
        /* 目标改变：重新开始一轮“到温判定”，清掉上一轮的完成锁存 */
        App_HeatClearDone();
        App_HeatClearNoRise();   /* A3：新目标 = 新会话，重新观察升温情况 */
    }
    s_targetC10 = targetC10;
}

uint8_t App_HeatTakeDoneEvent(void)
{
    uint8_t ev = s_doneEvent;

    s_doneEvent = 0U;
    return ev;
}

uint8_t App_HeatIsDone(void)
{
    return s_doneLatched;
}

void App_HeatClearDone(void)
{
    s_doneLatched = 0U;
    s_doneEvent   = 0U;
    s_heatingSeen = 0U;
}

int16_t App_HeatGetTargetTemp(void)
{
    return s_targetC10;
}

/* ---- A3：液体加热「升温异常」判定 ---- */

uint8_t App_HeatTakeNoRiseEvent(void)
{
    uint8_t ev = s_noRiseEvent;

    s_noRiseEvent = 0U;
    return ev;
}

uint8_t App_HeatIsNoRise(void)
{
    return s_noRiseTripped;
}

void App_HeatClearNoRise(void)
{
    s_noRiseMs      = 0U;
    s_noRiseBaseC10 = 0;
    s_noRiseTripped = 0U;
    s_noRiseEvent   = 0U;
}

/* 每 600ms（App_HeatControlStep 内）评估一次；只置告警标志，不改变加热输出。
 * 调用条件由调用点保证：正常加热路径、目标>0、占空比达门限、有加热许可。 */
static void App_HeatNoRiseStep(int16_t tempC10, uint8_t pct)
{
#if (APP_HEAT_NORISE_ENABLE != 0)
    uint8_t cond;

    cond = ((s_targetC10 > 0) && (pct >= APP_HEAT_NORISE_PCT_MIN) && (s_runPermit != 0U)) ? 1U : 0U;

    if (cond == 0U)
    {
        /* 条件消失（到温/停机/降功率）：窗口重新开始，不做判定 */
        s_noRiseMs      = 0U;
        s_noRiseBaseC10 = tempC10;
        return;
    }

    if (s_noRiseTripped != 0U)
    {
        return;   /* 已告警：本次加热会话内不重复判定 */
    }

    if (s_noRiseMs == 0U)
    {
        s_noRiseBaseC10 = tempC10;    /* 窗口起点 */
    }
    if (s_noRiseMs < (APP_HEAT_NORISE_WIN_MS + 600U))
    {
        s_noRiseMs += 600U;
    }

    if (s_noRiseMs >= APP_HEAT_NORISE_WIN_MS)
    {
        if ((int32_t)tempC10 - (int32_t)s_noRiseBaseC10 < (int32_t)APP_HEAT_NORISE_DELTA_C10)
        {
            s_noRiseTripped = 1U;
            s_noRiseEvent   = 1U;
            printf("[P11] liquid no-rise: %d.%dC in %us (pct=%u%%, base %d.%dC) -> warn only\r\n",
                   (int)(tempC10 / 10), (int)(tempC10 % 10),
                   (unsigned)(APP_HEAT_NORISE_WIN_MS / 1000U), (unsigned)pct,
                   (int)(s_noRiseBaseC10 / 10), (int)(s_noRiseBaseC10 % 10));
        }
        else
        {
            /* 有升温：窗口滑动，以前一拍温度为新基准 */
            s_noRiseBaseC10 = tempC10;
            s_noRiseMs      = 0U;
        }
    }
#else
    (void)tempC10;
    (void)pct;
#endif
}

void App_HeatPidSetGains(float kp, float ki, float kd)
{
    App_PidSetGains(&s_heatPid, kp, ki, kd);
    App_PidSetGains(&s_platePid, kp, ki, kd);   /* P10：板温闭环共用整定参数 */
}

void App_HeatSetPlateLimitTemp(int16_t limitC10)
{
    int16_t safe = App_HeatSanitizePlateLimit(limitC10);

    if (safe != limitC10)
    {
        printf("[P9] plate limit %d (0.1C) invalid -> no limit\r\n", (int)limitC10);
    }

    s_plateLimitC10 = safe;
    s_plateLimited = 0U;   /* 限值改变：清除限温锁存，按新门限重新判断 */
    App_HeatClearPlateStuck();   /* 设定改变：重新允许板温闭环尝试 */
}

int16_t App_HeatSanitizePlateLimit(int16_t limitC10)
{
    if (limitC10 <= 0)
    {
        return 0;   /* 0（或负值）：不限制 */
    }

    if (limitC10 < APP_HEAT_PLATE_LIMIT_MIN_C10)
    {
        /* 1~34.9℃ 无实际意义且会永久锁死加热，按不限制处理 */
        return 0;
    }

    return limitC10;
}

void App_HeatClearPlateLimit(void)
{
    s_plateLimited = 0U;
}

int16_t App_HeatGetPlateLimitTemp(void)
{
    return s_plateLimitC10;
}

uint8_t App_HeatIsPlateLimited(void)
{
    return s_plateLimited;
}

void App_HeatControlStep(void)
{
    int16_t tempC10;
    int16_t plateC10 = INT16_MIN;
    int16_t limitC10;
    uint8_t plateMode;
    float out;

    /* 故障/过流/传感器异常 -> 立即关加热（保护优先）。 */
    if ((App_HeatIsFault() != 0U) || (App_HeatIsOverCurrent() != 0U))
    {
        App_HeatEmergencyOff();
        return;
    }

    /* 运行联锁：电机未启动（目标转速=0）时一律不加热，防止静置状态下干烧铝板。 */
    if (s_runPermit == 0U)
    {
        App_HeatEmergencyOff();
        return;
    }

    /* P10：判断是否走“铝板目标加热”模式（液体目标 = 0 且铝板设定 > 0 时）。
     * 该模式下铝板设定值就是目标，硬限幅自动放宽 2.0℃ 以免与闭环打架；
     * 非该模式时 limitC10 与原来的 s_plateLimitC10 完全相同，既有行为不变。 */
    plateMode = ((s_plateAsTarget != 0U) && (s_targetC10 <= 0) && (s_plateLimitC10 > 0)) ? 1U : 0U;
    limitC10  = (plateMode != 0U) ? (int16_t)(s_plateLimitC10 + APP_HEAT_PLATE_TARGET_MARGIN_C10)
                                  : s_plateLimitC10;

    /* 铝板限温（带回差，0 = 不限制）：
     *   板温 ≥ 上限 → 锁存关断；板温 ≤ 上限-2.0℃ → 解除锁存，允许重新加热。
     * 锁存期间保持关断，避免板温贴上限时 600ms 节奏的断续通断。 */
    if (limitC10 > 0)
    {
        plateC10 = App_TempReadCelsius(APP_TEMP_SRC_BOARD);

        /* 板温采样失败（INT16_MIN）或 NTC 断线/短路：限温无从判断，
         * 按“失控风险”处理 —— 立即关加热（fail-safe），不允许在板温不可知时继续加热。
         * 这一条与 app_fault 的 NTC_PLATE 故障位互为备份，避免任何一条路径漏判。 */
        if ((plateC10 == INT16_MIN) || (App_TempIsBroken(APP_TEMP_SRC_BOARD) != 0U))
        {
            if (s_plateSensorBad == 0U)
            {
                s_plateSensorBad = 1U;
                printf("[P9] plate NTC invalid (raw=%u) -> heat off\r\n",
                       (unsigned)App_TempGetLastRaw(APP_TEMP_SRC_BOARD));
            }
            App_HeatEmergencyOff();
            return;
        }

        if (s_plateSensorBad != 0U)
        {
            s_plateSensorBad = 0U;
            printf("[P9] plate NTC back to normal\r\n");
        }

        if (s_plateLimited == 0U)
        {
            if (plateC10 >= limitC10)
            {
                s_plateLimited = 1U;
            }
        }
        else if (plateC10 <= (int16_t)(limitC10 - APP_HEAT_PLATE_LIMIT_HYST_C10))
        {
            s_plateLimited = 0U;
        }
        else
        {
            /* 处于回差带内：保持锁存 */
        }

        if (s_plateLimited != 0U)
        {
            App_HeatEmergencyOff();
            return;
        }
    }
    else
    {
        s_plateLimited = 0U;   /* 不限温时不保留锁存 */
    }

    /* P10：铝板目标加热 —— 独立分支、独立 PID，不与下面的液温闭环交叉。
     * 只设了铝板温度（如 40℃）时按板温闭环加热；不产生"到温完成"事件、
     * 不读写液温目标状态，液温模式的一切行为保持原样。 */
    if (plateMode != 0U)
    {
        if (plateC10 == INT16_MIN)
        {
            App_HeatEmergencyOff();
            return;
        }

        /* 板温闭环失控保护（仅本模式）：高占空比(≥50%)持续加热、板温原始值却
         * 30s 一动不动 → 说明板温传感与铝板脱开/通道卡死，闭环必然一路满功率烘烤。
         * 判定后锁存断电，需改铝板设定或重新启停电机才复位（见 App_HeatClearPlateStuck）。 */
        if ((s_pct >= 50U) && (App_TempIsRawStuck(APP_TEMP_SRC_BOARD) != 0U))
        {
            if (s_plateStuckMs < 60000U)
            {
                s_plateStuckMs = (uint16_t)(s_plateStuckMs + 600U);   /* 控制周期 600ms */
            }
        }
        else if (App_TempIsRawStuck(APP_TEMP_SRC_BOARD) == 0U)
        {
            s_plateStuckMs = 0U;   /* 板温在变：闭环有效，清零 */
        }
        else
        {
            /* 高占空比但不满足累计条件：保持 */
        }

        if (s_plateStuckMs >= 30000U)
        {
            if (s_plateStuckTripped == 0U)
            {
                s_plateStuckTripped = 1U;
                printf("[P10] plate temp stuck while heating %u%% -> heat off (check plate NTC)\r\n",
                       (unsigned)s_pct);
            }
            App_HeatEmergencyOff();
            return;
        }

        if (s_plateStuckTripped != 0U)
        {
            App_HeatEmergencyOff();
            return;
        }

        /* 起步加速：离目标还差 ≥5.0℃ 时直接满功率加热（用户要"启动电机后立即加热"），
         * 同时不累积积分，避免接近目标时冲过头。 */
        if ((int16_t)(s_plateLimitC10 - plateC10) >= 50)
        {
            s_platePid.integral = 0.0f;
            App_HeatSetPwmPercent(APP_HEAT_PWM_MAX);
            return;
        }

        out = App_PidCompute(&s_platePid, (float)s_plateLimitC10 / 10.0f, (float)plateC10 / 10.0f);
        if (out < 0.0f)
        {
            out = 0.0f;
        }
        App_HeatSetPwmPercent((uint8_t)(out + 0.5f));
        return;
    }

    tempC10 = App_TempReadCelsius(APP_TEMP_SRC_LIQUID);
    if (tempC10 == INT16_MIN)
    {
        App_HeatEmergencyOff();
        return;
    }

    /* P7：液温到温 → 判定“加热完成”，关加热并锁存（电机继续转，由 UI 蜂鸣提示）。
     * 只有先低于目标（确实在加热）后越过目标才算完成，上电即高于目标不误报。 */
    if (s_targetC10 > 0)
    {
        if (tempC10 < s_targetC10)
        {
            s_heatingSeen = 1U;
        }
        else if ((s_heatingSeen != 0U) && (s_doneLatched == 0U))
        {
            s_doneLatched = 1U;
            s_doneEvent   = 1U;
            App_HeatEmergencyOff();
            printf("[P7] liquid %d.%d >= target %d.%d -> heating done\r\n",
                   (int)(tempC10 / 10), (int)(tempC10 % 10),
                   (int)(s_targetC10 / 10), (int)(s_targetC10 % 10));
            return;
        }

        if (s_doneLatched != 0U)
        {
            App_HeatEmergencyOff();   /* 完成后保持关加热，直到修改目标/显式清除 */
            return;
        }
    }

    out = App_PidCompute(&s_heatPid, (float)s_targetC10 / 10.0f, (float)tempC10 / 10.0f);
    if (out < 0.0f)
    {
        out = 0.0f;
    }
    App_HeatSetPwmPercent((uint8_t)(out + 0.5f));

    /* A3：液体升温异常判定（放在正常加热路径末尾，不干预上面的输出） */
    App_HeatNoRiseStep(tempC10, s_pct);
}
