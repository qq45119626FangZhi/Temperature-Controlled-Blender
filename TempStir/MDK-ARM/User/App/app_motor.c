/*
 * app_motor.c
 * 搅拌电机执行器控制：封装 int_tim 的 PWM 设置与编码器读取，按采样周期计算 RPM。
 * 不直接碰 TIM 寄存器，全部经 BSP 层。PID 闭环在 P3 实现。
 */
#include "app_motor.h"
#include "FreeRTOS.h"
#include "int_tim.h"
#include "utils_pid.h"  /* 通用 PID */
#include "int_bsp_pins.h" /* MOTOR2(PB9) 固定低：DRV8870 严禁拉高 */
#include "task.h"     /* xTaskGetTickCount：RPM 采样计时 */
#include <stdio.h>    /* 堵转告警打印 */

/* 转速 PID 参数（决策 5：占位，待 P3 实测整定）。 */
#define APP_MOTOR_PID_KP 0.05f
#define APP_MOTOR_PID_KI 0.02f
#define APP_MOTOR_PID_KD 0.0f
#define APP_MOTOR_PID_DT 0.2f
#define APP_MOTOR_SLEW_MAX 5.0f     /* 软启动：每周期占空比最大变化(%)，慢启动防冲击 */

/* P3 验收统计判据 */
#define APP_MOTOR_PERF_BAND_DIV 20       /* 稳态带 = 目标/20 = ±5% */
#define APP_MOTOR_PERF_SETTLE_N 5U       /* 连续 5 个周期（1s）在带内视为稳定 */
#define APP_MOTOR_PERF_AVG_N 50U         /* 稳态后累计 50 个周期（10s）求平均 */
#define APP_MOTOR_PERF_TIMEOUT_MS 60000U /* 60s 仍未进入稳态带则判未达标（占空比不足/堵转） */

/* 转速性能统计状态（P3 验收：稳态误差/超调/调节时间）。 */
typedef struct {
    int32_t target;    /* 本轮统计对应的目标 */
    int32_t maxRpm;    /* 观测峰值（含超调） */
    int32_t steadySum; /* 稳态采样累加 */
    int32_t steadyRpm; /* 稳态平均值 */
    uint32_t steadyN;  /* 稳态采样个数 */
    uint32_t inBandN;  /* 连续在带内周期数 */
    uint32_t t0;       /* 统计起始 tick */
    uint32_t riseMs;   /* 上升时间 */
    uint32_t settleMs; /* 调节时间 */
    uint8_t state;     /* 0=空闲 1=上升 2=稳态累计 3=已完成 */
    uint8_t timeout;   /* 1=超时未进入稳态带 */
} App_Motor_Perf_t;

static uint8_t s_pct = 0U;

/* 编码器每转计数（线数×4）：可由 EEPROM 校准值覆盖，默认取宏。 */
static int32_t s_encoderPpr = APP_MOTOR_ENCODER_PPR;

/* RPM 计算所需的上次采样状态（由 App_MotorCalcRpm 内部维护）。 */
static int32_t s_lastCount = 0;
static uint32_t s_lastTick = 0;
static uint32_t s_lastDelta = 0U; /* 上次采样计数增量绝对值 */
static uint32_t s_lastDtMs = 0U;  /* 上次采样时间间隔(ms) */

static AppPid_t s_speedPid;
static int32_t s_targetRpm = 0; /* 目标转速 */
static float s_lastOut = 0.0f;  /* 软启动：上次输出占空比 */
static int32_t s_lastRpm = 0;   /* 上次有效 RPM（同周期重复调用时复用） */
static App_Motor_Perf_t s_perf;

/* P7：堵转检测状态（独立，不影响调速/性能统计） */
static uint16_t s_stallMs = 0U;      /* 持续满足堵转判据的累计时间(ms) */
static uint8_t  s_stallLatched = 0U; /* 1=已判定堵转，PWM 保持 0 直到人工清除 */

/* P9-3（M1/M3）：速度斜坡与加减速耗时测量（独立于 PID 与堵转逻辑） */
static int32_t  s_cmdRpm      = 0;    /* 斜坡当前指令转速 */
static uint16_t s_rampState   = APP_MOTOR_RAMP_STOP;
static uint16_t s_rampAccRpmS = APP_MOTOR_RAMP_ACC_RPM_S;
static uint16_t s_rampDecRpmS = APP_MOTOR_RAMP_DEC_RPM_S;
static uint32_t s_rampTargetTick = 0U; /* M3：目标变更时刻，加速/减速耗时统一起点 */
static uint32_t s_accTimeMs   = 0U;    /* M3：加速实测耗时 */
static uint32_t s_decTimeMs   = 0U;    /* M3：减速实测耗时 */
#if (APP_MOTOR_STALL_ASSIST_ENABLE != 0)
static uint8_t  s_stallAssistVib = 0U; /* M5：本周期是否检测到振动 */
/* M5 运行期开关：默认取 APP_MOTOR_STALL_ASSIST_DEFAULT(0)，由 0x003B bit0 改 */
static uint8_t  s_stallAssistEn  = APP_MOTOR_STALL_ASSIST_DEFAULT;
#endif

static uint32_t App_MotorTickToMs(uint32_t delta) { return delta * 1000U / (uint32_t)configTICK_RATE_HZ; }

static void App_MotorPerfStep(int32_t rpm);
static void App_MotorStallStep(int32_t rpm);
static void App_MotorRampStep(void);

void App_MotorInit(void)
{
    /* PB9 固定低（DRV8870：拉高会反向/刹车）；gpio.c 已配低，这里再确保一次。 */
    HAL_GPIO_WritePin(MOTOR2_GPIO_Port, MOTOR2_Pin, MOTOR2_FIXED_STATE);

    s_pct = 0U;
    s_targetRpm = 0;
    s_lastOut = 0.0f;
    /* 输出/积分上限必须等于实际可执行上限(PWM_LIMIT)，否则 PID 以为能用到 100，
     * 积分会累积到 100 而抗饱和永不触发 → 积分饱和(windup)，表现为长期顶满占空比、
     * 超调大且降不下来。 */
    App_PidInit(&s_speedPid, APP_MOTOR_PID_KP, APP_MOTOR_PID_KI, APP_MOTOR_PID_KD,
                0.0f, (float)APP_MOTOR_PWM_LIMIT, 0.0f, (float)APP_MOTOR_PWM_LIMIT, APP_MOTOR_PID_DT);
    Int_TimEncReset();
    s_lastCount = 0;
    s_lastTick = 0U;
    s_lastDelta = 0U;
    s_lastDtMs = 0U;
    App_MotorPerfReset();
    s_stallMs = 0U;
    s_stallLatched = 0U;

    /* P9-3：斜坡复位（上电指令值 = 目标值 = 0） */
    s_cmdRpm      = 0;
    s_rampState   = APP_MOTOR_RAMP_STOP;
    s_rampTargetTick = 0U;
    s_accTimeMs   = 0U;
    s_decTimeMs   = 0U;
#if (APP_MOTOR_STALL_ASSIST_ENABLE != 0)
    s_stallAssistVib = 0U;
#endif
}

void App_MotorSetSpeedPercent(uint8_t pct)
{
    if (pct > APP_MOTOR_PWM_MAX) {
        pct = APP_MOTOR_PWM_MAX;
    }
    /* PB9 固定低已在 MX_GPIO_Init 中配置；此处只需设置 PB8 的 PWM 占空比。 */
    Int_TimPwmSetPercent(&htim4, TIM_CHANNEL_3, pct);
    s_pct = pct;
}

uint8_t App_MotorGetSpeedPercent(void) { return s_pct; }

int32_t App_MotorCalcRpm(void)
{
    int32_t cur = (int32_t)Int_TimEncGetCount();
    uint32_t now = (uint32_t)xTaskGetTickCount();
    int32_t delta;
    uint32_t dt_ms;

    if (s_lastTick == 0U) {
        /* 首次采样，仅记录基线。 */
        s_lastCount = cur;
        s_lastTick = now;
        return 0;
    }

    /* TIM3 是 16bit 计数器，直接相减在 0x7FFF/0x8000 跨界时会跳变 ±65536，
     * 造成约十几秒一次的转速突跳。先按 uint16 求差再解释为 int16，得到环绕安全增量。 */
    delta = (int32_t)(int16_t)(uint16_t)((uint16_t)cur - (uint16_t)s_lastCount);
    dt_ms = now - s_lastTick;
    if (dt_ms >= 50U) /* 至少 50ms 才计算，避免除零/抖动 */
    {
        /* 电机轴转速(RPM, 每分钟转数) = 计数差 / 每转脉冲 / 时间(s) * 60
         * 注意：不再除减速比。速度设定值(0~500)与闭环反馈必须同量纲，
         * 否则“设定 100 实际只有 3”会让 PID 误判并一路饱和。减速比仅用于
         * 需要输出轴转速时再换算（输出轴RPM = 电机轴RPM / GEAR_RATIO）。 */
        s_lastRpm = (int32_t)((int64_t)delta * 60000LL / ((int64_t)s_encoderPpr * (int64_t)dt_ms));
        /* 编码器计数方向与“电机正转”定义相反时，PID 会形成正反馈（目标为正、测量为负，
         * 输出一路顶到占空比上限，表现为“怎么调都没有变化”）。此处按宏统一取反。 */
#if (APP_MOTOR_RPM_INVERT != 0)
        s_lastRpm = -s_lastRpm;
#endif
        /* 保留原始计数率，供串口打印反推 PPR/减速比。 */
        s_lastDelta = (uint32_t)((delta >= 0) ? delta : -delta);
        s_lastDtMs = dt_ms;
        s_lastCount = cur;
        s_lastTick = now;
    }

    /* 采样间隔不足时返回上次有效值，避免同一周期内重复调用返回 0。 */
    return s_lastRpm;
}

void App_MotorEmergencyStop(void)
{
    Int_TimPwmSetPercent(&htim4, TIM_CHANNEL_3, 0U);
    s_pct = 0U;
    s_lastOut = 0.0f;
    App_PidReset(&s_speedPid); /* 清积分与上次测量，避免再启动时突冲 */
    s_cmdRpm    = 0;           /* P9-3：急停同时把斜坡指令清零 */
    s_rampState = APP_MOTOR_RAMP_STOP;
}

void App_MotorSetTargetRpm(int32_t targetRpm)
{
    if (s_targetRpm != targetRpm) {
        s_targetRpm = targetRpm;
        /* M3 计时口径修正（2026-11-06）：加减速耗时从“目标变更这一刻”开始算。
         * 原实现在进入 ACC/DEC 状态时取起点，因为斜坡按 200ms 一拍离散推进，
         * 首拍已经把一步走完，导致统计值固定少算一个控制周期（600rpm/s、200rpm 时
         * 显示 203ms，实际爬升 400ms）。 */
        s_rampTargetTick = (uint32_t)xTaskGetTickCount();
        App_MotorPerfReset(); /* 目标变更：重新开始统计 */
        if (targetRpm == 0) {
            App_MotorClearStall();   /* P7：停机即解除堵转锁存，允许重新启动再试 */
        }
#if (APP_MOTOR_RAMP_ENABLE == 0)
        s_cmdRpm = targetRpm;        /* 斜坡关闭：指令值直接等于目标值（旧行为） */
#endif
    }
}

int32_t App_MotorGetTargetRpm(void) { return s_targetRpm; }

void App_MotorPidSetGains(float kp, float ki, float kd) { App_PidSetGains(&s_speedPid, kp, ki, kd); }

/* 编码器每转计数标定：ppr 必须 > 0，否则忽略（防止除零）。 */
void App_MotorSetEncoderPpr(int32_t ppr)
{
    if (ppr > 0) {
        s_encoderPpr = ppr;
    }
}

int32_t App_MotorGetEncoderPpr(void) { return s_encoderPpr; }

/* ---- P3 验收：性能统计实现 ---- */

void App_MotorPerfReset(void)
{
    s_perf.target = 0;
    s_perf.maxRpm = 0;
    s_perf.steadySum = 0;
    s_perf.steadyRpm = 0;
    s_perf.steadyN = 0U;
    s_perf.inBandN = 0U;
    s_perf.t0 = 0U;
    s_perf.riseMs = 0U;
    s_perf.settleMs = 0U;
    s_perf.state = 0U;
    s_perf.timeout = 0U;
}

uint8_t App_MotorPerfIsTimeout(void) { return s_perf.timeout; }

uint8_t App_MotorPerfIsReady(void) { return (uint8_t)((s_perf.state == 3U) ? 1U : 0U); }

int32_t App_MotorPerfGetMaxRpm(void) { return s_perf.maxRpm; }

int32_t App_MotorPerfGetSteadyRpm(void) { return s_perf.steadyRpm; }

int32_t App_MotorPerfGetSteadyError(void) { return s_perf.target - s_perf.steadyRpm; }

int32_t App_MotorPerfGetOvershootPermille(void)
{
    if (s_perf.target <= 0) {
        return 0;
    }
    if (s_perf.maxRpm <= s_perf.target) {
        return 0;
    }
    return (s_perf.maxRpm - s_perf.target) * 1000 / s_perf.target;
}

uint32_t App_MotorPerfGetRiseMs(void) { return s_perf.riseMs; }

uint32_t App_MotorPerfGetSettleMs(void) { return s_perf.settleMs; }

int32_t App_MotorGetCountRate(void)
{
    if (s_lastDtMs == 0U) {
        return 0;
    }
    return (int32_t)(s_lastDelta * 1000U / s_lastDtMs);
}

/* 每控制周期调用一次：统计超调、上升/调节时间、稳态误差。 */
static void App_MotorPerfStep(int32_t rpm)
{
    int32_t band;
    int32_t errAbs;
    uint32_t now;

    if (s_targetRpm <= 0) {
        s_perf.state = 0U; /* 无目标：不统计 */
        return;
    }

    now = (uint32_t)xTaskGetTickCount();

    /* 首次统计或目标变化：重新开始。 */
    if ((s_perf.state == 0U) || (s_perf.target != s_targetRpm)) {
        App_MotorPerfReset();
        s_perf.target = s_targetRpm;
        s_perf.t0 = now;
        s_perf.state = 1U;
    }

    /* 峰值只在响应/稳态累计阶段统计，结论产出后不再计入（避免稳态波动被当作超调）。 */
    if ((s_perf.state != 3U) && (rpm > s_perf.maxRpm)) {
        s_perf.maxRpm = rpm;
    }

    band = s_targetRpm / APP_MOTOR_PERF_BAND_DIV;
    if (band < 1) {
        band = 1;
    }
    errAbs = rpm - s_targetRpm;
    if (errAbs < 0) {
        errAbs = -errAbs;
    }

    if (s_perf.state == 1U) {
        /* 上升时间：首次达到目标的 90%。 */
        if ((s_perf.riseMs == 0U) && (rpm * 10 >= s_targetRpm * 9)) {
            s_perf.riseMs = App_MotorTickToMs(now - s_perf.t0);
        }

        if (errAbs <= band) {
            s_perf.inBandN++;
            if (s_perf.inBandN >= APP_MOTOR_PERF_SETTLE_N) {
                s_perf.settleMs = App_MotorTickToMs(now - s_perf.t0);
                s_perf.state = 2U;
                s_perf.steadyN = 0U;
                s_perf.steadySum = 0;
            }
        } else {
            s_perf.inBandN = 0U;
        }
    } else if (s_perf.state == 2U) {
        if (errAbs <= band) {
            s_perf.steadySum += rpm;
            s_perf.steadyN++;
            if (s_perf.steadyN >= APP_MOTOR_PERF_AVG_N) {
                s_perf.steadyRpm = s_perf.steadySum / (int32_t)s_perf.steadyN;
                s_perf.state = 3U; /* 结论已出，可打印 */
            }
        }
    }

    /* 统一超时兜底：60s 内没得出稳态结论（进不了带内或在带内反复掉出），
     * 判为未达标并强制输出结果，避免评估行永远不打印。 */
    if (((s_perf.state == 1U) || (s_perf.state == 2U)) && (App_MotorTickToMs(now - s_perf.t0) >= APP_MOTOR_PERF_TIMEOUT_MS)) {
        s_perf.steadyRpm = rpm;
        s_perf.timeout = 1U;
        s_perf.state = 3U;
    }
}

/* ---------------- P7：堵转检测 ---------------- */

uint8_t App_MotorIsStalled(void)
{
    return s_stallLatched;
}

void App_MotorClearStall(void)
{
    s_stallMs      = 0U;
    s_stallLatched = 0U;
}

/* 每控制周期（200ms）评估一次：有目标 + 占空比够大 + 转速起不来 + 连续保持 3s。 */
static void App_MotorStallStep(int32_t rpm)
{
    int32_t  rpmLimit;
    uint8_t  pctNow;
    uint32_t msNeed;

    if (s_stallLatched != 0U) {
        return;
    }

    /* P9-3（M5）：振动辅助——有振动说明电机在通电抖动（电气堵转），
     * 判定时间可由 3s 缩短到 1s；默认关闭（APP_MOTOR_STALL_ASSIST_ENABLE=0）。 */
    msNeed = APP_MOTOR_STALL_MS;
#if (APP_MOTOR_STALL_ASSIST_ENABLE != 0)
    /* 只有运行期开关打开（0x003B bit0 = 1）时才缩短判定时间；默认关 = 旧行为 */
    if ((s_stallAssistEn != 0U) && (s_stallAssistVib != 0U)) {
        msNeed = APP_MOTOR_STALL_ASSIST_MS;
    }
#endif

    if (s_targetRpm <= 0) {
        s_stallMs = 0U;     /* 无目标：不可能堵转 */
        return;
    }

    rpmLimit = (s_targetRpm * (int32_t)APP_MOTOR_STALL_RPM_RATIO) / 100;
    if (rpmLimit < (int32_t)APP_MOTOR_STALL_RPM_MIN) {
        rpmLimit = (int32_t)APP_MOTOR_STALL_RPM_MIN;
    }

    if ((s_pct >= APP_MOTOR_STALL_PCT_MIN) && (rpm < rpmLimit)) {
        if (s_stallMs < 60000U) {
            s_stallMs += 200U;   /* 控制周期 200ms */
        }
        if ((uint32_t)s_stallMs >= msNeed) {
            pctNow = s_pct;
            s_stallLatched = 1U;
            s_stallMs = 0U;
            App_MotorEmergencyStop();   /* 立即断 PWM + 清 PID 积分（同时清斜坡指令） */
            printf("[P7] motor stall: tgt=%ld rpm=%ld pct=%u -> stop\r\n",
                   (long)s_targetRpm, (long)rpm, (unsigned)pctNow);
        }
    } else {
        s_stallMs = 0U;      /* 任一条件不满足：重新计时，避免误判 */
    }
}

void App_MotorControlStep(void)
{
    int32_t rpm;
    float out;

    /* P7：堵转锁存期间保持停机输出，等人工清除（停机或上位机清故障）。 */
    if (s_stallLatched != 0U) {
        App_MotorSetSpeedPercent(0U);
        s_lastOut = 0.0f;
        s_cmdRpm = 0;              /* P9-3：锁存期间斜坡指令归零，避免清锁存后突冲 */
        s_rampState = APP_MOTOR_RAMP_STOP;
        return;
    }

    /* P9-3（M1）：先按斜率生成指令转速，PID 跟随指令值而非跳变的目标值。 */
    App_MotorRampStep();

    rpm = App_MotorCalcRpm();

    /* 停机收尾（2026-11-06 修）：目标=0 且斜坡指令已归零 → 输出强制 0 + 清 PID 积分。
     * 否则 err = 0-0-0，Ki×积分残值会一直作为输出（实测停机后 PWM 停在 ~20%，
     * 电机持续通电发热且堵转保护不动作，因为判据要求目标>0）。只在首次收敛时做一次，
     * 之后 s_pct/s_lastOut 都为 0，不再重复清。 */
    if ((s_targetRpm == 0) && (s_cmdRpm == 0)) {
        if ((s_pct != 0U) || (s_lastOut != 0.0f)) {
            App_MotorSetSpeedPercent(0U);
            App_PidReset(&s_speedPid);
            s_lastOut = 0.0f;
        }
        App_MotorPerfStep(rpm);
        return;
    }

    out = App_PidCompute(&s_speedPid, (float)s_cmdRpm, (float)rpm);

    /* 软启动：限制每周期占空比变化率，避免突加冲击。 */
    if (out > (s_lastOut + APP_MOTOR_SLEW_MAX)) {
        out = s_lastOut + APP_MOTOR_SLEW_MAX;
    } else if (out < (s_lastOut - APP_MOTOR_SLEW_MAX)) {
        out = s_lastOut - APP_MOTOR_SLEW_MAX;
    }

    /* 转速上限已按要求移除（原 APP_MOTOR_MAX_RPM=2500 的超速回退），
     * 目标转速由 UI 菜单直接给定，仅保留占空比硬上限做最后保护。 */
    if (out > (float)APP_MOTOR_PWM_LIMIT) {
        out = (float)APP_MOTOR_PWM_LIMIT;
    }
    if (out < 0.0f) {
        out = 0.0f;
    }
    s_lastOut = out;

    App_MotorSetSpeedPercent((uint8_t)(out + 0.5f));

    /* P3 验收：记录本周期转速，用于稳态误差/超调/调节时间统计。 */
    App_MotorPerfStep(rpm);

    /* P7：堵转判定（用本周期实际输出占空比 + 实测转速）。 */
    App_MotorStallStep(rpm);
}

/* ---------------- P9-3（M1/M3）：速度斜坡与加减速耗时 ---------------- */

static void App_MotorRampStep(void)
{
#if (APP_MOTOR_RAMP_ENABLE != 0)
    int32_t  stepAcc;
    int32_t  stepDec;
    int32_t  d;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint16_t newState;

    stepAcc = (int32_t)(((uint32_t)s_rampAccRpmS * (uint32_t)APP_MOTOR_CTRL_PERIOD_MS) / 1000U);
    stepDec = (int32_t)(((uint32_t)s_rampDecRpmS * (uint32_t)APP_MOTOR_CTRL_PERIOD_MS) / 1000U);
    if (stepAcc < 1) {
        stepAcc = 1;
    }
    if (stepDec < 1) {
        stepDec = 1;
    }

    /* 指令值按斜率逼近目标（加速/减速斜率可不同） */
    if (s_cmdRpm < s_targetRpm) {
        d = s_targetRpm - s_cmdRpm;
        s_cmdRpm += (d > stepAcc) ? stepAcc : d;
    } else if (s_cmdRpm > s_targetRpm) {
        d = s_cmdRpm - s_targetRpm;
        s_cmdRpm -= (d > stepDec) ? stepDec : d;
    } else {
        /* 已到位 */
    }

    if ((s_cmdRpm == 0) && (s_targetRpm == 0)) {
        newState = APP_MOTOR_RAMP_STOP;
    } else if (s_cmdRpm < s_targetRpm) {
        newState = APP_MOTOR_RAMP_ACC;
    } else if (s_cmdRpm > s_targetRpm) {
        newState = APP_MOTOR_RAMP_DEC;
    } else {
        newState = APP_MOTOR_RAMP_CRUISE;
    }

    /* 状态迁移时结算实测耗时（M3）：起点统一取 s_rampTargetTick（目标变更时刻），
     * 这样统计值与真实爬升时间一致（含第一拍）。 */
    if (newState != s_rampState) {
        if ((s_rampState == APP_MOTOR_RAMP_ACC) && (newState == APP_MOTOR_RAMP_CRUISE)) {
            s_accTimeMs = App_MotorTickToMs(now - s_rampTargetTick);
        }
        if ((s_rampState == APP_MOTOR_RAMP_DEC) && (newState == APP_MOTOR_RAMP_STOP)) {
            s_decTimeMs = App_MotorTickToMs(now - s_rampTargetTick);
        }
        s_rampState = newState;
    }
#else
    s_cmdRpm    = s_targetRpm;
    s_rampState = ((s_targetRpm == 0) ? APP_MOTOR_RAMP_STOP : APP_MOTOR_RAMP_CRUISE);
#endif
}

void App_MotorSetRampRate(uint16_t accRpmS, uint16_t decRpmS)
{
    if (accRpmS < 50U) {
        accRpmS = 50U;             /* 低于 50rpm/s 等同无限慢启动，钳到下限 */
    }
    if (decRpmS < 50U) {
        decRpmS = 50U;
    }
    s_rampAccRpmS = accRpmS;
    s_rampDecRpmS = decRpmS;
}

uint16_t App_MotorGetRampAccRpmS(void) { return s_rampAccRpmS; }

uint16_t App_MotorGetRampDecRpmS(void) { return s_rampDecRpmS; }

int32_t App_MotorGetCmdRpm(void) { return s_cmdRpm; }

uint16_t App_MotorGetRampState(void) { return s_rampState; }

uint32_t App_MotorGetAccTimeMs(void) { return s_accTimeMs; }

uint32_t App_MotorGetDecTimeMs(void) { return s_decTimeMs; }

int32_t App_MotorGetLastRpm(void) { return s_lastRpm; }

void App_MotorSetStallAssist(uint8_t vibDetected)
{
#if (APP_MOTOR_STALL_ASSIST_ENABLE != 0)
    s_stallAssistVib = (vibDetected != 0U) ? 1U : 0U;
#else
    (void)vibDetected;   /* 辅助堵转默认关闭：只保留接口，不参与判定 */
#endif
}

/* A8（M5）运行期开关：不改变调速/堵转判据本身，只决定是否允许缩短判定时间。 */
void App_MotorSetStallAssistEnable(uint8_t en)
{
#if (APP_MOTOR_STALL_ASSIST_ENABLE != 0)
    s_stallAssistEn = (en != 0U) ? 1U : 0U;
    printf("[P11] stall assist %s\r\n", (s_stallAssistEn != 0U) ? "on" : "off");
#else
    (void)en;   /* 编译级关闭：接口保留但无效果 */
#endif
}

uint8_t App_MotorGetStallAssistEnable(void)
{
#if (APP_MOTOR_STALL_ASSIST_ENABLE != 0)
    return s_stallAssistEn;
#else
    return 0U;
#endif
}
