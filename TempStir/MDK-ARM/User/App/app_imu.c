/*
 * @file    app_imu.c
 * @brief   P9：LSM6DSM 六轴采集 + 姿态/振动/打滑判据实现。
 *
 * 数据流（100Hz，IMU 任务）：
 *   突发读 14B -> 整数换算(mg / 0.1dps / 0.1℃) -> 扣零偏 -> 角加速度(低通)
 *   -> 振动 RMS(1s 窗) -> 姿态(20Hz) -> 打滑交叉校验 -> 快照 + 状态位
 *
 * 判据（状态位见 app_imu.h 的 APP_IMU_ST_*）：
 *   STILL   : 三轴 |gyro| < 3.0dps
 *   ROTATE  : 三轴 |gyro| >= 3.0dps
 *   VIB     : 振动 RMS >= vibTh（默认 300mg）
 *   TILT    : |roll| 或 |pitch| >= tiltTh（默认 45°）
 *   SLIP    : |ω_imu - ω_enc| / max(|ω_enc|,50rpm) > slipTh 持续 1s（M4）
 *
 * 安装：随搅拌轴刚性转动，旋转轴 = APP_IMU_ROT_AXIS（默认 Z）。
 * 日志：仅上电自检打印一次；运行期静默（异常时打一条 no-new-data）。
 */
#include "app_imu.h"
#include "app_main.h"
#include "app_fault.h"           /* P7：IMU 任务心跳（APP_HB_IMU） */
#include "app_motor.h"           /* 打滑交叉校验：编码器转速/目标/斜坡状态 */
#include "app_store.h"           /* 参数独立块读写（v1 空闲区 0x00/0x20） */
#include "dri_lsm6dsm.h"
#include "int_spi.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---------------- 模块内部状态 ---------------- */
static App_ImuSample_t app_imu_sample = {0};
static App_Imu_Params_t app_imu_params;
static uint8_t  app_imu_present = 0U;
static uint32_t app_imu_err     = 0U;
static uint32_t app_imu_seq     = 0U;
static uint8_t  app_imu_status  = 0U;   /* 最近一次 STATUS_REG（XLDA/GDA） */

static int16_t  app_imu_prevGyro = 0;   /* 上一拍旋转轴角速度(0.1dps) */
static int16_t  app_imu_alpha    = 0;   /* 低通后的角加速度(0.1dps/s) */
static int16_t  app_imu_peakAlpha = 0;
static uint8_t  app_imu_attDivCnt = 0U;
static int16_t  app_imu_rollC10  = 0;
static int16_t  app_imu_pitchC10 = 0;
static int32_t  app_imu_yawC10   = 0;

/* 零偏标定 */
static uint8_t  app_imu_calActive = 0U;
static uint16_t app_imu_calN      = 0U;
static int32_t  app_imu_calSum[3] = {0, 0, 0};

/* 振动 RMS 窗口 */
static uint32_t app_imu_vibSum   = 0U;
static uint16_t app_imu_vibN     = 0U;
static uint16_t app_imu_vibRms   = 0U;

/* 打滑判定 */
static uint8_t  app_imu_slipLatched = 0U;
#if (APP_IMU_SLIP_ENABLE != 0)
static uint16_t app_imu_slipMs      = 0U;
static uint16_t app_imu_slipClrMs   = 0U;
#endif

/* 倾斜保护（需求：>10° 长鸣 + 停机） */
static uint8_t  app_imu_tiltBlocked = 0U;
static uint16_t app_imu_tiltMs      = 0U;
static uint16_t app_imu_tiltClrMs   = 0U;

/* 姿态用加速度一阶低通状态（mg，浮点仅在此内部使用） */
static float    app_imu_lpfAx   = 0.0f;
static float    app_imu_lpfAy   = 0.0f;
static float    app_imu_lpfAz   = 0.0f;
static uint8_t  app_imu_lpfInit = 0U;

/* ---------------- 函数声明 ---------------- */
static uint32_t App_ImuIsqrt(uint32_t v);
static int16_t  App_ImuAccRawToMg(int16_t raw);
static int16_t  App_ImuGyroRawToDps10(int16_t raw);
static int16_t  App_ImuAbs16(int16_t v);
static void     App_ImuParamsDefault(App_Imu_Params_t *p);
static uint16_t App_ImuCrc16(const uint8_t *data, uint16_t len);
static void     App_ImuParamsLoad(void);
static uint8_t  App_ImuParamsSave(void);
static void     App_ImuBiasCalStep(const int16_t rawDps10[3]);
static void     App_ImuAttitudeStep(const int16_t accMg[3], int16_t gyroRot);
static void     App_ImuTiltStep(void);
#if (APP_IMU_SLIP_ENABLE != 0)
static void     App_ImuSlipStep(int16_t gyroRot, int16_t *slipErrX10);
#endif
static uint8_t  App_ImuReadAndConvert(App_ImuSample_t *out);

/* ---------------- 上电自检 ---------------- */
void App_ImuInit(void)
{
    uint8_t id = 0U;
    uint8_t xl = 0U;
    uint8_t g = 0U;
    uint8_t c3 = 0U;
    uint8_t c4 = 0U;
    uint8_t c6 = 0U;
    uint8_t c8 = 0U;
    uint8_t expXl;
    uint8_t expG;
    uint8_t cfgOk;
    int16_t tAbs;
    App_ImuSample_t s;

    App_ImuParamsDefault(&app_imu_params);
    App_ImuParamsLoad();          /* EEPROM 有有效块则覆盖默认值（含零偏） */

    printf("[P9] ===== IMU (Lsm6dsm) power-on self test =====\r\n");

    /* T1：SPI 链路与器件识别（WHO_AM_I 固定 0x6A） */
    (void)Lsm6dsm_WhoAmI(&id);
    if (id != LSM6DSM_WHO_AM_I_VAL) {
        printf("[P9] T1 WHO_AM_I=0x%02X expect=0x6A -> NG (check SPI2 PB13/14/15, CS=PB12)\r\n",
               (unsigned)id);
        printf("[P9] IMU disabled: judges/registers stay invalid, other features unaffected\r\n");
        app_imu_present = 0U;
        return;
    }
    printf("[P9] T1 WHO_AM_I=0x%02X -> OK\r\n", (unsigned)id);

    if (Lsm6dsm_Init(app_imu_params.accFs, app_imu_params.gyroFs, app_imu_params.odrCode) == 0U) {
        printf("[P9] T1 Lsm6dsm_Init failed -> IMU disabled\r\n");
        app_imu_present = 0U;
        return;
    }

    /* T2：回读配置寄存器 */
    xl = Int_SpiReadReg(LSM6DSM_REG_CTRL1_XL);
    g  = Int_SpiReadReg(LSM6DSM_REG_CTRL2_G);
    c3 = Int_SpiReadReg(LSM6DSM_REG_CTRL3_C);
    c4 = Int_SpiReadReg(LSM6DSM_REG_CTRL4_C);
    c6 = Int_SpiReadReg(LSM6DSM_REG_CTRL6_C);
    c8 = Int_SpiReadReg(LSM6DSM_REG_CTRL8_XL);
    expXl = (uint8_t)(((app_imu_params.odrCode & 0x0FU) << 4) | ((app_imu_params.accFs & 0x03U) << 2));
    expG  = (uint8_t)(((app_imu_params.odrCode & 0x0FU) << 4) | ((app_imu_params.gyroFs & 0x03U) << 2));
    cfgOk = ((xl == expXl) && (g == expG) && (c3 == 0x44U)) ? 1U : 0U;
    printf("[P9] T2 cfg XL=0x%02X G=0x%02X C3=0x%02X C4=0x%02X C6=0x%02X C8=0x%02X (exp XL=0x%02X G=0x%02X C3=0x44) -> %s\r\n",
           (unsigned)xl, (unsigned)g, (unsigned)c3, (unsigned)c4, (unsigned)c6, (unsigned)c8,
           (unsigned)expXl, (unsigned)expG, (cfgOk != 0U) ? "OK" : "NG");

    app_imu_present = 1U;

#if (APP_IMU_SELFTEST_ENABLE != 0)
    /* G1：CTRL5_C 硬件自检（默认关闭）。内部阻塞约 400ms，放在首帧读取之前。 */
    {
        uint8_t xlStOk = 0U;
        uint8_t gStOk  = 0U;

        if (Lsm6dsm_SelfTest(&xlStOk, &gStOk) != 0U) {
            printf("[P9] T2b selftest XL=%s G=%s\r\n",
                   (xlStOk != 0U) ? "OK" : "NG", (gStOk != 0U) ? "OK" : "NG");
        } else {
            printf("[P9] T2b selftest not executed (read failed)\r\n");
        }
    }
#endif

#if (APP_IMU_FIFO_ENABLE != 0)
    /* G7：FIFO 连续模式（默认关闭）。配置失败则退回寄存器直读，不影响判据。 */
    if (Lsm6dsm_FifoConfig(app_imu_params.odrCode, LSM6DSM_FIFO_MODE_CONT,
                           (uint16_t)APP_IMU_FIFO_WTM) != 0U) {
        printf("[P9] G7 FIFO on: odr=%u wtm=%u words\r\n",
               (unsigned)app_imu_params.odrCode, (unsigned)APP_IMU_FIFO_WTM);
    } else {
        printf("[P9] G7 FIFO config NG -> fallback to register read\r\n");
        Lsm6dsm_FifoDisable();
    }
#endif

    /* 加速度滤波链上电建立约需 150ms（首帧直接读只有 0.12g），等 200ms 再看首帧 */
    HAL_Delay(200U);

    if (App_ImuReadAndConvert(&s) != 0U) {
        tAbs = App_ImuAbs16(s.tempC10);
        printf("[P9] T3/T4/T5 first: A(mg)=%d %d %d |a|=%d G(0.1dps)=%d %d %d T=%s%d.%dC\r\n",
               (int)s.accMg[0], (int)s.accMg[1], (int)s.accMg[2], (int)s.accMagMg,
               (int)s.gyroDps10[0], (int)s.gyroDps10[1], (int)s.gyroDps10[2],
               (s.tempC10 < 0) ? "-" : "", (int)(tAbs / 10), (int)tAbs % 10);
    }

    printf("[P9] P9-2/P9-3 on: biasCal=%s rotAxis=%u slip=%s ramp=%u/%urpm/s slipTh=%u.%u%% vibTh=%umg tiltTh=%u.%u\r\n",
#if (APP_IMU_BIAS_AUTOCAL_ENABLE != 0)
           "auto(2s)",
#else
           "stored",
#endif
           (unsigned)APP_IMU_ROT_AXIS,
#if (APP_IMU_SLIP_ENABLE != 0)
           "on",
#else
           "off",
#endif
           (unsigned)app_imu_params.alphaLimitRpmS, (unsigned)app_imu_params.alphaLimitRpmS,
           (unsigned)(app_imu_params.slipThX10 / 10U), (unsigned)(app_imu_params.slipThX10 % 10U),
           (unsigned)app_imu_params.vibThMg,
           (unsigned)(app_imu_params.tiltThC10 / 10U), (unsigned)(app_imu_params.tiltThC10 % 10U));

    /* M1：把 EEPROM 里的加减速斜率下发给电机速度斜坡 */
    App_MotorSetRampRate(app_imu_params.alphaLimitRpmS, app_imu_params.alphaLimitRpmS);

#if (APP_IMU_BIAS_AUTOCAL_ENABLE != 0)
    App_ImuStartBiasCal();   /* 上电自动标定：需保持静止 2s */
#else
    if (app_imu_params.seq != 0U) {
        app_imu_sample.state |= APP_IMU_ST_CALIB;
    }
#endif
}

/* ---------------- 采集任务（100Hz） ---------------- */
void App_ImuTask(void *argument)
{
    App_ImuSample_t s;
    uint32_t noData = 0U;

    (void)argument;

    for (;;) {
        App_FaultHeartbeat(APP_HB_IMU);   /* P7：看门狗心跳（含器件缺省的静默分支） */

        if (app_imu_present == 0U) {
            vTaskDelay(pdMS_TO_TICKS(100U));   /* 未识别到器件：静默等待，心跳照报 */
            continue;
        }

        if (App_ImuReadAndConvert(&s) == 0U) {
            app_imu_err++;
        } else {
            taskENTER_CRITICAL();
            app_imu_sample = s;
            taskEXIT_CRITICAL();

            /* 数据就绪监督：连续 1s 两路 DRDY 都为 0 说明没有新样本 */
            if ((app_imu_status & 0x03U) == 0U) {
                if (noData < 1000U) {
                    noData++;
                }
                if (noData == 100U) {
                    printf("[P9] T6 no new data: STATUS=0x%02X for 1s -> check ODR/SPI\r\n",
                           (unsigned)app_imu_status);
                }
            } else {
                noData = 0U;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(APP_IMU_PERIOD_MS));
    }
}

/* ---------------- 对外查询 ---------------- */
uint8_t App_ImuIsPresent(void) { return app_imu_present; }

uint8_t App_ImuGetSample(App_ImuSample_t *out)
{
    if ((out == NULL) || (app_imu_present == 0U) || (app_imu_seq == 0U)) {
        return 0U;
    }

    taskENTER_CRITICAL();
    *out = app_imu_sample;
    taskEXIT_CRITICAL();
    return 1U;
}

uint8_t App_ImuIsFault(void)
{
    /* 器件在线但连续读失败（>=50 次 ≈ 0.5s）视为通信异常 */
    return ((app_imu_present != 0U) && (app_imu_err >= 50U)) ? 1U : 0U;
}

uint32_t App_ImuGetErrCount(void) { return app_imu_err; }

uint8_t App_ImuHasVibration(void)
{
    return (((app_imu_sample.state & APP_IMU_ST_VIB) != 0U) && (app_imu_present != 0U)) ? 1U : 0U;
}

uint8_t App_ImuIsSlipping(void) { return app_imu_slipLatched; }

uint8_t App_ImuIsTiltBlocked(void)
{
    /* 器件缺省/无有效数据时不得触发停机保护（回到旧行为） */
    if ((app_imu_present == 0U) || (app_imu_seq == 0U)) {
        return 0U;
    }
    return app_imu_tiltBlocked;
}

void App_ImuClearTiltBlock(void)
{
    app_imu_tiltBlocked = 0U;
    app_imu_tiltMs      = 0U;
    app_imu_tiltClrMs   = 0U;
}

int16_t App_ImuGetPeakAlpha(void) { return app_imu_peakAlpha; }

void App_ImuStartBiasCal(void)
{
    app_imu_calActive = 1U;
    app_imu_calN      = 0U;
    app_imu_calSum[0] = 0;
    app_imu_calSum[1] = 0;
    app_imu_calSum[2] = 0;
    app_imu_sample.state &= (uint16_t)~APP_IMU_ST_CALIB;
    printf("[P9] bias cal start: keep still %ums\r\n", (unsigned)(APP_IMU_BIAS_CAL_N * APP_IMU_PERIOD_MS));
}

void App_ImuClearEvents(void)
{
    app_imu_slipLatched = 0U;
#if (APP_IMU_SLIP_ENABLE != 0)
    app_imu_slipMs      = 0U;
    app_imu_slipClrMs   = 0U;
#endif
    app_imu_peakAlpha   = 0;
}

const App_Imu_Params_t *App_ImuGetParams(void) { return &app_imu_params; }

void App_ImuSetSlipThX10(uint16_t v)
{
    if (v < 10U) {
        v = 10U;                       /* 最低 1.0%，防止除零式误判 */
    }
    if (v > 5000U) {
        v = 5000U;                     /* 最高 500%：等于关闭判据 */
    }
    app_imu_params.slipThX10 = v;
}

void App_ImuSetVibThMg(uint16_t v)
{
    if (v < 20U) {
        v = 20U;
    }
    app_imu_params.vibThMg = v;
}

void App_ImuSetTiltThC10(uint16_t v)
{
    if (v < 50U) {
        v = 50U;                       /* 5.0° */
    }
    if (v > 900U) {
        v = 900U;                      /* 90.0° */
    }
    app_imu_params.tiltThC10 = v;
}

void App_ImuSetAlphaLimitRpmS(uint16_t v)
{
    if (v < 50U) {
        v = 50U;
    }
    if (v > 3000U) {
        v = 3000U;
    }
    app_imu_params.alphaLimitRpmS = v;
    App_MotorSetRampRate(v, v);        /* M1：斜坡斜率同源 */
}

void App_ImuSetSlipAssistStall(uint8_t en)
{
    app_imu_params.slipAssistStallEn = (en != 0U) ? 1U : 0U;
}

uint8_t App_ImuSaveParams(void) { return App_ImuParamsSave(); }

/* ---------------- 单拍采集 + 判据 ---------------- */
static uint8_t App_ImuReadAndConvert(App_ImuSample_t *out)
{
    Lsm6dsmRaw_t raw;
    App_ImuSample_t s;
    int16_t rawDps10[3];
    int16_t gyroRot;
    int16_t slipErr;
    int32_t mag2;
    int32_t d;
    int32_t aRaw;
    uint16_t state;
    uint8_t i;

    if ((out == NULL) || (app_imu_present == 0U)) {
        return 0U;
    }

#if (APP_IMU_FIFO_ENABLE != 0)
    /* G7：FIFO 路径——一次排空，只取最新一组样本参与判据。
     * 注意：FIFO 中每拍样本数可能 >1，本路径的时间相关判据仍按 APP_IMU_PERIOD_MS
     * 估算（默认关闭，若要长期启用需按实际样本数积分时间）。 */
    {
        uint16_t fifoWords = Lsm6dsm_FifoDrain(&raw);

        if (fifoWords == 0U) {
            return 0U;      /* FIFO 空：本拍没有新样本 */
        }
        app_imu_status = 0x03U;   /* 有样本 = 两路均有新数据 */
    }
#else
    /* 必须先读 STATUS_REG：数据寄存器读完后器件会清就绪位 */
    app_imu_status = Int_SpiReadReg(LSM6DSM_REG_STATUS_REG);
    if (Lsm6dsm_ReadRaw(&raw) == 0U) {
        return 0U;
    }
#endif

    for (i = 0U; i < 3U; i++) {
        s.accMg[i]     = App_ImuAccRawToMg(raw.accRaw[i]);
        rawDps10[i]    = App_ImuGyroRawToDps10(raw.gyroRaw[i]);
        s.gyroDps10[i] = rawDps10[i];
    }
    s.tempC10 = Lsm6dsm_TempRawToC10(raw.tempRaw);

    mag2 = ((int32_t)s.accMg[0] * (int32_t)s.accMg[0])
         + ((int32_t)s.accMg[1] * (int32_t)s.accMg[1])
         + ((int32_t)s.accMg[2] * (int32_t)s.accMg[2]);
    s.accMagMg = (int16_t)App_ImuIsqrt((uint32_t)mag2);

    /* 零偏标定：窗口内取均值（用未扣零偏的原始换算值） */
    App_ImuBiasCalStep(rawDps10);

    /* 扣零偏 */
    if (app_imu_calActive == 0U) {
        for (i = 0U; i < 3U; i++) {
            s.gyroDps10[i] = (int16_t)(rawDps10[i] - app_imu_params.biasDps10[i]);
        }
    }

    /* 旋转轴角速度 -> 角加速度（0.1dps/s = Δ(0.1dps)/0.01s = Δ × 100） */
    gyroRot = s.gyroDps10[APP_IMU_ROT_AXIS];
    aRaw = ((int32_t)gyroRot - (int32_t)app_imu_prevGyro) * (int32_t)(1000 / APP_IMU_PERIOD_MS);
    app_imu_prevGyro = gyroRot;
    app_imu_alpha = (int16_t)((int32_t)app_imu_alpha
                              + (aRaw - (int32_t)app_imu_alpha) / APP_IMU_ALPHA_LPF_DIV);
    s.alphaDps10 = app_imu_alpha;
    if (App_ImuAbs16(app_imu_alpha) > App_ImuAbs16(app_imu_peakAlpha)) {
        app_imu_peakAlpha = app_imu_alpha;
    }

    /* 振动 RMS：合矢量偏离 1g 的平方和，1s 窗求 RMS */
#if (APP_IMU_VIB_ENABLE != 0)
    d = (int32_t)s.accMagMg - 1000;
    if (d > 3000) {
        d = 3000;                      /* 限幅，避免冲击把 32 位平方和打满 */
    } else if (d < -3000) {
        d = -3000;
    }
    app_imu_vibSum += (uint32_t)(d * d);
    app_imu_vibN++;
    if (app_imu_vibN >= APP_IMU_VIB_WIN) {
        app_imu_vibRms = (uint16_t)App_ImuIsqrt(app_imu_vibSum / (uint32_t)app_imu_vibN);
        app_imu_vibSum = 0U;
        app_imu_vibN   = 0U;
    }
#endif
    s.vibRmsMg = app_imu_vibRms;

    /* 姿态（20Hz）与 yaw 积分 */
#if (APP_IMU_ATTITUDE_ENABLE != 0)
    App_ImuAttitudeStep(s.accMg, gyroRot);
#else
    app_imu_rollC10  = 0;
    app_imu_pitchC10 = 0;
    app_imu_yawC10   = 0;
#endif
    s.rollC10  = app_imu_rollC10;
    s.pitchC10 = app_imu_pitchC10;
    s.yawC10   = (int16_t)app_imu_yawC10;

    /* 倾斜保护判定（需求：>10° 蜂鸣器长鸣 + 电机运转中停机） */
    App_ImuTiltStep();

    /* 打滑交叉校验（M4） */
    slipErr = 0;
#if (APP_IMU_SLIP_ENABLE != 0)
    App_ImuSlipStep(gyroRot, &slipErr);
#endif
    s.slipErrX10 = slipErr;

    /* ---- 状态位 ---- */
    state = APP_IMU_ST_VALID;
    if (app_imu_calActive == 0U) {
        state |= APP_IMU_ST_CALIB;
    }
    if ((App_ImuAbs16(s.gyroDps10[0]) < APP_IMU_STILL_GYRO_MAX) &&
        (App_ImuAbs16(s.gyroDps10[1]) < APP_IMU_STILL_GYRO_MAX) &&
        (App_ImuAbs16(s.gyroDps10[2]) < APP_IMU_STILL_GYRO_MAX)) {
        state |= APP_IMU_ST_STILL;
    } else {
        state |= APP_IMU_ST_ROTATE;
    }
    if (s.vibRmsMg >= app_imu_params.vibThMg) {
        state |= APP_IMU_ST_VIB;
    }
    if ((App_ImuAbs16(app_imu_rollC10) >= (int16_t)app_imu_params.tiltThC10) ||
        (App_ImuAbs16(app_imu_pitchC10) >= (int16_t)app_imu_params.tiltThC10)) {
        state |= APP_IMU_ST_TILT;
    }
    if (app_imu_slipLatched != 0U) {
        state |= APP_IMU_ST_SLIP;
    }
    if (app_imu_tiltBlocked != 0U) {
        state |= APP_IMU_ST_TILT_LOCK;
    }
    s.state = state;

    app_imu_seq++;
    s.seq = app_imu_seq;

    *out = s;
    return 1U;
}

/* 姿态：roll/pitch 用加速度，yaw 用陀螺积分（相对角、会漂移） */
static void App_ImuAttitudeStep(const int16_t accMg[3], int16_t gyroRot)
{
    float ax;
    float ay;
    float az;
    float roll;
    float pitch;

    /* yaw 积分：0.1° += (0.1dps)/1000 * 10ms * 10 = gyroRot/100 */
    app_imu_yawC10 += (int32_t)gyroRot / 100;
    if (app_imu_yawC10 > 1800) {
        app_imu_yawC10 -= 3600;
    } else if (app_imu_yawC10 < -1800) {
        app_imu_yawC10 += 3600;
    }

    /* 加速度一阶低通（10ms 拍）：搅拌振动会让瞬时角度抖动十几度，
     * 不滤波会把“振动”误判成“倾斜”从而误停机。 */
    if (app_imu_lpfInit == 0U) {
        app_imu_lpfAx = (float)accMg[0];
        app_imu_lpfAy = (float)accMg[1];
        app_imu_lpfAz = (float)accMg[2];
        app_imu_lpfInit = 1U;
    } else {
        app_imu_lpfAx += ((float)accMg[0] - app_imu_lpfAx) / (float)APP_IMU_ACC_LPF_DIV;
        app_imu_lpfAy += ((float)accMg[1] - app_imu_lpfAy) / (float)APP_IMU_ACC_LPF_DIV;
        app_imu_lpfAz += ((float)accMg[2] - app_imu_lpfAz) / (float)APP_IMU_ACC_LPF_DIV;
    }

    app_imu_attDivCnt++;
    if (app_imu_attDivCnt < APP_IMU_ATT_DIV) {
        return;
    }
    app_imu_attDivCnt = 0U;

    ax = app_imu_lpfAx;
    ay = app_imu_lpfAy;
    az = app_imu_lpfAz;
    roll  = atan2f(ay, az) * 57.29578f;
    pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 57.29578f;

    app_imu_rollC10  = (int16_t)(roll * 10.0f);
    app_imu_pitchC10 = (int16_t)(pitch * 10.0f);
}

/* 倾斜保护判定：倾角（|roll| 或 |pitch| 取大者）超门限持续 500ms 确认；
 * 回到 门限-3° 并持续 500ms 才解除。执行动作（停机/鸣叫）由 CTRL 侧完成。 */
static void App_ImuTiltStep(void)
{
#if (APP_IMU_TILT_PROTECT_ENABLE != 0)
    int16_t r;
    int16_t p;
    int16_t maxTilt;
    int16_t th;
    int16_t rel;

    r = App_ImuAbs16(app_imu_rollC10);
    p = App_ImuAbs16(app_imu_pitchC10);
    maxTilt = (r > p) ? r : p;
    th  = (int16_t)app_imu_params.tiltThC10;
    rel = (int16_t)((th > (int16_t)APP_IMU_TILT_HYST_C10) ? (th - (int16_t)APP_IMU_TILT_HYST_C10) : 1);

    if (maxTilt >= th) {
        app_imu_tiltClrMs = 0U;
        if (app_imu_tiltMs < 60000U) {
            app_imu_tiltMs = (uint16_t)(app_imu_tiltMs + APP_IMU_PERIOD_MS);
        }
        if ((app_imu_tiltBlocked == 0U) && (app_imu_tiltMs >= APP_IMU_TILT_LATCH_MS)) {
            app_imu_tiltBlocked = 1U;
            printf("[P9] TILT latch: roll=%d.%d pitch=%d.%d th=%d.%d deg -> stop motor + alarm\r\n",
                   (int)(app_imu_rollC10 / 10), (int)App_ImuAbs16((int16_t)(app_imu_rollC10 % 10)),
                   (int)(app_imu_pitchC10 / 10), (int)App_ImuAbs16((int16_t)(app_imu_pitchC10 % 10)),
                   (int)(th / 10), (int)(th % 10));
        }
    } else {
        app_imu_tiltMs = 0U;
        if (app_imu_tiltBlocked != 0U) {
            if (maxTilt <= rel) {
                app_imu_tiltClrMs = (uint16_t)(app_imu_tiltClrMs + APP_IMU_PERIOD_MS);
                if (app_imu_tiltClrMs >= APP_IMU_TILT_CLEAR_MS) {
                    app_imu_tiltBlocked = 0U;
                    app_imu_tiltClrMs   = 0U;
                    printf("[P9] TILT cleared (roll=%d.%d pitch=%d.%d)\r\n",
                           (int)(app_imu_rollC10 / 10), (int)App_ImuAbs16((int16_t)(app_imu_rollC10 % 10)),
                           (int)(app_imu_pitchC10 / 10), (int)App_ImuAbs16((int16_t)(app_imu_pitchC10 % 10)));
                }
            } else {
                app_imu_tiltClrMs = 0U;
            }
        }
    }
#endif
}

/* 打滑：ω_imu(rpm) 与编码器 rpm 交叉校验，偏差持续 1s 置位、恢复 1s 清除 */
#if (APP_IMU_SLIP_ENABLE != 0)
static void App_ImuSlipStep(int16_t gyroRot, int16_t *slipErrX10)
{
    int32_t encRpm;
    int32_t imuRpm;
    int32_t diff;
    int32_t base;
    int32_t errX10;

    encRpm = App_MotorGetLastRpm();
    imuRpm = (int32_t)gyroRot / 60;             /* 0.1dps -> rpm：dps/6 = (g/10)/6 */
    diff   = imuRpm - encRpm;
    if (diff < 0) {
        diff = -diff;
    }
    base = (encRpm >= 0) ? encRpm : -encRpm;

    if ((App_MotorGetTargetRpm() == 0) || (base < APP_IMU_SLIP_MIN_RPM)) {
        /* 停机 / 低速：不判打滑，偏差清零并逐步解除锁存 */
        *slipErrX10 = 0;
        if (app_imu_slipLatched != 0U) {
            app_imu_slipClrMs = (uint16_t)(app_imu_slipClrMs + APP_IMU_PERIOD_MS);
            if (app_imu_slipClrMs >= APP_IMU_SLIP_CLEAR_MS) {
                app_imu_slipLatched = 0U;
                app_imu_slipClrMs   = 0U;
            }
        }
        return;
    }

    errX10 = diff * 1000 / base;                 /* 0.1% 单位（15.0% -> 150） */
    if (errX10 > 32767) {
        errX10 = 32767;
    }
    *slipErrX10 = (int16_t)errX10;

    if (errX10 > (int32_t)app_imu_params.slipThX10) {
        app_imu_slipClrMs = 0U;
        if (app_imu_slipMs < 60000U) {
            app_imu_slipMs = (uint16_t)(app_imu_slipMs + APP_IMU_PERIOD_MS);
        }
        if (app_imu_slipMs >= APP_IMU_SLIP_LATCH_MS) {
            app_imu_slipLatched = 1U;
            app_imu_slipMs      = 0U;
        }
    } else {
        app_imu_slipMs = 0U;
        if (app_imu_slipLatched != 0U) {
            if (errX10 < (int32_t)(app_imu_params.slipThX10 / 2U)) {
                app_imu_slipClrMs = (uint16_t)(app_imu_slipClrMs + APP_IMU_PERIOD_MS);
                if (app_imu_slipClrMs >= APP_IMU_SLIP_CLEAR_MS) {
                    app_imu_slipLatched = 0U;
                    app_imu_slipClrMs   = 0U;
                }
            } else {
                app_imu_slipClrMs = 0U;
            }
        }
    }
}
#endif /* APP_IMU_SLIP_ENABLE */

/* 零偏标定：窗口内累加原始换算值，结束后求均值并保存 */
static void App_ImuBiasCalStep(const int16_t rawDps10[3])
{
    uint8_t i;

    if (app_imu_calActive == 0U) {
        return;
    }

    for (i = 0U; i < 3U; i++) {
        app_imu_calSum[i] += (int32_t)rawDps10[i];
    }
    app_imu_calN++;

    if (app_imu_calN < APP_IMU_BIAS_CAL_N) {
        return;
    }

    app_imu_calActive = 0U;
    for (i = 0U; i < 3U; i++) {
        int32_t mean = app_imu_calSum[i] / (int32_t)APP_IMU_BIAS_CAL_N;
        if ((mean > -1000) && (mean < 1000)) {          /* |均值| < 100dps 才可信 */
            app_imu_params.biasDps10[i] = (int16_t)mean;
        }
    }
    app_imu_sample.state |= APP_IMU_ST_CALIB;
    (void)App_ImuParamsSave();
    printf("[P9] bias cal done: bias(0.1dps)=%d %d %d saved=%u\r\n",
           (int)app_imu_params.biasDps10[0], (int)app_imu_params.biasDps10[1],
           (int)app_imu_params.biasDps10[2], (unsigned)app_imu_params.seq);
}

/* ---------------- 参数（EEPROM 独立块 0x00 / 0x20） ---------------- */

static void App_ImuParamsDefault(App_Imu_Params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->magic            = APP_IMU_PARAM_MAGIC;
    p->version          = APP_IMU_PARAM_VERSION;
    p->seq              = 0U;
    p->enable           = 1U;
    p->accFs            = APP_IMU_XL_FS;
    p->gyroFs           = APP_IMU_GY_FS;
    p->odrCode          = APP_IMU_ODR_CODE;
    p->biasDps10[0]     = 0;
    p->biasDps10[1]     = 0;
    p->biasDps10[2]     = 0;
    p->alphaLimitRpmS   = APP_IMU_ALPHA_LIMIT_DEFAULT;
    p->slipThX10        = APP_IMU_SLIP_TH_DEFAULT_X10;
    p->vibThMg          = APP_IMU_VIB_TH_DEFAULT_MG;
    p->tiltThC10        = APP_IMU_TILT_TH_DEFAULT_C10;
    p->slipAssistStallEn = 0U;
    p->reserved         = 0U;
    p->crc              = 0U;
}

static uint16_t App_ImuCrc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static uint8_t App_ImuParamsValid(const App_Imu_Params_t *p)
{
    uint16_t crcCalc;

    /* 接受 v1（旧块）以便迁移；CRC 覆盖 version 字段，旧块同样能校验通过 */
    if ((p->magic != APP_IMU_PARAM_MAGIC) ||
        ((p->version != 1U) && (p->version != APP_IMU_PARAM_VERSION))) {
        return 0U;
    }
    crcCalc = App_ImuCrc16((const uint8_t *)p, (uint16_t)sizeof(App_Imu_Params_t) - 2U);
    return (crcCalc == p->crc) ? 1U : 0U;
}

static void App_ImuParamsLoad(void)
{
    App_Imu_Params_t a;
    App_Imu_Params_t b;
    uint8_t okA;
    uint8_t okB;
    const App_Imu_Params_t *pick = NULL;

    okA = (App_StoreRawRead(APP_IMU_BLOCK_A_ADDR, &a, (uint16_t)sizeof(a)) != 0U)
          ? App_ImuParamsValid(&a) : 0U;
    okB = (App_StoreRawRead(APP_IMU_BLOCK_B_ADDR, &b, (uint16_t)sizeof(b)) != 0U)
          ? App_ImuParamsValid(&b) : 0U;

    if ((okA != 0U) && (okB != 0U)) {
        pick = (a.seq >= b.seq) ? &a : &b;
    } else if (okA != 0U) {
        pick = &a;
    } else if (okB != 0U) {
        pick = &b;
    }

    if (pick != NULL) {
        app_imu_params = *pick;

        /* v1 -> v2 迁移：旧默认倾斜门限 45.0° 自动改为 10.0°（用户改过的其它值保持） */
        if (app_imu_params.version < APP_IMU_PARAM_VERSION) {
            if (app_imu_params.tiltThC10 == 450U) {
                app_imu_params.tiltThC10 = APP_IMU_TILT_TH_DEFAULT_C10;
            }
            app_imu_params.version = APP_IMU_PARAM_VERSION;
            printf("[P9] params migrated v1 -> v2 (tiltTh=%u)\r\n",
                   (unsigned)app_imu_params.tiltThC10);
        }

        printf("[P9] params loaded: seq=%u bias=%d/%d/%d slipTh=%u vibTh=%u tiltTh=%u\r\n",
               (unsigned)app_imu_params.seq,
               (int)app_imu_params.biasDps10[0], (int)app_imu_params.biasDps10[1],
               (int)app_imu_params.biasDps10[2], (unsigned)app_imu_params.slipThX10,
               (unsigned)app_imu_params.vibThMg, (unsigned)app_imu_params.tiltThC10);
    }
}

static uint8_t App_ImuParamsSave(void)
{
    App_Imu_Params_t p;
    uint8_t addr;
    uint8_t ok;

    p = app_imu_params;
    p.magic   = APP_IMU_PARAM_MAGIC;
    p.version = APP_IMU_PARAM_VERSION;
    p.seq     = (uint16_t)(app_imu_params.seq + 1U);
    p.crc     = App_ImuCrc16((const uint8_t *)&p, (uint16_t)sizeof(p) - 2U);

    /* 交替写两份，任一失败仍保留旧数据 */
    addr = ((p.seq & 1U) != 0U) ? (uint8_t)APP_IMU_BLOCK_A_ADDR : (uint8_t)APP_IMU_BLOCK_B_ADDR;
    ok   = App_StoreRawWrite(addr, &p, (uint16_t)sizeof(p));
    if (ok != 0U) {
        app_imu_params = p;
    }
    return ok;
}

/* ---------------- 小工具 ---------------- */

static int16_t App_ImuAbs16(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

static int16_t App_ImuAccRawToMg(int16_t raw)
{
    return (int16_t)(((int32_t)raw * (int32_t)Lsm6dsm_AccLsbToUg(app_imu_params.accFs)) / 1000);
}

static int16_t App_ImuGyroRawToDps10(int16_t raw)
{
    return (int16_t)(((int32_t)raw * (int32_t)Lsm6dsm_GyroLsbToUdps(app_imu_params.gyroFs)) / 100000);
}

static uint32_t App_ImuIsqrt(uint32_t v)
{
    uint32_t res = 0U;
    uint32_t bit = 1UL << 30;

    while (bit > v) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (v >= (res + bit)) {
            v -= (res + bit);
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}
