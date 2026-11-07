/*
 * @file    app_tune.c
 * @brief   A2：现场实测整定支撑——一行式统计快照（默认关闭，按需打开）。
 *
 * 数据来源全部是既有模块的只读查询接口，不复算、不改任何状态：
 *   液温/板温  : App_TempReadCelsius()
 *   转速/斜坡  : App_MotorGetLastRpm/GetTargetRpm/GetCmdRpm/GetSpeedPercent/RampState
 *   加减速耗时 : App_MotorGetAccTimeMs/GetDecTimeMs
 *   六轴       : App_ImuGetSample()（打滑偏差/振动 RMS/角加速度/温度）、App_ImuGetPeakAlpha()
 *   零偏       : App_ImuGetParams()->biasDps10[]（配合静止时 gyroZ 读值看温漂）
 *   加热       : App_HeatGetPwmPercent()/App_HeatIsNoRise()
 */
#include "app_tune.h"

#if (APP_TUNE_ENABLE != 0)

#include <stdio.h>
#include <string.h>

#include "app_temp.h"
#include "app_motor.h"
#include "app_heat.h"
#include "app_imu.h"

#include "FreeRTOS.h"
#include "task.h"

static uint16_t s_ctrl    = 0U;   /* 只保存 bit0（周期打印开关） */
static uint32_t s_accMs   = 0U;   /* 周期打印计时 */
static uint32_t s_prints  = 0U;   /* 已打印行数 */

void App_TuneInit(void)
{
    s_ctrl   = 0U;
    s_accMs  = 0U;
    s_prints = 0U;
}

uint16_t App_TuneGetCtrl(void)
{
    return (uint16_t)(s_ctrl & 0x0001U);
}

/* 写 0x003C：bit0 持久（本次运行内），bit1/bit2 立即执行后自清零。 */
uint16_t App_TuneSetCtrl(uint16_t bits)
{
    s_ctrl = (uint16_t)(bits & 0x0001U);   /* bit0 周期打印开关 */

    if ((bits & 0x0002U) != 0U)
    {
        App_TunePrint("once");
    }
    if ((bits & 0x0004U) != 0U)
    {
        s_accMs  = 0U;
        s_prints = 0U;
        printf("[TUNE] stats cleared\r\n");
    }
    return App_TuneGetCtrl();
}

void App_TunePrint(const char *why)
{
    App_ImuSample_t imu;
    const App_Imu_Params_t *ip = App_ImuGetParams();
    int16_t tLiq   = App_TempReadCelsius(APP_TEMP_SRC_LIQUID);
    int16_t tPlate = App_TempReadCelsius(APP_TEMP_SRC_BOARD);
    int16_t gyroRot;
    int16_t biasRot;
    uint8_t imuOk;

    (void)memset(&imu, 0, sizeof(imu));
    imuOk = App_ImuGetSample(&imu);

    gyroRot = imu.gyroDps10[APP_IMU_ROT_AXIS];
    biasRot = (int16_t)ip->biasDps10[APP_IMU_ROT_AXIS];

    /* 一行式快照：n=序号 why=触发源；数值统一用整数 + 0.1/1 单位标注，避免浮点 printf。
     * liq/plate/t 为 0.1℃；gyroZ/biasZ 为 0.1dps；slip/vib 分别对门限。 */
    printf("[TUNE] #%lu %s liq=%d.%d plate=%d.%d T=%d.%d rpm=%ld/%ld cmd=%ld pct=%u heat=%u "
           "ramp=%u acc=%lums dec=%lums slip=%d.%d/%u vib=%u/%u pkA=%d gyroZ=%d.%d biasZ=%d.%d "
           "imuSt=0x%04X novRise=%u assist=%u\r\n",
           (unsigned long)s_prints, (why != NULL) ? why : "-",
           (int)(tLiq / 10), (int)(tLiq % 10),
           (int)(tPlate / 10), (int)(tPlate % 10),
           (int)(imu.tempC10 / 10), (int)(imu.tempC10 % 10),
           (long)App_MotorGetLastRpm(), (long)App_MotorGetTargetRpm(), (long)App_MotorGetCmdRpm(),
           (unsigned)App_MotorGetSpeedPercent(), (unsigned)App_HeatGetPwmPercent(),
           (unsigned)App_MotorGetRampState(),
           (unsigned long)App_MotorGetAccTimeMs(), (unsigned long)App_MotorGetDecTimeMs(),
           (int)(imu.slipErrX10 / 10), (int)(imu.slipErrX10 % 10),
           (unsigned)ip->slipThX10,
           (unsigned)imu.vibRmsMg, (unsigned)ip->vibThMg,
           (int)App_ImuGetPeakAlpha(),
           (int)(gyroRot / 10), (int)(gyroRot % 10),
           (int)(biasRot / 10), (int)(biasRot % 10),
           (unsigned)imu.state,
           (unsigned)App_HeatIsNoRise(),
           (unsigned)App_MotorGetStallAssistEnable());

    if (imuOk == 0U)
    {
        /* 无有效 IMU 数据时补一句，避免上面全 0 被误读成“姿态为 0” */
        printf("[TUNE] (IMU no valid sample)\r\n");
    }
    s_prints++;
}

void App_TuneStep(uint32_t dtMs)
{
    if ((s_ctrl & 0x0001U) == 0U)
    {
        s_accMs = 0U;
        return;
    }

    s_accMs += dtMs;
    if (s_accMs >= APP_TUNE_PERIOD_MS)
    {
        s_accMs = 0U;
        App_TunePrint("periodic");
    }
}

#else  /* APP_TUNE_ENABLE == 0 */

#include <stddef.h>

void App_TuneInit(void) { }
void App_TuneStep(uint32_t dtMs) { (void)dtMs; }
uint16_t App_TuneGetCtrl(void) { return 0U; }
uint16_t App_TuneSetCtrl(uint16_t bits) { (void)bits; return 0U; }
void App_TunePrint(const char *why) { (void)why; }

#endif /* APP_TUNE_ENABLE */
