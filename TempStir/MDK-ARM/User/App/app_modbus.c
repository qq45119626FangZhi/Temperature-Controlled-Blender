/*
 * @file    app_modbus.c
 * @brief   P6：Modbus RTU 从机应用层（FreeModbus）。
 *
 * 寄存器表见 app_modbus.h。读回调内部先 App_ModbusSyncRegs() 刷新快照，
 * 写回调逐个交给 App_CommWriteReg()（面板仲裁/范围校验/保护联锁），
 * 因此上位机写入与面板操作走同一条下发路径，不存在两套状态。
 *
 * 异常码映射：
 *   地址越界/只读  -> MB_ENOREG -> 02 非法数据地址
 *   数值非法/保护  -> MB_EINVAL -> 04 从机设备故障（FreeModbus 默认映射）
 *   面板占用       -> MB_ETIMEDOUT -> 06 从机忙
 */
#include "app_modbus.h"
#include "app_comm.h"
#include "app_ble.h"
#include "app_main.h"
#include "app_motor.h"
#include "app_heat.h"
#include "app_temp.h"
#include "app_fan.h"
#include "app_store.h"
#include "app_rtc.h"
#include "app_imu.h"     /* P9：六轴/姿态/打滑/斜坡寄存器 */
#include "app_tune.h"    /* A2：实测统计控制字回读 */
#include "menu_ui.h"

#include <stdio.h>
#include <string.h>

static USHORT s_regs[APP_MB_REG_HOLDING_NREGS];

/* ---------------- 寄存器快照 ---------------- */

USHORT App_ModbusReadReg(USHORT usAddress)
{
    if (usAddress >= APP_MB_REG_HOLDING_NREGS)
    {
        return 0U;
    }

    return s_regs[usAddress];
}

static USHORT App_ModbusClampI16(int32_t value)
{
    if (value > 32767)
    {
        value = 32767;
    }
    else if (value < -32768)
    {
        value = -32768;
    }
    else
    {
        /* 范围内 */
    }

    return (USHORT)(int16_t)value;
}

void App_ModbusSyncRegs(void)
{
    const App_Store_Params_t *p = App_StoreGet();
    uint16_t faultBits = App_CommGetFaultBits();
    uint16_t state = 0U;
    uint8_t  heatPct = App_HeatGetPwmPercent();
    uint8_t  running = Menu_UiIsRunning();
    uint32_t totalRun = p->totalRunSec;
    App_ImuSample_t imu;
    const App_Imu_Params_t *ip = App_ImuGetParams();
    uint8_t  imuOk;

    imuOk = App_ImuGetSample(&imu);   /* P9：一次拷贝，下面状态位与寄存器共用 */
    if (imuOk == 0U)
    {
        (void)memset(&imu, 0, sizeof(imu));
    }

    /* 温度：int16 / 0.1℃ */
    s_regs[APP_MB_REG_CUR_TEMP]   = (USHORT)App_TempReadCelsius(APP_TEMP_SRC_LIQUID);
    s_regs[APP_MB_REG_PLATE_TEMP] = (USHORT)App_TempReadCelsius(APP_TEMP_SRC_BOARD);

    /* 目标值来自面板/通信同一份设定（menu_ui 侧维护） */
    s_regs[APP_MB_REG_TARGET_TEMP]  = (USHORT)(int16_t)(Menu_UiGetLiquid() * 10);

    /* 转速：int16 RPM（电机轴） */
    s_regs[APP_MB_REG_CUR_SPEED]    = App_ModbusClampI16(app_main_state.cur_speed);
    s_regs[APP_MB_REG_TARGET_SPEED] = App_ModbusClampI16(Menu_UiGetSpeed());

    /* 时间 */
    s_regs[APP_MB_REG_REMAIN_TIME] = (USHORT)Menu_UiGetRemainSec();
    s_regs[APP_MB_REG_SET_TIME]    = (USHORT)Menu_UiGetTime();

    /* 占空比/风扇 */
    s_regs[APP_MB_REG_HEAT_PCT]  = (USHORT)heatPct;
    s_regs[APP_MB_REG_MOTOR_PCT] = (USHORT)App_MotorGetSpeedPercent();
    s_regs[APP_MB_REG_FAN_STATE] = (USHORT)((Fan_IsOn() != 0U) ? 1U : 0U);

    /* 状态字 */
    if (running != 0U)
    {
        state |= APP_MB_ST_MOTOR_RUNNING;
    }
    if (heatPct > 0U)
    {
        state |= APP_MB_ST_HEATING;
    }
    if (Fan_IsOn() != 0U)
    {
        state |= APP_MB_ST_FAN_ON;
    }
    if (faultBits != 0U)
    {
        state |= APP_MB_ST_FAULT;
    }
    if ((running != 0U) && (Menu_UiGetTime() > 0U))
    {
        state |= APP_MB_ST_TIMING;
    }
    if (App_RtcIsValid() != 0U)
    {
        state |= APP_MB_ST_RTC_VALID;
    }
    if (App_CommIsPanelBusy() != 0U)
    {
        state |= APP_MB_ST_PANEL_ACTIVE;
    }
    if (App_BleIsOnline() != 0U)
    {
        state |= APP_MB_ST_BLE_ONLINE;
    }
    if (App_HeatIsDone() != 0U)
    {
        state |= APP_MB_ST_HEAT_DONE;   /* P7：液温到温，加热已结束（电机可能仍在转） */
    }
    if ((imuOk != 0U) && ((imu.state & APP_IMU_ST_VALID) != 0U))
    {
        state |= APP_MB_ST_IMU_VALID;   /* P9：六轴数据有效 */
    }
    if (App_ImuIsSlipping() != 0U)
    {
        state |= APP_MB_ST_SLIP;        /* P9：打滑告警 */
    }
    if (App_HeatIsNoRise() != 0U)
    {
        state |= APP_MB_ST_HEAT_NO_RISE;   /* A3：液体升温异常（仅告警，不进故障字） */
    }
    s_regs[APP_MB_REG_SYS_STATE]  = state;
    s_regs[APP_MB_REG_FAULT_CODE] = faultBits;

    /* 命令寄存器为“写后自动清零”语义，读回恒 0 */
    s_regs[APP_MB_REG_CTRL_CMD] = 0U;

    /* 设备信息（device_id 由 UID 计算，保存在全局状态里） */
    s_regs[APP_MB_REG_DEV_ID_LO]  = (USHORT)(app_main_state.device_id & 0xFFFFU);
    s_regs[APP_MB_REG_DEV_ID_HI]  = (USHORT)((app_main_state.device_id >> 16) & 0xFFFFU);
    s_regs[APP_MB_REG_FW_VERSION] = APP_MB_FW_VERSION;

    /* 通信参数（改后重启生效，读回的是当前生效值） */
    s_regs[APP_MB_REG_SLAVE_ADDR] = (USHORT)App_CommGetSlaveAddr();
    s_regs[APP_MB_REG_BAUD_CODE]  = App_CommGetBaudCode();

    /* 累计运行时间 32 位拆两个寄存器 */
    s_regs[APP_MB_REG_TOTAL_RUN_LO] = (USHORT)(totalRun & 0xFFFFU);
    s_regs[APP_MB_REG_TOTAL_RUN_HI] = (USHORT)((totalRun >> 16) & 0xFFFFU);

    /* PID 增益只读镜像（存储里本就是 ×1000 定点，直接映射） */
    s_regs[APP_MB_REG_PID_MOTOR_KP] = App_ModbusClampI16(p->motorPidKp);
    s_regs[APP_MB_REG_PID_MOTOR_KI] = App_ModbusClampI16(p->motorPidKi);
    s_regs[APP_MB_REG_PID_MOTOR_KD] = App_ModbusClampI16(p->motorPidKd);
    s_regs[APP_MB_REG_PID_HEAT_KP]  = App_ModbusClampI16(p->heatPidKp);
    s_regs[APP_MB_REG_PID_HEAT_KI]  = App_ModbusClampI16(p->heatPidKi);
    s_regs[APP_MB_REG_PID_HEAT_KD]  = App_ModbusClampI16(p->heatPidKd);

    /* 面板焦点项 / 铝板限温上限：蓝牙上位机可读回，用于让 App 的当前调节项
     * 与 OLED 上的 ">" 箭头保持一致（v1.1 新增） */
    s_regs[APP_MB_REG_UI_FOCUS]    = (USHORT)Menu_UiGetFocus();
    s_regs[APP_MB_REG_PLATE_LIMIT] = (USHORT)Menu_UiGetPlate();

    /* 0x0016~0x0017、0x001E~0x001F：保留，读 0 */
    s_regs[0x0016U] = 0U;
    s_regs[0x0017U] = 0U;
    s_regs[0x001EU] = 0U;
    s_regs[0x001FU] = 0U;

    /* ---------------- P9：六轴 / 姿态 / 斜坡 / 打滑（0x0020~0x0039） ---------------- */
    s_regs[APP_MB_REG_ACC_X]  = App_ModbusClampI16(imu.accMg[0]);
    s_regs[APP_MB_REG_ACC_Y]  = App_ModbusClampI16(imu.accMg[1]);
    s_regs[APP_MB_REG_ACC_Z]  = App_ModbusClampI16(imu.accMg[2]);
    s_regs[APP_MB_REG_GYRO_X] = App_ModbusClampI16(imu.gyroDps10[0]);
    s_regs[APP_MB_REG_GYRO_Y] = App_ModbusClampI16(imu.gyroDps10[1]);
    s_regs[APP_MB_REG_GYRO_Z] = App_ModbusClampI16(imu.gyroDps10[2]);
    s_regs[APP_MB_REG_ROLL]   = App_ModbusClampI16(imu.rollC10);
    s_regs[APP_MB_REG_PITCH]  = App_ModbusClampI16(imu.pitchC10);
    s_regs[APP_MB_REG_YAW]    = App_ModbusClampI16(imu.yawC10);
    s_regs[APP_MB_REG_IMU_TEMP] = App_ModbusClampI16(imu.tempC10);
    s_regs[APP_MB_REG_IMU_STATE] = (USHORT)imu.state;
    s_regs[APP_MB_REG_VIB_RMS]   = (USHORT)imu.vibRmsMg;
    s_regs[APP_MB_REG_ANG_ACCEL] = App_ModbusClampI16(imu.alphaDps10);
    s_regs[APP_MB_REG_RAMP_STATE]   = (USHORT)App_MotorGetRampState();
    s_regs[APP_MB_REG_RAMP_CMD_RPM] = App_ModbusClampI16(App_MotorGetCmdRpm());
    s_regs[APP_MB_REG_SLIP_ERR]     = App_ModbusClampI16(imu.slipErrX10);

    /* 0x0030 控制字为“写后清零”语义，读回恒 0 */
    s_regs[APP_MB_REG_IMU_CTRL] = 0U;

    /* 配置回读：低字节量程编码、高字节 ODR 编码 */
    s_regs[APP_MB_REG_ACC_CONFIG]  = (USHORT)(((USHORT)ip->odrCode << 8) | (USHORT)ip->accFs);
    s_regs[APP_MB_REG_GYRO_CONFIG] = (USHORT)(((USHORT)ip->odrCode << 8) | (USHORT)ip->gyroFs);
    s_regs[APP_MB_REG_ALPHA_LIMIT] = (USHORT)ip->alphaLimitRpmS;
    s_regs[APP_MB_REG_SLIP_TH]     = (USHORT)ip->slipThX10;
    s_regs[APP_MB_REG_VIB_TH]      = (USHORT)ip->vibThMg;
    s_regs[APP_MB_REG_TILT_TH]     = (USHORT)ip->tiltThC10;

    /* M3：上次加减速实测耗时（ms，16 位饱和） */
    s_regs[APP_MB_REG_ACC_TIME_MS] = (USHORT)((App_MotorGetAccTimeMs() > 0xFFFFU) ? 0xFFFFU : App_MotorGetAccTimeMs());
    s_regs[APP_MB_REG_DEC_TIME_MS] = (USHORT)((App_MotorGetDecTimeMs() > 0xFFFFU) ? 0xFFFFU : App_MotorGetDecTimeMs());
    s_regs[APP_MB_REG_PEAK_ALPHA]  = App_ModbusClampI16(App_ImuGetPeakAlpha());

    /* ---------------- v1.3（A2/A4）：现场可调参数回读 0x003A~0x003C ---------------- */
    s_regs[APP_MB_REG_HEAT_OC]      = (USHORT)App_HeatGetOverCurrentThreshold();
    s_regs[APP_MB_REG_MOTOR_ASSIST] = (USHORT)((App_MotorGetStallAssistEnable() != 0U)
                                               ? APP_MB_ASSIST_STALL_EN : 0U);
    s_regs[APP_MB_REG_TUNE_CTRL]    = App_TuneGetCtrl();

    /* 0x003D~0x003F：保留，读 0 */
    s_regs[0x003DU] = 0U;
    s_regs[0x003EU] = 0U;
    s_regs[0x003FU] = 0U;
}

/* ---------------- FreeModbus 回调 ---------------- */

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs, eMBRegisterMode eMode)
{
    USHORT i;

    if (pucRegBuffer == NULL)
    {
        return MB_EINVAL;
    }

    if ((usAddress >= APP_MB_REG_HOLDING_NREGS) ||
        (((uint32_t)usAddress + (uint32_t)usNRegs) > (uint32_t)APP_MB_REG_HOLDING_NREGS))
    {
        return MB_ENOREG;
    }

    if (eMode == MB_REG_READ)
    {
        App_ModbusSyncRegs();

        for (i = 0U; i < usNRegs; i++)
        {
            USHORT value = s_regs[usAddress + i];

            *pucRegBuffer++ = (UCHAR)(value >> 8);
            *pucRegBuffer++ = (UCHAR)(value & 0xFFU);
        }

        return MB_ENOERR;
    }

    /* 写：逐个寄存器校验并下发（校验失败即整帧失败） */
    for (i = 0U; i < usNRegs; i++)
    {
        USHORT value = (USHORT)(((USHORT)pucRegBuffer[0] << 8) | (USHORT)pucRegBuffer[1]);
        App_CommResult_t result;

        result = App_CommWriteReg((uint16_t)(usAddress + i), value);
        pucRegBuffer += 2;

        if (result == APP_COMM_ERR_ADDR)
        {
            return MB_ENOREG;
        }
        if (result == APP_COMM_ERR_BUSY)
        {
            return MB_ETIMEDOUT;
        }
        if (result != APP_COMM_OK)
        {
            return MB_EINVAL;
        }
    }

    return MB_ENOERR;
}

/* ---------------- 启动与轮询 ---------------- */

uint8_t App_ModbusStart(uint16_t addr, uint32_t baud)
{
    eMBErrorCode status;

    status = eMBInit(MB_RTU, (UCHAR)addr, 0U, baud, MB_PAR_NONE, 1U);
    if (status != MB_ENOERR)
    {
        printf("[P6] eMBInit FAIL (%d)\r\n", (int)status);
        return 0U;
    }

    status = eMBEnable();
    if (status != MB_ENOERR)
    {
        printf("[P6] eMBEnable FAIL (%d)\r\n", (int)status);
        return 0U;
    }

    return 1U;
}

void App_ModbusPoll(void)
{
    (void)eMBPoll();
}
