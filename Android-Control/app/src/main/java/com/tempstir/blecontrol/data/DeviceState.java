package com.tempstir.blecontrol.data;

/**
 * 下位机寄存器快照。所有数值保持寄存器原始量纲：
 *   温度 0.1℃（int16 补码）、转速 RPM、时间 s、百分比 %。
 */
public class DeviceState {

    public int curTempC10;      // 0x0000 液温实测
    public int targetTempC10;   // 0x0001 液温设定
    public int curRpm;          // 0x0002 转速实测
    public int targetRpm;       // 0x0003 转速设定
    public int remainSec;       // 0x0004 剩余时间
    public int sysState;        // 0x0005 状态字
    public int faultCode;       // 0x0006 故障码
    public int plateTempC10;    // 0x0008 铝板实测
    public int heatPct;         // 0x0009 加热占空比
    public int motorPct;        // 0x000A 电机占空比
    public int setTimeSec;      // 0x000B 设定时间
    public int fanState;        // 0x000C 风扇
    public int devIdLo;         // 0x000D
    public int devIdHi;         // 0x000E
    public int fwVersion;       // 0x000F
    public int uiFocus;         // 0x0014 面板焦点项 0~3（FW>=1.1）
    public int plateLimit;      // 0x0015 铝板上限温度 ℃（FW>=1.1）

    /* ---- P9（FW>=1.2）六轴 / 姿态 / 斜坡 / 打滑 ---- */
    public int accX, accY, accZ;        // 0x0020~22 mg
    public int gyroX, gyroY, gyroZ;     // 0x0023~25 0.1dps（已扣零偏）
    public int rollC10, pitchC10, yawC10; // 0x0026~28 0.1°
    public int imuTempC10;              // 0x0029 0.1℃
    public int imuState;                // 0x002A 位域
    public int vibRms;                  // 0x002B mg
    public int angAccel;                // 0x002C 0.1dps/s
    public int rampState;               // 0x002D 0停 1加 2匀 3减
    public int rampCmdRpm;              // 0x002E rpm
    public int slipErr;                 // 0x002F 0.1%
    public int alphaLimit;              // 0x0033 rpm/s
    public int slipTh;                  // 0x0034 0.1%
    public int vibTh;                   // 0x0035 mg
    public int tiltTh;                  // 0x0036 0.1°
    public int accTimeMs;               // 0x0037 ms
    public int decTimeMs;               // 0x0038 ms
    public int peakAlpha;               // 0x0039 0.1dps/s

    /* ---- A4/A2（FW>=1.3）现场可调参数 ---- */
    public int heatOcRaw;               // 0x003A 加热过流阈值 ADC raw
    public int motorAssist;             // 0x003B bit0 振动辅助堵转
    public int tuneCtrl;                // 0x003C bit0 实测统计打印开关
    public boolean paramEverRead;       // 是否收到过 0x003A~0x003C

    public boolean imuEverRead;         // 是否收到过 IMU 区块

    public long lastUpdateMs;
    public boolean everRead;

    /** 把一次 03 应答写进快照。startReg 为本次读取的起始地址。 */
    public void applyRead(int startReg, int[] v) {
        if (v == null) {
            return;
        }
        for (int i = 0; i < v.length; i++) {
            int addr = startReg + i;
            int raw = v[i] & 0xFFFF;
            switch (addr) {
                case ModbusProtocol.REG_CUR_TEMP:     curTempC10 = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_TARGET_TEMP:  targetTempC10 = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_CUR_SPEED:    curRpm = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_TARGET_SPEED: targetRpm = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_REMAIN_TIME:  remainSec = raw; break;
                case ModbusProtocol.REG_SYS_STATE:    sysState = raw; break;
                case ModbusProtocol.REG_FAULT_CODE:   faultCode = raw; break;
                case ModbusProtocol.REG_PLATE_TEMP:   plateTempC10 = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_HEAT_PCT:     heatPct = raw; break;
                case ModbusProtocol.REG_MOTOR_PCT:    motorPct = raw; break;
                case ModbusProtocol.REG_SET_TIME:     setTimeSec = raw; break;
                case ModbusProtocol.REG_FAN_STATE:    fanState = raw; break;
                case ModbusProtocol.REG_DEV_ID_LO:    devIdLo = raw; break;
                case ModbusProtocol.REG_DEV_ID_HI:    devIdHi = raw; break;
                case ModbusProtocol.REG_FW_VERSION:   fwVersion = raw; break;
                case ModbusProtocol.REG_UI_FOCUS:     uiFocus = raw; break;
                case ModbusProtocol.REG_PLATE_LIMIT:  plateLimit = raw; break;

                /* ---- P9：六轴 / 姿态 / 斜坡 / 打滑 ---- */
                case ModbusProtocol.REG_ACC_X:        accX = ModbusProtocol.signed16(raw); imuEverRead = true; break;
                case ModbusProtocol.REG_ACC_Y:        accY = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_ACC_Z:        accZ = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_GYRO_X:       gyroX = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_GYRO_Y:       gyroY = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_GYRO_Z:       gyroZ = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_ROLL:         rollC10 = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_PITCH:        pitchC10 = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_YAW:          yawC10 = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_IMU_TEMP:     imuTempC10 = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_IMU_STATE:    imuState = raw; break;
                case ModbusProtocol.REG_VIB_RMS:      vibRms = raw; break;
                case ModbusProtocol.REG_ANG_ACCEL:    angAccel = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_RAMP_STATE:   rampState = raw; break;
                case ModbusProtocol.REG_RAMP_CMD_RPM: rampCmdRpm = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_SLIP_ERR:     slipErr = ModbusProtocol.signed16(raw); break;
                case ModbusProtocol.REG_ALPHA_LIMIT:  alphaLimit = raw; break;
                case ModbusProtocol.REG_SLIP_TH:      slipTh = raw; break;
                case ModbusProtocol.REG_VIB_TH:       vibTh = raw; break;
                case ModbusProtocol.REG_TILT_TH:      tiltTh = raw; break;
                case ModbusProtocol.REG_ACC_TIME_MS:  accTimeMs = raw; break;
                case ModbusProtocol.REG_DEC_TIME_MS:  decTimeMs = raw; break;
                case ModbusProtocol.REG_PEAK_ALPHA:   peakAlpha = ModbusProtocol.signed16(raw); break;
                /* ---- A4/A2：现场可调参数（FW>=1.3） ---- */
                case ModbusProtocol.REG_HEAT_OC:      heatOcRaw = raw; paramEverRead = true; break;
                case ModbusProtocol.REG_MOTOR_ASSIST: motorAssist = raw; paramEverRead = true; break;
                case ModbusProtocol.REG_TUNE_CTRL:    tuneCtrl = raw; paramEverRead = true; break;
                default: break; // 0x0007 命令字读回恒 0，不需要保存
            }
        }
        lastUpdateMs = System.currentTimeMillis();
        everRead = true;
    }

    public boolean isRunning() {
        return (sysState & ModbusProtocol.ST_MOTOR_RUNNING) != 0;
    }

    public static String tempText(int c10) {
        if (c10 <= -3276) { // 接近 -327.6℃，视为传感器异常哨兵值
            return "异常";
        }
        return String.format(java.util.Locale.CHINA, "%.1f ℃", c10 / 10.0f);
    }

    public String statusText() {
        StringBuilder sb = new StringBuilder();
        if ((sysState & ModbusProtocol.ST_MOTOR_RUNNING) != 0) append(sb, "电机运行");
        if ((sysState & ModbusProtocol.ST_HEATING) != 0) append(sb, "加热中");
        if ((sysState & ModbusProtocol.ST_FAN_ON) != 0) append(sb, "风扇转");
        // 只剩 RTC 掉电这一个故障位时不算"故障"，首页不体现（见 FAULT_HIDE_RTC_POWER）
        if ((sysState & ModbusProtocol.ST_FAULT) != 0 && displayFaultCode() != 0) append(sb, "故障");
        if ((sysState & ModbusProtocol.ST_TIMING) != 0) append(sb, "计时中");
        if ((sysState & ModbusProtocol.ST_BLE_ONLINE) != 0) append(sb, "BLE在线");
        if ((sysState & ModbusProtocol.ST_PANEL_ACTIVE) != 0) append(sb, "面板占用");
        return sb.length() == 0 ? "空闲" : sb.toString();
    }

    private static void append(StringBuilder sb, String s) {
        if (sb.length() > 0) {
            sb.append(" · ");
        }
        sb.append(s);
    }

    /**
     * 首页不展示的故障位：RTC 掉电（0x0080）。
     * 这是下位机时钟电池掉电，与上位机控制无关，首页不再提示。
     * 注意：只影响"显示"，{@link #faultCode} 原始值、启动联锁、协议应答判断都不变。
     */
    public static final int FAULT_HIDE_RTC_POWER = 0x0080;

    /** 界面显示用的故障码：屏蔽掉首页不展示的位（RTC 掉电）。 */
    public int displayFaultCode() {
        return faultCode & ~FAULT_HIDE_RTC_POWER;
    }

    public String faultText() {
        int code = displayFaultCode();          // 用显示码，RTC 掉电位不会出现在文字里
        if (code == 0) {
            return "无";
        }
        StringBuilder sb = new StringBuilder();
        if ((code & 0x0001) != 0) append(sb, "加热硬件故障");
        if ((code & 0x0002) != 0) append(sb, "加热过流");
        if ((code & 0x0004) != 0) append(sb, "液温NTC异常");
        if ((code & 0x0008) != 0) append(sb, "铝板NTC异常");
        if ((code & 0x0010) != 0) append(sb, "铝板限温");
        if ((code & 0x0020) != 0) append(sb, "电机堵转");
        if ((code & 0x0040) != 0) append(sb, "EEPROM异常");
        if ((code & ModbusProtocol.FLT_IMU) != 0) append(sb, "IMU通信异常");
        return sb.toString();
    }

    /** P9：IMU 状态位文字（0x002A）。 */
    public String imuStateText() {
        StringBuilder sb = new StringBuilder();
        if ((imuState & ModbusProtocol.IMUST_VALID) != 0) append(sb, "有效");
        if ((imuState & ModbusProtocol.IMUST_CALIB) != 0) append(sb, "已标定");
        if ((imuState & ModbusProtocol.IMUST_STILL) != 0) append(sb, "静止");
        if ((imuState & ModbusProtocol.IMUST_ROTATE) != 0) append(sb, "转动");
        if ((imuState & ModbusProtocol.IMUST_VIB) != 0) append(sb, "振动超限");
        if ((imuState & ModbusProtocol.IMUST_TILT) != 0) append(sb, "倾斜");
        if ((imuState & ModbusProtocol.IMUST_SLIP) != 0) append(sb, "打滑");
        if ((imuState & ModbusProtocol.IMUST_TILT_LOCK) != 0) append(sb, "倾斜保护(已停机+长鸣)");
        return sb.length() == 0 ? "无数据" : sb.toString();
    }

    /** P9：斜坡状态文字（0x002D）。 */
    public String rampStateText() {
        switch (rampState) {
            case 0: return "停机";
            case 1: return "加速中";
            case 2: return "匀速";
            case 3: return "减速中";
            default: return "--";
        }
    }

    public String deviceIdText() {
        return String.format("ID %04X%04X / FW %d.%d",
                devIdHi & 0xFFFF, devIdLo & 0xFFFF,
                (fwVersion >> 8) & 0xFF, fwVersion & 0xFF);
    }
}
