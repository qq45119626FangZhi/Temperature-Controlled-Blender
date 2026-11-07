package com.tempstir.blecontrol.data;

/**
 * TempStir 下位机蓝牙通道协议：BLE 透传 Modbus RTU 帧。
 *
 * 依据《TempStir/MDK-ARM/Doc/通信协议.md》13-B / 13-C：
 *   - 从机地址 1，功能码 03 / 06 / 10，异常码 02 / 04 / 06
 *   - CRC16/MODBUS，低字节在前
 *   - 大端 16bit 寄存器
 *
 * 本类只负责组帧、校验与十六进制显示，不含任何 Android / BLE 依赖。
 */
public final class ModbusProtocol {

    public static final int SLAVE_ADDR = 0x01;

    /* 保持寄存器地址（与 app_modbus.h 一致） */
    public static final int REG_CUR_TEMP     = 0x0000; // 当前液温    int16 / 0.1℃  R
    public static final int REG_TARGET_TEMP  = 0x0001; // 目标液温    int16 / 0.1℃  R/W
    public static final int REG_CUR_SPEED    = 0x0002; // 当前转速    int16 / RPM    R
    public static final int REG_TARGET_SPEED = 0x0003; // 目标转速    int16 / RPM    R/W
    public static final int REG_REMAIN_TIME  = 0x0004; // 剩余时间    uint16 / s     R
    public static final int REG_SYS_STATE    = 0x0005; // 系统状态字               R
    public static final int REG_FAULT_CODE   = 0x0006; // 故障码                   R
    public static final int REG_CTRL_CMD     = 0x0007; // 控制命令字               R/W
    public static final int REG_PLATE_TEMP   = 0x0008; // 铝板温度    int16 / 0.1℃  R
    public static final int REG_HEAT_PCT     = 0x0009; // 加热占空比  %            R
    public static final int REG_MOTOR_PCT    = 0x000A; // 电机占空比  %            R
    public static final int REG_SET_TIME     = 0x000B; // 设定时间    uint16 / s     R/W
    public static final int REG_FAN_STATE    = 0x000C; // 风扇状态                 R
    public static final int REG_DEV_ID_LO    = 0x000D; // 设备 ID 低 16 位         R
    public static final int REG_DEV_ID_HI    = 0x000E; // 设备 ID 高 16 位         R
    public static final int REG_FW_VERSION   = 0x000F; // 固件版本                 R
    public static final int REG_UI_FOCUS     = 0x0014; // 面板焦点项 0~3           R/W  (FW>=1.1)
    public static final int REG_PLATE_LIMIT  = 0x0015; // 铝板上限温度 ℃           R/W  (FW>=1.1)

    /* ---- P9（FW>=1.2）：六轴 / 姿态 / 斜坡 / 打滑区块 0x0020~0x0039 ---- */
    public static final int REG_ACC_X        = 0x0020; // 加速度 X    int16 mg      R
    public static final int REG_ACC_Y        = 0x0021;
    public static final int REG_ACC_Z        = 0x0022; // 静止 ≈ +1000 mg
    public static final int REG_GYRO_X       = 0x0023; // 角速度 X    int16 0.1dps  R（已扣零偏）
    public static final int REG_GYRO_Y       = 0x0024;
    public static final int REG_GYRO_Z       = 0x0025;
    public static final int REG_ROLL         = 0x0026; // 横滚角      int16 0.1°    R
    public static final int REG_PITCH        = 0x0027; // 俯仰角      int16 0.1°    R
    public static final int REG_YAW          = 0x0028; // 相对偏航    int16 0.1°    R（会漂移）
    public static final int REG_IMU_TEMP     = 0x0029; // 片内温度    int16 0.1℃    R
    public static final int REG_IMU_STATE    = 0x002A; // IMU 状态位                R
    public static final int REG_VIB_RMS      = 0x002B; // 振动 RMS    uint16 mg     R
    public static final int REG_ANG_ACCEL    = 0x002C; // 角加速度    int16 0.1dps/s R
    public static final int REG_RAMP_STATE   = 0x002D; // 斜坡状态 0停1加2匀3减     R
    public static final int REG_RAMP_CMD_RPM = 0x002E; // 斜坡指令转速 int16 rpm     R
    public static final int REG_SLIP_ERR     = 0x002F; // 打滑偏差    int16 0.1%    R
    public static final int REG_IMU_CTRL     = 0x0030; // IMU 控制字  bit1标定/bit2存/bit3清 R/W
    public static final int REG_ACC_CONFIG   = 0x0031; // 低字节量程 高字节ODR（重启生效） R/W
    public static final int REG_GYRO_CONFIG  = 0x0032;
    public static final int REG_ALPHA_LIMIT  = 0x0033; // 加减速斜率  uint16 rpm/s   R/W
    public static final int REG_SLIP_TH      = 0x0034; // 打滑门限    uint16 0.1%    R/W
    public static final int REG_VIB_TH       = 0x0035; // 振动门限    uint16 mg      R/W
    public static final int REG_TILT_TH      = 0x0036; // 倾斜门限    uint16 0.1°    R/W
    public static final int REG_ACC_TIME_MS  = 0x0037; // 上次加速耗时 uint16 ms     R
    public static final int REG_DEC_TIME_MS  = 0x0038; // 上次减速耗时 uint16 ms     R
    public static final int REG_PEAK_ALPHA   = 0x0039; // 峰值角加速度 int16 0.1dps/s R

    /* ---- A4/A2（FW>=1.3）：现场可调参数与实测统计 0x003A~0x003C ---- */
    public static final int REG_HEAT_OC      = 0x003A; // 加热过流阈值 ADC raw 100~4095  R/W
    public static final int REG_MOTOR_ASSIST = 0x003B; // bit0 振动辅助堵转  R/W
    public static final int REG_TUNE_CTRL    = 0x003C; // bit0 周期打印 / bit1 立即打印 / bit2 清 R/W

    /* 堵转辅助位（0x003B） */
    public static final int ASSIST_STALL_EN  = 0x0001;
    /* 实测统计控制位（0x003C） */
    public static final int TUNECTL_PRINT    = 0x0001;
    public static final int TUNECTL_ONCE     = 0x0002;
    public static final int TUNECTL_CLEAR    = 0x0004;

    /* 固件版本门限：>= 1.1 才有 0x0014/0x0015（面板焦点 / 铝板限温可远程读写） */
    public static final int FW_VER_REMOTE_FOCUS = 0x0101;
    /* >= 1.2 才有 0x0020~0x0039（P9 六轴区块） */
    public static final int FW_VER_IMU = 0x0102;
    /* >= 1.3 才有 0x003A~0x003C（加热过流阈值 / 堵转辅助 / 实测统计） */
    public static final int FW_VER_TUNE = 0x0103;

    /* 面板焦点项序号（与 menu_ui.c 的 KEY1 循环顺序一致） */
    public static final int FOCUS_SPEED = 0;
    public static final int FOCUS_PLATE = 1;
    public static final int FOCUS_LIQUID = 2;
    public static final int FOCUS_TIME = 3;

    /* 系统状态字 bit（0x0005） */
    public static final int ST_MOTOR_RUNNING = 0x0001;
    public static final int ST_HEATING       = 0x0002;
    public static final int ST_FAN_ON        = 0x0004;
    public static final int ST_FAULT         = 0x0008;
    public static final int ST_TIMING        = 0x0010;
    public static final int ST_RTC_VALID     = 0x0020;
    public static final int ST_PANEL_ACTIVE  = 0x0040;
    public static final int ST_BLE_ONLINE    = 0x0100;
    public static final int ST_IMU_VALID     = 0x0400; // P9：六轴数据有效
    public static final int ST_SLIP          = 0x0800; // P9：打滑告警

    /* P9：IMU 状态位（0x002A，与 app_imu.h 的 APP_IMU_ST_* 一致） */
    public static final int IMUST_VALID  = 0x0001;
    public static final int IMUST_STILL  = 0x0002;
    public static final int IMUST_ROTATE = 0x0004;
    public static final int IMUST_VIB    = 0x0008;
    public static final int IMUST_TILT   = 0x0010;
    public static final int IMUST_CALIB  = 0x0040;
    public static final int IMUST_SLIP   = 0x0080;
    public static final int IMUST_TILT_LOCK = 0x0100; // 倾斜保护已锁存（已停机 + 长鸣）

    /* P9：IMU 控制字（0x0030） */
    public static final int IMUCTL_CAL_BIAS  = 0x0002;
    public static final int IMUCTL_SAVE      = 0x0004;
    public static final int IMUCTL_CLEAR_EVT = 0x0008;

    /* 故障码 bit：P9 IMU 通信异常 */
    public static final int FLT_IMU = 0x0100;

    /* 控制命令字 bit（0x0007），写后自动清零 */
    public static final int CMD_START        = 0x0001;
    public static final int CMD_STOP         = 0x0002;
    public static final int CMD_CLEAR_FAULT  = 0x0004;
    public static final int CMD_SAVE_PARAM   = 0x0008;
    public static final int CMD_LOAD_DEFAULT = 0x0010;

    /* 参数上限（与 menu_ui.h 一致） */
    public static final int SPEED_MAX_RPM   = 500;
    public static final int LIQUID_MAX_C10  = 1000; // 100.0℃
    public static final int TIME_MAX_SEC    = 999;

    private static final int FC_READ_HOLDING  = 0x03;
    private static final int FC_WRITE_SINGLE  = 0x06;
    private static final int FC_WRITE_MULTI   = 0x10;

    private ModbusProtocol() {
    }

    /** 03：读保持寄存器。 */
    public static byte[] buildRead(int startReg, int count) {
        byte[] f = new byte[8];
        f[0] = (byte) SLAVE_ADDR;
        f[1] = (byte) FC_READ_HOLDING;
        f[2] = (byte) ((startReg >> 8) & 0xFF);
        f[3] = (byte) (startReg & 0xFF);
        f[4] = (byte) ((count >> 8) & 0xFF);
        f[5] = (byte) (count & 0xFF);
        appendCrc(f, 6);
        return f;
    }

    /** 06：写单个保持寄存器。 */
    public static byte[] buildWriteSingle(int reg, int value) {
        byte[] f = new byte[8];
        f[0] = (byte) SLAVE_ADDR;
        f[1] = (byte) FC_WRITE_SINGLE;
        f[2] = (byte) ((reg >> 8) & 0xFF);
        f[3] = (byte) (reg & 0xFF);
        f[4] = (byte) ((value >> 8) & 0xFF);
        f[5] = (byte) (value & 0xFF);
        appendCrc(f, 6);
        return f;
    }

    /** 10：写多个保持寄存器（保留给批量下发）。 */
    public static byte[] buildWriteMultiple(int startReg, int[] values) {
        int n = values.length;
        byte[] f = new byte[7 + n * 2 + 2];
        f[0] = (byte) SLAVE_ADDR;
        f[1] = (byte) FC_WRITE_MULTI;
        f[2] = (byte) ((startReg >> 8) & 0xFF);
        f[3] = (byte) (startReg & 0xFF);
        f[4] = (byte) ((n >> 8) & 0xFF);
        f[5] = (byte) (n & 0xFF);
        f[6] = (byte) (n * 2);
        int p = 7;
        for (int v : values) {
            f[p++] = (byte) ((v >> 8) & 0xFF);
            f[p++] = (byte) (v & 0xFF);
        }
        appendCrc(f, p);
        return f;
    }

    private static void appendCrc(byte[] frame, int dataLen) {
        int crc = crc16(frame, dataLen);
        frame[dataLen] = (byte) (crc & 0xFF);         // CRC 低字节在前
        frame[dataLen + 1] = (byte) ((crc >> 8) & 0xFF);
    }

    /** CRC16/MODBUS：初值 0xFFFF，多项式 0xA001，低位先处理。 */
    public static int crc16(byte[] data, int len) {
        int crc = 0xFFFF;
        for (int i = 0; i < len; i++) {
            crc ^= (data[i] & 0xFF);
            for (int b = 0; b < 8; b++) {
                if ((crc & 0x0001) != 0) {
                    crc = (crc >>> 1) ^ 0xA001;
                } else {
                    crc >>>= 1;
                }
            }
        }
        return crc & 0xFFFF;
    }

    public static boolean crcOk(byte[] frame) {
        if (frame == null || frame.length < 5) {
            return false;
        }
        int n = frame.length - 2;
        int crc = crc16(frame, n);
        return (crc & 0xFF) == (frame[n] & 0xFF)
                && ((crc >> 8) & 0xFF) == (frame[n + 1] & 0xFF);
    }

    /**
     * 依据已收到的字节推断完整 RTU 应答帧长度。
     * 返回 0 = 数据还不够；返回 -1 = 无法识别（应清空缓冲重新同步）。
     */
    public static int expectedFrameLength(byte[] buf, int len) {
        if (len < 3) {
            return 0;
        }
        if ((buf[0] & 0xFF) != SLAVE_ADDR) {
            return -1; // 站号不符（例如模块透传的文本串），重新同步
        }
        int fc = buf[1] & 0xFF;
        if ((fc & 0x80) != 0) {
            return 5;                        // 异常响应：addr fc|80 code crc crc
        }
        switch (fc) {
            case FC_READ_HOLDING:
                if (len < 4) {
                    return 0;
                }
                return 3 + (buf[2] & 0xFF) + 2; // addr fc count + data + crc
            case FC_WRITE_SINGLE:
            case FC_WRITE_MULTI:
                return 8;
            default:
                return -1;
        }
    }

    /** 从 03 应答中取出大端 16bit 寄存器数组；非法帧返回 null。 */
    public static int[] parseReadValues(byte[] frame) {
        if (frame == null || frame.length < 5 || (frame[1] & 0xFF) != FC_READ_HOLDING) {
            return null;
        }
        int byteCount = frame[2] & 0xFF;
        if (byteCount < 2 || (byteCount & 1) != 0 || frame.length != 3 + byteCount + 2) {
            return null;
        }
        int[] v = new int[byteCount / 2];
        for (int i = 0; i < v.length; i++) {
            int hi = frame[3 + i * 2] & 0xFF;
            int lo = frame[4 + i * 2] & 0xFF;
            v[i] = (hi << 8) | lo;
        }
        return v;
    }

    /** 16 位寄存器视为有符号补码。 */
    public static int signed16(int raw) {
        return (short) raw;
    }

    public static String exceptionText(int code) {
        switch (code) {
            case 0x02: return "02 非法地址 / 只读寄存器";
            case 0x04: return "04 数值越界或保护联锁拒绝";
            case 0x06: return "06 面板按键占用中（按键后 8 秒），请稍候重试";
            default:   return String.format("0x%02X 从机异常", code);
        }
    }

    public static String hex(byte[] data) {
        if (data == null) {
            return "";
        }
        StringBuilder sb = new StringBuilder(data.length * 3);
        for (byte b : data) {
            sb.append(String.format("%02X ", b & 0xFF));
        }
        return sb.toString().trim();
    }
}
