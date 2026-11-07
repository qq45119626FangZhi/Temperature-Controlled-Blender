/*
 * @file    app_imu.h
 * @brief   P9：LSM6DSM 六轴采集 + 姿态判据（App 层）。
 *
 * 分层：app_imu（本文件，业务/判据/日志） -> dri_lsm6dsm（Dri 层，器件寄存器级）
 *       -> Int_Spi*（Int 层，SPI2 + CS=PB12）。
 *
 * 单位约定（除姿态角外全部整数定点，无浮点输出）：
 *   加速度  accMg[3]      mg         （±2g 档：mg = raw * 61 / 1000）
 *   角速度  gyroDps10[3]  0.1 dps    （±245dps 档；已扣上电零偏）
 *   温度    tempC10       0.1 ℃      （256 LSB/℃、0 LSB = 25℃）
 *   姿态    rollC10 等    0.1 °      （roll/pitch 由加速度解算，yaw 为陀螺积分相对角，会漂移）
 *   角加速  alphaDps10    0.1 dps/s  （陀螺微分 + 一阶低通）
 *   振动    vibRmsMg      mg         （加速度合矢量偏离 1g 的 RMS，1s 窗）
 *
 * 安装约定：本板随搅拌轴刚性转动（P9 决策），打滑判定的旋转轴由
 *          APP_IMU_ROT_AXIS 指定（0=X 1=Y 2=Z，默认 Z）。
 *
 * 独立性：IMU 全程只读旁路；未贴件/无效时各判据自动降级（state 位清零），
 *         电机/加热/UI/通信行为不受影响。
 */
#ifndef APP_IMU_H
#define APP_IMU_H

#include <stdint.h>

/* ---------------- 开关（每个功能独立，可单独回退） ---------------- */
#define APP_IMU_TASK_ENABLE        1U   /* 1 = 创建 IMU 采集任务 */
#define APP_IMU_ATTITUDE_ENABLE    1U   /* 1 = 解算 roll/pitch/yaw（20Hz 更新） */
#define APP_IMU_VIB_ENABLE         1U   /* 1 = 计算振动 RMS 与振动判据 */
/* 2026-11-06 现场确认：IMU 固定在底座、不随搅拌轴转动，陀螺无法感知转速，
 * ω_imu 恒 0 → 打滑误差恒为 100%，属必然假告警（状态位 bit7 / 状态字 bit11 常亮）。
 * 故默认关闭 M4 打滑判据；若以后改为让 IMU 随轴装配，把本宏改回 1 即恢复（判据代码保留）。 */
#define APP_IMU_SLIP_ENABLE        0U   /* 1 = 陀螺×编码器打滑交叉校验（M4） */
#define APP_IMU_BIAS_AUTOCAL_ENABLE 1U  /* 1 = 上电自动零偏标定 */

/* ---- A8：六轴扩展两项（默认全部关闭，打开后需实机复验，不影响默认行为） ---- */
#define APP_IMU_SELFTEST_ENABLE    0U   /* G1：上电做 CTRL5_C 硬件自检（约 +400ms 启动时间） */
#define APP_IMU_FIFO_ENABLE        0U   /* G7：FIFO 连续采样（每 10ms 排空取最新样本） */
#define APP_IMU_FIFO_WTM           6U   /* G7：FIFO 水位（字），1 组 6 轴样本 = 6 字 */

/* ---------------- 采样配置 ---------------- */
#define APP_IMU_XL_FS         0U    /* ±2 g   */
#define APP_IMU_GY_FS         0U    /* ±245 dps */
#define APP_IMU_ODR_CODE      4U    /* 208 Hz */
#define APP_IMU_PERIOD_MS     10U   /* 100Hz 采集 */

/* 自动零偏标定窗口：上电后前 200 拍（2s）取角速度均值作为零偏 */
#define APP_IMU_BIAS_CAL_N    200U
/* 姿态更新分频：每 5 拍更新一次姿态（20Hz，省 CPU；atan2 为软件浮点） */
#define APP_IMU_ATT_DIV       5U
/* 振动 RMS 窗口：100 拍 = 1s */
#define APP_IMU_VIB_WIN       100U
/* 角加速度一阶低通系数：alphaF += (alpha - alphaF)/8 */
#define APP_IMU_ALPHA_LPF_DIV 8
/* 冲击判据：|alpha| > 2000 dps/s = 20000（0.1dps/s） */
#define APP_IMU_IMPACT_ALPHA  20000

/* 旋转轴（随轴安装约定）：0=X 1=Y 2=Z */
#define APP_IMU_ROT_AXIS      2U

/* 判据阈值默认值（可由 EEPROM / Modbus 覆盖） */
#define APP_IMU_VIB_TH_DEFAULT_MG      300U    /* 振动 RMS 告警门限(mg) */
#define APP_IMU_TILT_TH_DEFAULT_C10    100U    /* 倾斜门限(0.1°) = 10.0°（需求：>10° 报警停机） */
#define APP_IMU_SLIP_TH_DEFAULT_X10    150U    /* 打滑门限 = 15.0% */

/* ---- 倾斜保护（需求：倾角 >10° 蜂鸣器长鸣；电机运转中检测到倾斜立即停机） ----
 * 判定在 IMU 侧完成（加速度低通 + 持续时间 + 回差），执行（停机/鸣叫）在 CTRL 侧，
 * 各自由开关控制，便于单独回退。 */
#define APP_IMU_TILT_PROTECT_ENABLE    1U     /* 1 = 启用倾斜判定（置 0 完全恢复旧行为） */
#define APP_IMU_TILT_HYST_C10          30U    /* 解除回差 3.0°（低于 门限-回差 才解除） */
#define APP_IMU_TILT_LATCH_MS          500U   /* 倾角持续超限 500ms 才确认（滤振动瞬态） */
#define APP_IMU_TILT_CLEAR_MS          500U   /* 回到安全角持续 500ms 才解除 */
#define APP_IMU_ACC_LPF_DIV            16     /* 姿态用加速度一阶低通：lpf += (raw-lpf)/16 */
#define APP_IMU_ALPHA_LIMIT_DEFAULT    600U    /* 加减速斜率(rpm/s) */
#define APP_IMU_SLIP_MIN_RPM           50      /* 转速低于此值不判打滑 */
#define APP_IMU_SLIP_LATCH_MS          1000U   /* 偏差持续 1s 才置打滑 */
#define APP_IMU_SLIP_CLEAR_MS          1000U   /* 偏差消失持续 1s 才清打滑 */
#define APP_IMU_STILL_GYRO_MAX         30      /* |gyro| < 3.0 dps 视为静止 */

/* IMU_STATE（0x002A）位定义 */
#define APP_IMU_ST_VALID       0x0001U  /* bit0 数据有效 */
#define APP_IMU_ST_STILL       0x0002U  /* bit1 静止 */
#define APP_IMU_ST_ROTATE      0x0004U  /* bit2 转动 */
#define APP_IMU_ST_VIB         0x0008U  /* bit3 振动超阈值 */
#define APP_IMU_ST_TILT        0x0010U  /* bit4 倾斜超阈值 */
#define APP_IMU_ST_FIFO        0x0020U  /* bit5 FIFO 溢出（本期未用，恒 0） */
#define APP_IMU_ST_CALIB       0x0040U  /* bit6 零偏已标定 */
#define APP_IMU_ST_SLIP        0x0080U  /* bit7 打滑告警（M4） */
#define APP_IMU_ST_TILT_LOCK   0x0100U  /* bit8 倾斜保护已锁存（已停机 + 蜂鸣器长鸣） */

/* ---------------- 数据结构 ---------------- */

/* 一次换算后的采样快照 */
typedef struct
{
    int16_t  accMg[3];      /* X/Y/Z 加速度(mg) */
    int16_t  gyroDps10[3];  /* X/Y/Z 角速度(0.1dps，已扣零偏) */
    int16_t  tempC10;       /* 片内温度(0.1℃) */
    int16_t  accMagMg;      /* 合矢量 |a|(mg)，静止≈1000 */
    int16_t  rollC10;       /* 横滚角(0.1°) */
    int16_t  pitchC10;      /* 俯仰角(0.1°) */
    int16_t  yawC10;        /* 相对偏航角(0.1°，陀螺积分，会漂移) */
    int16_t  alphaDps10;    /* 旋转轴角加速度(0.1dps/s，滤波后) */
    int16_t  slipErrX10;    /* 打滑偏差(0.1%) */
    uint16_t vibRmsMg;      /* 振动 RMS(mg) */
    uint16_t state;         /* APP_IMU_ST_* 位域 */
    uint32_t seq;           /* 成功采样序号 */
} App_ImuSample_t;

/* 持久化参数（EEPROM 独立块，28 字节，双份 + CRC） */
typedef struct
{
    uint16_t magic;             /* APP_IMU_PARAM_MAGIC */
    uint16_t version;           /* APP_IMU_PARAM_VERSION */
    uint16_t seq;               /* 写入序号，大者为新 */
    uint8_t  enable;            /* 1=IMU 参与判据/寄存器 */
    uint8_t  accFs;             /* ±g 档编码 */
    uint8_t  gyroFs;            /* dps 档编码 */
    uint8_t  odrCode;           /* ODR 编码 */
    int16_t  biasDps10[3];      /* 陀螺零偏(0.1dps) */
    uint16_t alphaLimitRpmS;    /* 加减速斜率(rpm/s) */
    uint16_t slipThX10;         /* 打滑门限(0.1%) */
    uint16_t vibThMg;           /* 振动门限(mg) */
    uint16_t tiltThC10;         /* 倾斜门限(0.1°) */
    uint8_t  slipAssistStallEn; /* 1=振动辅助堵转判定（默认 0） */
    uint8_t  reserved;
    uint16_t crc;               /* 覆盖 crc 之前全部字节 */
} App_Imu_Params_t;

#define APP_IMU_PARAM_MAGIC    0x494DU   /* 'I''M' */
#define APP_IMU_PARAM_VERSION  2U        /* v2：倾斜门限默认值 45°→10°（v1 载入时自动迁移） */

/* 参数独立块地址（M24C02 的 v1 遗留空闲区，主参数槽 0x40/0xA0 不动）：
 *   A 份 0x00~0x1F，B 份 0x20~0x3F，交替写入 + seq/crc 校验。 */
#define APP_IMU_BLOCK_A_ADDR   0x00U
#define APP_IMU_BLOCK_B_ADDR   0x20U

/* ---------------- 接口 ---------------- */

/* 上电自检 + 配置 + 载入 EEPROM 参数（须在 Int_SpiInit() 之后、调度器启动前）。 */
void App_ImuInit(void);

/* 采集任务体：100Hz 采样/判据，上电自检日志一次，运行期静默。 */
void App_ImuTask(void *argument);

/* 1 = 器件在线且已配置 */
uint8_t App_ImuIsPresent(void);

/* 取最近一次快照（临界区拷贝）。1 = 有有效数据。 */
uint8_t App_ImuGetSample(App_ImuSample_t *out);

/* 1 = 通信/自检异常（供故障字 bit8 使用） */
uint8_t App_ImuIsFault(void);

/* 累计读取失败次数 */
uint32_t App_ImuGetErrCount(void);

/* 1 = 当前振动超过门限（供电机堵转辅助 M5 使用） */
uint8_t App_ImuHasVibration(void);

/* 1 = 当前打滑已锁存（M4） */
uint8_t App_ImuIsSlipping(void);

/* 1 = 倾斜超限已确认（倾角持续 > 门限 500ms）。
 * CTRL 任务据此：电机运转则停机 + 蜂鸣器长鸣；回到 门限-3° 且持续 500ms 才解除。 */
uint8_t App_ImuIsTiltBlocked(void);

/* 手动清除倾斜锁存（例如人工确认已放平但角度判据未回落时使用） */
void App_ImuClearTiltBlock(void);

/* 启动一次零偏标定（1~2s 内需保持静止；完成后置 CALIB 位并自动保存） */
void App_ImuStartBiasCal(void);

/* 清打滑锁存与冲击峰值 */
void App_ImuClearEvents(void);

/* 参数读写（EEPROM 独立块 0x00/0x20） */
const App_Imu_Params_t *App_ImuGetParams(void);
void App_ImuSetSlipThX10(uint16_t v);
void App_ImuSetVibThMg(uint16_t v);
void App_ImuSetTiltThC10(uint16_t v);
void App_ImuSetAlphaLimitRpmS(uint16_t v);   /* 同时下发给电机速度斜坡（M1） */
void App_ImuSetSlipAssistStall(uint8_t en);
uint8_t App_ImuSaveParams(void);             /* 1=写入成功 */

/* 本次启动峰值角加速度（0.1dps/s，绝对值） */
int16_t App_ImuGetPeakAlpha(void);

#endif /* APP_IMU_H */
