/*
 * dri_lsm6dsm.h
 * P9（第二版需求）：LSM6DSM 六轴传感器（3 轴加速度 + 3 轴陀螺 + 片上温度）驱动。
 *
 * 分层约定（与工程其它 Utils 驱动一致）：
 *   - 本文件只做“器件寄存器级”访问：识别、软复位、量程/ODR 配置、突发读取原始值；
 *   - 不含任何业务逻辑（滤波/姿态/判据都在 App 层 app_imu.c）；
 *   - 物理链路复用 Int 层 int_spi.c（SPI2，CS=PB12），不直接操作 GPIO。
 *
 * 硬件前提：CS=PB12（软件片选，低有效），SPI2 由 Core 层 MX_SPI2_Init() 初始化
 *          （SCK=PB13/MISO=PB14/MOSI=PB15，APB1 36MHz / 8 ≈ 4.5 MHz，模式 0）。
 * 时序：统一使用 4 线 SPI 模式 0（CPOL=0/CPHA=1Edge），与 int_spi.c 的现有约定一致。
 */
#ifndef DRI_LSM6DSM_H
#define DRI_LSM6DSM_H

#include <stdint.h>

/* ---------------- 关键寄存器地址（实现与排查时对照） ---------------- */
#define LSM6DSM_REG_FIFO_CTRL1      0x06U   /* FIFO 水位低字节 FTH[7:0] */
#define LSM6DSM_REG_FIFO_CTRL2      0x07U   /* FTH[12:8] / TIMER_PEDO_FIFO_EN 等 */
#define LSM6DSM_REG_FIFO_CTRL3      0x08U   /* DEC_FIFO_G[3:0] / DEC_FIFO_XL[3:0]，1=不分频 */
#define LSM6DSM_REG_FIFO_CTRL4      0x09U   /* FIFO_MODE 相关（v1.3/G7 用） */
#define LSM6DSM_REG_FIFO_CTRL5      0x0AU   /* FIFO_ODR[3:0] / FIFO_MODE[2:0] */
#define LSM6DSM_REG_FIFO_STATUS1    0x3AU   /* DIFF_FIFO[7:0]：未读“字”数低字节 */
#define LSM6DSM_REG_FIFO_STATUS2    0x3BU   /* bit7 FIFO_EMPTY / bit6 FIFO_FULL / DIFF_FIFO[11:8] */
#define LSM6DSM_REG_FIFO_DATA_OUT_L 0x3EU   /* FIFO 数据出口（读该地址按样本顺序自动出队） */
#define LSM6DSM_REG_INT1_CTRL       0x0DU   /* bit7 DRDY_XL / bit6 DRDY_G / FIFO 阈值 */
#define LSM6DSM_REG_INT2_CTRL       0x0EU
#define LSM6DSM_REG_WHO_AM_I        0x0FU   /* 期望 0x6A */
#define LSM6DSM_REG_CTRL1_XL        0x10U   /* ODR_XL[7:4] / FS_XL[3:2] / LPF1_BW_SEL */
#define LSM6DSM_REG_CTRL2_G         0x11U   /* ODR_G[7:4] / FS_G[3:2] / FS_125(bit1) */
#define LSM6DSM_REG_CTRL3_C         0x12U   /* BOOT(7) / BDU(6) / H_LACTIVE(5) / PP_OD(4)
                                             * SIM(3) / IF_INC(2) / BDR_XL[1:0] */
#define LSM6DSM_REG_CTRL4_C         0x13U   /* LPF1_SEL_G(0) / I2C_disable(2) */
#define LSM6DSM_REG_CTRL5_C         0x14U   /* round / ST_XL[2:1] / ST_G[2:1] 自检 */
#define LSM6DSM_REG_CTRL6_C         0x15U   /* XL_HM_MODE(4)：1=低功耗 */
#define LSM6DSM_REG_CTRL8_XL        0x17U   /* LPF2_XL_EN(7) / HPCF_XL[6:5] / HP_REF_MODE */
#define LSM6DSM_REG_CTRL10_C        0x19U   /* FUNC_CFG_EN(2)：内嵌功能总开关 */
#define LSM6DSM_REG_STATUS_REG      0x1EU   /* bit1 GDA / bit0 XLDA */
#define LSM6DSM_REG_OUT_TEMP_L      0x20U   /* 温度低字节（8 LSB/℃，有符号） */
#define LSM6DSM_REG_OUT_TEMP_H      0x21U
#define LSM6DSM_REG_OUTX_L_G        0x22U   /* 角速度 0x22~0x27（X/Y/Z 各 2 字节） */
#define LSM6DSM_REG_OUTX_L_XL       0x28U   /* 加速度 0x28~0x2D */
#define LSM6DSM_REG_TAP_CFG         0x58U
#define LSM6DSM_REG_TAP_THS_6D      0x59U
#define LSM6DSM_REG_WAKE_UP_THS     0x5BU
#define LSM6DSM_REG_WAKE_UP_DUR     0x5CU
#define LSM6DSM_REG_FREE_FALL       0x5DU
#define LSM6DSM_REG_MD1_CFG         0x5EU   /* 事件路由到 INT1 */
#define LSM6DSM_REG_MD2_CFG         0x5FU

/* WHO_AM_I 期望值（LSM6DSM 固定 0x6A） */
#define LSM6DSM_WHO_AM_I_VAL        0x6AU

/* ---------------- CTRL5_C 自检位（G1） ----------------
 * bit2:1 = ST_XL[2:1]（01=正自检, 10=负自检, 00=关）
 * bit4:3 = ST_G [2:1]（10=正自检, 01=负自检, 00=关）  */
#define LSM6DSM_CTRL5_ST_XL_POS     0x02U
#define LSM6DSM_CTRL5_ST_XL_NEG     0x04U
#define LSM6DSM_CTRL5_ST_G_POS      0x08U
#define LSM6DSM_CTRL5_ST_G_NEG      0x10U

/* ---------------- FIFO_MODE（FIFO_CTRL5[2:0]） ---------------- */
#define LSM6DSM_FIFO_MODE_BYPASS    0x00U
#define LSM6DSM_FIFO_MODE_FIFO      0x01U   /* 满则停（不覆盖） */
#define LSM6DSM_FIFO_MODE_CONT      0x03U   /* 连续（覆盖最旧） */
#define LSM6DSM_FIFO_MODE_BYPASS_CONT 0x04U /* 触发前旁路，触发后连续 */

/* ---------------- 量程枚举（值即 FS_XL/FS_G 编码） ---------------- */
typedef enum
{
    LSM6DSM_XL_FS_2G  = 0,   /* ±2 g   （≈0.061 mg/LSB） */
    LSM6DSM_XL_FS_4G  = 1,   /* ±4 g   （≈0.122 mg/LSB） */
    LSM6DSM_XL_FS_8G  = 2,   /* ±8 g   （≈0.244 mg/LSB） */
    LSM6DSM_XL_FS_16G = 3    /* ±16 g  （≈0.488 mg/LSB） */
} Lsm6dsmXlFs_t;

typedef enum
{
    LSM6DSM_G_FS_245  = 0,   /* ±245 dps  （8.75 mdps/LSB） */
    LSM6DSM_G_FS_500  = 1,   /* ±500 dps  （17.50 mdps/LSB） */
    LSM6DSM_G_FS_1000 = 2,   /* ±1000 dps （35.0 mdps/LSB） */
    LSM6DSM_G_FS_2000 = 3    /* ±2000 dps （70.0 mdps/LSB） */
} Lsm6dsmGyFs_t;

/* ---------------- ODR 枚举（值即 ODR 编码，加速度与陀螺通用） ---------------- */
typedef enum
{
    LSM6DSM_ODR_12_5_HZ = 0,
    LSM6DSM_ODR_26_HZ   = 1,
    LSM6DSM_ODR_52_HZ   = 2,
    LSM6DSM_ODR_104_HZ  = 3,
    LSM6DSM_ODR_208_HZ  = 4,
    LSM6DSM_ODR_416_HZ  = 5,
    LSM6DSM_ODR_833_HZ  = 6,
    LSM6DSM_ODR_1660_HZ = 7,
    LSM6DSM_ODR_3330_HZ = 8,
    LSM6DSM_ODR_6660_HZ = 9
} Lsm6dsmOdr_t;

/* 一次突发读取的结果（均为器件原始补码值） */
typedef struct
{
    int16_t accRaw[3];    /* X/Y/Z 加速度原始值 */
    int16_t gyroRaw[3];   /* X/Y/Z 角速度原始值 */
    int16_t tempRaw;      /* 片内温度原始值（8 LSB/℃） */
} Lsm6dsmRaw_t;

/* 初始化：WHO_AM_I 校验 -> 软复位 -> BDU/IF_INC -> 量程/ODR/滤波。
 * 返回 1 = 器件在线且配置成功；0 = 不在线（未贴件或接线问题）。
 * 注意：初始化失败不影响系统其它功能，调用方按“IMU 无效”降级处理。 */
uint8_t Lsm6dsm_Init(uint8_t xlFs, uint8_t gyFs, uint8_t odrCode);

/* 读 WHO_AM_I（1=读成功，*id 为读回值）。 */
uint8_t Lsm6dsm_WhoAmI(uint8_t *id);

/* 器件是否已成功初始化（1=在线可用）。 */
uint8_t Lsm6dsm_IsPresent(void);

/* 一次突发读取 0x20~0x2D（温度 + 角速度 + 加速度，共 14 字节）。返回 1=成功。 */
uint8_t Lsm6dsm_ReadRaw(Lsm6dsmRaw_t *raw);

/* 动态改 ODR（唤醒/降功耗用）。odrCode 见 Lsm6dsmOdr_t。 */
void Lsm6dsm_SetOdr(uint8_t odrCode);

/* 单位换算系数：每 1 LSB 对应的物理量（纯整数，无浮点）。
 *   加速度：返回 µg/LSB，mg   = raw * 该值 / 1000
 *   角速度：返回 µdps/LSB，0.1dps = raw * 该值 / 100000
 *           （8.75 mdps = 875 µdps；若按 0.1dps 单位存储系数会不足 1，故用 µdps） */
uint16_t Lsm6dsm_AccLsbToUg(uint8_t xlFs);      /* µg/LSB（2g 档 = 61） */
uint16_t Lsm6dsm_GyroLsbToUdps(uint8_t gyFs);   /* µdps/LSB（245 档 = 875） */

/* 片内温度换算：返回 0.1℃。LSM6DSM 为 256 LSB/℃，0 LSB = 25℃，
 * 即 0.1℃ = 250 + raw * 10 / 256。 */
int16_t Lsm6dsm_TempRawToC10(int16_t tempRaw);

/* ---------------- G1：器件自检（CTRL5_C 的 ST_XL / ST_G） ----------------
 * 流程：记基线 -> 打开正自检 -> 等 200ms -> 读输出 -> 关闭自检 -> 等 200ms。
 * 判据（宽松阈值，仅用于判断“传感器链是否活着”，不做精度标定）：
 *   加速度任一轴 |Δ| ≥ 50 mg、角速度任一轴 |Δ| ≥ 30 dps 视为通过。
 * 返回 1 = 已执行（xlOk、gOk 两个出参给出各项结论）；0 = 器件不在线或读失败（未执行）。
 * 调用须在 Lsm6dsm_Init() 成功之后、常规 100Hz 采样开始之前（内部有 400ms 阻塞）。 */
uint8_t Lsm6dsm_SelfTest(uint8_t *xlOk, uint8_t *gOk);

/* ---------------- G7：FIFO 连续采样 ----------------
 * 配置 FIFO：FIFO_CTRL5 = FIFO_ODR | FIFO_MODE；FIFO_CTRL3 = 0x11（XL/G 均不分频）。
 * watermark = 触发水位（字，1 字 = 2 字节；一组 6 轴样本 = 6 字 = 12 字节）。
 * 返回 1 = 配置成功。 */
uint8_t Lsm6dsm_FifoConfig(uint8_t odrCode, uint8_t mode, uint16_t watermark);

/* 关闭 FIFO（回到 BYPASS，恢复正常寄存器直读）。 */
void Lsm6dsm_FifoDisable(void);

/* 读 FIFO 中未读“字”数（含 status2 高 4 位），0 = 空或器件不在线。 */
uint16_t Lsm6dsm_FifoGetWords(void);

/* 排空 FIFO：内部按 ≤10 字（20 字节）分段读（Int_SpiReadBuf 上限 23 字节），
 * 只保留“最后一组完整 6 轴样本”写入 *raw（数据顺序：陀螺 X/Y/Z 然后加速度 X/Y/Z，
 * 与 FIFO_CTRL3 只开 XL/G、不分频的默认顺序一致）。
 * 返回本次排空的“字”数（0 = 无数据）。*raw 可能未被更新（不足一组时）。 */
uint16_t Lsm6dsm_FifoDrain(Lsm6dsmRaw_t *raw);

#endif /* DRI_LSM6DSM_H */
