/*
 * dri_lsm6dsm.c
 * P9（第二版需求）：LSM6DSM 六轴传感器驱动实现（器件寄存器级，无业务逻辑）。
 *
 * 数据通路：app_imu（App 层） -> Lsm6dsm_ReadRaw() -> Int_SpiReadBuf()（Int 层）
 *          -> SPI2(9MHz, CS=PB12) -> LSM6DSM。
 *
 * 读取方式说明：
 *   0x20~0x2D 共 14 字节一次突发读完（温度 + 角速度 + 加速度），
 *   依靠 CTRL3_C.IF_INC=1 的地址自增；BDU=1 保证高低字节不会被采样点撕裂。
 *
 * 单位换算（全部整数运算，避免浮点开销）：
 *   加速度 mg    = raw * AccLsbToUg / 1000       （2g 档 61 µg/LSB）
 *   角速度 0.1dps = raw * GyroLsbToUdps / 100000 （245 档 875 µdps/LSB = 8.75 mdps）
 *   温度 0.1℃   = 250 + raw * 10 / 256          （256 LSB/℃，0 LSB = 25℃）
 */
#include "dri_lsm6dsm.h"
#include "int_spi.h"
#include "stm32f1xx_hal.h"
#include <stddef.h>

/* 器件状态（本驱动内的唯一全局，便于排查时直接看） */
static uint8_t s_present = 0U;   /* 1=WHO_AM_I 校验通过且配置完成 */
static uint8_t s_xlFs    = 0U;   /* 当前加速度量程编码 */
static uint8_t s_gyFs    = 0U;   /* 当前角速度量程编码 */
static uint8_t s_odr     = 4U;   /* 当前 ODR 编码（默认 208Hz） */

/* 读 WHO_AM_I（1=读成功） */
uint8_t Lsm6dsm_WhoAmI(uint8_t *id)
{
    if (id == NULL)
    {
        return 0U;
    }

    *id = Int_SpiReadReg(LSM6DSM_REG_WHO_AM_I);
    return 1U;
}

uint8_t Lsm6dsm_IsPresent(void)
{
    return s_present;
}

uint8_t Lsm6dsm_Init(uint8_t xlFs, uint8_t gyFs, uint8_t odrCode)
{
    uint8_t id = 0U;

    s_present = 0U;

    /* 1) 识别：先确认线上确实是 LSM6DSM，避免对空片写配置 */
    if (Lsm6dsm_WhoAmI(&id) == 0U)
    {
        return 0U;
    }
    if (id != LSM6DSM_WHO_AM_I_VAL)
    {
        return 0U;
    }

    /* 2) 软复位（CTRL3_C.SW_RESET），复位后需等待 BOOT 完成（数据手册 ~15ms） */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL3_C, 0x01U);
    HAL_Delay(20U);

    /* 3) BDU=1（高低字节一致）+ IF_INC=1（地址自增，突发读必须） */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL3_C, 0x44U);

    /* 4) 加速度：ODR[7:4] + FS[3:2]（LPF1 带宽位保持默认） */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL1_XL,
                          (uint8_t)(((odrCode & 0x0FU) << 4) | ((xlFs & 0x03U) << 2)));

    /* 5) 角速度：ODR[7:4] + FS[3:2] */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL2_G,
                          (uint8_t)(((odrCode & 0x0FU) << 4) | ((gyFs & 0x03U) << 2)));

    /* 6) 加速度 LPF2 使能（抑制高频混叠，适合电机 PWM 环境） */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL8_XL, 0x80U);

    /* 7) 陀螺 LPF1 使能 + 关闭 I2C 接口（本板只用 SPI） */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL4_C, 0x05U);

    /* 8) 高性能模式（不用低功耗，保证 100Hz 采样时噪声可控） */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL6_C, 0x00U);

    /* 9) 内嵌功能（tap/6D/free-fall/FSM）先全部关闭，P9-4 再按需开启 */
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL10_C, 0x00U);

    /* 10) 数据就绪中断暂不开启（P9-1 用 100Hz 轮询，避免改 Core 的 EXTI 配置） */
    (void)Int_SpiWriteReg(LSM6DSM_REG_INT1_CTRL, 0x00U);
    (void)Int_SpiWriteReg(LSM6DSM_REG_INT2_CTRL, 0x00U);

    s_xlFs    = (uint8_t)(xlFs & 0x03U);
    s_gyFs    = (uint8_t)(gyFs & 0x03U);
    s_odr     = (uint8_t)(odrCode & 0x0FU);
    s_present = 1U;

    return 1U;
}

uint8_t Lsm6dsm_ReadRaw(Lsm6dsmRaw_t *raw)
{
    uint8_t buf[14];

    if ((raw == NULL) || (s_present == 0U))
    {
        return 0U;
    }

    /* 0x20(温度L) ~ 0x2D(加速度Z H)：一次 CS 内突发读 14 字节 */
    if (Int_SpiReadBuf(LSM6DSM_REG_OUT_TEMP_L, buf, 14U) == 0U)
    {
        return 0U;
    }

    raw->tempRaw    = (int16_t)((uint16_t)buf[0]  | ((uint16_t)buf[1] << 8));
    raw->gyroRaw[0] = (int16_t)((uint16_t)buf[2]  | ((uint16_t)buf[3] << 8));
    raw->gyroRaw[1] = (int16_t)((uint16_t)buf[4]  | ((uint16_t)buf[5] << 8));
    raw->gyroRaw[2] = (int16_t)((uint16_t)buf[6]  | ((uint16_t)buf[7] << 8));
    raw->accRaw[0]  = (int16_t)((uint16_t)buf[8]  | ((uint16_t)buf[9] << 8));
    raw->accRaw[1]  = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8));
    raw->accRaw[2]  = (int16_t)((uint16_t)buf[12] | ((uint16_t)buf[13] << 8));

    return 1U;
}

void Lsm6dsm_SetOdr(uint8_t odrCode)
{
    if (s_present == 0U)
    {
        return;
    }

    s_odr = (uint8_t)(odrCode & 0x0FU);

    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL1_XL,
                          (uint8_t)((s_odr << 4) | ((s_xlFs & 0x03U) << 2)));
    (void)Int_SpiWriteReg(LSM6DSM_REG_CTRL2_G,
                          (uint8_t)((s_odr << 4) | ((s_gyFs & 0x03U) << 2)));
}

uint16_t Lsm6dsm_AccLsbToUg(uint8_t xlFs)
{
    switch (xlFs & 0x03U)
    {
        case 1U:  return 122U;   /* ±4 g  */
        case 2U:  return 244U;   /* ±8 g  */
        case 3U:  return 488U;   /* ±16 g */
        default:  return 61U;    /* ±2 g  */
    }
}

uint16_t Lsm6dsm_GyroLsbToUdps(uint8_t gyFs)
{
    switch (gyFs & 0x03U)
    {
        case 1U:  return 1750U;  /* ±500 dps  -> 17.50 mdps/LSB */
        case 2U:  return 3500U;  /* ±1000 dps -> 35.0  mdps/LSB */
        case 3U:  return 7000U;  /* ±2000 dps -> 70.0  mdps/LSB */
        default:  return 875U;   /* ±245 dps  -> 8.75  mdps/LSB */
    }
}

int16_t Lsm6dsm_TempRawToC10(int16_t tempRaw)
{
    /* LSM6DSM 片内温度：16bit、256 LSB/℃，0 LSB 对应 25℃（数据手册表）。
     * 0.1℃ = 250 + raw * 10 / 256  （整数运算，误差 < 0.05℃） */
    return (int16_t)(250 + (((int32_t)tempRaw * 10) / 256));
}

/* ==================== G1：CTRL5_C 自检 ==================== */

/* 取三个轴绝对值的最大变化量（mg / 0.1dps 两种单位分别算） */
static int32_t Lsm6dsm_AbsDeltaMax(int32_t a, int32_t b)
{
    int32_t d = a - b;

    return (d < 0) ? -d : d;
}

uint8_t Lsm6dsm_SelfTest(uint8_t *xlOk, uint8_t *gOk)
{
    Lsm6dsmRaw_t base;
    Lsm6dsmRaw_t st;
    int32_t dAccMax = 0;
    int32_t dGyMax  = 0;
    uint32_t accUg;
    uint32_t gyUdps;
    uint8_t  i;
    uint8_t  buf[6];

    if (xlOk != NULL) { *xlOk = 0U; }
    if (gOk  != NULL) { *gOk  = 0U; }
    if (s_present == 0U) { return 0U; }

    if (Lsm6dsm_ReadRaw(&base) == 0U) { return 0U; }

    /* 打开加速度 + 陀螺正自检，等输出稳定（数据手册要求 ~200ms） */
    Int_SpiWriteReg(LSM6DSM_REG_CTRL5_C,
                    (uint8_t)(LSM6DSM_CTRL5_ST_XL_POS | LSM6DSM_CTRL5_ST_G_POS));
    HAL_Delay(200U);

    /* 0x22~0x27 角速度 + 0x28~0x2D 加速度：分两次读（每次 6 字节，避开 23 字节上限） */
    if (Int_SpiReadBuf(LSM6DSM_REG_OUTX_L_G, buf, 6U) == 0U)
    {
        Int_SpiWriteReg(LSM6DSM_REG_CTRL5_C, 0x00U);
        return 0U;
    }
    for (i = 0U; i < 3U; i++)
    {
        st.gyroRaw[i] = (int16_t)((uint16_t)buf[i * 2U] | ((uint16_t)buf[i * 2U + 1U] << 8));
        if (i == 0U)
        {
            dGyMax = Lsm6dsm_AbsDeltaMax((int32_t)st.gyroRaw[0], (int32_t)base.gyroRaw[0]);
        }
        else if (dGyMax < Lsm6dsm_AbsDeltaMax((int32_t)st.gyroRaw[i], (int32_t)base.gyroRaw[i]))
        {
            dGyMax = Lsm6dsm_AbsDeltaMax((int32_t)st.gyroRaw[i], (int32_t)base.gyroRaw[i]);
        }
    }

    if (Int_SpiReadBuf(LSM6DSM_REG_OUTX_L_XL, buf, 6U) == 0U)
    {
        Int_SpiWriteReg(LSM6DSM_REG_CTRL5_C, 0x00U);
        return 0U;
    }
    for (i = 0U; i < 3U; i++)
    {
        st.accRaw[i] = (int16_t)((uint16_t)buf[i * 2U] | ((uint16_t)buf[i * 2U + 1U] << 8));
        if (i == 0U)
        {
            dAccMax = Lsm6dsm_AbsDeltaMax((int32_t)st.accRaw[0], (int32_t)base.accRaw[0]);
        }
        else if (dAccMax < Lsm6dsm_AbsDeltaMax((int32_t)st.accRaw[i], (int32_t)base.accRaw[i]))
        {
            dAccMax = Lsm6dsm_AbsDeltaMax((int32_t)st.accRaw[i], (int32_t)base.accRaw[i]);
        }
    }

    /* 关闭自检并等待回到正常输出（避免污染后续采样） */
    Int_SpiWriteReg(LSM6DSM_REG_CTRL5_C, 0x00U);
    HAL_Delay(200U);

    /* raw -> 物理量：加速度 mg = raw*µg/1000；角速度 0.1dps = raw*µdps/100000 */
    accUg  = (uint32_t)Lsm6dsm_AccLsbToUg(s_xlFs);
    gyUdps = (uint32_t)Lsm6dsm_GyroLsbToUdps(s_gyFs);

    /* 50 mg = 50 * 1000 µg；30 dps = 300 * 100000 µdps */
    if (((uint32_t)dAccMax * accUg / 1000U) >= 50U)   { if (xlOk != NULL) { *xlOk = 1U; } }
    if (((uint32_t)dGyMax * gyUdps / 100000U) >= 300U) { if (gOk  != NULL) { *gOk  = 1U; } }

    return 1U;
}

/* ==================== G7：FIFO 连续采样 ==================== */

uint8_t Lsm6dsm_FifoConfig(uint8_t odrCode, uint8_t mode, uint16_t watermark)
{
    uint8_t val;

    if (s_present == 0U)
    {
        return 0U;
    }

    /* FIFO_CTRL3：DEC_FIFO_G[3:0]=1、DEC_FIFO_XL[3:0]=1（都不分频，每样本 6 字） */
    Int_SpiWriteReg(LSM6DSM_REG_FIFO_CTRL3, 0x11U);

    /* 水位：FIFO_CTRL1 = FTH[7:0]，FIFO_CTRL2 低 5 位 = FTH[12:8] */
    Int_SpiWriteReg(LSM6DSM_REG_FIFO_CTRL1, (uint8_t)(watermark & 0x00FFU));
    Int_SpiWriteReg(LSM6DSM_REG_FIFO_CTRL2, (uint8_t)((watermark >> 8) & 0x1FU));

    /* FIFO_CTRL5：bit7:5 = FIFO_MODE[2:0]，bit3:0 = FIFO_ODR[3:0]（LSM6DSM 数据手册） */
    val = (uint8_t)(((mode & 0x07U) << 5) | (odrCode & 0x0FU));
    Int_SpiWriteReg(LSM6DSM_REG_FIFO_CTRL5, val);

    /* 回读 FIFO_CTRL5 确认写入生效（SPI 写无返回值，用回读判定） */
    return (Int_SpiReadReg(LSM6DSM_REG_FIFO_CTRL5) == val) ? 1U : 0U;
}

void Lsm6dsm_FifoDisable(void)
{
    if (s_present == 0U)
    {
        return;
    }

    (void)Int_SpiWriteReg(LSM6DSM_REG_FIFO_CTRL5, LSM6DSM_FIFO_MODE_BYPASS);
}

uint16_t Lsm6dsm_FifoGetWords(void)
{
    uint8_t st[2];

    if (s_present == 0U)
    {
        return 0U;
    }
    if (Int_SpiReadBuf(LSM6DSM_REG_FIFO_STATUS1, st, 2U) == 0U)
    {
        return 0U;
    }

    return (uint16_t)((((uint16_t)st[1] & 0x0FU) << 8) | (uint16_t)st[0]);
}

uint16_t Lsm6dsm_FifoDrain(Lsm6dsmRaw_t *raw)
{
    uint16_t words;
    uint16_t total;
    uint16_t n;
    uint8_t  buf[20];      /* 10 字 = 20 字节（Int_SpiReadBuf 上限 23 字节） */
    uint8_t  last[12];
    uint8_t  haveLast = 0U;
    uint8_t  i;

    if (s_present == 0U)
    {
        return 0U;
    }

    total = Lsm6dsm_FifoGetWords();
    words = total;
    if (words == 0U)
    {
        return 0U;
    }

    while (words > 0U)
    {
        n = (words > 10U) ? 10U : words;

        if (Int_SpiReadBuf(LSM6DSM_REG_FIFO_DATA_OUT_L, buf, (uint8_t)(n * 2U)) == 0U)
        {
            break;
        }

        /* 只有这一段里含完整一组 6 字（12 字节）时才保留“最新一组” */
        if (n >= 6U)
        {
            for (i = 0U; i < 12U; i++)
            {
                last[i] = buf[(n - 6U) * 2U + i];
            }
            haveLast = 1U;
        }
        words = (uint16_t)(words - n);
    }

    if ((haveLast != 0U) && (raw != NULL))
    {
        /* 顺序：陀螺 X/Y/Z（0x22 起）然后加速度 X/Y/Z（0x28 起） */
        raw->gyroRaw[0] = (int16_t)((uint16_t)last[0]  | ((uint16_t)last[1]  << 8));
        raw->gyroRaw[1] = (int16_t)((uint16_t)last[2]  | ((uint16_t)last[3]  << 8));
        raw->gyroRaw[2] = (int16_t)((uint16_t)last[4]  | ((uint16_t)last[5]  << 8));
        raw->accRaw[0]  = (int16_t)((uint16_t)last[6]  | ((uint16_t)last[7]  << 8));
        raw->accRaw[1]  = (int16_t)((uint16_t)last[8]  | ((uint16_t)last[9]  << 8));
        raw->accRaw[2]  = (int16_t)((uint16_t)last[10] | ((uint16_t)last[11] << 8));
        raw->tempRaw    = 0;   /* FIFO 不成组输出温度，保持 0（上层不参与判据） */
    }

    return total;
}
