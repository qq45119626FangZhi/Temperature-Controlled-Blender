/*
 * app_store.c
 * 参数掉电保存实现：M24C02 双槽 + magic/version + CRC16-CCITT，
 * 支持 v1 -> v2 自动迁移，并提供运行时镜像与“静置 2s 落盘”的防抖写入。
 */
#include "app_store.h"
#include "dri_m24c02.h"
#include "int_bsp_i2c2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

/* 编译期检查：结构体必须能放进一个槽，否则 M24C02 地址会互相覆盖。 */
typedef char app_store_size_check[(sizeof(App_Store_Params_t) <= APP_STORE_SLOT_SIZE) ? 1 : -1];

/* v1 旧结构：仅用于迁移读取（槽 0x00 / 0x20，28 字节） */
typedef struct
{
    uint16_t magic;
    uint16_t version;
    uint16_t seq;
    int32_t  speed;
    int32_t  plateTemp;
    int32_t  liquidTemp;
    uint32_t runTimeSec;
    uint16_t crc;
} App_Store_ParamsV1_t;

typedef char app_store_v1_size_check[(sizeof(App_Store_ParamsV1_t) <= APP_STORE_V1_SLOT_SIZE) ? 1 : -1];

/* 运行统计的秒内余量 */
#define APP_STORE_MS_PER_SEC  1000U

static uint8_t  app_store_ready      = 0U;   /* 1 = EEPROM 在线 */
static uint8_t  app_store_activeSlot = 0U;   /* 当前有效槽：0 或 1 */
static uint16_t app_store_lastSeq    = 0U;   /* 当前有效槽的写入序号 */

/* 运行时参数镜像：各模块读写这里，落盘时整体写入 EEPROM。 */
static App_Store_Params_t app_store_params;

/* 防抖落盘状态 */
static uint8_t  app_store_savePending = 0U;
static uint32_t app_store_saveTick    = 0U;

/* 运行统计状态 */
static uint8_t  app_store_statsDirty = 0U;
static uint32_t app_store_runMs      = 0U;   /* 电机运行累计不足 1s 的部分 */
static uint32_t app_store_heatMs     = 0U;   /* 加热累计不足 1s 的部分 */

/* CRC16-CCITT（多项式 0x1021，初值 0xFFFF） */
static uint16_t App_Store_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t  bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

/* 校验指定版本的槽（magic/version/CRC）。v2 与 v3 布局相同，仅版本号不同。 */
static uint8_t App_Store_SlotValidVer(const App_Store_Params_t *p, uint16_t version)
{
    uint16_t crcCalc;

    if ((p->magic != APP_STORE_MAGIC) || (p->version != version))
    {
        return 0U;
    }

    crcCalc = App_Store_Crc16((const uint8_t *)p, (uint16_t)offsetof(App_Store_Params_t, crc));
    return (crcCalc == p->crc) ? 1U : 0U;
}

/* v2~v4 槽布局：前 88 字节与 v5 完全一致，crc 固定在第 88 字节
 * （v5 把 crc 移到了结构末尾，因此旧数据的 crc 必须按固定偏移校验）。 */
#define APP_STORE_LEGACY_CRC_OFF 88U

static uint8_t App_Store_SlotValidLegacy(const App_Store_Params_t *p)
{
    uint16_t crcStored = 0U;
    uint16_t crcCalc;

    if (p->magic != APP_STORE_MAGIC)
    {
        return 0U;
    }
    if ((p->version != APP_STORE_VERSION_V4) &&
        (p->version != APP_STORE_VERSION_V3) &&
        (p->version != APP_STORE_VERSION_V2))
    {
        return 0U;
    }

    (void)memcpy(&crcStored, (const uint8_t *)p + APP_STORE_LEGACY_CRC_OFF, sizeof(crcStored));
    crcCalc = App_Store_Crc16((const uint8_t *)p, APP_STORE_LEGACY_CRC_OFF);
    return (crcCalc == crcStored) ? 1U : 0U;
}

/* 校验 v1 槽（迁移用） */
static uint8_t App_Store_SlotValidV1(const App_Store_ParamsV1_t *p)
{
    uint16_t crcCalc;

    if ((p->magic != APP_STORE_MAGIC) || (p->version != APP_STORE_VERSION_LEGACY))
    {
        return 0U;
    }

    crcCalc = App_Store_Crc16((const uint8_t *)p, (uint16_t)offsetof(App_Store_ParamsV1_t, crc));
    return (crcCalc == p->crc) ? 1U : 0U;
}

/* 写入“非当前有效”的槽：先算 CRC 与 seq，写成功后才切换有效槽。 */
static uint8_t App_Store_WriteSlot(const App_Store_Params_t *in)
{
    App_Store_Params_t p;
    uint8_t targetSlot;
    uint8_t addr;

    if ((in == NULL) || (app_store_ready == 0U))
    {
        return 0U;
    }

    p = *in;
    p.magic     = APP_STORE_MAGIC;
    p.version   = APP_STORE_VERSION;
    p.reserved2 = 0U;
    p.seq       = (uint16_t)(app_store_lastSeq + 1U);
    p.crc       = App_Store_Crc16((const uint8_t *)&p, (uint16_t)offsetof(App_Store_Params_t, crc));

    targetSlot = (uint8_t)(1U - app_store_activeSlot);
    addr = (targetSlot == 0U) ? (uint8_t)APP_STORE_SLOT0_ADDR : (uint8_t)APP_STORE_SLOT1_ADDR;

    if (M24C02_Write(addr, (const uint8_t *)&p, (uint16_t)sizeof(p)) == 0U)
    {
        return 0U;
    }

    app_store_activeSlot = targetSlot;
    app_store_lastSeq    = p.seq;

    return 1U;
}

/* v1 -> v2 迁移：读旧槽，取较新者，保留 UI 参数，其余填默认值。
 * 成功后立即回写为 v2（写 0xA0 槽，与 v1 的 0x00/0x20 不重叠，
 * 即使回写失败，下次上电仍能再次迁移）。 */
static uint8_t App_Store_MigrateV1(App_Store_Params_t *out)
{
    App_Store_ParamsV1_t s0;
    App_Store_ParamsV1_t s1;
    App_Store_ParamsV1_t *pick;
    uint8_t v0;
    uint8_t v1;

    (void)memset(&s0, 0, sizeof(s0));
    (void)memset(&s1, 0, sizeof(s1));

    if (M24C02_Read((uint8_t)APP_STORE_V1_SLOT0_ADDR, (uint8_t *)&s0, (uint16_t)sizeof(s0)) == 0U)
    {
        return 0U;
    }
    if (M24C02_Read((uint8_t)APP_STORE_V1_SLOT1_ADDR, (uint8_t *)&s1, (uint16_t)sizeof(s1)) == 0U)
    {
        return 0U;
    }

    v0 = App_Store_SlotValidV1(&s0);
    v1 = App_Store_SlotValidV1(&s1);

    if ((v0 == 0U) && (v1 == 0U))
    {
        return 0U;   /* 全新 EEPROM：无历史数据可迁移 */
    }

    if ((v0 != 0U) && (v1 != 0U))
    {
        pick = ((uint16_t)(s1.seq - s0.seq) < 0x8000U) ? &s1 : &s0;
    }
    else if (v0 != 0U)
    {
        pick = &s0;
    }
    else
    {
        pick = &s1;
    }

    App_StoreDefault(out);
    out->speed      = pick->speed;
    out->plateTemp  = pick->plateTemp;
    out->liquidTemp = pick->liquidTemp;
    out->runTimeSec = pick->runTimeSec;
    out->seq        = pick->seq;

    /* v4：v1 时代"0=不限制"其实是默认值，升级时补 40℃ 兜底 */
    if (out->plateTemp <= 0)
    {
        out->plateTemp = APP_STORE_PLATE_DEFAULT_C;
    }

    /* 以“旧槽的 seq”为基线继续编号，回写到槽 1。 */
    app_store_activeSlot = 0U;
    app_store_lastSeq    = pick->seq;
    if (App_Store_WriteSlot(out) != 0U)
    {
        out->seq = app_store_lastSeq;   /* 与 flash 中的序号保持一致 */
    }

    return 1U;
}

void App_StoreDefault(App_Store_Params_t *out)
{
    if (out == NULL)
    {
        return;
    }

    (void)memset(out, 0, sizeof(*out));

    out->magic   = APP_STORE_MAGIC;
    out->version = APP_STORE_VERSION;

    /* UI 参数：上电电机停止、不加热、不限时（与 menu_ui 初值一致）。
     * P7（v4）起铝板上限默认 40℃，作为"板子最热"的兜底限值。 */
    out->speed      = 0;
    out->plateTemp  = APP_STORE_PLATE_DEFAULT_C;
    out->liquidTemp = 0;
    out->runTimeSec = 0U;

    /* P3 控制参数初值（与 app_motor / app_heat 的原硬编码一致，×1000 定点）。 */
    out->motorPidKp = 2000;   /* 2.0  */
    out->motorPidKi = 500;    /* 0.5  */
    out->motorPidKd = 0;      /* 0.0  */
    out->heatPidKp  = 2000;   /* 2.0  */
    out->heatPidKi  = 50;     /* 0.05 */
    out->heatPidKd  = 0;      /* 0.0  */

    /* 校准系数：先沿用占位值，待实测标定后按需改写。 */
    out->encoderPpr     = 44;
    out->gearRatioX100  = 30 * APP_STORE_GEAR_SCALE;
    out->liquidOffsetC10 = 0;
    out->boardOffsetC10  = 0;

    /* 通信参数（P6）：9600 8N1、从机地址 1、通道使能 = RS485 + 蓝牙。 */
    out->modbusBaud = 9600U;
    out->slaveAddr  = 1U;
    out->commEn     = APP_STORE_COMM_EN_DEFAULT;

    /* P8 现场标定参数：NTC 曲线 + 加热过流阈值。 */
    out->ntcR25    = APP_STORE_NTC_R25_DEFAULT;
    out->ntcB      = APP_STORE_NTC_B_DEFAULT;
    out->ntcRup    = APP_STORE_NTC_RUP_DEFAULT;
    out->heatOcRaw = APP_STORE_HEAT_OC_DEFAULT;

    /* 运行统计从 0 开始。 */
    out->crc = 0U;
}

void App_StoreInit(void)
{
    /* I2C2 与 DS3231 共用，互斥量必须在使用前创建。 */
    Bsp_I2c2Init();

    App_StoreDefault(&app_store_params);

    app_store_activeSlot  = 0U;
    app_store_lastSeq     = 0U;
    app_store_savePending = 0U;
    app_store_saveTick    = 0U;
    app_store_statsDirty  = 0U;
    app_store_runMs       = 0U;
    app_store_heatMs      = 0U;

    app_store_ready = M24C02_IsReady();
    if (app_store_ready != 0U)
    {
        /* v2 有效数据 -> 直接用；否则尝试 v1 迁移；都没有则保留默认值。 */
        (void)App_StoreLoad(&app_store_params);
    }

    /* 上电次数 +1，随任意一次保存一起落盘（不为此单独写 EEPROM）。 */
    App_StoreAddBoot();
}

uint8_t App_StoreIsReady(void)
{
    return app_store_ready;
}

/* P9：原始字节块读写（IMU 参数独立块用）。
 * 只允许落在 v1 遗留空闲区 0x00~0x3F，避免误写主参数槽 0x40/0xA0。 */
#define APP_STORE_RAW_LO  0x00U
#define APP_STORE_RAW_HI  0x3FU

uint8_t App_StoreRawRead(uint8_t addr, void *buf, uint16_t len)
{
    if ((buf == NULL) || (app_store_ready == 0U) || (len == 0U))
    {
        return 0U;
    }
    /* addr 为 uint8 天然 >= 0，只需检查上界（0x00~0x3F） */
    if (((uint16_t)addr + len) > (APP_STORE_RAW_HI + 1U))
    {
        return 0U;
    }

    return M24C02_Read(addr, (uint8_t *)buf, len);
}

uint8_t App_StoreRawWrite(uint8_t addr, const void *buf, uint16_t len)
{
    if ((buf == NULL) || (app_store_ready == 0U) || (len == 0U))
    {
        return 0U;
    }
    if (((uint16_t)addr + len) > (APP_STORE_RAW_HI + 1U))
    {
        return 0U;
    }

    return M24C02_Write(addr, (const uint8_t *)buf, len);
}

const App_Store_Params_t *App_StoreGet(void)
{
    return &app_store_params;
}

uint8_t App_StoreLoad(App_Store_Params_t *out)
{
    App_Store_Params_t s0;
    App_Store_Params_t s1;
    uint8_t valid0;
    uint8_t valid1;

    if ((out == NULL) || (app_store_ready == 0U))
    {
        return 0U;
    }

    (void)memset(&s0, 0, sizeof(s0));
    (void)memset(&s1, 0, sizeof(s1));

    if (M24C02_Read((uint8_t)APP_STORE_SLOT0_ADDR, (uint8_t *)&s0, (uint16_t)sizeof(s0)) == 0U)
    {
        return 0U;
    }
    if (M24C02_Read((uint8_t)APP_STORE_SLOT1_ADDR, (uint8_t *)&s1, (uint16_t)sizeof(s1)) == 0U)
    {
        return 0U;
    }

    valid0 = App_Store_SlotValidVer(&s0, APP_STORE_VERSION);
    valid1 = App_Store_SlotValidVer(&s1, APP_STORE_VERSION);

    if ((valid0 == 0U) && (valid1 == 0U))
    {
        /* 当前版本无有效数据：先尝试 v2/v3/v4 -> v5 迁移（前 88 字节布局相同），
         * 再尝试 v1 -> v5。 */
        uint8_t v20 = App_Store_SlotValidLegacy(&s0);
        uint8_t v21 = App_Store_SlotValidLegacy(&s1);

        if ((v20 != 0U) || (v21 != 0U))
        {
            const App_Store_Params_t *pick;

            if ((v20 != 0U) && (v21 != 0U))
            {
                pick = ((uint16_t)(s1.seq - s0.seq) < 0x8000U) ? &s1 : &s0;
                app_store_activeSlot = (pick == &s1) ? 1U : 0U;
            }
            else if (v20 != 0U)
            {
                pick = &s0;
                app_store_activeSlot = 0U;
            }
            else
            {
                pick = &s1;
                app_store_activeSlot = 1U;
            }

            *out = *pick;                          /* 前 88 字节布局一致，字段直接沿用 */
            out->version   = APP_STORE_VERSION;    /* 升级为 v5 */
            out->heatOcRaw = APP_STORE_HEAT_OC_DEFAULT;   /* v5 新字段：补默认值 */
            out->ntcR25    = APP_STORE_NTC_R25_DEFAULT;
            out->ntcB      = APP_STORE_NTC_B_DEFAULT;
            out->ntcRup    = APP_STORE_NTC_RUP_DEFAULT;
            if ((out->commEn == 0U) || (out->commEn == 0xFFU))
            {
                out->commEn = APP_STORE_COMM_EN_DEFAULT;   /* 旧数据无此字段：补默认值 */
            }
            if (out->plateTemp <= 0)
            {
                /* v4 起"未设置"不再等于"不限制"：升级时补 40℃ 兜底。
                 * 升级完成后用户显式把上限设为 0（不限制）会原样保存，不再被改动。 */
                out->plateTemp = APP_STORE_PLATE_DEFAULT_C;
            }
            app_store_lastSeq = out->seq;
            (void)App_Store_WriteSlot(out);        /* 回写成 v4（写另一槽，旧数据保留） */
            out->seq = app_store_lastSeq;
            return 1U;
        }

        return App_Store_MigrateV1(out);
    }

    if ((valid0 != 0U) && (valid1 != 0U))
    {
        /* 两槽都有效：取序号更大的（较新的那次保存） */
        if ((uint16_t)(s1.seq - s0.seq) < 0x8000U)
        {
            *out = s1;
            app_store_activeSlot = 1U;
        }
        else
        {
            *out = s0;
            app_store_activeSlot = 0U;
        }
    }
    else if (valid0 != 0U)
    {
        *out = s0;
        app_store_activeSlot = 0U;
    }
    else
    {
        *out = s1;
        app_store_activeSlot = 1U;
    }

    app_store_lastSeq = out->seq;
    return 1U;
}

uint8_t App_StoreSave(const App_Store_Params_t *in)
{
    return App_Store_WriteSlot(in);
}

/* ---- 镜像更新 ---- */

void App_StoreSetUi(int32_t speed, int32_t plateTemp, int32_t liquidTemp, uint32_t runTimeSec)
{
    app_store_params.speed      = speed;
    app_store_params.plateTemp  = plateTemp;
    app_store_params.liquidTemp = liquidTemp;
    app_store_params.runTimeSec = runTimeSec;
}

void App_StoreSetMotorPid(float kp, float ki, float kd)
{
    app_store_params.motorPidKp = APP_STORE_PID_TO_MILLI(kp);
    app_store_params.motorPidKi = APP_STORE_PID_TO_MILLI(ki);
    app_store_params.motorPidKd = APP_STORE_PID_TO_MILLI(kd);
}

void App_StoreSetHeatPid(float kp, float ki, float kd)
{
    app_store_params.heatPidKp = APP_STORE_PID_TO_MILLI(kp);
    app_store_params.heatPidKi = APP_STORE_PID_TO_MILLI(ki);
    app_store_params.heatPidKd = APP_STORE_PID_TO_MILLI(kd);
}

void App_StoreSetCalib(int32_t encoderPpr, int32_t gearRatioX100, int32_t liquidOffsetC10, int32_t boardOffsetC10)
{
    app_store_params.encoderPpr      = encoderPpr;
    app_store_params.gearRatioX100   = gearRatioX100;
    app_store_params.liquidOffsetC10 = liquidOffsetC10;
    app_store_params.boardOffsetC10  = boardOffsetC10;
}

void App_StoreSetComm(uint32_t baud, uint16_t slaveAddr)
{
    app_store_params.modbusBaud = baud;
    app_store_params.slaveAddr  = slaveAddr;
}

/* P8：NTC 曲线参数（R25/B/上拉）。超出合理范围的按默认值处理，
 * 避免误标定（例如 0Ω）导致除零或温度恒定。 */
void App_StoreSetNtc(uint16_t r25, uint16_t b, uint16_t rup)
{
    app_store_params.ntcR25 = ((r25 >= 100U) && (r25 <= 100000U)) ? r25 : APP_STORE_NTC_R25_DEFAULT;
    app_store_params.ntcB   = ((b   >= 500U) && (b   <= 10000U)) ? b   : APP_STORE_NTC_B_DEFAULT;
    app_store_params.ntcRup = ((rup >= 100U) && (rup <= 100000U)) ? rup : APP_STORE_NTC_RUP_DEFAULT;
}

/* P8：加热过流判定阈值（ADC raw）。 */
void App_StoreSetHeatOc(uint16_t raw)
{
    app_store_params.heatOcRaw = ((raw >= 100U) && (raw <= 4095U)) ? raw : APP_STORE_HEAT_OC_DEFAULT;
}

void App_StoreSetCommEn(uint8_t commEn)
{
    app_store_params.commEn = commEn;
}

void App_StoreAddMotorStart(void)
{
    app_store_params.motorStartCnt++;
    app_store_statsDirty = 1U;
}

void App_StoreAddBoot(void)
{
    app_store_params.bootCnt++;
    app_store_statsDirty = 1U;
}

void App_StoreStatsTick(uint32_t dtMs, uint8_t motorRunning, uint8_t heating)
{
    if (motorRunning != 0U)
    {
        app_store_runMs += dtMs;
        while (app_store_runMs >= APP_STORE_MS_PER_SEC)
        {
            app_store_runMs -= APP_STORE_MS_PER_SEC;
            app_store_params.totalRunSec++;
            app_store_statsDirty = 1U;
        }
    }

    if (heating != 0U)
    {
        app_store_heatMs += dtMs;
        while (app_store_heatMs >= APP_STORE_MS_PER_SEC)
        {
            app_store_heatMs -= APP_STORE_MS_PER_SEC;
            app_store_params.heatRunSec++;
            app_store_statsDirty = 1U;
        }
    }
}

uint8_t App_StoreStatsDirty(void)
{
    return app_store_statsDirty;
}

/* ---- 防抖落盘 ---- */

void App_StoreRequestSave(void)
{
    app_store_savePending = 1U;
    app_store_saveTick    = (uint32_t)xTaskGetTickCount();
}

uint8_t App_StoreFlushSave(void)
{
    uint32_t now;

    if (app_store_savePending == 0U)
    {
        return 1U;
    }

    if (app_store_ready == 0U)
    {
        /* 无 EEPROM：放弃本次，避免每个周期都重试。 */
        app_store_savePending = 0U;
        return 1U;
    }

    now = (uint32_t)xTaskGetTickCount();
    if ((now - app_store_saveTick) < pdMS_TO_TICKS(APP_STORE_SAVE_DELAY_MS))
    {
        return 0U;   /* 还在静置窗口内 */
    }

    if (App_Store_WriteSlot(&app_store_params) == 0U)
    {
        return 0U;   /* 写失败：保留 pending，下个周期重试 */
    }

    app_store_params.seq = app_store_lastSeq;
    app_store_savePending = 0U;
    app_store_statsDirty  = 0U;

    return 1U;
}

uint8_t App_StoreIsSavePending(void)
{
    return app_store_savePending;
}
