/*
 * app_store.h
 * 参数掉电保存（M24C02，I2C2）：版本号 + CRC16 + 双备份 + 运行时镜像。
 *
 * 存储布局（M24C02 共 256 字节，16 字节/页）：
 *   槽 0（v2）：0x40 ~ 0x9F（96 字节）
 *   槽 1（v2）：0xA0 ~ 0xFF（96 字节）
 *   旧版（v1）：0x00 ~ 0x1F、0x20 ~ 0x3F（28 字节，仅 UI 参数），只读保留
 * 每次保存写入“非当前有效”的那个槽，并递增 seq；上电时校验两槽的
 * magic/version/CRC，取 seq 较大的有效槽。任一槽损坏仍可恢复。
 *
 * 版本迁移：v2 两槽均无效时读取 v1 槽（0x00/0x20），把 UI 参数搬到 v2 并
 * 立即回写为 v2（写入与旧槽不重叠的地址，失败也不影响下次再迁移）。
 * 新增字段在迁移时填默认值，运行统计从 0 开始。
 *
 * 写入策略（防抖）：App_StoreRequestSave() 只置标志并记录时刻，
 * App_StoreFlushSave() 在最后一次修改静置 APP_STORE_SAVE_DELAY_MS 后统一
 * 写一次，避免长按连发改参数时频繁擦写 EEPROM（写一页约 5ms，92 字节约 6 页）。
 * 单次保存阻塞 30~40ms，应在任务上下文调用。
 */
#ifndef APP_STORE_H
#define APP_STORE_H

#include <stdint.h>

#define APP_STORE_MAGIC          0x5453U   /* 'T''S' */
#define APP_STORE_VERSION        5U        /* v5：P8 起 NTC 参数(R25/B/上拉) + 加热过流阈值可标定 */
#define APP_STORE_VERSION_V4     4U        /* v4：P7 起铝板上限默认 40℃（布局与 v2/v3 相同） */
#define APP_STORE_VERSION_V3     3U        /* v3：v2 + 通信通道使能位（P6） */
#define APP_STORE_VERSION_V2     2U        /* v2：PID/校准/通信/统计 */
#define APP_STORE_VERSION_LEGACY 1U        /* v1：仅 UI 参数 */

/* P7：铝板上限默认值（℃）。0 = 不限制仍保留为合法值，仅默认值改变。 */
#define APP_STORE_PLATE_DEFAULT_C 40

/* P8：现场可标定参数默认值（NTC 曲线 + 加热过流阈值） */
#define APP_STORE_NTC_R25_DEFAULT  10000U   /* NTC 25℃ 阻值(Ω) */
#define APP_STORE_NTC_B_DEFAULT     3950U   /* NTC B 常量(K) */
#define APP_STORE_NTC_RUP_DEFAULT  10000U   /* 分压上拉电阻(Ω) */
#define APP_STORE_HEAT_OC_DEFAULT   3500U   /* 加热过流判定阈值(ADC raw，0~4095) */

/* 通信通道使能位默认值：RS485 + 蓝牙（CAN 本期未实现） */
#define APP_STORE_COMM_EN_DEFAULT 0x03U

/* v2 槽大小与起始地址（结构体必须 <= 96 字节，编译期已校验） */
#define APP_STORE_SLOT_SIZE      96U
#define APP_STORE_SLOT0_ADDR     0x40U
#define APP_STORE_SLOT1_ADDR     0xA0U

/* v1 旧布局（只读迁移用） */
#define APP_STORE_V1_SLOT_SIZE   32U
#define APP_STORE_V1_SLOT0_ADDR  0x00U
#define APP_STORE_V1_SLOT1_ADDR  0x20U

/* 定标：PID 增益 ×1000、减速比 ×100、温度偏移 ×10(0.1℃) */
#define APP_STORE_PID_SCALE      1000.0f
#define APP_STORE_GEAR_SCALE     100
#define APP_STORE_MILLI_TO_PID(m) ((float)(m) / APP_STORE_PID_SCALE)
#define APP_STORE_PID_TO_MILLI(k) ((int32_t)((k) * APP_STORE_PID_SCALE))

/* 参数停手后多久落盘(ms) */
#define APP_STORE_SAVE_DELAY_MS  2000U

/* 持久化参数（与 UI 菜单项 / 控制参数 / 运行统计一一对应） */
typedef struct
{
    uint16_t magic;
    uint16_t version;
    uint16_t seq;         /* 写入序号，越大越新 */
    uint16_t heatOcRaw;   /* P8：加热过流判定阈值(ADC raw)；v4 及以前此处为保留字段(0) */

    /* ---- UI 参数 ---- */
    int32_t  speed;       /* 目标转速（电机轴 RPM，0~500） */
    int32_t  plateTemp;   /* 铝板限温（℃，0 = 不限制） */
    int32_t  liquidTemp;  /* 液体目标温度（℃，0 = 不加热） */
    uint32_t runTimeSec;  /* 定时运行时间（秒，0 = 不限时） */

    /* ---- PID 增益（×1000 定点，2.0 -> 2000） ---- */
    int32_t  motorPidKp;
    int32_t  motorPidKi;
    int32_t  motorPidKd;
    int32_t  heatPidKp;
    int32_t  heatPidKi;
    int32_t  heatPidKd;

    /* ---- 校准系数 ---- */
    int32_t  encoderPpr;       /* 编码器每转计数（线数×4，默认 44） */
    int32_t  gearRatioX100;    /* 减速比 ×100（30.00 -> 3000） */
    int32_t  liquidOffsetC10;  /* 液体温度校准偏移（0.1℃） */
    int32_t  boardOffsetC10;   /* 铝板温度校准偏移（0.1℃） */

    /* ---- 通信参数（P6） ---- */
    uint32_t modbusBaud;       /* RS485/Modbus 波特率（默认 9600，8N1） */
    uint16_t slaveAddr;        /* Modbus 从机地址（1~247） */
    uint8_t  commEn;           /* 通道使能位：bit0 RS485 / bit1 蓝牙 / bit2 CAN */
    uint8_t  reserved2;

    /* ---- 运行统计 ---- */
    uint32_t totalRunSec;      /* 累计电机运行秒数 */
    uint32_t heatRunSec;       /* 累计加热秒数 */
    uint32_t motorStartCnt;    /* 累计启动次数 */
    uint32_t bootCnt;          /* 上电次数 */

    /* ---- P8 现场标定参数（v5 新增，CRC 覆盖） ---- */
    uint16_t ntcR25;           /* NTC 25℃ 阻值(Ω)，默认 10000 */
    uint16_t ntcB;             /* NTC B 常量(K)，默认 3950 */
    uint16_t ntcRup;           /* 分压上拉电阻(Ω)，默认 10000 */

    uint16_t crc;              /* 覆盖本结构 crc 之前的全部字节 */
} App_Store_Params_t;

/* 初始化：创建 I2C2 互斥量、探测 EEPROM、加载参数到运行时镜像。
 * 须在 vTaskStartScheduler() 之前、且在其他模块读取参数之前调用。 */
void App_StoreInit(void);

/* EEPROM 是否在线（1=在线）。离线时镜像仍可用（默认值），但不会落盘。 */
uint8_t App_StoreIsReady(void);

/* 运行时参数镜像（只读）。写入请用下面的 Set/Add 接口，再 RequestSave。 */
const App_Store_Params_t *App_StoreGet(void);

/* 填充默认参数（与 UI 上电初值、P3 控制参数初值一致）。 */
void App_StoreDefault(App_Store_Params_t *out);

/* 低层读写（立即访问 EEPROM；一般用“镜像 + Request/Flush”代替）。
 * Load：1=成功（out 被填充），0=无有效数据（out 不变）。 */
uint8_t App_StoreLoad(App_Store_Params_t *out);
uint8_t App_StoreSave(const App_Store_Params_t *in);

/* ---- 镜像更新（只改 RAM，不写 EEPROM） ---- */
void App_StoreSetUi(int32_t speed, int32_t plateTemp, int32_t liquidTemp, uint32_t runTimeSec);
void App_StoreSetMotorPid(float kp, float ki, float kd);
void App_StoreSetHeatPid(float kp, float ki, float kd);
void App_StoreSetCalib(int32_t encoderPpr, int32_t gearRatioX100, int32_t liquidOffsetC10, int32_t boardOffsetC10);
void App_StoreSetComm(uint32_t baud, uint16_t slaveAddr);
/* P8：NTC 曲线参数（R25/B/上拉）与加热过流阈值；非法值按默认值处理 */
void App_StoreSetNtc(uint16_t r25, uint16_t b, uint16_t rup);
void App_StoreSetHeatOc(uint16_t raw);
void App_StoreSetCommEn(uint8_t commEn);   /* 通道使能位（bit0 RS485 / bit1 蓝牙 / bit2 CAN） */
void App_StoreAddMotorStart(void);
void App_StoreAddBoot(void);

/* 运行统计累加：每个控制周期调用，不足 1s 的部分内部保留。
 * motorRunning/heating 为 1 表示本周期该计时器在走。 */
void App_StoreStatsTick(uint32_t dtMs, uint8_t motorRunning, uint8_t heating);

/* 统计自上次落盘后是否变化（用于低频批量保存，减少擦写）。 */
uint8_t App_StoreStatsDirty(void);

/* ---- P9：原始字节块读写（IMU 参数独立块，仅允许 0x00~0x3F 的 v1 遗留区） ----
 * 不参与主参数槽（0x40/0xA0）的 magic/CRC/双备份机制，调用方自行保证块内格式。
 * 返回 1=成功。 */
uint8_t App_StoreRawRead(uint8_t addr, void *buf, uint16_t len);
uint8_t App_StoreRawWrite(uint8_t addr, const void *buf, uint16_t len);

/* ---- 防抖落盘 ---- */
void App_StoreRequestSave(void);   /* 标记待保存并重置静置计时 */
uint8_t App_StoreFlushSave(void);  /* 1=已写入/无待保存；0=仍在静置或写失败（下周期重试） */
uint8_t App_StoreIsSavePending(void);

#endif /* APP_STORE_H */
