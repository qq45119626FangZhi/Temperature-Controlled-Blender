/*
 * @file    app_modbus.h
 * @brief   P6：Modbus RTU 应用层——保持寄存器映射、协议栈启动与轮询。
 *
 * 寄存器表按《Doc/通信协议.md》第 6.3 节定稿（0x0000~0x001F，共 32 个保持寄存器）。
 * 读：由 App_ModbusSyncRegs() 把运行状态刷入寄存器区；
 * 写：统一走 App_CommWriteReg()（校验 + 面板仲裁 + 保护联锁后再下发）。
 *
 * 注意：FreeModbus 只实例化一路 RTU（UART4/RS485）；蓝牙通道由 app_ble 用同一张
 * 寄存器表独立解析，两者不共用协议栈实例。
 */
#ifndef APP_MODBUS_H
#define APP_MODBUS_H

#include <stdint.h>
#include "mb.h"

/* ---------------- 保持寄存器地址表 ---------------- */
#define APP_MB_REG_CUR_TEMP       0x0000U  /* 当前液温  int16 0.1℃   R     */
#define APP_MB_REG_TARGET_TEMP    0x0001U  /* 目标液温  int16 0.1℃   R/W   */
#define APP_MB_REG_CUR_SPEED      0x0002U  /* 当前转速  int16 RPM     R     */
#define APP_MB_REG_TARGET_SPEED   0x0003U  /* 目标转速  int16 RPM     R/W   */
#define APP_MB_REG_REMAIN_TIME    0x0004U  /* 剩余时间  uint16 s      R     */
#define APP_MB_REG_SYS_STATE      0x0005U  /* 系统状态  位域          R     */
#define APP_MB_REG_FAULT_CODE     0x0006U  /* 故障码    位域          R     */
#define APP_MB_REG_CTRL_CMD       0x0007U  /* 控制命令  位域          R/W   */
#define APP_MB_REG_PLATE_TEMP     0x0008U  /* 铝板温度  int16 0.1℃   R     */
#define APP_MB_REG_HEAT_PCT       0x0009U  /* 加热占空比 uint16 %     R     */
#define APP_MB_REG_MOTOR_PCT      0x000AU  /* 电机占空比 uint16 %     R     */
#define APP_MB_REG_SET_TIME       0x000BU  /* 设定时间  uint16 s      R/W   */
#define APP_MB_REG_FAN_STATE      0x000CU  /* 风扇状态  uint16        R     */
#define APP_MB_REG_DEV_ID_LO      0x000DU  /* 设备 ID 低 16 位        R     */
#define APP_MB_REG_DEV_ID_HI      0x000EU  /* 设备 ID 高 16 位        R     */
#define APP_MB_REG_FW_VERSION     0x000FU  /* 固件版本  高字节主/低字节次 R */
#define APP_MB_REG_SLAVE_ADDR     0x0010U  /* 从机地址  1~247         R/W   */
#define APP_MB_REG_BAUD_CODE      0x0011U  /* 波特率代码 0~4          R/W   */
#define APP_MB_REG_TOTAL_RUN_LO   0x0012U  /* 累计运行时间 低 16 位   R     */
#define APP_MB_REG_TOTAL_RUN_HI   0x0013U  /* 累计运行时间 高 16 位   R     */
#define APP_MB_REG_UI_FOCUS       0x0014U  /* 面板焦点项 0=速度 1=铝板 2=液温 3=时间 R/W */
#define APP_MB_REG_PLATE_LIMIT    0x0015U  /* 铝板上限温度 整数 ℃ 0~120  R/W */
#define APP_MB_REG_PID_MOTOR_KP   0x0018U  /* PID 只读镜像（D6 决定：只读预留） */
#define APP_MB_REG_PID_MOTOR_KI   0x0019U
#define APP_MB_REG_PID_MOTOR_KD   0x001AU
#define APP_MB_REG_PID_HEAT_KP    0x001BU
#define APP_MB_REG_PID_HEAT_KI    0x001CU
#define APP_MB_REG_PID_HEAT_KD    0x001DU

/* ---------------- P9：六轴 / 电机斜坡区块 0x0020~0x0039 ----------------
 * 注意：寄存器总数由 32 扩到 64（0x0000~0x003F）。
 * 旧上位机只读前 32 个，不受影响；新上位机可读完整 64 个。 */
#define APP_MB_REG_ACC_X          0x0020U  /* 加速度 X   int16 mg       R */
#define APP_MB_REG_ACC_Y          0x0021U  /* 加速度 Y   int16 mg       R */
#define APP_MB_REG_ACC_Z          0x0022U  /* 加速度 Z   int16 mg       R（静止≈+1000） */
#define APP_MB_REG_GYRO_X         0x0023U  /* 角速度 X   int16 0.1dps   R（已扣零偏） */
#define APP_MB_REG_GYRO_Y         0x0024U  /* 角速度 Y   int16 0.1dps   R */
#define APP_MB_REG_GYRO_Z         0x0025U  /* 角速度 Z   int16 0.1dps   R */
#define APP_MB_REG_ROLL           0x0026U  /* 横滚角     int16 0.1°     R */
#define APP_MB_REG_PITCH          0x0027U  /* 俯仰角     int16 0.1°     R */
#define APP_MB_REG_YAW            0x0028U  /* 相对偏航   int16 0.1°     R（陀螺积分，会漂移） */
#define APP_MB_REG_IMU_TEMP       0x0029U  /* 片内温度   int16 0.1℃     R */
#define APP_MB_REG_IMU_STATE      0x002AU  /* IMU 状态位 APP_IMU_ST_*   R */
#define APP_MB_REG_VIB_RMS        0x002BU  /* 振动 RMS   uint16 mg      R */
#define APP_MB_REG_ANG_ACCEL      0x002CU  /* 角加速度   int16 0.1dps/s R */
#define APP_MB_REG_RAMP_STATE     0x002DU  /* 斜坡状态   0停 1加速 2匀速 3减速 R */
#define APP_MB_REG_RAMP_CMD_RPM   0x002EU  /* 斜坡指令转速 int16 rpm    R */
#define APP_MB_REG_SLIP_ERR       0x002FU  /* 打滑偏差   int16 0.1%     R */
#define APP_MB_REG_IMU_CTRL       0x0030U  /* IMU 控制位 APP_MB_IMUCTL_* R/W */
#define APP_MB_REG_ACC_CONFIG     0x0031U  /* 加速度配置 低字节量程 高字节ODR R/W */
#define APP_MB_REG_GYRO_CONFIG    0x0032U  /* 陀螺配置   低字节量程 高字节ODR R/W */
#define APP_MB_REG_ALPHA_LIMIT    0x0033U  /* 加减速斜率 uint16 rpm/s   R/W */
#define APP_MB_REG_SLIP_TH        0x0034U  /* 打滑门限   uint16 0.1%    R/W */
#define APP_MB_REG_VIB_TH         0x0035U  /* 振动门限   uint16 mg      R/W */
#define APP_MB_REG_TILT_TH        0x0036U  /* 倾斜门限   uint16 0.1°    R/W */
#define APP_MB_REG_ACC_TIME_MS    0x0037U  /* 上次加速耗时 uint16 ms    R */
#define APP_MB_REG_DEC_TIME_MS    0x0038U  /* 上次减速耗时 uint16 ms    R */
#define APP_MB_REG_PEAK_ALPHA     0x0039U  /* 本次启动峰值角加速度 int16 0.1dps/s R */

/* ---------------- v1.3（A4/A2）：现场可调参数与实测统计 0x003A~0x003C ----------------
 * 目的：把「只能改宏重烧」的阈值搬到现场可调（App/网页/面板统一走这 3 个寄存器）。
 * 旧上位机只读前 32/64 个中的已知地址，新增地址不影响既有功能。 */
#define APP_MB_REG_HEAT_OC        0x003AU  /* 加热过流阈值 ADC raw 100~4095  R/W（落 EEPROM） */
#define APP_MB_REG_MOTOR_ASSIST   0x003BU  /* 堵转辅助位域 APP_MB_ASSIST_*  R/W（落 EEPROM） */
#define APP_MB_REG_TUNE_CTRL      0x003CU  /* 实测统计控制位域 APP_MB_TUNECTL_* R/W（不落盘） */

#define APP_MB_REG_HOLDING_NREGS  0x0040U  /* 共 64 个寄存器（P9 起） */

/* ---------------- 系统状态字（0x0005）位定义 ---------------- */
#define APP_MB_ST_MOTOR_RUNNING   0x0001U  /* bit0 电机运行中 */
#define APP_MB_ST_HEATING         0x0002U  /* bit1 加热输出中 */
#define APP_MB_ST_FAN_ON          0x0004U  /* bit2 风扇运行中 */
#define APP_MB_ST_FAULT           0x0008U  /* bit3 故障态 */
#define APP_MB_ST_TIMING          0x0010U  /* bit4 定时计时中 */
#define APP_MB_ST_RTC_VALID       0x0020U  /* bit5 RTC 有效 */
#define APP_MB_ST_PANEL_ACTIVE    0x0040U  /* bit6 面板刚被操作（8s 内） */
#define APP_MB_ST_BLE_ONLINE      0x0100U  /* bit8 蓝牙已连接（模块上报 CONNECT OK） */
#define APP_MB_ST_HEAT_DONE       0x0200U  /* bit9 液温已达目标，加热结束（P7 锁存） */
#define APP_MB_ST_IMU_VALID       0x0400U  /* bit10 P9：IMU 数据有效 */
#define APP_MB_ST_SLIP            0x0800U  /* bit11 P9：打滑告警（陀螺×编码器偏差） */
#define APP_MB_ST_HEAT_NO_RISE    0x1000U  /* bit12 A3：液体升温异常告警（仅告警，不属故障字） */

/* ---------------- 故障码（0x0006）位定义 ---------------- */
#define APP_MB_FLT_HEAT_HW        0x0001U  /* bit0 加热硬件故障锁存 */
#define APP_MB_FLT_HEAT_OC        0x0002U  /* bit1 加热过流 */
#define APP_MB_FLT_NTC_LIQUID     0x0004U  /* bit2 液体 NTC 异常 */
#define APP_MB_FLT_NTC_PLATE      0x0008U  /* bit3 铝板 NTC 异常 */
#define APP_MB_FLT_PLATE_LIMIT    0x0010U  /* bit4 铝板限温触发 */
#define APP_MB_FLT_MOTOR_STALL    0x0020U  /* bit5 电机堵转（P7 启用） */
#define APP_MB_FLT_EEPROM         0x0040U  /* bit6 EEPROM 异常 */
#define APP_MB_FLT_RTC_LOST       0x0080U  /* bit7 RTC 掉电丢时 */
#define APP_MB_FLT_IMU            0x0100U  /* bit8 P9：IMU 通信/自检异常 */

/* ---------------- 控制命令字（0x0007）位定义 ---------------- */
#define APP_MB_CMD_START          0x0001U  /* bit0 启动 */
#define APP_MB_CMD_STOP           0x0002U  /* bit1 停止 */
#define APP_MB_CMD_CLEAR_FAULT    0x0004U  /* bit2 清故障 */
#define APP_MB_CMD_SAVE_PARAM     0x0008U  /* bit3 立即保存参数 */
#define APP_MB_CMD_LOAD_DEFAULT   0x0010U  /* bit4 恢复默认参数 */
#define APP_MB_CMD_VALID_MASK     0x001FU

/* ---------------- IMU 控制字（0x0030）位定义 ---------------- */
#define APP_MB_IMUCTL_CAL_BIAS    0x0002U  /* bit1 启动零偏标定（1~2s 内保持静止） */
#define APP_MB_IMUCTL_SAVE        0x0004U  /* bit2 保存 IMU 参数到 EEPROM */
#define APP_MB_IMUCTL_CLEAR_EVT   0x0008U  /* bit3 清打滑锁存/峰值角加速度 */

/* ---------------- 堵转辅助位域（0x003B） ---------------- */
#define APP_MB_ASSIST_STALL_EN    0x0001U  /* bit0 1=启用振动辅助堵转判定（M5，默认 0） */

/* ---------------- 实测统计控制位域（0x003C，写后只保留 bit0） ---------------- */
#define APP_MB_TUNECTL_PRINT      0x0001U  /* bit0 1=周期打印 [TUNE] 统计行（默认 0，不落盘） */
#define APP_MB_TUNECTL_ONCE       0x0002U  /* bit1 立即打印一行（自清零） */
#define APP_MB_TUNECTL_CLEAR      0x0004U  /* bit2 清统计（自清零） */

/* 固件版本：高字节主版本、低字节次版本
 * v1.1 新增 0x0014 面板焦点 / 0x0015 铝板限温上限
 * v1.2（P9）寄存器表扩到 64 个：新增 0x0020~0x0039（六轴/姿态/斜坡/打滑）
 * v1.3（A2/A3/A4）新增 0x003A 加热过流阈值 / 0x003B 堵转辅助 / 0x003C 实测统计控制，
 *      状态字 bit12 = 液体升温异常告警（仅告警） */
#define APP_MB_FW_VERSION         0x0103U

/* 启动协议栈：addr=从机地址（1~247），baud=波特率。1=成功。 */
uint8_t App_ModbusStart(uint16_t addr, uint32_t baud);

/* 协议栈轮询（在 COMM 任务中循环调用，内部阻塞等待事件）。 */
void App_ModbusPoll(void);

/* 把当前运行状态刷入寄存器区（读回调里调用，保证数据新鲜）。 */
void App_ModbusSyncRegs(void);

/* 读写单个保持寄存器（供蓝牙通道复用；地址越界时读返回 0、写忽略）。 */
USHORT App_ModbusReadReg(USHORT usAddress);

#endif /* APP_MODBUS_H */
