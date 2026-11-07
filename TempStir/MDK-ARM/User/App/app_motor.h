/*
 * app_motor.h
 * 搅拌电机控制（App 层）：驱动 TIM4_CH3 (PB8) PWM 调速（PB9 固定低已由 GPIO 配置），
 * 读取 TIM3 编码器计数并计算 RPM。PID 调速闭环在 P3 实现。
 * 电机参数（减速比/编码器线数）为决策 7 待器件资料，先用占位值（TODO）。
 */
#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* 电机参数（决策 7：减速比/编码器线数待器件资料，TODO 占位）
 * 速度闭环与 UI 显示的 RPM 均为【电机轴】转速（每分钟转数），
 * 不除减速比；需要输出轴转速时用 电机轴RPM / APP_MOTOR_GEAR_RATIO 换算。 */
#define APP_MOTOR_ENCODER_PPR   44      /* 每转脉冲 = 线数*4（正交），TODO 按资料修正 */
#define APP_MOTOR_GEAR_RATIO    30      /* 减速比，TODO 按资料修正 */
#define APP_MOTOR_PWM_MAX       100U
#define APP_MOTOR_PWM_LIMIT     100U    /* 占空比硬上限(%)：已放开到 100，使 0~500 设定值全程可调 */
#define APP_MOTOR_RPM_INVERT    1       /* 1=转速符号取反（编码器 A/B 相与电机转向定义相反时用） */

void App_MotorInit(void);                       /* 复位编码器计数 */
void App_MotorSetSpeedPercent(uint8_t pct);     /* 0~100 -> TIM4_CH3 */
uint8_t App_MotorGetSpeedPercent(void);
int32_t App_MotorCalcRpm(void);                 /* 基于两次采样计数差计算 RPM（参数 TODO） */
void App_MotorEmergencyStop(void);

/* ---- P9-3（M1）：速度斜坡（S/梯形速度曲线） ----
 * 做法：把“目标转速”经斜率限制生成“指令转速”，PID 跟随指令值；
 *       0 = 关闭，行为与 P8 完全一致（指令值 = 目标值）。
 * 斜率默认 600 rpm/s（200ms 周期 → 每周期 120rpm）。 */
#define APP_MOTOR_RAMP_ENABLE       1U     /* 1 = 启用速度斜坡 */
#define APP_MOTOR_RAMP_ACC_RPM_S    600U   /* 加速斜率(rpm/s) */
#define APP_MOTOR_RAMP_DEC_RPM_S    600U   /* 减速斜率(rpm/s) */
#define APP_MOTOR_CTRL_PERIOD_MS    200U   /* 控制周期，与 CTRL 任务一致 */

/* ---- P9-3（M5）：振动辅助堵转判定 ----
 * 开启后：有振动（IMU 检测到通电抖动）时堵转判定时间由 3s 缩短到 1s。
 * A8 起改为「编译开关 + 运行期开关」两级，默认行为仍是关闭：
 *   APP_MOTOR_STALL_ASSIST_ENABLE  = 1 只表示“把辅助判定代码编译进来”；
 *   APP_MOTOR_STALL_ASSIST_DEFAULT = 0 表示上电默认不启用（与旧版行为完全一致）。
 * 运行期由 0x003B bit0（或 EEPROM 的 IMU 参数 slipAssistStallEn）开启/关闭。 */
#define APP_MOTOR_STALL_ASSIST_ENABLE  1U
#define APP_MOTOR_STALL_ASSIST_DEFAULT 0U
#define APP_MOTOR_STALL_ASSIST_MS     1000U

/* 运行期开关（掉电保持由调用方负责写 EEPROM） */
void    App_MotorSetStallAssistEnable(uint8_t en);   /* 1=启用振动辅助判定 */
uint8_t App_MotorGetStallAssistEnable(void);

/* 斜坡状态（对应 0x002D RAMP_STATE） */
#define APP_MOTOR_RAMP_STOP     0U
#define APP_MOTOR_RAMP_ACC      1U
#define APP_MOTOR_RAMP_CRUISE   2U
#define APP_MOTOR_RAMP_DEC      3U

void    App_MotorSetRampRate(uint16_t accRpmS, uint16_t decRpmS);  /* 0 或越界按默认/钳制 */
uint16_t App_MotorGetRampAccRpmS(void);
uint16_t App_MotorGetRampDecRpmS(void);
int32_t App_MotorGetCmdRpm(void);        /* 斜坡当前指令转速(rpm) */
uint16_t App_MotorGetRampState(void);    /* APP_MOTOR_RAMP_* */
uint32_t App_MotorGetAccTimeMs(void);    /* M3：上次“指令值爬升到目标”的实测耗时 */
uint32_t App_MotorGetDecTimeMs(void);    /* M3：上次“指令值降到 0”的实测耗时 */
int32_t App_MotorGetLastRpm(void);       /* 最近一次编码器测得的转速（不触发采样，供 IMU 打滑校验） */
void    App_MotorSetStallAssist(uint8_t vibDetected);  /* M5：由 CTRL 任务每周期喂入 */

/* ---- P7：堵转检测（故障位 bit5 = APP_MB_FLT_MOTOR_STALL） ----
 * 判据（每 200ms 控制周期评估一次，全部为独立新增逻辑，不改变调速行为）：
 *   - 有目标转速（>0）；
 *   - 输出占空比已 ≥ APP_MOTOR_STALL_PCT_MIN（说明 PID 在持续推）；
 *   - 实测转速 < 「目标×RATIO%」且 < 绝对门限 RPM_MIN；
 *   - 上述条件连续保持 APP_MOTOR_STALL_MS 毫秒。
 * 动作：立即 PWM=0 并锁存（目标转速保留，UI 仍显示 Running 便于定位）；
 * 锁存期间 App_MotorControlStep 不再输出。恢复：把目标转速设为 0（面板 KEY4 停止）
 * 或上位机写 0x0007 清故障位。 */
#define APP_MOTOR_STALL_PCT_MIN    40U     /* 占空比门限(%)：低于此值不判堵转（软启动/轻载阶段） */
#define APP_MOTOR_STALL_RPM_RATIO  25U     /* 实测 < 目标×25% 视为转不起来 */
#define APP_MOTOR_STALL_RPM_MIN    10      /* 且实测 < 10 RPM（低目标时用绝对门限兜底） */
#define APP_MOTOR_STALL_MS         3000U   /* 连续满足判据 3s 判定堵转 */
uint8_t App_MotorIsStalled(void);          /* 1=堵转已锁存（通信故障位/联锁用） */
void    App_MotorClearStall(void);         /* 人工清堵转锁存 */

/* ---- P3 转速闭环 ---- */
void App_MotorSetTargetRpm(int32_t targetRpm);
int32_t App_MotorGetTargetRpm(void);
void App_MotorControlStep(void);        /* 单步 PID：读 RPM -> 电机 PWM，周期调用（含软启动限斜率） */
void App_MotorPidSetGains(float kp, float ki, float kd);

/* ---- 校准：编码器每转计数（线数×4） ----
 * 默认取 APP_MOTOR_ENCODER_PPR；实测标定值由 app_store 保存并在启动时下发。 */
void App_MotorSetEncoderPpr(int32_t ppr);
int32_t App_MotorGetEncoderPpr(void);

/* ---- P3 验收：转速性能自动评估 ----
 * ControlStep 内部按 200ms 周期自动统计；目标变更自动重新开始。
 * 判据：稳态带 = 目标 ±5%（至少 ±1RPM），连续 1s 在带内视为稳定，
 *       稳定后再累计 10s 求平均转速作为稳态值。 */
void     App_MotorPerfReset(void);                 /* 手动复位统计（目标变更时自动调用） */
uint8_t  App_MotorPerfIsReady(void);               /* 1=已得出稳态结论，可打印评估结果 */
uint8_t  App_MotorPerfIsTimeout(void);             /* 1=60s 未进入稳态带（占空比不足/堵转等） */
int32_t  App_MotorPerfGetMaxRpm(void);             /* 峰值转速（含超调） */
int32_t  App_MotorPerfGetSteadyRpm(void);          /* 稳态平均转速 */
int32_t  App_MotorPerfGetSteadyError(void);        /* 稳态误差 = 目标 - 稳态平均 */
int32_t  App_MotorPerfGetOvershootPermille(void);  /* 超调量（相对目标的千分比，无超调为 0） */
uint32_t App_MotorPerfGetRiseMs(void);             /* 上升时间：首次达到目标 90% 的耗时(ms) */
uint32_t App_MotorPerfGetSettleMs(void);           /* 调节时间：进入 ±5% 带并保持 1s 的耗时(ms) */
int32_t  App_MotorGetCountRate(void);              /* 编码器计数率(counts/s)：用于反推 PPR 与减速比 */

#endif /* APP_MOTOR_H */
