/*
 * utils_pid.h
 * 通用 PID 控制器（位置式），供温控、转速闭环复用。
 * 特性：积分限幅 + 输出限幅 + 抗积分饱和 + 微分先行（对测量值微分，抑制设定值突变冲击）。
 * 参数（Kp/Ki/Kd）按决策 5 不预置凭空数值，由 P3 实测整定后经 App_PidSetGains 配置。
 */
#ifndef UTILS_PID_H
#define UTILS_PID_H

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float out_min;      /* 输出下限 */
    float out_max;      /* 输出上限 */
    float i_min;        /* 积分项下限（抗饱和） */
    float i_max;        /* 积分项上限 */

    float integral;     /* 积分累积 */
    float prev_meas;    /* 上次测量值（微分先行） */
    float dt;           /* 采样周期(s) */

    uint8_t inited;
} AppPid_t;

/* 初始化：设定增益、输出/积分限幅与采样周期。 */
void App_PidInit(AppPid_t *pid, float kp, float ki, float kd,
                 float out_min, float out_max, float i_min, float i_max, float dt);

/* 运行时更新增益（实测整定用）。 */
void App_PidSetGains(AppPid_t *pid, float kp, float ki, float kd);

/* 清空内部状态（积分/上次测量）。 */
void App_PidReset(AppPid_t *pid);

/* 单次计算：setpoint 目标值，meas 当前测量值，返回限幅后的控制量。 */
float App_PidCompute(AppPid_t *pid, float setpoint, float meas);

#endif /* UTILS_PID_H */
