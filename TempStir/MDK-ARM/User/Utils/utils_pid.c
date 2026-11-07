/*
 * utils_pid.c
 * 位置式 PID 实现。微分项采用“微分先行”（对测量值求导），避免设定值阶跃时
 * 微分冲击；积分项限幅并在输出饱和时做抗饱和处理。
 */
#include "utils_pid.h"

void App_PidInit(AppPid_t *pid, float kp, float ki, float kd,
                 float out_min, float out_max, float i_min, float i_max, float dt)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_min  = i_min;
    pid->i_max  = i_max;
    pid->dt     = (dt > 0.0f) ? dt : 0.001f;

    pid->integral  = 0.0f;
    pid->prev_meas = 0.0f;
    pid->inited    = 1U;
}

void App_PidSetGains(AppPid_t *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void App_PidReset(AppPid_t *pid)
{
    if (pid == 0)
    {
        return;
    }
    pid->integral  = 0.0f;
    pid->prev_meas = 0.0f;
}

float App_PidCompute(AppPid_t *pid, float setpoint, float meas)
{
    float error;
    float deriv;
    float output;
    float p_term;

    if ((pid == 0) || (pid->inited == 0U))
    {
        return 0.0f;
    }

    error = setpoint - meas;

    /* 积分累积（先累加，再限幅）。 */
    pid->integral += pid->ki * error * pid->dt;
    if (pid->integral > pid->i_max)
    {
        pid->integral = pid->i_max;
    }
    if (pid->integral < pid->i_min)
    {
        pid->integral = pid->i_min;
    }

    /* 微分先行：对测量值变化率求导（取负，使测量上升时抑制输出）。 */
    deriv = -(meas - pid->prev_meas) / pid->dt;

    p_term = pid->kp * error;
    output = p_term + pid->integral + pid->kd * deriv;

    /* 输出限幅 + 抗积分饱和（饱和且误差同向时回退积分）。 */
    if (output > pid->out_max)
    {
        output = pid->out_max;
        if (error > 0.0f)
        {
            pid->integral -= pid->ki * error * pid->dt;
        }
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
        if (error < 0.0f)
        {
            pid->integral -= pid->ki * error * pid->dt;
        }
    }

    pid->prev_meas = meas;

    return output;
}
