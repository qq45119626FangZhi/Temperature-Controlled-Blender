/*
 * int_tim.c
 * TIM 板级封装。CubeMX 已配好各 TIM 时基与通道（PWM/PWM/PWM/编码器），
 * 本封装负责启动外设并提供易用的占空比/计数读写接口。
 *
 * 各 PWM 时基：
 *   TIM1_CH1 (PA8 蜂鸣器)  ARR=17999  72MHz/18000 = 4kHz
 *   TIM2_CH1 (PA0 加热)    ARR=3599   72MHz/3600  = 20kHz（本文件把 CubeMX 的 1kHz 改到 20kHz）
 *   TIM4_CH3 (PB8 电机)    ARR=3599   72MHz/3600 = 20kHz
 *   TIM3 编码器 (PB4/5)    ARR=65535  TI12 模式
 *
 * 加热 PWM 频率说明：CubeMX 原配置为 PSC=71/ARR=999 → 1kHz，正好落在人耳最敏感的
 * 频段。24V 加热组件（CN1/CN2）与线束在 1kHz 开关下会发出明显“低鸣”，实测确认
 * 就是板上那个间歇性噪声（加热占空比间歇开关 → 声音断续 3~5s）。
 * 现改为 PSC=0/ARR=3599 → 20kHz，与电机 PWM 同频，超出听觉范围。
 * 注意：.ioc 已同步改为 20kHz；即使以后重新生成代码，本函数也会再次强制校正。
 */
#include "int_tim.h"

/* 加热 PWM 目标频率 20kHz：PSC=0、ARR=3599（72MHz/3600）。 */
#define INT_TIM_HEAT_PSC      (0U)
#define INT_TIM_HEAT_ARR      (3599U)

void Int_TimInit(void)
{
    /* 把加热 PWM 从 1kHz 提高到 20kHz（必须在启动 PWM 之前改：PSC/ARR 立即生效）。 */
    htim2.Init.Prescaler = INT_TIM_HEAT_PSC;
    htim2.Init.Period    = INT_TIM_HEAT_ARR;
    __HAL_TIM_SET_PRESCALER(&htim2, INT_TIM_HEAT_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim2, INT_TIM_HEAT_ARR);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);

    /* 启动 PWM 通道输出（占空比默认 0，即不驱动）。 */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* 蜂鸣器 PA8 */
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   /* 加热 PA0 */
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);   /* 电机 PB8 */

    /* 启动编码器接口（TIM3，PB4/5）；计数从当前值开始。 */
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void Int_TimPwmSetPercent(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t percent)
{
    uint32_t pulse;

    if (percent > 100U)
    {
        percent = 100U;
    }

    /* 占空比 = percent/100 * (ARR+1)；ARR 来自当前自动重装载值。 */
    pulse = ((uint32_t)htim->Instance->ARR + 1U) * (uint32_t)percent / 100U;
    if (pulse > (uint32_t)htim->Instance->ARR)
    {
        pulse = (uint32_t)htim->Instance->ARR;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)pulse);
}

int16_t Int_TimEncGetCount(void)
{
    return (int16_t)(htim3.Instance->CNT & 0xFFFFU);
}

void Int_TimEncReset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
}
