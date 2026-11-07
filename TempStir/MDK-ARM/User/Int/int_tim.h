/*
 * int_tim.h
 * BSP 层 TIM 封装：PWM 输出（蜂鸣器 TIM1_CH1 / 加热 TIM2_CH1 / 电机 TIM4_CH3）
 * 的占空比设置，以及编码器（TIM3）计数值读取。
 * TIM 句柄 htim1/2/3/4 由 Core 层 MX_TIMx_Init 初始化，本封装负责启动与读写。
 */
#ifndef INT_TIM_H
#define INT_TIM_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "tim.h"

/* 启动所有 PWM 输出通道与编码器接口；应在调度器启动前调用一次。
 * 启动后各通道占空比均为 0（CubeMX 配置 Pulse=0），不会立即驱动负载。 */
void Int_TimInit(void);

/* 设置某路 PWM 占空比，percent: 0~100（整数百分比）。
 * 超出范围按边界钳制；0 表示停止输出。 */
void Int_TimPwmSetPercent(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t percent);

/* 读取编码器（TIM3）当前计数值（16bit，正负反映转向）。 */
int16_t Int_TimEncGetCount(void);

/* 复位编码器计数值为 0。 */
void Int_TimEncReset(void);

#endif /* INT_TIM_H */
