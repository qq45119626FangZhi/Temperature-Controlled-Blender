/*
 * @file    dri_buzzer.h
 * @brief   蜂鸣器模块（PA8 / TIM1_CH1，PWM 方波，50% 占空比）。
 */
#ifndef DRI_BUZZER_H
#define DRI_BUZZER_H

#include "FreeRTOS.h"
#include "timers.h"
#include <stdint.h>

/* 初始化（须在 vTaskStartScheduler 之前调用）：
 *  - 启动 TIM1_CH1 PWM（PA8），占空比固定 50%；
 *  - 初始静音（占空比 0%，蜂鸣器不响）；
 *  - 创建“短鸣”单次软件定时器。 */
void Buzzer_Init(void);

/* 短鸣（非阻塞）：启动一次软件定时器，鸣叫 buzz_ms 毫秒后由定时器回调自动停止。 */
void Buzzer_ShortBeep(uint32_t buzz_ms);

/* 长鸣（阻塞）：持续鸣叫 buzz_ms 毫秒（内部 vTaskDelay），须在任务上下文中调用。 */
void Buzzer_LongBeep(uint32_t buzz_ms);

/* 立即停止鸣叫（占空比置 0）。 */
void Buzzer_Stop(void);

#endif /* DRI_BUZZER_H */
