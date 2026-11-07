/*
 * @file    dri_buzzer.c
 * @brief   蜂鸣器驱动：PA8 / TIM1_CH1，PWM 方波，50% 占空比。
 */

#include "dri_buzzer.h"
#include "tim.h"    /* htim1、TIM_CHANNEL_1（经 tim.h -> main.h 引入 HAL） */

/* TIM1 自动重装值（与 CubeMX 配置一致：Period = 17999，即 18000 个计数）。
 * 50% 占空比 => 比较值 = (ARR + 1) / 2 = 9000。
 * PWM 频率 = TIM1_CLK / (ARR + 1)；TIM1_CLK≈72MHz 时约 4kHz。 */
#define BUZZER_TIM_ARR           (17999U)
#define BUZZER_TIM_PULSE_50PCT   (((BUZZER_TIM_ARR) + 1U) / 2U)   /* = 9000 */

/* 短鸣用的单次软件定时器（到期回调中停止鸣叫）。 */
static TimerHandle_t buzzer_short_timer = NULL;

/* 私有函数声明 */
static void Buzzer_On(void);
static void Buzzer_ShortTimerCallback(TimerHandle_t xTimer);

/* 打开鸣叫：占空比 50%。 */
static void Buzzer_On(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BUZZER_TIM_PULSE_50PCT);
}

/* 关闭鸣叫：占空比 0%（PWM 仍运行，但输出恒低，蜂鸣器不响）。 */
void Buzzer_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
}

/* 软件定时器回调：短鸣时间到，停止鸣叫。 */
static void Buzzer_ShortTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    Buzzer_Stop();
}

void Buzzer_Init(void)
{
    /* 先静音，再启动 PWM 输出。 */
    Buzzer_Stop();
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    /* TIM1 为高级定时器，需使能主输出（MOE）才能驱动 PA8；HAL 一般已置位，这里再确保一次。 */
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 创建短鸣单次软件定时器（实际周期由 Buzzer_ShortBeep 动态设置）。 */
    if (buzzer_short_timer == NULL) {
        buzzer_short_timer = xTimerCreate("BUZZ_S",
                                          pdMS_TO_TICKS(100),
                                          pdFALSE,          /* 单次触发 */
                                          NULL,
                                          Buzzer_ShortTimerCallback);
    }
}

void Buzzer_ShortBeep(uint32_t buzz_ms)
{
    if (buzzer_short_timer == NULL) {
        return;
    }

    Buzzer_On();
    (void)xTimerChangePeriod(buzzer_short_timer, pdMS_TO_TICKS(buzz_ms), 0);
    (void)xTimerStart(buzzer_short_timer, 0);
}

void Buzzer_LongBeep(uint32_t buzz_ms)
{
    Buzzer_On();
    vTaskDelay(pdMS_TO_TICKS(buzz_ms));
    Buzzer_Stop();
}
