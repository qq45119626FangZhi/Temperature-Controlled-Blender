/*
 * dri_fan.h
 * 风扇引脚驱动（MOTOR_ON / PA1：高电平转动、低电平停止）。
 *
 * 依据 int_bsp_pins.h：
 *   MOTOR_ON_GPIO_Port / MOTOR_ON_Pin
 *   MOTOR_ON_ACTIVE_STATE   = GPIO_PIN_SET   （高 -> 转动）
 *   MOTOR_ON_INACTIVE_STATE = GPIO_PIN_RESET （低 -> 停止）
 * GPIO 电气属性（推挽输出）已由 Core/Src/gpio.c 的 MX_GPIO_Init 配置，
 * 本模块只负责写电平，不重复配置引脚。
 *
 * 本层不含任何温度判定/回差策略（35℃ 开 / 32℃ 关 / 5s 间隔在 App/app_fan.c）。
 */
#ifndef DRI_FAN_H
#define DRI_FAN_H

#include <stdint.h>

/* 引脚初始化：确保默认停止（MOTOR_ON 拉低）。建议在 vTaskStartScheduler() 之前调用。 */
void Fan_PinInit(void);

/* 直接置位/清除 MOTOR_ON 引脚：on != 0 转动，on == 0 停止。 */
void Fan_PinSet(uint8_t on);

#endif /* DRI_FAN_H */
