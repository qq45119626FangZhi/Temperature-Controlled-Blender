/*
 * dri_fan.c
 * 风扇引脚驱动实现：只写 MOTOR_ON(PA1) 电平，无任何温度策略。
 */
#include "dri_fan.h"
#include "int_bsp_pins.h"   /* MOTOR_ON_* 与有效电平定义（内含 HAL 头） */

/* 初始化：默认停止（拉低引脚）。 */
void Fan_PinInit(void)
{
    Fan_PinSet(0U);
}

/* 置位/清除引脚：on != 0 转动，on == 0 停止。 */
void Fan_PinSet(uint8_t on)
{
    HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin,
                      (on != 0U) ? MOTOR_ON_ACTIVE_STATE : MOTOR_ON_INACTIVE_STATE);
}
