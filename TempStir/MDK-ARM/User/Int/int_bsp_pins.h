/*
* int_bsp_pins.h
* 原理图网络名到 STM32F103RCT6 GPIO 的唯一映射表。
*/
#ifndef BSP_PINS_H
#define BSP_PINS_H

#include "main.h"

/*
* 本文件只定义“网络连接与有效电平”，不配置 GPIO 电气属性。推挽、开漏、
* 上拉/下拉、复用和 ADC 模式统一由 Core/Src/gpio.c 及 HAL MSP 初始化负责；
* 应用层必须使用这些名称，不能再次硬编码端口、引脚号或有效电平。
*/

/* 四个按键：内部上拉，按下时接地，因此低电平有效。 */
#define KEY1_GPIO_Port   GPIOC
#define KEY1_Pin    GPIO_PIN_6
#define KEY2_GPIO_Port   GPIOC
#define KEY2_Pin    GPIO_PIN_7
#define KEY3_GPIO_Port   GPIOC
#define KEY3_Pin    GPIO_PIN_8
#define KEY4_GPIO_Port   GPIOC
#define KEY4_Pin    GPIO_PIN_9
#define KEY_PRESSED_STATE   GPIO_PIN_RESET

/* OLED：I2C1，PB6=SCL、PB7=SDA，复用开漏。 */
#define OLED_SCL_GPIO_Port  GPIOB
#define OLED_SCL_Pin   GPIO_PIN_6
#define OLED_SDA_GPIO_Port  GPIOB
#define OLED_SDA_Pin   GPIO_PIN_7

/* I2C2：板上第二路 I2C，PB10=SCL、PB11=SDA，复用开漏。与 OLED 的 I2C1 分开，
 * 挂载参数存储器件（EEPROM 类）。复用与电气属性由 I2C2 外设管理，应用层通过
 * hi2c2 访问；此处登记连接关系，便于接线核对。器件网络名以原理图为准。 */
#define EEPROM_SCL_GPIO_Port  GPIOB
#define EEPROM_SCL_Pin   GPIO_PIN_10
#define EEPROM_SDA_GPIO_Port  GPIOB
#define EEPROM_SDA_Pin   GPIO_PIN_11

/* LSM6DSM：SPI2，片选由普通推挽 GPIO 控制，中断输入位于 PC4/PC5。
 * SCK/MISO/MOSI 为 SPI2 复用引脚，收发由 SPI2 外设与 hspi2 完成，应用层不要
 * 再用 GPIO 方式读写，这里仅登记连接关系以与 LSM_CS 配套。 */
#define LSM_CS_GPIO_Port  GPIOB
#define LSM_CS_Pin    GPIO_PIN_12
#define LSM_SCK_GPIO_Port GPIOB
#define LSM_SCK_Pin   GPIO_PIN_13
#define LSM_MISO_GPIO_Port  GPIOB
#define LSM_MISO_Pin    GPIO_PIN_14
#define LSM_MOSI_GPIO_Port  GPIOB
#define LSM_MOSI_Pin    GPIO_PIN_15
#define LSM_INT1_GPIO_Port  GPIOC
#define LSM_INT1_Pin   GPIO_PIN_4
#define LSM_INT2_GPIO_Port  GPIOC
#define LSM_INT2_Pin   GPIO_PIN_5
#define LSM_CS_ACTIVE_STATE  GPIO_PIN_RESET
#define LSM_CS_INACTIVE_STATE  GPIO_PIN_SET

/* 蜂鸣器：TIM1_CH1 PWM 输出。 */
#define BUZZ_GPIO_Port   GPIOA
#define BUZZ_Pin    GPIO_PIN_8

/* 电机：PB4/PB5 为 TIM3 编码器输入；PB8 为 TIM4_CH3 复用（PWM 调速输出，
 * 时基 PSC=0/ARR=3599 -> 20kHz，上电占空比 0），
 * PB9 为 U7 的控制输出，实测是电机的**通电/使能**脚（高有效）。 */
#define M1_ENC_A_GPIO_Port  GPIOB
#define M1_ENC_A_Pin   GPIO_PIN_4
#define M1_ENC_B_GPIO_Port  GPIOB
#define M1_ENC_B_Pin   GPIO_PIN_5

#define MOTOR1_GPIO_Port  GPIOB
#define MOTOR1_Pin    GPIO_PIN_8
#define MOTOR2_GPIO_Port  GPIOB
#define MOTOR2_Pin    GPIO_PIN_9
/*
 * MOTOR2(PB9)：电机驱动器 U7 的**另一个控制输入**，必须**固定为低电平**。
 *
 * 结论来自 2026-10-08 的四次实机观察（PB8 电平 + PB9 电平 -> 现象）：
 *   PB9=低, PB8=低 -> 不转     （两个输入都低：不驱动）
 *   PB9=低, PB8=高 -> 转       （正常正转：PB8 的高电平就是"导通"）
 *   PB9=高, PB8=低 -> 转       （反向驱动！！高 PB9 + 低 PB8 会倒转）
 *   PB9=高, PB8=高 -> 不转     （两个输入都高：刹车）
 * 即 U7 是"双输入"结构（类似 H 桥 IN1/IN2）：PB8 给 PWM 调速，PB9 必须保持低，
 * 这样 PB8 的"高 = 导通、低 = 断开"才成立。
 *
 * 早前把 PB9 当成"电机使能脚"是误判：拉高它并不是"通电"，
 * 而是与 PB8 一起组成"反向/刹车"状态 —— 这正是当时看到"0% 占空比反而转"的原因。
 *
 * ⚠️ 因此：PB9 只在初始化时拉低一次，之后**不要再动**；
 *     严禁把"拉高 PB9"当作停机手段（那会造成反向驱动）。
 */
#define MOTOR2_FIXED_STATE  GPIO_PIN_RESET     /* 固定低：与 PB8 的 PWM 配合实现正转 */

/* 加热与风扇：PA0 为 TIM2_CH1 加热 PWM，PA1 只做风扇高低电平使能。 */
#define HEAT_PWM_GPIO_Port  GPIOA
#define HEAT_PWM_Pin   GPIO_PIN_0
#define MOTOR_ON_GPIO_Port  GPIOA
#define MOTOR_ON_Pin   GPIO_PIN_1
#define MOTOR_ON_ACTIVE_STATE  GPIO_PIN_SET
#define MOTOR_ON_INACTIVE_STATE GPIO_PIN_RESET

/* ADC1 输入：板载热敏、液体热敏、加热电流以及 24 V 分压检测。 */
#define BOARD_ADC_GPIO_Port  GPIOB
#define BOARD_ADC_Pin    GPIO_PIN_0
#define LIQUID_ADC_GPIO_Port GPIOB
#define LIQUID_ADC_Pin   GPIO_PIN_1
#define HEAT_I_ADC_GPIO_Port GPIOA
#define HEAT_I_ADC_Pin   GPIO_PIN_4
#define SUPPLY_24V_ADC_GPIO_Port GPIOC
#define SUPPLY_24V_ADC_Pin   GPIO_PIN_0

/* DS3231 INT/SQW 为开漏低有效输入，硬件已有外部上拉。 */
#define DS_INT_GPIO_Port  GPIOC
#define DS_INT_Pin    GPIO_PIN_12
#define DS_INT_ACTIVE_STATE  GPIO_PIN_RESET

/*
* U21 硬件故障锁存接口：FAULT_RST_N 为开漏复位输出，HEAT_FAULT_N 为
* 低有效锁存状态输入。释放开漏输出时写高，由外部上拉建立高电平。
*/
#define FAULT_RST_N_GPIO_Port  GPIOC
#define FAULT_RST_N_Pin   GPIO_PIN_2
#define FAULT_RST_ASSERT_STATE GPIO_PIN_RESET
#define FAULT_RST_RELEASE_STATE GPIO_PIN_SET
#define HEAT_FAULT_N_GPIO_Port GPIOC
#define HEAT_FAULT_N_Pin  GPIO_PIN_3
#define HEAT_FAULT_ACTIVE_STATE GPIO_PIN_RESET

/*
* 通信口：以下均为 USART/UART/CAN 的复用引脚，收发由各自外设完成，应用层
* 通过 huart1/huart2/huart4/hcan 句柄访问，不得再用 GPIO 方式读写。登记在
* 此仅用于接线核对与故障排查。
*   USART1 PA9/PA10   115200 -> 日志/调试口（printf 重定向目标）
*   USART2 PA2/PA3    9600(自动识别) -> 蓝牙（PA2=BL-RXD，PA3=BL-TXD）
*   UART4  PC10/PC11    9600 -> Modbus RTU/RS485（PC10=MCU_485_TX，PC11=MCU_485_RX）
*   CAN    PA11/PA12          -> CAN 协议通道
* 依据：Doc/SCH_主控_2_2026-08-27new.pdf（早期文档把 USART2/UART4 写成相反）
*/
#define DEBUG_TX_GPIO_Port  GPIOA
#define DEBUG_TX_Pin    GPIO_PIN_9
#define DEBUG_RX_GPIO_Port  GPIOA
#define DEBUG_RX_Pin    GPIO_PIN_10

#define MODBUS_TX_GPIO_Port GPIOC
#define MODBUS_TX_Pin   GPIO_PIN_10
#define MODBUS_RX_GPIO_Port GPIOC
#define MODBUS_RX_Pin   GPIO_PIN_11

#define BT_TX_GPIO_Port   GPIOA
#define BT_TX_Pin     GPIO_PIN_2
#define BT_RX_GPIO_Port   GPIOA
#define BT_RX_Pin     GPIO_PIN_3

#define CAN_RX_GPIO_Port  GPIOA
#define CAN_RX_Pin    GPIO_PIN_11
#define CAN_TX_GPIO_Port  GPIOA
#define CAN_TX_Pin    GPIO_PIN_12

/*
* 系统保留引脚（不提供别名，应用层不得占用）：
*   PA13/PA14 -> SWDIO/SWCLK 调试口（SYS Debug = Serial Wire）
*   PD0/PD1   -> HSE 8MHz 外部晶振（PLL 倍频 9 后为 72MHz 系统时钟）
*/

#endif /* BSP_PINS_H */
