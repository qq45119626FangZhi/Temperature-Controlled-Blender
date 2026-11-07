/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SUPPLY_24V_ADC_Pin GPIO_PIN_0
#define SUPPLY_24V_ADC_GPIO_Port GPIOC
#define FAULT_RST_N_Pin GPIO_PIN_2
#define FAULT_RST_N_GPIO_Port GPIOC
#define HEAT_FAULT_N_Pin GPIO_PIN_3
#define HEAT_FAULT_N_GPIO_Port GPIOC
#define HEAT_PWM_Pin GPIO_PIN_0
#define HEAT_PWM_GPIO_Port GPIOA
#define MOTOR_ON_Pin GPIO_PIN_1
#define MOTOR_ON_GPIO_Port GPIOA
#define HEAT_I_ADC_Pin GPIO_PIN_4
#define HEAT_I_ADC_GPIO_Port GPIOA
#define LSM_INT1_Pin GPIO_PIN_4
#define LSM_INT1_GPIO_Port GPIOC
#define LSM_INT1_EXTI_IRQn EXTI4_IRQn
#define LSM_INT2_Pin GPIO_PIN_5
#define LSM_INT2_GPIO_Port GPIOC
#define LSM_INT2_EXTI_IRQn EXTI9_5_IRQn
#define BOARD_ADC_Pin GPIO_PIN_0
#define BOARD_ADC_GPIO_Port GPIOB
#define LIQUID_ADC_Pin GPIO_PIN_1
#define LIQUID_ADC_GPIO_Port GPIOB
#define LSM_NSS_Pin GPIO_PIN_12
#define LSM_NSS_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_6
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_7
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_8
#define KEY3_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_9
#define KEY4_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_8
#define BUZZ_GPIO_Port GPIOA
#define DS_INT_Pin GPIO_PIN_12
#define DS_INT_GPIO_Port GPIOC
#define DS_INT_EXTI_IRQn EXTI15_10_IRQn
#define M1_ENC_A_Pin GPIO_PIN_4
#define M1_ENC_A_GPIO_Port GPIOB
#define M1_ENC_B_Pin GPIO_PIN_5
#define M1_ENC_B_GPIO_Port GPIOB
#define MOTOR1_Pin GPIO_PIN_8
#define MOTOR1_GPIO_Port GPIOB
#define MOTOR2_Pin GPIO_PIN_9
#define MOTOR2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
