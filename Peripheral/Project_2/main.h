/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Other includes and definitions
extern UART_HandleTypeDef huart1;   // UART handle
extern ADC_HandleTypeDef hadc1;      // ADC handle
extern IWDG_HandleTypeDef hiwdg;     // IWDG handle

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define REVERSE__Pin GPIO_PIN_13
#define REVERSE__GPIO_Port GPIOC
#define AC_SENSE_Pin GPIO_PIN_0
#define AC_SENSE_GPIO_Port GPIOA
#define TEMP_SENSE_Pin GPIO_PIN_1
#define TEMP_SENSE_GPIO_Port GPIOA
#define BATTERY_SENSE_Pin GPIO_PIN_2
#define BATTERY_SENSE_GPIO_Port GPIOA
#define CHARGER_SENSE_Pin GPIO_PIN_3
#define CHARGER_SENSE_GPIO_Port GPIOA
#define VOLTAGE_SET_Pin GPIO_PIN_5
#define VOLTAGE_SET_GPIO_Port GPIOA
#define CURRENT_SET_Pin GPIO_PIN_6
#define CURRENT_SET_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB

#define SHUTDOWN_Pin GPIO_PIN_10
#define SHUTDOWN_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_11
#define FAN_GPIO_Port GPIOB
#define AGING_Pin GPIO_PIN_13
#define AGING_GPIO_Port GPIOB
#define AC_LED_Pin GPIO_PIN_0
#define AC_LED_GPIO_Port GPIOD
#define LED_30__Pin GPIO_PIN_1
#define LED_30__GPIO_Port GPIOD
#define LED_50__Pin GPIO_PIN_2
#define LED_50__GPIO_Port GPIOD
#define LED_80__Pin GPIO_PIN_3
#define LED_80__GPIO_Port GPIOD
#define LED_100__Pin GPIO_PIN_3
#define LED_100__GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_8
#define RELAY_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
