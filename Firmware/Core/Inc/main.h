/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
//Define prototype
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
// Define GPIO pins for motor and valve control
#define valve_Pin GPIO_PIN_2
#define valve_GPIO_Port GPIOA

#define motor_Pin GPIO_PIN_3
#define motor_GPIO_Port GPIOA

// Define GPIO pins for buttons
#define bt_start1_Pin GPIO_PIN_5
#define bt_start1_GPIO_Port GPIOA

#define bt_start2_Pin GPIO_PIN_6
#define bt_start2_GPIO_Port GPIOA

#define bt_stop_Pin GPIO_PIN_7
#define bt_stop_GPIO_Port GPIOA

// Define ADC pins
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOA

#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOA

// Define motor states
#define on GPIO_PIN_SET
#define off GPIO_PIN_RESET

// Define states for motor control
#define startState 0
#define inflate1State 1  // Bơm hơi
#define inflate2State 2
#define deflateState 3   // Xả hơi
#define displayState 4
#define resetState 5

// Define states for Measure control
#define Sys_Measure 6    // �?o tâm thu
#define Sys_Cal 7        // Tính toán tâm thu
#define Rate_Measure 8   // �?o tỷ lệ
#define dias_Measure 9   // �?o tâm trương
#define dias_Cal 10      // Tính toán tâm trương
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
