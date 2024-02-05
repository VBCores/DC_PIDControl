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
#include "stm32g4xx_hal.h"

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
extern uint32_t millis_k;
[[noreturn]] void main_cpp();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Vref_Pin GPIO_PIN_4
#define Vref_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOA
#define DIP_SW1_Pin GPIO_PIN_10
#define DIP_SW1_GPIO_Port GPIOB
#define DIP_SW2_Pin GPIO_PIN_11
#define DIP_SW2_GPIO_Port GPIOB
#define DIP_SW3_Pin GPIO_PIN_12
#define DIP_SW3_GPIO_Port GPIOB
#define DIP_SW4_Pin GPIO_PIN_13
#define DIP_SW4_GPIO_Port GPIOB
#define DIP_SW5_Pin GPIO_PIN_14
#define DIP_SW5_GPIO_Port GPIOB
#define DIP_SW6_Pin GPIO_PIN_15
#define DIP_SW6_GPIO_Port GPIOB
#define Encoder_2_Pin GPIO_PIN_7
#define Encoder_2_GPIO_Port GPIOC
#define DIP_SW7_Pin GPIO_PIN_8
#define DIP_SW7_GPIO_Port GPIOC
#define DIP_SW8_Pin GPIO_PIN_9
#define DIP_SW8_GPIO_Port GPIOC
#define Motor_A_Pin GPIO_PIN_8
#define Motor_A_GPIO_Port GPIOA
#define Motor_B_Pin GPIO_PIN_9
#define Motor_B_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define Sleep_Pin GPIO_PIN_3
#define Sleep_GPIO_Port GPIOB
#define Fault_Pin GPIO_PIN_5
#define Fault_GPIO_Port GPIOB
#define Fault_EXTI_IRQn EXTI9_5_IRQn
#define Encoder_1_Pin GPIO_PIN_6
#define Encoder_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
