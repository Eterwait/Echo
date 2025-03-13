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
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

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
#define BOOT1_Pin LL_GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define Sense_Lvl_1_Pin LL_GPIO_PIN_12
#define Sense_Lvl_1_GPIO_Port GPIOB
#define Sense_Lvl_2_Pin LL_GPIO_PIN_13
#define Sense_Lvl_2_GPIO_Port GPIOB
#define Sense_Lvl_3_Pin LL_GPIO_PIN_14
#define Sense_Lvl_3_GPIO_Port GPIOB
#define EXTI15_10_Selecting_the_sensitivity_mode_Pin LL_GPIO_PIN_15
#define EXTI15_10_Selecting_the_sensitivity_mode_GPIO_Port GPIOB
#define EXTI15_10_Selecting_the_sensitivity_mode_EXTI_IRQn EXTI15_10_IRQn
#define RE_DE_Pin LL_GPIO_PIN_8
#define RE_DE_GPIO_Port GPIOC
#define Buzzer_Pin LL_GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define En_right_motor_Pin LL_GPIO_PIN_10
#define En_right_motor_GPIO_Port GPIOC
#define Dir_right_motor_Pin LL_GPIO_PIN_11
#define Dir_right_motor_GPIO_Port GPIOC
#define En_left_motor_Pin LL_GPIO_PIN_3
#define En_left_motor_GPIO_Port GPIOB
#define Dir_left_motor_Pin LL_GPIO_PIN_5
#define Dir_left_motor_GPIO_Port GPIOB
#define EXTI9_5_start_dataset_collection_Pin LL_GPIO_PIN_8
#define EXTI9_5_start_dataset_collection_GPIO_Port GPIOB
#define EXTI9_5_start_dataset_collection_EXTI_IRQn EXTI9_5_IRQn
#define start_dataset_collection_led_Pin LL_GPIO_PIN_9
#define start_dataset_collection_led_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
