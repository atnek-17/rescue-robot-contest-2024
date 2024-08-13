/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define enable_7_Pin GPIO_PIN_2
#define enable_7_GPIO_Port GPIOC
#define enable_6_Pin GPIO_PIN_3
#define enable_6_GPIO_Port GPIOC
#define Botton_Black_Pin GPIO_PIN_2
#define Botton_Black_GPIO_Port GPIOB
#define Botton_Black_EXTI_IRQn EXTI2_IRQn
#define Botton_Red_Pin GPIO_PIN_11
#define Botton_Red_GPIO_Port GPIOF
#define Botton_Red_EXTI_IRQn EXTI15_10_IRQn
#define Botton_Blue_Pin GPIO_PIN_12
#define Botton_Blue_GPIO_Port GPIOF
#define Botton_Blue_EXTI_IRQn EXTI15_10_IRQn
#define Enable_1_Pin GPIO_PIN_0
#define Enable_1_GPIO_Port GPIOG
#define Enable_2_Pin GPIO_PIN_1
#define Enable_2_GPIO_Port GPIOG
#define Enable_4_Pin GPIO_PIN_7
#define Enable_4_GPIO_Port GPIOE
#define Enable_3_5_Pin GPIO_PIN_8
#define Enable_3_5_GPIO_Port GPIOE
#define LED_White_Pin GPIO_PIN_10
#define LED_White_GPIO_Port GPIOE
#define LED_Green_Pin GPIO_PIN_11
#define LED_Green_GPIO_Port GPIOE
#define LED_Red_Pin GPIO_PIN_12
#define LED_Red_GPIO_Port GPIOE
#define LED_Yellow_Pin GPIO_PIN_13
#define LED_Yellow_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
char rxbuf[1];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
