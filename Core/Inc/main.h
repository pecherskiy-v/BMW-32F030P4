/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void getEndPointStatus();
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define endPointA_Pin GPIO_PIN_0
#define endPointA_GPIO_Port GPIOA
#define endPointB_Pin GPIO_PIN_1
#define endPointB_GPIO_Port GPIOA
#define back_Pin GPIO_PIN_2
#define back_GPIO_Port GPIOA
#define back_EXTI_IRQn EXTI2_3_IRQn
#define forward_Pin GPIO_PIN_3
#define forward_GPIO_Port GPIOA
#define forward_EXTI_IRQn EXTI2_3_IRQn
#define aeration_Pin GPIO_PIN_4
#define aeration_GPIO_Port GPIOA
#define aeration_EXTI_IRQn EXTI4_15_IRQn
#define outA_Pin GPIO_PIN_5
#define outA_GPIO_Port GPIOA
#define outB_Pin GPIO_PIN_6
#define outB_GPIO_Port GPIOA
#define PWM_Pin GPIO_PIN_7
#define PWM_GPIO_Port GPIOA
#define motorCurrentSense_Pin GPIO_PIN_1
#define motorCurrentSense_GPIO_Port GPIOB
#define statusLed_Pin GPIO_PIN_9
#define statusLed_GPIO_Port GPIOA
#define motorEN_DIAG_Pin GPIO_PIN_10
#define motorEN_DIAG_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define leftRotation 1                    // todo биты левого вращение
#define rightRotation 2                   // todo биты правого вращения
#define rotationStopped ((uint8_t)0x00U)  // togo биты остановки

#define aeration    ((uint8_t)0x03U)  // 0011 A=1 B=1 првоертивание
#define toAeration  ((uint8_t)0x01U)  // 0001 A=1 B=0 в процессе открытия на првоертивание
#define closed      ((uint8_t)0x00U)  // 0000 A=0 B=0 закрыт
#define toOpen      ((uint8_t)0x02U)  // 0010 A=0 B=1 в процессе полного открытия
#define open        ((uint8_t)0x03U)  // 0011 A=1 B=1 полностью открылся

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
