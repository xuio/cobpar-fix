/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define DIP0_Pin GPIO_PIN_3
#define DIP0_GPIO_Port GPIOA
#define DIP1_Pin GPIO_PIN_4
#define DIP1_GPIO_Port GPIOA
#define DIP2_Pin GPIO_PIN_5
#define DIP2_GPIO_Port GPIOA
#define DIP3_Pin GPIO_PIN_6
#define DIP3_GPIO_Port GPIOA
#define DIP4_Pin GPIO_PIN_7
#define DIP4_GPIO_Port GPIOA
#define DIP5_Pin GPIO_PIN_0
#define DIP5_GPIO_Port GPIOB
#define DIP6_Pin GPIO_PIN_1
#define DIP6_GPIO_Port GPIOB
#define DIP7_Pin GPIO_PIN_2
#define DIP7_GPIO_Port GPIOB
#define DIP8_Pin GPIO_PIN_10
#define DIP8_GPIO_Port GPIOB
#define PWM4_Pin GPIO_PIN_8
#define PWM4_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_9
#define PWM3_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_10
#define PWM2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_11
#define PWM1_GPIO_Port GPIOA
#define PWMF_Pin GPIO_PIN_4
#define PWMF_GPIO_Port GPIOB
#define DMX_TX_Pin GPIO_PIN_6
#define DMX_TX_GPIO_Port GPIOB
#define DMX_RX_Pin GPIO_PIN_7
#define DMX_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/