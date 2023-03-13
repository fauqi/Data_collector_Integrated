/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define TRIG2_AKTUATOR2_Pin GPIO_PIN_2
#define TRIG2_AKTUATOR2_GPIO_Port GPIOA
#define TRIG1_AKTUATOR2_Pin GPIO_PIN_3
#define TRIG1_AKTUATOR2_GPIO_Port GPIOA
#define TRIG2_AKTUATOR1_Pin GPIO_PIN_4
#define TRIG2_AKTUATOR1_GPIO_Port GPIOA
#define TRIG1_AKTUATOR1_Pin GPIO_PIN_5
#define TRIG1_AKTUATOR1_GPIO_Port GPIOA
#define TRIG_ALARM_Pin GPIO_PIN_7
#define TRIG_ALARM_GPIO_Port GPIOA
#define GNDBMSWAKEUP1_Pin GPIO_PIN_2
#define GNDBMSWAKEUP1_GPIO_Port GPIOB
#define GNDBMSWAKEUP2_Pin GPIO_PIN_8
#define GNDBMSWAKEUP2_GPIO_Port GPIOA
#define TX_En_Pin GPIO_PIN_6
#define TX_En_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
