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
#include "stm32f4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define frontIn2_Pin GPIO_PIN_0
#define frontIn2_GPIO_Port GPIOC
#define frontIn3_Pin GPIO_PIN_1
#define frontIn3_GPIO_Port GPIOC
#define frontIn4_Pin GPIO_PIN_2
#define frontIn4_GPIO_Port GPIOC
#define frontIn1_Pin GPIO_PIN_3
#define frontIn1_GPIO_Port GPIOC
#define topRightEncA_Pin GPIO_PIN_0
#define topRightEncA_GPIO_Port GPIOA
#define topRightEncB_Pin GPIO_PIN_1
#define topRightEncB_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define meLeduwu_Pin GPIO_PIN_5
#define meLeduwu_GPIO_Port GPIOA
#define topLeftSpeed_Pin GPIO_PIN_6
#define topLeftSpeed_GPIO_Port GPIOA
#define topRightSpeed_Pin GPIO_PIN_7
#define topRightSpeed_GPIO_Port GPIOA
#define bottomLeftSpeed_Pin GPIO_PIN_0
#define bottomLeftSpeed_GPIO_Port GPIOB
#define bottomRightSpeed_Pin GPIO_PIN_1
#define bottomRightSpeed_GPIO_Port GPIOB
#define bottomRightEncA_Pin GPIO_PIN_8
#define bottomRightEncA_GPIO_Port GPIOA
#define bottomRightEncB_Pin GPIO_PIN_9
#define bottomRightEncB_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define bottomLeftEncA_Pin GPIO_PIN_15
#define bottomLeftEncA_GPIO_Port GPIOA
#define backIn4_Pin GPIO_PIN_10
#define backIn4_GPIO_Port GPIOC
#define backIn3_Pin GPIO_PIN_11
#define backIn3_GPIO_Port GPIOC
#define backIn2_Pin GPIO_PIN_12
#define backIn2_GPIO_Port GPIOC
#define backIn1_Pin GPIO_PIN_2
#define backIn1_GPIO_Port GPIOD
#define bottomLeftEncB_Pin GPIO_PIN_3
#define bottomLeftEncB_GPIO_Port GPIOB
#define topLeftEncA_Pin GPIO_PIN_6
#define topLeftEncA_GPIO_Port GPIOB
#define topLeftEncB_Pin GPIO_PIN_7
#define topLeftEncB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
