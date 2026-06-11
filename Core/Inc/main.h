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
#include "stm32l1xx_hal.h"

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
#define topLeftR_EN_Pin GPIO_PIN_0
#define topLeftR_EN_GPIO_Port GPIOC
#define topLeftL_EN_Pin GPIO_PIN_1
#define topLeftL_EN_GPIO_Port GPIOC
#define topRightL_EN_Pin GPIO_PIN_2
#define topRightL_EN_GPIO_Port GPIOC
#define topRightR_EN_Pin GPIO_PIN_3
#define topRightR_EN_GPIO_Port GPIOC
#define bottomRightEncA_Pin GPIO_PIN_0
#define bottomRightEncA_GPIO_Port GPIOA
#define bottomRightEncB_Pin GPIO_PIN_1
#define bottomRightEncB_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define solenoid_Pin GPIO_PIN_4
#define solenoid_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define topRightEncA_Pin GPIO_PIN_6
#define topRightEncA_GPIO_Port GPIOA
#define topRightEncB_Pin GPIO_PIN_7
#define topRightEncB_GPIO_Port GPIOA
#define leftRightIr_Pin GPIO_PIN_5
#define leftRightIr_GPIO_Port GPIOC
#define backMiddleIR_Pin GPIO_PIN_1
#define backMiddleIR_GPIO_Port GPIOB
#define backRightIR_Pin GPIO_PIN_2
#define backRightIR_GPIO_Port GPIOB
#define rightLeftIR_Pin GPIO_PIN_12
#define rightLeftIR_GPIO_Port GPIOB
#define upperColourIn_Pin GPIO_PIN_13
#define upperColourIn_GPIO_Port GPIOB
#define downColourIn_Pin GPIO_PIN_14
#define downColourIn_GPIO_Port GPIOB
#define servoOutput_Pin GPIO_PIN_15
#define servoOutput_GPIO_Port GPIOB
#define leftMiddleIR_Pin GPIO_PIN_6
#define leftMiddleIR_GPIO_Port GPIOC
#define backLeftIR_Pin GPIO_PIN_7
#define backLeftIR_GPIO_Port GPIOC
#define leftLeftIR_Pin GPIO_PIN_8
#define leftLeftIR_GPIO_Port GPIOC
#define frontLeftIR_Pin GPIO_PIN_8
#define frontLeftIR_GPIO_Port GPIOA
#define frontMiddleIR_Pin GPIO_PIN_9
#define frontMiddleIR_GPIO_Port GPIOA
#define frontRightIR_Pin GPIO_PIN_10
#define frontRightIR_GPIO_Port GPIOA
#define rightMiddleIR_Pin GPIO_PIN_11
#define rightMiddleIR_GPIO_Port GPIOA
#define rightRightIr_Pin GPIO_PIN_12
#define rightRightIr_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define topLeftEncB_Pin GPIO_PIN_15
#define topLeftEncB_GPIO_Port GPIOA
#define bottomRightL_EN_Pin GPIO_PIN_10
#define bottomRightL_EN_GPIO_Port GPIOC
#define bottomLeftR_EN_Pin GPIO_PIN_11
#define bottomLeftR_EN_GPIO_Port GPIOC
#define bottomRightR_EN_Pin GPIO_PIN_12
#define bottomRightR_EN_GPIO_Port GPIOC
#define bottomLeftL_EN_Pin GPIO_PIN_2
#define bottomLeftL_EN_GPIO_Port GPIOD
#define topLeftEncA_Pin GPIO_PIN_3
#define topLeftEncA_GPIO_Port GPIOB
#define bottomLeftEncB_Pin GPIO_PIN_6
#define bottomLeftEncB_GPIO_Port GPIOB
#define bottomLeftEncA_Pin GPIO_PIN_7
#define bottomLeftEncA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
