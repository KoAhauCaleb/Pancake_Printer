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
#include "stm32l4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Screen_CS_Pin GPIO_PIN_0
#define Screen_CS_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SD_SCK_Pin GPIO_PIN_5
#define SD_SCK_GPIO_Port GPIOA
#define SD_MISO_Pin GPIO_PIN_6
#define SD_MISO_GPIO_Port GPIOA
#define SD_MOSI_Pin GPIO_PIN_7
#define SD_MOSI_GPIO_Port GPIOA
#define Touch_CS_Pin GPIO_PIN_1
#define Touch_CS_GPIO_Port GPIOB
#define Touch_IRQ_Pin GPIO_PIN_2
#define Touch_IRQ_GPIO_Port GPIOB
#define Y_DIR_Pin GPIO_PIN_10
#define Y_DIR_GPIO_Port GPIOB
#define Touch_SCK_Pin GPIO_PIN_13
#define Touch_SCK_GPIO_Port GPIOB
#define Touch_MISO_Pin GPIO_PIN_14
#define Touch_MISO_GPIO_Port GPIOB
#define Touch_MOSI_Pin GPIO_PIN_15
#define Touch_MOSI_GPIO_Port GPIOB
#define X_STOP_Pin GPIO_PIN_7
#define X_STOP_GPIO_Port GPIOC
#define Z_DIR_Pin GPIO_PIN_8
#define Z_DIR_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_9
#define ENABLE_GPIO_Port GPIOA
#define X_STEP_Pin GPIO_PIN_10
#define X_STEP_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Screen_DC_Pin GPIO_PIN_15
#define Screen_DC_GPIO_Port GPIOA
#define Screen_SCK_Pin GPIO_PIN_10
#define Screen_SCK_GPIO_Port GPIOC
#define Screen_MISO_Pin GPIO_PIN_11
#define Screen_MISO_GPIO_Port GPIOC
#define Screen_MOSI_Pin GPIO_PIN_12
#define Screen_MOSI_GPIO_Port GPIOC
#define Screen_RST_Pin GPIO_PIN_2
#define Screen_RST_GPIO_Port GPIOD
#define Y_STEP_Pin GPIO_PIN_3
#define Y_STEP_GPIO_Port GPIOB
#define X_DIR_Pin GPIO_PIN_4
#define X_DIR_GPIO_Port GPIOB
#define Z_STEP_Pin GPIO_PIN_5
#define Z_STEP_GPIO_Port GPIOB
#define Y_STOP_Pin GPIO_PIN_6
#define Y_STOP_GPIO_Port GPIOB
#define Screen_LED_Pin GPIO_PIN_7
#define Screen_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
