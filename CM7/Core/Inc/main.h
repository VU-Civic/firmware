/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define LSE_CLK_IN_Pin GPIO_PIN_14
#define LSE_CLK_IN_GPIO_Port GPIOC
#define LSE_CLK_OUT_Pin GPIO_PIN_15
#define LSE_CLK_OUT_GPIO_Port GPIOC
#define HSE_CLK_Pin GPIO_PIN_0
#define HSE_CLK_GPIO_Port GPIOH
#define GPS_TIME_TRIGGER_Pin GPIO_PIN_2
#define GPS_TIME_TRIGGER_GPIO_Port GPIOA
#define AUDIO_SD_Pin GPIO_PIN_11
#define AUDIO_SD_GPIO_Port GPIOE
#define AUDIO_CLK_Pin GPIO_PIN_12
#define AUDIO_CLK_GPIO_Port GPIOE
#define AUDIO_FSYNC_Pin GPIO_PIN_13
#define AUDIO_FSYNC_GPIO_Port GPIOE
#define MICS_CS_Pin GPIO_PIN_12
#define MICS_CS_GPIO_Port GPIOB
#define MICS_SCK_Pin GPIO_PIN_13
#define MICS_SCK_GPIO_Port GPIOB
#define MICS_MISO_Pin GPIO_PIN_14
#define MICS_MISO_GPIO_Port GPIOB
#define MICS_MOSI_Pin GPIO_PIN_15
#define MICS_MOSI_GPIO_Port GPIOB
#define MICS_NRESET_Pin GPIO_PIN_8
#define MICS_NRESET_GPIO_Port GPIOD
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define DEBUG_JTDI_Pin GPIO_PIN_15
#define DEBUG_JTDI_GPIO_Port GPIOA
#define DEBUG_SWO_Pin GPIO_PIN_3
#define DEBUG_SWO_GPIO_Port GPIOB
#define DEBUG_RST_Pin GPIO_PIN_4
#define DEBUG_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
