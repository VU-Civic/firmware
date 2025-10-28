#ifndef __MAIN_H
#define __MAIN_H

#include "stm32h7xx_hal.h"
#include "revisions.h"

void Error_Handler(void);

#define LSE_CLK_IN_Pin GPIO_PIN_14
#define LSE_CLK_IN_GPIO_Port GPIOC
#define LSE_CLK_OUT_Pin GPIO_PIN_15
#define LSE_CLK_OUT_GPIO_Port GPIOC
#define HSE_CLK_Pin GPIO_PIN_0
#define HSE_CLK_GPIO_Port GPIOH
#define AUDIO_SD_Pin GPIO_PIN_11
#define AUDIO_SD_GPIO_Port GPIOE
#define AUDIO_CLK_Pin GPIO_PIN_12
#define AUDIO_CLK_GPIO_Port GPIOE
#define AUDIO_FSYNC_Pin GPIO_PIN_13
#define AUDIO_FSYNC_GPIO_Port GPIOE
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define DEBUG_SWO_Pin GPIO_PIN_3
#define DEBUG_SWO_GPIO_Port GPIOB

#if REV_ID == REV_A

  #define GPS_TIME_TRIGGER_Pin GPIO_PIN_2
  #define GPS_TIME_TRIGGER_GPIO_Port GPIOA
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
  #define DEBUG_JTDI_Pin GPIO_PIN_15
  #define DEBUG_JTDI_GPIO_Port GPIOA
  #define DEBUG_RST_Pin GPIO_PIN_4
  #define DEBUG_RST_GPIO_Port GPIOB

#else

  #define GPS_TIME_TRIGGER_Pin GPIO_PIN_9
  #define GPS_TIME_TRIGGER_GPIO_Port GPIOE
  #define MICS_SCL_Pin GPIO_PIN_10
  #define MICS_SCL_GPIO_Port GPIOB
  #define MICS_SDA_Pin GPIO_PIN_11
  #define MICS_SDA_GPIO_Port GPIOB

#endif  // #if REV_ID == REV_A

#endif  // __MAIN_H
