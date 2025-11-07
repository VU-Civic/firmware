#ifndef __MAIN_H
#define __MAIN_H

#include "stm32h7xx_hal.h"
#include "revisions.h"

typedef struct
{
   GPIO_TypeDef *port;
   uint16_t pin;
} gpio_pin_t;

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

  #define UNUSED_PINS { { GPIOE, GPIO_PIN_3 } }

#else

  #define GPS_TIME_TRIGGER_Pin GPIO_PIN_9
  #define GPS_TIME_TRIGGER_GPIO_Port GPIOE
  #define MICS_SCL_Pin GPIO_PIN_10
  #define MICS_SCL_GPIO_Port GPIOB
  #define MICS_SDA_Pin GPIO_PIN_11
  #define MICS_SDA_GPIO_Port GPIOB

  #define UNUSED_PINS { \
     { GPIOA, GPIO_PIN_6 }, { GPIOA, GPIO_PIN_7 }, { GPIOA, GPIO_PIN_9 }, { GPIOA, GPIO_PIN_10 }, { GPIOA, GPIO_PIN_15 }, \
     { GPIOB, GPIO_PIN_0 }, { GPIOB, GPIO_PIN_2 }, { GPIOB, GPIO_PIN_4 }, { GPIOB, GPIO_PIN_5 }, { GPIOB, GPIO_PIN_9 }, \
     { GPIOC, GPIO_PIN_0 }, { GPIOC, GPIO_PIN_1 }, { GPIOC, GPIO_PIN_2 }, { GPIOC, GPIO_PIN_3 }, { GPIOC, GPIO_PIN_4 }, { GPIOC, GPIO_PIN_5 }, { GPIOC, GPIO_PIN_6 }, { GPIOC, GPIO_PIN_8 }, { GPIOC, GPIO_PIN_10 }, { GPIOC, GPIO_PIN_11 }, { GPIOC, GPIO_PIN_12 }, { GPIOC, GPIO_PIN_13 }, \
     { GPIOD, GPIO_PIN_0 }, { GPIOD, GPIO_PIN_1 }, { GPIOD, GPIO_PIN_2 }, { GPIOD, GPIO_PIN_3 }, { GPIOD, GPIO_PIN_4 }, { GPIOD, GPIO_PIN_5 }, { GPIOD, GPIO_PIN_6 }, { GPIOD, GPIO_PIN_7 }, { GPIOD, GPIO_PIN_8 }, { GPIOD, GPIO_PIN_9 }, { GPIOD, GPIO_PIN_10 }, { GPIOD, GPIO_PIN_11 }, { GPIOD, GPIO_PIN_12 }, { GPIOD, GPIO_PIN_13 }, { GPIOD, GPIO_PIN_14 }, { GPIOD, GPIO_PIN_15 }, \
     { GPIOE, GPIO_PIN_0 }, { GPIOE, GPIO_PIN_1 }, { GPIOE, GPIO_PIN_2 }, { GPIOE, GPIO_PIN_3 }, { GPIOE, GPIO_PIN_4 }, { GPIOE, GPIO_PIN_5 }, { GPIOE, GPIO_PIN_6 }, { GPIOE, GPIO_PIN_14 }, { GPIOE, GPIO_PIN_15 }, \
     { GPIOF, GPIO_PIN_6 }, { GPIOF, GPIO_PIN_7 }, { GPIOF, GPIO_PIN_8 }, { GPIOF, GPIO_PIN_9 }, { GPIOF, GPIO_PIN_10 }, { GPIOF, GPIO_PIN_11 }, { GPIOF, GPIO_PIN_14 }, { GPIOF, GPIO_PIN_15 }, \
     { GPIOG, GPIO_PIN_6 }, { GPIOG, GPIO_PIN_7 }, { GPIOG, GPIO_PIN_8 }, { GPIOG, GPIO_PIN_9 }, { GPIOG, GPIO_PIN_10 }, { GPIOG, GPIO_PIN_11 }, { GPIOG, GPIO_PIN_12 }, { GPIOG, GPIO_PIN_13 }, { GPIOG, GPIO_PIN_14 }, \
     { GPIOH, GPIO_PIN_1 } \
  }

#endif  // #if REV_ID == REV_A

#endif  // __MAIN_H
