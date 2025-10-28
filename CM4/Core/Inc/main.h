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
#define CELL_NPWR_ON_Pin GPIO_PIN_4
#define CELL_NPWR_ON_GPIO_Port GPIOA
#define CELL_NRESET_Pin GPIO_PIN_5
#define CELL_NRESET_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define DEBUG_SWO_Pin GPIO_PIN_3
#define DEBUG_SWO_GPIO_Port GPIOB

#if REV_ID == REV_A

  #define GPS_TX_Pin GPIO_PIN_0
  #define GPS_TX_GPIO_Port GPIOA
  #define GPS_RX_Pin GPIO_PIN_1
  #define GPS_RX_GPIO_Port GPIOA
  #define GPS_TIMEPULSE_Pin GPIO_PIN_3
  #define GPS_TIMEPULSE_GPIO_Port GPIOA
  #define RPI_PWR_SWITCH_Pin GPIO_PIN_6
  #define RPI_PWR_SWITCH_GPIO_Port GPIOA
  #define RPI_NPWR_OFF_Pin GPIO_PIN_7
  #define RPI_NPWR_OFF_GPIO_Port GPIOA
  #define IMU_INT_Pin GPIO_PIN_11
  #define IMU_INT_GPIO_Port GPIOF
  #define IMU_SCL_Pin GPIO_PIN_14
  #define IMU_SCL_GPIO_Port GPIOF
  #define IMU_SDA_Pin GPIO_PIN_15
  #define IMU_SDA_GPIO_Port GPIOF
  #define CELL_RX_Pin GPIO_PIN_7
  #define CELL_RX_GPIO_Port GPIOE
  #define CELL_TX_Pin GPIO_PIN_8
  #define CELL_TX_GPIO_Port GPIOE
  #define CELL_RTS_Pin GPIO_PIN_9
  #define CELL_RTS_GPIO_Port GPIOE
  #define CELL_CTS_Pin GPIO_PIN_10
  #define CELL_CTS_GPIO_Port GPIOE
  #define LED_GPS_STATUS_Pin GPIO_PIN_12
  #define LED_GPS_STATUS_GPIO_Port GPIOD
  #define LED_USB_STATUS_Pin GPIO_PIN_13
  #define LED_USB_STATUS_GPIO_Port GPIOD
  #define DEBUG_JTDI_Pin GPIO_PIN_15
  #define DEBUG_JTDI_GPIO_Port GPIOA
  #define DEBUG_RST_Pin GPIO_PIN_4
  #define DEBUG_RST_GPIO_Port GPIOB

  #define GPS_UART_TYPE UART
  #define GPS_UART_NUMBER 4
  #define GPS_UART_AF GPIO_AF8_UART4

  #define IMU_I2C_NUMBER 4
  #define IMU_I2C_AF GPIO_AF4_I2C4

  #define CELL_UART_TYPE UART
  #define CELL_UART_NUMBER 7
  #define CELL_UART_AF GPIO_AF7_UART7

#else

  #define GPS_TX_Pin GPIO_PIN_8
  #define GPS_TX_GPIO_Port GPIOE
  #define GPS_RX_Pin GPIO_PIN_7
  #define GPS_RX_GPIO_Port GPIOE
  #define GPS_TIMEPULSE_Pin GPIO_PIN_10
  #define GPS_TIMEPULSE_GPIO_Port GPIOE
  #define IMU_INT_Pin GPIO_PIN_6
  #define IMU_INT_GPIO_Port GPIOB
  #define IMU_SCL_Pin GPIO_PIN_8
  #define IMU_SCL_GPIO_Port GPIOB
  #define IMU_SDA_Pin GPIO_PIN_7
  #define IMU_SDA_GPIO_Port GPIOB
  #define CELL_RX_Pin GPIO_PIN_3
  #define CELL_RX_GPIO_Port GPIOA
  #define CELL_TX_Pin GPIO_PIN_2
  #define CELL_TX_GPIO_Port GPIOA
  #define CELL_RTS_Pin GPIO_PIN_1
  #define CELL_RTS_GPIO_Port GPIOA
  #define CELL_CTS_Pin GPIO_PIN_0
  #define CELL_CTS_GPIO_Port GPIOA
  #define LED_GPS_STATUS_Pin GPIO_PIN_1
  #define LED_GPS_STATUS_GPIO_Port GPIOB
  #define FROM_AI_SCL_Pin GPIO_PIN_8
  #define FROM_AI_SCL_GPIO_Port GPIOA
  #define FROM_AI_SDA_Pin GPIO_PIN_9
  #define FROM_AI_SDA_GPIO_Port GPIOC
  #define FROM_AI_INT_Pin GPIO_PIN_7
  #define FROM_AI_INT_GPIO_Port GPIOC
  #define TO_AI_CS_Pin GPIO_PIN_12
  #define TO_AI_CS_GPIO_Port GPIOB
  #define TO_AI_SCK_Pin GPIO_PIN_13
  #define TO_AI_SCK_GPIO_Port GPIOB
  #define TO_AI_MISO_Pin GPIO_PIN_14
  #define TO_AI_MISO_GPIO_Port GPIOB
  #define TO_AI_MOSI_Pin GPIO_PIN_15
  #define TO_AI_MOSI_GPIO_Port GPIOB

  #define GPS_UART_TYPE UART
  #define GPS_UART_NUMBER 7
  #define GPS_UART_AF GPIO_AF7_UART7

  #define IMU_I2C_NUMBER 4
  #define IMU_I2C_AF GPIO_AF6_I2C4

  #define CELL_UART_TYPE USART
  #define CELL_UART_NUMBER 2
  #define CELL_UART_AF GPIO_AF7_USART2

#endif  // #if REV_ID == REV_A

#endif  // __MAIN_H
