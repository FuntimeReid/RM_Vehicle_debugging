/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
  extern CAN_HandleTypeDef hcan1;
  extern CAN_HandleTypeDef hcan2;
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
  void LED_WHITE(void);
  void LED_RED(void);
  void LED_GREEN(void);
  void LED_BLUE(void);
  void LED_OFF(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

  typedef struct
  {
   struct
    {
      int16_t ch0;
      int16_t ch1;
      int16_t ch2;
      int16_t ch3;
      uint8_t s1;
      uint8_t s2;
    }rc;
    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
      uint8_t press_l;
      uint8_t press_r;
    }mouse;
    struct
    {
      uint16_t v;
    }key;
  }RC_Ctl_t;

  extern SPI_HandleTypeDef hspi1;
  extern volatile uint8_t RCMode;
  extern volatile int32_t RCMode_cnt;
  extern UART_HandleTypeDef huart3;
  extern volatile uint8_t dbus_rx_buffer[18];
  extern volatile RC_Ctl_t RC_Ctl;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
