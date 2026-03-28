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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define wbjz_Pin GPIO_PIN_0
#define wbjz_GPIO_Port GPIOH
#define wbjzH1_Pin GPIO_PIN_1
#define wbjzH1_GPIO_Port GPIOH
#define blue_tx_Pin GPIO_PIN_2
#define blue_tx_GPIO_Port GPIOA
#define blue_rx_Pin GPIO_PIN_3
#define blue_rx_GPIO_Port GPIOA
#define E1A_Pin GPIO_PIN_6
#define E1A_GPIO_Port GPIOA
#define E2A_Pin GPIO_PIN_7
#define E2A_GPIO_Port GPIOA
#define ain1_Pin GPIO_PIN_0
#define ain1_GPIO_Port GPIOB
#define ain2_Pin GPIO_PIN_1
#define ain2_GPIO_Port GPIOB
#define FRAM_CS_Pin GPIO_PIN_12
#define FRAM_CS_GPIO_Port GPIOB
#define pwnA_Pin GPIO_PIN_8
#define pwnA_GPIO_Port GPIOA
#define pwmB_Pin GPIO_PIN_9
#define pwmB_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_10
#define BIN2_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_11
#define BIN1_GPIO_Port GPIOC
#define BUZZER_HIGH_Pin GPIO_PIN_4
#define BUZZER_HIGH_GPIO_Port GPIOB
#define BULB_LOW_Pin GPIO_PIN_5
#define BULB_LOW_GPIO_Port GPIOB
#define E2B_Pin GPIO_PIN_6
#define E2B_GPIO_Port GPIOB
#define E1B_Pin GPIO_PIN_7
#define E1B_GPIO_Port GPIOB
#define GYRO_SCL_Pin GPIO_PIN_8
#define GYRO_SCL_GPIO_Port GPIOB
#define GYRO_SDA_Pin GPIO_PIN_9
#define GYRO_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
