/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_Servo_Rotate_Pin GPIO_PIN_8
#define PWM_Servo_Rotate_GPIO_Port GPIOF
#define PWM_Servo_Slider_Pin GPIO_PIN_9
#define PWM_Servo_Slider_GPIO_Port GPIOF
#define Vaccuum_Sensor_Pin GPIO_PIN_4
#define Vaccuum_Sensor_GPIO_Port GPIOA
#define Encoder_X_A_Pin GPIO_PIN_6
#define Encoder_X_A_GPIO_Port GPIOA
#define PWM_X_Pin GPIO_PIN_10
#define PWM_X_GPIO_Port GPIOB
#define PWM_Z_Pin GPIO_PIN_11
#define PWM_Z_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define Encoder_Z_A_Pin GPIO_PIN_12
#define Encoder_Z_A_GPIO_Port GPIOD
#define Encoder_Z_B_Pin GPIO_PIN_13
#define Encoder_Z_B_GPIO_Port GPIOD
#define Encoder_X_B_Pin GPIO_PIN_5
#define Encoder_X_B_GPIO_Port GPIOB
#define RPI_UART_Tx_Pin GPIO_PIN_6
#define RPI_UART_Tx_GPIO_Port GPIOB
#define RPI_UART_Rx_Pin GPIO_PIN_7
#define RPI_UART_Rx_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
