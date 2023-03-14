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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_USART3_UART_Init(void);
void MX_I2C1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM13_Init(void);
void MX_ETH_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
void MX_TIM14_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DigiIN_OUT_Z_Pin GPIO_PIN_2
#define DigiIN_OUT_Z_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define PWM_Servo_Rotate_Pin GPIO_PIN_8
#define PWM_Servo_Rotate_GPIO_Port GPIOF
#define PWM_Servo_Slider_Pin GPIO_PIN_9
#define PWM_Servo_Slider_GPIO_Port GPIOF
#define Rotary_Switch_C8_Pin GPIO_PIN_10
#define Rotary_Switch_C8_GPIO_Port GPIOF
#define Rotary_Switch_C2_Pin GPIO_PIN_0
#define Rotary_Switch_C2_GPIO_Port GPIOC
#define Ready_Z_Pin GPIO_PIN_0
#define Ready_Z_GPIO_Port GPIOA
#define Rotary_Switch_C1_Pin GPIO_PIN_3
#define Rotary_Switch_C1_GPIO_Port GPIOA
#define Vaccuum_Sensor_Pin GPIO_PIN_4
#define Vaccuum_Sensor_GPIO_Port GPIOA
#define Encoder_X_A_Pin GPIO_PIN_6
#define Encoder_X_A_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define Rotary_Switch_C4_Pin GPIO_PIN_1
#define Rotary_Switch_C4_GPIO_Port GPIOB
#define Homing_Z_Pin GPIO_PIN_2
#define Homing_Z_GPIO_Port GPIOB
#define GPIO_EXTI14_Pin GPIO_PIN_14
#define GPIO_EXTI14_GPIO_Port GPIOF
#define Pushbutton_Rotary_Switch_Pin GPIO_PIN_9
#define Pushbutton_Rotary_Switch_GPIO_Port GPIOE
#define Homing_X_Pin GPIO_PIN_11
#define Homing_X_GPIO_Port GPIOE
#define Endstop_X_Pin GPIO_PIN_12
#define Endstop_X_GPIO_Port GPIOE
#define Endstop_Z_Pin GPIO_PIN_13
#define Endstop_Z_GPIO_Port GPIOE
#define PWM_X_Pin GPIO_PIN_10
#define PWM_X_GPIO_Port GPIOB
#define PWM_Z_Pin GPIO_PIN_11
#define PWM_Z_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define Encoder_Z_A_Pin GPIO_PIN_12
#define Encoder_Z_A_GPIO_Port GPIOD
#define Encoder_Z_B_Pin GPIO_PIN_13
#define Encoder_Z_B_GPIO_Port GPIOD
#define DigiIN_OUT_X_Pin GPIO_PIN_14
#define DigiIN_OUT_X_GPIO_Port GPIOD
#define Ready_X_Pin GPIO_PIN_15
#define Ready_X_GPIO_Port GPIOD
#define Vaccuum_Valve_Pin GPIO_PIN_6
#define Vaccuum_Valve_GPIO_Port GPIOC
#define Vaccuum_Pump_Pin GPIO_PIN_7
#define Vaccuum_Pump_GPIO_Port GPIOC
#define Solenoid_Pin GPIO_PIN_8
#define Solenoid_GPIO_Port GPIOC
#define Coin_INT_Pin GPIO_PIN_10
#define Coin_INT_GPIO_Port GPIOC
#define Enable_PWR_Pin GPIO_PIN_0
#define Enable_PWR_GPIO_Port GPIOD
#define Read_EMO_Pin GPIO_PIN_1
#define Read_EMO_GPIO_Port GPIOD
#define Direction_X_Pin GPIO_PIN_12
#define Direction_X_GPIO_Port GPIOG
#define Encoder_X_B_Pin GPIO_PIN_5
#define Encoder_X_B_GPIO_Port GPIOB
#define Direction_Z_Pin GPIO_PIN_0
#define Direction_Z_GPIO_Port GPIOE
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
