/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "i2c_dev.h"
#include "VCNL4010.h"
#include "global_var4.h"
#include "motor_master.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#ifndef HSEM_ID_0
//#define HSEM_ID_0 (0U) /* HW semaphore 0*/
//#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
VCNL4010 S1;
uint16_t valProxy = 0;

uint8_t data[2];

uint8_t kolom[7] = {KOLOM_1, KOLOM_2, KOLOM_3, KOLOM_4, KOLOM_5, KOLOM_6, KOLOM_7};

uint8_t humanMove = 0;
uint8_t cheat = 0;

uint8_t state = STATE_INIT;

uint32_t counter = 0;
int32_t position_mm = 0;

uint8_t kalibration = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(HSEM_WAKEUP_CPU2_MASK);
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(HSEM_WAKEUP_CPU2_MASK);

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
//  initMotors();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

  S1 = VCNL4010_Create(0x13, &hi2c1);
  VCNL4010_Init(&S1);

  HAL_HSEM_ActivateNotification(HSEM_CM7_TO_CM4_MASK);
  HAL_HSEM_ActivateNotification(HSEM_ROBOT_MOVE_MASK);
  HAL_HSEM_ActivateNotification(HSEM_HUMAN_MOVE_MASK);

//  HomeMotors(1, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*Code kan in een timer interrupt*/
	  /***************************************************************/
	  valProxy = VCNL4010_ReceiveProxy(&S1);

	  if (valProxy >= 4500){
		  if(state == STATE_HUMAN_MOVE && !humanMove){
			  humanMove = 1;
		  }
//		  if(state != STATE_HUMAN_MOVE || humanMove){
//			  cheat = 1;
//		  }
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  } else {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  }
	  /***************************************************************/

	  if(state == STATE_INIT){
		  kalibration = HomeMotors(1, 0);
		  if(kalibration){
			  state = STATE_IDLE;
			  HSEM_TAKE_RELEASE(HSEM_CM4_DONE);
		  }
	  }
	  if(state == STATE_IDLE){
		  //Do nothing
	  }
	  if(state == STATE_HUMAN_MOVE){
		  if(humanMove){
			  HSEM_TAKE_RELEASE(HSEM_COIN_COLUMN);
			  state = STATE_IDLE;
		  }
	  }
	  if(state == STATE_ROBOT_MOVE){
		  MoveToPos(kolom[data[0]-1], 0);
		  HAL_Delay(2000);
		  MoveToPos(5,0);
		  HSEM_TAKE_RELEASE(HSEM_CM4_DONE);
		  humanMove = 0;
		  state = STATE_IDLE;
	  }
	  HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void HAL_HSEM_FreeCallback(uint32_t SemMask){
	if(SemMask == HSEM_ROBOT_MOVE_MASK){
		memcpy(data, SharedBuf, 1);
		state = STATE_ROBOT_MOVE;
		HAL_HSEM_ActivateNotification(HSEM_ROBOT_MOVE_MASK);
	}
	if(SemMask == HSEM_HUMAN_MOVE_MASK){
		humanMove = 0;
		state = STATE_HUMAN_MOVE;
		HAL_HSEM_ActivateNotification(HSEM_HUMAN_MOVE_MASK);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
