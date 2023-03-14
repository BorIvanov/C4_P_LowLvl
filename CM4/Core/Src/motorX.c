/*
 * motorX.c
 *
 *  Created on: Jun 20, 2022
 *      Author: Pascal
 */
#include "motorX.h"

extern uint32_t counter;
extern int32_t position_mm;

uint8_t slowspeed = 0;

uint8_t homeDone = 0;
uint8_t i = 0;

void initMotorX(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
}

uint8_t homeMotorX(){
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 250);
	if(HAL_GPIO_ReadPin(Home_X_GPIO_Port, Home_X_Pin) == 0){
		HAL_GPIO_WritePin(Direction_X_GPIO_Port, Direction_X_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Ready_X_GPIO_Port, Ready_X_Pin, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(Direction_X_GPIO_Port, Direction_X_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ready_X_GPIO_Port, Ready_X_Pin, GPIO_PIN_SET);
	}
	while(!homeDone){
	}
	HAL_Delay(200);
	position_mm = 0;
	counter = 0;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	return 1;
}

uint8_t moveToPosX(int16_t Xpos){
	int16_t delta = Xpos - position_mm;
	if(delta >= 0){
		HAL_GPIO_WritePin(Direction_X_GPIO_Port, Direction_X_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(Direction_X_GPIO_Port, Direction_X_Pin, GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 245);
	HAL_GPIO_WritePin(Ready_X_GPIO_Port, Ready_X_Pin, GPIO_PIN_SET);
	if (abs(delta) > 60 && Xpos > 5){
		slowspeed = 1;
	}
	while(Xpos != position_mm){
		if(slowspeed == 1){
			uint16_t y = 0;
			delta = Xpos - position_mm;
			if(abs(delta) < 60){
				y = 2*(pow(abs(delta), 0.49)) + 215;
				if(y>250){
					y = 250;
				}
			} else {
				y = 250;
			}

			i++;
			if(i == 200){
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, y);
				i = 0;
			}
		}
	}
	slowspeed = 0;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(Ready_X_GPIO_Port, Ready_X_Pin, GPIO_PIN_RESET);
	return 1;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim2){
		counter = __HAL_TIM_GET_COUNTER(htim);

		position_mm = (int32_t) counter / 1104;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Home_X_Pin){
    	if(!HAL_GPIO_ReadPin(Home_X_GPIO_Port, Home_X_Pin)){
			/* do something */
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
			HAL_GPIO_WritePin(Ready_X_GPIO_Port, Ready_X_Pin, GPIO_PIN_RESET);
			homeDone = 1;
		}
    	if(HAL_GPIO_ReadPin(Home_X_GPIO_Port, Home_X_Pin)){ //check pin state
			/* do something */
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Ready_X_GPIO_Port, Ready_X_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 240);
			HAL_GPIO_WritePin(Direction_X_GPIO_Port, Direction_X_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Ready_X_GPIO_Port, Ready_X_Pin, GPIO_PIN_SET);
		}
    }
}
