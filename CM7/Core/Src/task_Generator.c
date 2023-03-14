/*
 * task_Generator.c
 *
 *  Created on: 20 mei 2022
 *      Author: Pascal
 */

#include "task_Generator.h"


uint8_t* subRobotM;
uint8_t* subHumanM;
uint8_t* st;
uint8_t* data;

void initTaskGenerator(uint8_t* state, uint8_t* substateRM, uint8_t* substateHM, uint8_t* dataIn){
	subRobotM = substateRM;
	subHumanM = substateHM;
	st = state;
	data = dataIn;

	HAL_HSEM_ActivateNotification(HSEM_CM4_DONE_MASK);
	HAL_HSEM_ActivateNotification(HSEM_COIN_COLUMN_MASK);
	memset(SharedBuf, 0, 10);
}

void taskToDo(uint8_t task){
	if(task == TASK_ROBOT_MOVE){
		memset(SharedBuf, (int)(data[0]-'0'), 1);
		HSEM_TAKE_RELEASE(HSEM_ROBOT_MOVE);
	}
	if(task == TASK_HUMAN_MOVE){
		HSEM_TAKE_RELEASE(HSEM_HUMAN_MOVE);
	}
	if(task == TASK_CLEAN_UP){
		HSEM_TAKE_RELEASE(HSEM_CLEAN_UP);
	}
}

void HAL_HSEM_FreeCallback(uint32_t SemMask){
	if(SemMask == HSEM_CM4_DONE_MASK){
		if(*st == STATE_INIT){
			*st = STATE_START;
		}
		if(*st == STATE_ROBOT_MOVE){
			*subRobotM = SUBSTATE_RM_CM4_DONE;
		}
		HAL_HSEM_ActivateNotification(HSEM_CM4_DONE_MASK);
	}
	if(SemMask == HSEM_COIN_COLUMN_MASK){
		*subHumanM = SUBSTATE_HM_ACTIVATE;
		*st = STATE_ROBOT_MOVE;
		HAL_HSEM_ActivateNotification(HSEM_COIN_COLUMN_MASK);
	}
}
