/*
 * motor_master.c
 *
 *  Created on: Jun 20, 2022
 *      Author: Pascal
 */
#include "motor_master.h"

void initMotors(){
	initMotorX();
	//add Z motor
}

uint8_t MoveToPos(int16_t posX, int16_t posZ){
	moveToPosX(posX);
	return 1;
}

uint8_t HomeMotors(uint8_t homeX, uint8_t homeZ){
	uint8_t homeXflag = 0;
	if(homeX){
		homeMotorX();
	}
	if(homeZ){
		/* implement home Z */
	}
	return 1;
}
