/*
 * motor_master.h
 *
 *  Created on: Jun 20, 2022
 *      Author: Pascal
 */

#ifndef INC_MOTOR_MASTER_H_
#define INC_MOTOR_MASTER_H_

#include "motorX.h"

void initMotors();

uint8_t MoveToPos(int16_t posX, int16_t posZ);

uint8_t HomeMotors(uint8_t homeX, uint8_t homeZ);

#endif /* INC_MOTOR_MASTER_H_ */
