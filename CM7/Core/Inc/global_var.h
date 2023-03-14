/*
 * global_var.h
 *
 *  Created on: 20 mei 2022
 *      Author: Pascal
 */

#ifndef INC_GLOBAL_VAR_H_
#define INC_GLOBAL_VAR_H_

#include <stdio.h>

#define STATE_INIT							0
#define STATE_START							1
#define STATE_ROBOT_MOVE					2
#define STATE_HUMAN_MOVE					3
#define STATE_CLEAN_UP						4

#define SUBSTATE_RM_REQ_MOVE				0
#define SUBSTATE_RM_REC_MOVE				1
#define SUBSTATE_RM_MOVE_TO_CM4				2
#define SUBSTATE_RM_WAIT_CM4				3
#define SUBSTATE_RM_CM4_DONE				4

#define SUBSTATE_HM_ACTIVATE				0
#define SUBSTATE_HM_WAIT					1

#define TASK_HUMAN_MOVE						0
#define TASK_ROBOT_MOVE						1
#define TASK_CLEAN_UP						2

#endif /* INC_GLOBAL_VAR_H_ */
