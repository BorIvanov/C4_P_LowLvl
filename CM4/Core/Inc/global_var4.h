/*
 * global_var4.h
 *
 *  Created on: 20 jun. 2022
 *      Author: Pascal
 */

#ifndef INC_GLOBAL_VAR4_H_
#define INC_GLOBAL_VAR4_H_

#define STATE_INIT							0
#define STATE_IDLE							1
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

#define KOLOM_1								10
#define KOLOM_2								40
#define KOLOM_3								78
#define KOLOM_4								113
#define KOLOM_5								147
#define KOLOM_6								182
#define KOLOM_7								216

#endif /* INC_GLOBAL_VAR4_H_ */
