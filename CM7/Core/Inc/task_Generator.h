/*
 * task_Generator.h
 *
 *  Created on: 20 mei 2022
 *      Author: Pascal
 */

#ifndef INC_TASK_GENERATOR_H_
#define INC_TASK_GENERATOR_H_

#include "global_var.h"
#include "common.h"
#include <string.h>

void initTaskGenerator(uint8_t* state, uint8_t* substateRM, uint8_t* substateHM, uint8_t* dataIn);

void taskToDo(uint8_t task);

void HAL_HSEM_FreeCallback(uint32_t SemMask);

#endif /* INC_TASK_GENERATOR_H_ */
