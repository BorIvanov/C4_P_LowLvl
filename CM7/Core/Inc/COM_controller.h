/*
 * COM_controller.h
 *
 *  Created on: Apr 29, 2022
 *      Author: Pascal
 */

#ifndef INC_COM_CONTROLLER_H_
#define INC_COM_CONTROLLER_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "global_var.h"

void Init_COM_controller(UART_HandleTypeDef* const RPIbus, uint8_t* data, uint8_t* substate);

void RPI_Request_Move(UART_HandleTypeDef* const RPIbus, uint8_t insertColumn);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void UART_WriteString(UART_HandleTypeDef* const bus, char* buf);

void UART_WriteValue(UART_HandleTypeDef* const bus, int value);

#endif /* INC_COM_CONTROLLER_H_ */
