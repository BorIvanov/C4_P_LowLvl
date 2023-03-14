/*
 * COM_controlller.c
 *
 *  Created on: Apr 29, 2022
 *      Author: Pascal
 */

#include "COM_controller.h"

uint8_t* rxdata;
uint8_t* subRM;

void Init_COM_controller(UART_HandleTypeDef* const RPIbus, uint8_t* data, uint8_t* substate){
	rxdata = data;
	subRM = substate;
	HAL_UART_Receive_IT(RPIbus, rxdata, 3);
	srand(18);
}

void RPI_Request_Move(UART_HandleTypeDef* const RPIbus, uint8_t insertColumn){
	int random = rand() % 7 + 1;
	uint8_t buf[32];
	sprintf((char *) buf, "Coin inserted in column : %i\r\n\n", random);
	UART_WriteString(&huart3, (char *)buf);
	HAL_Delay(2000);
	UART_WriteValue(RPIbus, random);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t buf[28];
	sprintf((char*) buf, "Received RPI value : %.*s\r\n\n", 2, rxdata);
	UART_WriteString(&huart3, "-----------------------------------------------\r\n");
	UART_WriteString(&huart3, (char *)buf);
	*subRM = SUBSTATE_RM_REC_MOVE;
	HAL_UART_Receive_IT(huart, rxdata, 3);
}

void UART_WriteString(UART_HandleTypeDef* const bus, char* buf){
	HAL_UART_Transmit(bus, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

void UART_WriteValue(UART_HandleTypeDef* const bus, int value){
	uint8_t buf[12];
	sprintf((char*) buf, "%i\r\n", value);
	HAL_UART_Transmit(bus, (uint8_t*)buf, strlen((const char*)buf), HAL_MAX_DELAY);
}
