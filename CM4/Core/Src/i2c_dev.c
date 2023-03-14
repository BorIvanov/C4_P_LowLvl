/*
 * i2c_dev.c
 *
 *  Created on: May 2, 2022
 *      Author: Pascal
 */

#include "i2c_dev.h"

HAL_StatusTypeDef i2c_CheckDev(I2C_HandleTypeDef* bus, uint8_t DevAddress){
	HAL_StatusTypeDef retFunc;
	uint8_t write_addr = DevAddress << 1;
	retFunc = HAL_I2C_IsDeviceReady(bus, write_addr, 1, TIME_OUT);
	return retFunc;
}

HAL_StatusTypeDef i2c_Transmit(I2C_HandleTypeDef* bus, uint8_t DevAddress, uint8_t MemAddress, uint8_t MemAddSize, uint8_t* pData, uint8_t pData_size){
	HAL_StatusTypeDef retFunc;
	uint8_t write_addr = DevAddress << 1;
	retFunc = HAL_I2C_Mem_Write(bus, write_addr, MemAddress, MemAddSize, pData, pData_size, TIME_OUT);
	return retFunc;
}

HAL_StatusTypeDef i2c_Receive(I2C_HandleTypeDef* bus, uint8_t DevAddress, uint8_t MemAddress, uint8_t MemAddSize, uint8_t* pData, uint8_t pData_size){
	HAL_StatusTypeDef retFunc;
	uint8_t read_addr = (DevAddress << 1) | 0x01;
	retFunc = HAL_I2C_Mem_Read(bus, read_addr, MemAddress, MemAddSize, pData, pData_size, TIME_OUT);
	return retFunc;
}
