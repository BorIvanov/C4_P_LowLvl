/*
 * i2c_dev.h
 *
 *  Created on: May 2, 2022
 *      Author: Pascal
 */

#ifndef INC_I2C_DEV_H_
#define INC_I2C_DEV_H_

#include "i2c.h"

#define TIME_OUT 50

HAL_StatusTypeDef i2c_CheckDev(I2C_HandleTypeDef* bus, uint8_t DevAddress);

HAL_StatusTypeDef i2c_Transmit(I2C_HandleTypeDef* bus, uint8_t DevAddress, uint8_t MemAddress, uint8_t MemAddSize, uint8_t* pData, uint8_t pData_size);

HAL_StatusTypeDef i2c_Receive(I2C_HandleTypeDef* bus, uint8_t DevAddress, uint8_t MemAddress, uint8_t MemAddSize, uint8_t* pData, uint8_t pData_size);

#endif /* INC_I2C_DEV_H_ */
