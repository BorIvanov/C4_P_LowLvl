/*
 * VCNL4010.c
 *
 *  Created on: May 2, 2022
 *      Author: Pascal
 */

#include "VCNL4010.h"

void VCNL4010_Init(const VCNL4010* const self){
	uint8_t led_ma = 0x0A;
	uint8_t com_en = 0x03;
	HAL_StatusTypeDef retFunc;
	retFunc = i2c_CheckDev(self->bus, self->base_addr);

	if (retFunc == HAL_OK){
		i2c_Transmit(self->bus, self->base_addr, VCNL4010_LED_REG, 1, &led_ma, 1);
		i2c_Transmit(self->bus, self->base_addr, VCNL4010_COM_REG, 1, &com_en, 1);
		HAL_Delay(1);
	} else {
	}
}

uint16_t VCNL4010_ReceiveProxy(const VCNL4010* const self){
	uint8_t buf[2];
	uint16_t val = 0;
	HAL_StatusTypeDef retFunc;

	retFunc = i2c_Receive(self->bus, self->base_addr, VCNL4010_PROXY_REG, 1, buf, sizeof(buf));
	if (retFunc != HAL_OK){
		val = 0;
	} else {
		val = ((uint16_t)buf[0]<<8) | buf[1];
	}
	return val;
}

VCNL4010 VCNL4010_Create(uint8_t addr, I2C_HandleTypeDef* inBus){
	VCNL4010 create = { addr, inBus};
	return create;
}
