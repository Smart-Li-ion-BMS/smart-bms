/*
 * ina219.h
 *
 *  Created on: 02-Aug-2026
 *      Author: sunbeam
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_


#include "stm32f4xx_hal.h"

void INA219_Init(I2C_HandleTypeDef *hi2c);
float INA219_GetBusVoltage_V(void);
float INA219_GetCurrent_mA(void);
HAL_StatusTypeDef INA219_GetLastStatus(void);
uint16_t INA219_GetLastBusRaw(void);
int16_t INA219_GetLastShuntRaw(void);
uint8_t INA219_IsDeviceReady(void);



#endif /* INC_INA219_H_ */





