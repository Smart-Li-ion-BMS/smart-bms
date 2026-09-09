/*
 * ina219.h
 *
 *  Created on: 22-Jul-2026
 *      Author: sunbeam
 */

#ifndef __INA219_H
#define __INA219_H

#include "stm32f4xx_hal.h"

void INA219_Init(I2C_HandleTypeDef *hi2c);
float INA219_GetBusVoltage_V(void);
float INA219_GetCurrent_mA(void);

#endif
