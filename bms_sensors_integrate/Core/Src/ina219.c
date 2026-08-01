/*
 * ina219.c
 *
 *  Created on: 22-Jul-2026
 *      Author: sunbeam
 */
#include "ina219.h"

#define INA219_ADDR (0x40 << 1)

static I2C_HandleTypeDef *ina_i2c;

void INA219_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t data[3];
    data[0] = reg;
    data[1] = value >> 8;
    data[2] = value & 0xFF;

    HAL_I2C_Master_Transmit(ina_i2c, INA219_ADDR, data, 3, 100);
}

uint16_t INA219_ReadReg(uint8_t reg)
{
    uint8_t data[2];

    HAL_I2C_Master_Transmit(ina_i2c, INA219_ADDR, &reg, 1, 100);
    HAL_I2C_Master_Receive(ina_i2c, INA219_ADDR, data, 2, 100);

    return (data[0] << 8) | data[1];
}

void INA219_Init(I2C_HandleTypeDef *hi2c)
{
    ina_i2c = hi2c;

    /* Configuration register */
    INA219_WriteReg(0x00, 0x399F);

    /* Calibration register */
    INA219_WriteReg(0x05, 4096);
}

float INA219_GetBusVoltage_V(void)
{
    uint16_t raw = INA219_ReadReg(0x02);

    raw >>= 3;

    return raw * 0.004f;   // 4mV/bit
}

float INA219_GetCurrent_mA(void)
{
    int16_t raw = (int16_t)INA219_ReadReg(0x04);

    return raw * 0.1f;     // 0.1mA/bit (for calibration 4096)
}

