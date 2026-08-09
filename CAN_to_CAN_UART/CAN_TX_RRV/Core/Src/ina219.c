/*
 * INA219.c
 *
 *  Created on: 02-Aug-2026
 *      Author: sunbeam
 */

#include "ina219.h"

#define INA219_ADDR                  (0x40U << 1)
#define INA219_REG_CONFIG            0x00U
#define INA219_REG_SHUNT_VOLTAGE     0x01U
#define INA219_REG_BUS_VOLTAGE       0x02U
#define INA219_SHUNT_RESISTOR_OHM    0.1f

static I2C_HandleTypeDef *ina_i2c = NULL;
static HAL_StatusTypeDef last_status = HAL_OK;
static uint16_t last_bus_raw = 0U;
static int16_t last_shunt_raw = 0;

static HAL_StatusTypeDef INA219_WriteReg(uint8_t reg, uint16_t value)
{
    uint8_t data[2];
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);

    if (ina_i2c == NULL)
    {
        last_status = HAL_ERROR;
        return last_status;
    }

    last_status = HAL_I2C_Mem_Write(ina_i2c, INA219_ADDR, reg,
                                    I2C_MEMADD_SIZE_8BIT, data, 2U, 100U);
    return last_status;
}

static HAL_StatusTypeDef INA219_ReadReg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2] = {0U, 0U};

    if ((ina_i2c == NULL) || (value == NULL))
    {
        last_status = HAL_ERROR;
        return last_status;
    }

    last_status = HAL_I2C_Mem_Read(ina_i2c, INA219_ADDR, reg,
                                   I2C_MEMADD_SIZE_8BIT, data, 2U, 100U);
    if (last_status == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return last_status;
}

void INA219_Init(I2C_HandleTypeDef *hi2c)
{
    ina_i2c = hi2c;
    (void)INA219_WriteReg(INA219_REG_CONFIG, 0x399FU);
}

uint8_t INA219_IsDeviceReady(void)
{
    if (ina_i2c == NULL)
    {
        last_status = HAL_ERROR;
        return 0U;
    }
    last_status = HAL_I2C_IsDeviceReady(ina_i2c, INA219_ADDR, 3U, 100U);
    return (last_status == HAL_OK) ? 1U : 0U;
}

float INA219_GetBusVoltage_V(void)
{
    uint16_t raw = 0U;
    if (INA219_ReadReg(INA219_REG_BUS_VOLTAGE, &raw) != HAL_OK)
    {
        last_bus_raw = 0U;
        return 0.0f;
    }
    last_bus_raw = raw;
    return (float)((raw >> 3) * 4U) / 1000.0f;
}

float INA219_GetCurrent_mA(void)
{
    uint16_t raw_u16 = 0U;
    float shunt_voltage_v;

    if (INA219_ReadReg(INA219_REG_SHUNT_VOLTAGE, &raw_u16) != HAL_OK)
    {
        last_shunt_raw = 0;
        return 0.0f;
    }

    last_shunt_raw = (int16_t)raw_u16;
    shunt_voltage_v = (float)last_shunt_raw * 0.000010f;
    return (shunt_voltage_v / INA219_SHUNT_RESISTOR_OHM) * 1000.0f;
}

HAL_StatusTypeDef INA219_GetLastStatus(void) { return last_status; }
uint16_t INA219_GetLastBusRaw(void) { return last_bus_raw; }
int16_t INA219_GetLastShuntRaw(void) { return last_shunt_raw; }

