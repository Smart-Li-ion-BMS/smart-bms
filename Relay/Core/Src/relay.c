/*
 * relay.c
 *
 *  Created on: 02-Aug-2026
 *      Author: sunbeam
 */
#include "relay.h"

/*----------------------------------------------------
 * Initialize Relay
 *---------------------------------------------------*/
void Relay_Init(void)
{
    HAL_GPIO_WritePin(RELAY_GPIO_PORT,
                      RELAY_GPIO_PIN,
                      GPIO_PIN_RESET);
}

/*----------------------------------------------------
 * Relay ON
 *---------------------------------------------------*/
void Relay_ON(void)
{
    HAL_GPIO_WritePin(RELAY_GPIO_PORT,
                      RELAY_GPIO_PIN,
                      GPIO_PIN_SET);
}

/*----------------------------------------------------
 * Relay OFF
 *---------------------------------------------------*/
void Relay_OFF(void)
{
    HAL_GPIO_WritePin(RELAY_GPIO_PORT,
                      RELAY_GPIO_PIN,
                      GPIO_PIN_RESET);
}

/*----------------------------------------------------
 * Toggle Relay
 *---------------------------------------------------*/
void Relay_Toggle(void)
{
    HAL_GPIO_TogglePin(RELAY_GPIO_PORT,
                       RELAY_GPIO_PIN);
}

/*----------------------------------------------------
 * Read Relay Status
 *---------------------------------------------------*/
GPIO_PinState Relay_Status(void)
{
    return HAL_GPIO_ReadPin(RELAY_GPIO_PORT,
                            RELAY_GPIO_PIN);
}

