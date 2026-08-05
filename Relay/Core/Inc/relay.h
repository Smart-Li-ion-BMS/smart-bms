/*
 * relay.h
 *
 *  Created on: 02-Aug-2026
 *      Author: sunbeam
 */

#ifndef INC_RELAY_H_
#define INC_RELAY_H_

#include "main.h"

/* Relay GPIO Configuration */
#define RELAY_GPIO_PORT     GPIOD
#define RELAY_GPIO_PIN      GPIO_PIN_13

/* Function Prototypes */
void Relay_Init(void);
void Relay_ON(void);
void Relay_OFF(void);
void Relay_Toggle(void);
GPIO_PinState Relay_Status(void);


#endif /* INC_RELAY_H_ */
