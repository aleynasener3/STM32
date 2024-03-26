/*
 * led.h
 *
 *  Created on: Aug 7, 2023
 *      Author: Aleyna
 */

#ifndef MODULES_LED_LED_H_
#define MODULES_LED_LED_H_

#include "stm32f4xx.h"
#include <stdbool.h>

typedef struct {
	GPIO_TypeDef * p_Port;
	uint16_t u16_Pin;
}t_LedHandler;

// GPIO Port and Pin definitions
#define LED_GPIO_PORT GPIOA
#define LED_PIN GPIO_PIN_4



void Led_Init(t_LedHandler* led, GPIO_TypeDef* port ,uint16_t pin );

void Led_Read(t_LedHandler* led);


void Led_Light(t_LedHandler* led, uint8_t newStatus);

void Led_Toggle(t_LedHandler* led, int t);

#endif /* MODULES_LED_LED_H_ */
