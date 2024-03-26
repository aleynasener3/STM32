/*
 * led.c
 *
 *  Created on: Aug 7, 2023
 *      Author: Aleyna
 */

#include "led.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>


void Led_Init(t_LedHandler* led, GPIO_TypeDef* port ,uint16_t pin ){
	__HAL_RCC_GPIOA_CLK_ENABLE();

	led->p_Port =  port;
	led->u16_Pin = pin;
	HAL_Init();
	// Initialize the GPIO pin as an output
	    GPIO_InitTypeDef GPIO_InitStruct;
	    GPIO_InitStruct.Pin = led->u16_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low-speed output
	    HAL_GPIO_Init(led->p_Port, &GPIO_InitStruct);
}

void Led_Read(t_LedHandler* led){

	HAL_GPIO_ReadPin(led->p_Port,led->u16_Pin);
}

void Led_Light(t_LedHandler* led, uint8_t newStatus){
	if (newStatus){
			HAL_GPIO_WritePin(led->p_Port,led->u16_Pin,GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(led->p_Port,led->u16_Pin, GPIO_PIN_RESET);
		}

	//led->p_Port->ODR |= (1 << led->u16_Pin);


}

void led_toggle(t_LedHandler* led, int t){

	HAL_GPIO_TogglePin(led->p_Port, led->u16_Pin);
	HAL_Delay(t);
}

