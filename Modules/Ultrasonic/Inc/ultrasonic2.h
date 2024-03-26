/*
 * ultrasonic.h
 *
 *  Created on: Aug 7, 2023
 *      Author: Aleyna
 */

#ifndef MODULES_ULTRASONIC_INC_ULTRASONIC2_H_
#define MODULES_ULTRASONIC_INC_ULTRASONIC2_H_

#include "stm32f4xx.h"
#include <stdbool.h>

typedef struct {
	GPIO_TypeDef * p_Port;
	uint16_t u16_Pin;
	TIM_HandleTypeDef *h_htim;
	uint32_t u32_Channel;
	uint32_t IC_Val1 ;
	uint32_t IC_Val2 ;
	uint32_t Difference;
	uint8_t Is_First_Captured;  // is the first value captured ?
	uint8_t Distance  ;
	uint32_t u32_Activechannel;

}t_TriggerHandler;

void Ultrasonic_Init(t_TriggerHandler* trig, GPIO_TypeDef* port ,uint16_t pin,TIM_HandleTypeDef *htim, uint32_t channel,uint32_t achannel);

void delay (uint16_t time, t_TriggerHandler* trig);

void HAL_TIM_IC_CaptureCallback2(t_TriggerHandler* trig,TIM_HandleTypeDef *htim);

int HCSR04_Read (t_TriggerHandler* trig);



#endif /* MODULES_ULTRASONIC_INC_ULTRASONIC2_H_ */
