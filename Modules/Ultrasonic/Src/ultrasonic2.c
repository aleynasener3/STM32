


//Includes ------------------------------------------------------------



#include "../Inc/ultrasonic2.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

//Functions ------------------------------------------------------------


void Ultrasonic_Init(t_TriggerHandler* trig, GPIO_TypeDef* port ,uint16_t pin, TIM_HandleTypeDef *htim, uint32_t channel,uint32_t achannel){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	  HAL_TIM_IC_Start_IT(htim, channel);

	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	trig->p_Port =  port;
	trig->u16_Pin = pin;
	trig->h_htim = htim;
	trig->u32_Channel = channel;
	trig->u32_Activechannel = achannel;
	trig->Difference = 0;
	trig->Distance = 0;
	trig->IC_Val1 = 0;
	trig->IC_Val2 = 0;
	trig->Is_First_Captured = 0;


}


void delay (uint16_t time, t_TriggerHandler* trig)
{
	__HAL_TIM_SET_COUNTER(trig->h_htim, 0);
	while (__HAL_TIM_GET_COUNTER (trig->h_htim) < time);
}


void HAL_TIM_IC_CaptureCallback2(t_TriggerHandler* trig,TIM_HandleTypeDef *htim)
{
	if (htim->Channel == trig->u32_Activechannel)  // if the interrupt source is channel1
	{
		if (trig->Is_First_Captured==0) // if the first value is not captured
		{
			trig->IC_Val1 = HAL_TIM_ReadCapturedValue(trig->h_htim, trig->u32_Channel); // read the first value
			trig->Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(trig->h_htim, trig->u32_Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (trig->Is_First_Captured==1)   // if the first is already captured
		{
			trig->IC_Val2 = HAL_TIM_ReadCapturedValue(trig->h_htim, trig->u32_Channel);  // read second value
			__HAL_TIM_SET_COUNTER(trig->h_htim, 0);  // reset the counter

			if (trig->IC_Val2 > trig->IC_Val1)
			{
				trig->Difference = trig->IC_Val2-trig->IC_Val1;
			}

			else if (trig->IC_Val1 > trig->IC_Val2)
			{
				trig->Difference = (0xffff - trig->IC_Val1) + trig->IC_Val2;
			}

			trig->Distance = trig->Difference / 58;
			trig->Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(trig->h_htim, trig->u32_Channel, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(trig->h_htim, TIM_IT_CC1);
		}
	}
}



int HCSR04_Read (t_TriggerHandler* trig){
	HAL_GPIO_WritePin(trig->p_Port, trig->u16_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10,trig);  // wait for 10 us
	HAL_GPIO_WritePin(trig->p_Port, trig->u16_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(trig->h_htim, TIM_IT_CC1);
	return trig->Distance;
}

