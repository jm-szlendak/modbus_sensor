/*
 * delay.c
 *
 *  Created on: 18 gru 2014
 *      Author: jakub
 */

#ifndef DELAY_C_
#define DELAY_C_

#include "delay.h"

volatile char delay_flag = 0;

void delay_init(){
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 1);
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7->CR1 = (uint32_t)0;
	TIM7->DIER |= TIM_DIER_UIE; //Update Interrupt Enabled
	TIM7->PSC = (uint16_t) 63;
	TIM7->ARR = (uint16_t) 1000;
}

void delay(uint32_t time, DelayType type){

	if(type == DELAY_MS)
		TIM7->PSC = 63999;
	else if(type == DELAY_US)
		TIM7->PSC = 63;
	TIM7->ARR = (uint16_t) time;
	delay_flag = 1;
	TIM7->CR1 |= TIM_CR1_CEN;
	while(delay_flag);

}


void TIM7_IRQHandler(){
	TIM7->SR &= ~TIM_SR_UIF;
	delay_flag = 0;
	TIM7->CNT = 0;
	TIM7->CR1 &= ~(TIM_CR1_CEN);
}



#endif /* DELAY_C_ */
