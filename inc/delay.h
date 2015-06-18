/*
 * delay.h
 *
 *  Created on: 18 gru 2014
 *      Author: jakub
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f30x.h"

typedef enum{
	DELAY_MS,
	DELAY_US
}DelayType;

void delay_init();
void delay(uint32_t time, DelayType type);


void TIM7_IRQHandler();


#endif /* DELAY_H_ */
