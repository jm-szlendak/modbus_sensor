/*
 * GPIO.c
 *
 *  Created on: 5 gru 2014
 *      Author: jakub
 */

#include "GPIO.h"

void GPIO_SetInput(GPIO_TypeDef* GPIOx, uint32_t pins32, SpeedEnum speed, PUPDEnum pupd) {
	GPIOx->MODER &= ~(pins32);
	GPIOx->OSPEEDR &= ~(pins32);					//Reset OSPEEDR (Low speed)
	if (speed == SPEED_FAST)
		GPIOx->OSPEEDR |= pins32;
	else if (speed == SPEED_MEDIUM)
		GPIOx->OSPEEDR |= (GPIO_OSPEEDR_MEDIUM & pins32);

	GPIOx->PUPDR &= ~pins32;					//Reset PUPDR (Floating mode)
	if (pupd == PULLUP)
		GPIOx->PUPDR |= (GPIO_PUPDR_PULLUP & pins32);
	else if (pupd == PULLDOWN)
		GPIOx->PUPDR |= (GPIO_PUPDR_PULLDOWN & pins32);

}
void GPIO_SetOutput(GPIO_TypeDef* GPIOx, uint32_t pins32, SpeedEnum speed,PUPDEnum pupd, uint8_t otype) {
	//Convert 32 bit mask to 16 bit
	uint16_t pins16 = 0x0000;
	uint32_t mask = 0x00000001;
	uint32_t check = 0x00000000;
	for (unsigned char i = 0; i < 16; i++) {
		check = pins32 & mask;
		if (check != (uint32_t) 0) {
			pins16 = check >> i;
		}
		mask = mask << 2;
	}
	GPIOx->MODER &= ~(pins32); 						//Reset MODER
	GPIOx->MODER |= (GPIO_MODER_OUTPUT & pins32);	//Set MODER as output

	GPIOx->OSPEEDR &= ~(pins32);					//Reset OSPEEDR (Low speed)
	if (speed == SPEED_FAST)
		GPIOx->OSPEEDR |= pins32;
	else if (speed == SPEED_MEDIUM)
		GPIOx->OSPEEDR |= (GPIO_OSPEEDR_MEDIUM & pins32);

	GPIOx->PUPDR &= ~pins32;					//Reset PUPDR (Floating mode)
	if (pupd == PULLUP)
		GPIOx->PUPDR |= (GPIO_PUPDR_PULLUP & pins32);
	else if (pupd == PULLDOWN)
		GPIOx->PUPDR |= (GPIO_PUPDR_PULLDOWN & pins32);

	if (otype == OPEN_DRAIN)
		GPIOx->OTYPER |= pins16;
	else
		GPIOx->OTYPER &= ~pins16;
}


