/*
 * GPIO.h
 *
 *  Created on: 4 gru 2014
 *      Author: jakub
 */

#ifndef GPIO_H_
#define GPIO_H_
#include "stm32f30x.h"

#define PIN_0 	(uint32_t)0x00000003
#define PIN_1 	(uint32_t)0x0000000C
#define PIN_2	(uint32_t)0x00000030
#define PIN_3 	(uint32_t)0x000000C0
#define PIN_4	(uint32_t)0x00000300
#define PIN_5 	(uint32_t)0x00000C00
#define PIN_6	(uint32_t)0x00003000
#define PIN_7 	(uint32_t)0x0000C000
#define PIN_8	(uint32_t)0x00030000
#define PIN_9 	(uint32_t)0x000C0000
#define PIN_10	(uint32_t)0x00300000
#define PIN_11 	(uint32_t)0x00C00000
#define PIN_12	(uint32_t)0x03000000
#define PIN_13 	(uint32_t)0x0C000000
#define PIN_14	(uint32_t)0x30000000
#define PIN_15 	(uint32_t)0xC0000000

#define GPIO_MODER_OUTPUT 0x55555555
#define GPIO_MODER_ALTERNATE 0xCCCCCCCC

#define GPIO_OSPEEDR_MEDIUM 0x55555555

#define GPIO_PUPDR_PULLUP 0x55555555
#define GPIO_PUPDR_PULLDOWN 0xCCCCCCCC

typedef enum{
	SPEED_FAST,
	SPEED_MEDIUM,
	SPEED_SLOW
}SpeedEnum;

typedef enum{
	OUTPUT,
	INPUT,
	ALTERNATE
}ModeEnum;

typedef enum{
	PULLUP,
	PULLDOWN,
	FLOATING
}PUPDEnum;

typedef enum{
	OPEN_DRAIN,
	PUSH_PULL
}OutputModeEnum;

void GPIO_SetInput(GPIO_TypeDef* GPIOx, uint32_t pins, SpeedEnum speed, PUPDEnum pupd);
void GPIO_SetOutput(GPIO_TypeDef* GPIOx, uint32_t pins, SpeedEnum speed, PUPDEnum pupd, OutputModeEnum otype);




#endif /* GPIO_H_ */
