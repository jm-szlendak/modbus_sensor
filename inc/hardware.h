/*
 * hardware.h
 *
 *  Created on: 10 sty 2015
 *      Author: jakub
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

#include "stm32f30x.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_syscfg.h"
#include "GPIO.h"
#include "RCC.h"
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "imu.h"

#define LED_GPIO		GPIOE
#define LED_GPIO_CLK	RCC_AHBPeriph_GPIOE
#define LED3			PIN_9
#define LED4			PIN_8
#define LED5			PIN_10
#define LED6			PIN_15
#define LED7			PIN_11
#define LED8			PIN_14
#define LED9			PIN_12
#define LED10			PIN_13
#define LED3_NUMBER		9
#define LED4_NUMBER		8
#define LED5_NUMBER		10
#define LED6_NUMBER		15
#define LED7_NUMBER		11
#define LED8_NUMBER		14
#define LED9_NUMBER		12
#define LED10_NUMBER	13

#define USER_BUTTON_PIN                PIN_0
#define USER_BUTTON_GPIO	           GPIOA
#define USER_BUTTON_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define USER_BUTTON_EXTI_LINE          EXTI_Line0
#define USER_BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource0
#define USER_BUTTON_EXTI_IRQn          EXTI0_IRQn





void vhHardwareSetup(void);
void vhToggleLED(GPIO_TypeDef* GPIOx, unsigned long LEDPin);
void vUsartStartTx(void);
void vUsartStopTx(void);
void vUsartSendByte(char cByte);



#endif /* HARDWARESETUP_H_ */
