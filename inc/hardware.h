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



#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */


void vhHardwareSetup(void);
void vhToggleLED(GPIO_TypeDef* GPIOx, unsigned long LEDPin);
void vUsartStartTx(void);
void vUsartStopTx(void);
void vUsartSendByte(char cByte);



#endif /* HARDWARESETUP_H_ */
