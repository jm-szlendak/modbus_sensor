/*
 * HD44780.h
 *
 *  Created on: 18 gru 2014
 *      Author: jakub
 */

#ifndef HD44780_H_
#define HD44780_H_

#include "stm32f30x.h"
#include "delay.h"
#include "GPIO.h"

#define LCD_GPIO			GPIOD
#define LCD_GPIO_RCC	  	RCC_AHBENR_GPIODEN
#define LCD_RS_PIN			PIN_0
	#define LCD_RS_BS			GPIO_BSRR_BS_0
	#define LCD_RS_BR			GPIO_BSRR_BR_0
#define LCD_E_PIN			PIN_1
	#define LCD_E_BS			GPIO_BSRR_BS_1
	#define LCD_E_BR			GPIO_BSRR_BR_1
#define LCD_DB4_PIN			PIN_2
	#define LCD_DB4_BS			GPIO_BSRR_BS_2
	#define LCD_DB4_BR			GPIO_BSRR_BR_2
#define LCD_DB5_PIN			PIN_3
	#define LCD_DB5_BS			GPIO_BSRR_BS_3
	#define LCD_DB5_BR			GPIO_BSRR_BR_3
#define LCD_DB6_PIN			PIN_4
	#define LCD_DB6_BS			GPIO_BSRR_BS_4
	#define LCD_DB6_BR			GPIO_BSRR_BR_4
#define LCD_DB7_PIN			PIN_5
	#define LCD_DB7_BS			GPIO_BSRR_BS_5
	#define LCD_DB7_BR			GPIO_BSRR_BR_5

/*** ***/
#define HD44780_CLEAR					0x01

#define HD44780_HOME					0x02

#define HD44780_ENTRY_MODE				0x04
	#define HD44780_EM_SHIFT_CURSOR		0
	#define HD44780_EM_SHIFT_DISPLAY	1
	#define HD44780_EM_DECREMENT		0
	#define HD44780_EM_INCREMENT		2

#define HD44780_DISPLAY_ONOFF			0x08
	#define HD44780_DISPLAY_OFF			0
	#define HD44780_DISPLAY_ON			4
	#define HD44780_CURSOR_OFF			0
	#define HD44780_CURSOR_ON			2
	#define HD44780_CURSOR_NOBLINK		0
	#define HD44780_CURSOR_BLINK		1

#define HD44780_DISPLAY_CURSOR_SHIFT	0x10
	#define HD44780_SHIFT_CURSOR		0
	#define HD44780_SHIFT_DISPLAY		8
	#define HD44780_SHIFT_LEFT			0
	#define HD44780_SHIFT_RIGHT			4

#define HD44780_FUNCTION_SET			0x20
	#define HD44780_FONT5x7				0
	#define HD44780_FONT5x10			4
	#define HD44780_ONE_LINE			0
	#define HD44780_TWO_LINE			8
	#define HD44780_4_BIT				0
	#define HD44780_8_BIT				16

#define HD44780_CGRAM_SET				0x40

#define HD44780_DDRAM_SET				0x80


void LCD_Init(void);
void LCD_WriteCommand(unsigned char);
void LCD_WriteData(unsigned char);
void LCD_WriteText(char *);
void LCD_GoTo(unsigned char, unsigned char);
void LCD_Clear(void);
void LCD_Home(void);



#endif /* HD44780_H_ */
