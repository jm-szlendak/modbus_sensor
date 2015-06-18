/*
 * HD44780.c
 *
 *  Created on: 19 gru 2014
 *      Author: jakub
 */

#include "HD44780.h"


void _LCD_OutNibble(unsigned char nibble){
	if(nibble & 0x01)
		LCD_GPIO->BSRR |= LCD_DB4_BS;
	else
		LCD_GPIO->BSRR |= LCD_DB4_BR;

	if(nibble & 0x02)
		LCD_GPIO->BSRR |= LCD_DB5_BS;
	else
		LCD_GPIO->BSRR |= LCD_DB5_BR;

	if(nibble & 0x04)
		LCD_GPIO->BSRR |= LCD_DB6_BS;
	else
		LCD_GPIO->BSRR |= LCD_DB6_BR;

	if(nibble & 0x08)
		LCD_GPIO->BSRR |= LCD_DB7_BS;
	else
		LCD_GPIO->BSRR |= LCD_DB7_BR;
}

void _LCD_Write(unsigned char data){
	LCD_GPIO->BSRR |= LCD_E_BS;
	_LCD_OutNibble(data >> 4);
	LCD_GPIO->BSRR |= LCD_E_BR;

	LCD_GPIO->BSRR |= LCD_E_BS;
	_LCD_OutNibble(data);
	LCD_GPIO->BSRR |= LCD_E_BR;
	delay(50, DELAY_US);
}

void LCD_Init(void){
	RCC->AHBENR |= LCD_GPIO_RCC;
	GPIO_SetOutput(LCD_GPIO, LCD_RS_PIN | LCD_E_PIN | LCD_DB4_PIN | LCD_DB5_PIN | LCD_DB6_PIN | LCD_DB7_PIN, SPEED_FAST, FLOATING, PUSH_PULL);
	LCD_GPIO->BSRR |= LCD_RS_BR | LCD_E_BR | LCD_DB4_BR | LCD_DB5_BR | LCD_DB6_BR | LCD_DB7_BR;
	delay(15, DELAY_MS);
	LCD_GPIO->BSRR |= LCD_RS_BR | LCD_E_BR;
	unsigned char i;
	for(i = 0; i<3; i++){
		LCD_GPIO->BSRR |= LCD_E_BS;
		_LCD_OutNibble(0x03);
		LCD_GPIO->BSRR |= LCD_E_BR;
		delay(5, DELAY_MS);
	}
	LCD_GPIO->BSRR |= LCD_E_BS;
	_LCD_OutNibble(0x02);
	LCD_GPIO->BSRR |= LCD_E_BR;
	delay(1, DELAY_MS);
	LCD_WriteCommand(HD44780_FUNCTION_SET | HD44780_FONT5x7 | HD44780_TWO_LINE | HD44780_4_BIT);
	LCD_WriteCommand(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_OFF);
	LCD_WriteCommand(HD44780_CLEAR);
	delay(2, DELAY_MS);
	LCD_WriteCommand(HD44780_ENTRY_MODE | HD44780_EM_SHIFT_CURSOR | HD44780_EM_INCREMENT);
	LCD_WriteCommand(HD44780_DISPLAY_ONOFF | HD44780_DISPLAY_ON | HD44780_CURSOR_OFF | HD44780_CURSOR_NOBLINK);

}

void LCD_WriteCommand(unsigned char command){
	LCD_GPIO->BSRR |= LCD_RS_BR;
	_LCD_Write(command);
}

void LCD_WriteData(unsigned char data){
	LCD_GPIO->BSRR |= LCD_RS_BS;
	_LCD_Write(data);
}

void LCD_WriteText(char* text){
	while(*text)
		LCD_WriteData(*text++);
}

void LCD_GoTo(unsigned char x, unsigned char y){
	LCD_WriteCommand(HD44780_DDRAM_SET | (x + (0x40 * y)));
}

void LCD_Clear(){
	LCD_WriteCommand(HD44780_CLEAR);
	delay(2, DELAY_MS);
}

void LCD_Home(){
	LCD_WriteCommand(HD44780_HOME);
	delay(2, DELAY_MS);
}
