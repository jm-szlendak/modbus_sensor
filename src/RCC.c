/*
 * RCC.c
 *
 *  Created on: 4 gru 2014
 *      Author: jakub
 *      Functions to set Reset and Clock Control registers in STM32F30x
 */
#include "RCC.h"


unsigned char CPU_Freq;

void ResetRCC(){
	RCC->CR |= RCC_CR_HSION;			//Turn on High Speed Internal Osc.
	RCC->CFGR = (uint32_t)0x0000;   	//Reset CFGR
	RCC->CR &= (uint32_t)0xF8FFC000;	//Reset SW[1:0], HPRE[3:0], PPRE[2:0] and MCOSEL[2:0] bits
	RCC->CR &= (uint32_t)0xFEF6FFFF;	//Reset HSEON, CSSON and PLLON bits
	RCC->CR &= (uint32_t)0xFFFBFFFF;	//Reset HSEBYP bit
	RCC->CFGR &= (uint32_t)0xFF80FFFF;	//Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits
	RCC->CFGR2 &= (uint32_t)0xFFFFC000; //Reset PREDIV1[3:0] and ADCPRE[13:4] bits
	RCC->CFGR3 &= (uint32_t)0xF00FCCC;  //Reset USARTSW[1:0], I2CSW and TIMSW bits
	RCC->CIR = 0x00000000;				//Disable all interrupts
}

/**
 *  @brief Function to set RCC on HSI, maximum possible speed.
*/
void SetRCC(){
	ResetRCC();
	RCC->CR |= RCC_CR_HSION;		//Turn on HSI, just in case
	RCC->CR &= ~(RCC_CR_HSEON);		//Turn off HSE, just in case

	do
	{;}while((RCC->CR & RCC_CR_HSIRDY) != RCC_CR_HSIRDY); //Wait for HSI to be ready...

	RCC->CFGR &= ~(RCC_CFGR_PLLSRC);		//HSI as PLL source
	RCC->CFGR |= RCC_CFGR_PLLMULL16;		//16x PLL multiplier
	RCC->CR |= RCC_CR_PLLON;				//start PLL
	do
	{;}while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);	//Wait for PLL to be ready...

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; //APB low-speed prescaler /1
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; //APB high-speed prescaler /2
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  //HCLK prescaler /1

	FLASH->ACR |= FLASH_ACR_PRFTBE;		//Flash prefetch buffer enable
	FLASH->ACR |= FLASH_ACR_LATENCY_1;  //2WS flash latency
	RCC->CFGR |= RCC_CFGR_SW_PLL;		//Set PLL as SYSCLK source
	do
	{;}
	while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);



}



