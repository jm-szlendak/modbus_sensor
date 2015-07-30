/*
 * hardware.c
 *
 *  Created on: 10 sty 2015
 *      Author: jakub
 */

#include "hardware.h"
#include "stm32f30x_usart.h"
#include "imu.h"


void vhHardwareSetup(void){

	SystemInit();
	//SetRCC();
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	RCC->AHBENR |= LED_GPIO_CLK | USER_BUTTON_GPIO_CLK;
	RCC->APB2ENR |= RCC_APB2Periph_SYSCFG;

	GPIO_SetOutput(LED_GPIO,LED3 | LED6 | LED10 | LED4,SPEED_FAST,FLOATING,PUSH_PULL);

	/*User button in EXTI mode setting*/
	GPIO_SetInput(USER_BUTTON_GPIO, USER_BUTTON_PIN, SPEED_FAST,FLOATING);
	SYSCFG_EXTILineConfig(USER_BUTTON_EXTI_PORT_SOURCE,USER_BUTTON_EXTI_PIN_SOURCE);
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	EXTI->FTSR |= EXTI_FTSR_TR0;

	NVIC_SetPriority(EXTI0_IRQn,NVIC_EncodePriority(NVIC_PriorityGroup_4,0x0F,0x0F));
	NVIC_EnableIRQ(EXTI0_IRQn);

	/*USART1*/
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1;
	GPIOC->OTYPER |= GPIO_OTYPER_OT_4;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR4;
	GPIOC->AFR[0] |= (7<<(4*4)) | (7<<(5*4));

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->BRR = 72000000/115200;
	USART1->CR1 &= ~USART_CR1_OVER8;
	//USART1->CR1 |= USART_CR1_PCE;
	//USART1->CR1 |= USART_CR1_M;

	USART1->CR2 |= USART_CR2_RTOEN;
	USART1->RTOR =(uint32_t)(22) & USART_RTOR_RTO;
	//USART1->CR1 |= USART_CR1_TE;
	USART1->CR1 |= USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;

	//USART1->CR1 |= USART_CR1_TXEIE; //później
	USART1->CR1 |= USART_CR1_RXNEIE;
	USART1->CR1 |= USART_CR1_RTOIE;



	NVIC_SetPriority(USART1_IRQn,NVIC_EncodePriority(NVIC_PriorityGroup_4,0x0F,0x0F)); //??
	NVIC_EnableIRQ(USART1_IRQn);

    imuHardwareSetup();




}
void vhToggleLED(GPIO_TypeDef* GPIOx, unsigned long LEDPinNumber){
	GPIOx->ODR ^= (1<<LEDPinNumber);
}

void vUsartStartTx(void){
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_TXEIE;
}

void vUsartStopTx(void){
    USART1->CR1 &= ~USART_CR1_TE;
    USART1->CR1 &= ~USART_CR1_TXEIE;
}

void vUsartSendByte(char cByte){
	USART1->TDR = cByte;
}


void vAcc_Config(void){
	LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
	LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

	LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
	LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
	LSM303DLHCAcc_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
	LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
	LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
	LSM303DLHCAcc_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
	LSM303DLHCAcc_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;
	LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

	LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection = LSM303DLHC_HPM_NORMAL_MODE;
	LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
	LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

	LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

/**
 * @brief Configure the gyro
 * @param None
 * @retval None
 */
void vGyro_Config(void){
	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
	L3GD20_Init(&L3GD20_InitStructure);

	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);

}


/**
 * @brief Configure the magnetometer
 * @param None
 * @retval None
 */
void vMagnetometer_Config(void){
	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;

	LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
	LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
	LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
	LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;

	LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);
}



uint32_t LSM303DLHC_TIMEOUT_UserCallback(){
	return 0;
}
uint32_t L3GD20_TIMEOUT_UserCallback(){
	return 0;
}
