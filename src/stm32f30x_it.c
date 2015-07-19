/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_it.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern SemaphoreHandle_t xButtonSemaphore;
extern SemaphoreHandle_t xFrameReceivedSemaphore;


extern QueueHandle_t xUsartTxBuffer;
extern QueueHandle_t xUsartRxBuffer;
extern volatile modbus_frame_t xFrame;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void SysTick_Handler(void)
{

}

/**
  * @brief  This function handles EXTI 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	long lHigherPriorityTaskWoken = pdFALSE;
	EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	xSemaphoreGiveFromISR(xButtonSemaphore,&lHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);

}

void USART1_IRQHandler(void){
	long lHigherPriorityTaskWoken = pdFALSE;
	if(USART1->ISR & USART_FLAG_RXNE)
    {

        xFrame.frame[xFrame.size++]=USART1->RDR;
	}

	if(USART1->ISR & USART_FLAG_TXE)
    {
        static unsigned char index = 0;
		char ch;
		if(index>=xFrame.size)
        {
            vUsartStopTx();
            index = 0;
        }
        else
        {
            vUsartSendByte(xFrame.frame[index]);
            index++;
        }

	}
	if(USART1->ISR & USART_FLAG_RTO)
    {
        USART1->ICR |= USART_ICR_RTOCF;
        xSemaphoreGiveFromISR(xFrameReceivedSemaphore, &lHigherPriorityTaskWoken);
    }
    portEND_SWITCHING_ISR(lHigherPriorityTaskWoken);
}

/**
  * @brief  This function handles EXTI 15-10 interrupt request.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void EXTI15_10_IRQHandler(void)
{
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
