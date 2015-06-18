/*
 * usartTxTask.h
 *
 *  Created on: 14 sty 2015
 *      Author: jakub
 */

#ifndef USARTTXTASK_H_
#define USARTTXTASK_H_


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware.h"
#include "stm32f30x.h"
#include "protocol.h"

extern SemaphoreHandle_t xTxCompletedSemaphore;
extern TaskHandle_t xUsartTxTaskHandle;
extern QueueHandle_t xUsartTxBuffer;
extern QueueHandle_t xTxRequest;

void vStartUsartTxTask( UBaseType_t uxPriority);
void vUsartTxTask(void* pvParameters);

#endif /* USARTTXTASK_H_ */
