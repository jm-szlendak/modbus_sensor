/*
 * buttonTask.h
 *
 *  Created on: 10 sty 2015
 *      Author: jakub
 */

#ifndef BUTTONTASK_H_
#define BUTTONTASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware.h"
#include "stm32f30x.h"

extern TaskHandle_t xLedTaskHandle;
extern SemaphoreHandle_t xButtonSemaphore;
extern SemaphoreHandle_t xStatusSemaphore;
extern unsigned long ulButtonPressCount;

void vStartButtonTask( UBaseType_t uxPriority);
void vButtonTask(void* pvParameters);

#endif /* BUTTONTASK_H_ */
