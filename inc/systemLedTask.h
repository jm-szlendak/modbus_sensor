/*
 * systemLedTask.h
 *
 *  Created on: 10 sty 2015
 *      Author: jakub
 */

#ifndef SYSTEMLEDTASK_H_
#define SYSTEMLEDTASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware.h"
#include "stm32f30x.h"

extern SemaphoreHandle_t xStatusSemaphore;


void vStartSystemLEDTask( UBaseType_t uxPriority);
void vSystemLEDTask(void* pvParameters);

#endif /* SYSTEMLEDTASK_H_ */
