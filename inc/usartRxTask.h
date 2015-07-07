#ifndef USARTRXTASK_H_INCLUDED
#define USARTRXTASK_H_INCLUDED


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware.h"
#include "stm32f30x.h"
#include "protocol.h"
#include <stdlib.h>

extern SemaphoreHandle_t xFrameReceivedSemaphore;
extern SemaphoreHandle_t xDataReadySemaphore;
extern TaskHandle_t xUsartRxTaskHandle;

extern float board_status[NUMBER_OF_FIELDS];

void vStartUsartRxTask( UBaseType_t uxPriority);
void vUsartRxTask(void* pvParameters);


#endif /* USARTRXTASK_H_INCLUDED */
