#ifndef USARTRXTASK_H_INCLUDED
#define USARTRXTASK_H_INCLUDED


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware.h"
#include "stm32f30x.h"
#include "protocol.h"
#include <stdlib.h>

extern SemaphoreHandle_t xFrameReceivedSemaphore;;
extern TaskHandle_t xUsartRxTaskHandle;
extern QueueHandle_t xUsartRxBuffer;

void vStartUsartRxTask( UBaseType_t uxPriority);
void vUsartRxTask(void* pvParameters);


#endif /* USARTRXTASK_H_INCLUDED */
