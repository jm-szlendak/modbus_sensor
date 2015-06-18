#ifndef SENSORTASK_H_INCLUDED
#define SENSORTASK_H_INCLUDED



#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware.h"
#include "stm32f30x.h"
#include "protocol.h"


extern TaskHandle_t xSensorTaskHandle;
extern QueueHandle_t xTxRequest;


void vStartSensorTask( UBaseType_t uxPriority);
void vSensorTask(void* pvParameters);

#endif /* SENSORTASK_H_INCLUDED */
