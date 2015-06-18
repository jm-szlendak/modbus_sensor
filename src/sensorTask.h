#ifndef SENSORTASK_H_INCLUDED
#define SENSORTASK_H_INCLUDED

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware.h"
#include "stm32f30x.h"
#include "protocol.h"
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery_lsm303dlhc.h"

extern TaskHandle_t xSensorTaskHandle;


void vStartSensorTask( UBaseType_t uxPriority);
void vSensorTask(void* pvParameters);


#endif /* SENSORTASK_H_INCLUDED */
