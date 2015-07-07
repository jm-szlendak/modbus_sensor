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

extern TaskHandle_t xSensorTaskHandle;  /**< Task handle */
extern SemaphoreHandle_t xDataReadySemaphore;
extern float board_status[NUMBER_OF_FIELDS];
/**
  * \brief  Starts Sensor and Control task
  * \param  uxPriority: Priority of task
  * \retval None
  */
void vStartSensorTask( UBaseType_t uxPriority);

/**
  * \brief  Main function of Sensor and Control task
  * \param  pvParameters: pointer that may pass parameters to task. Unused.
  * \retval None
  */
void vSensorTask(void* pvParameters);


#endif /* SENSORTASK_H_INCLUDED */
