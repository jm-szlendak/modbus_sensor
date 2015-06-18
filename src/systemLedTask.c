/*
 * systemLedTask.c

 *
 *  Created on: 10 sty 2015
 *      Author: jakub
 */

#include "systemLedTask.h"

#define systemledSTACK_SIZE			configMINIMAL_STACK_SIZE
#define systemledBLINK_RATE_FAST	( (TickType_t) 125 )
#define systemledBLINK_RATE_SLOW	( (TickType_t) 500 )

void vStartSystemLEDTask( UBaseType_t uxPriority){
	if(xStatusSemaphore==NULL)
		{
			xStatusSemaphore = xSemaphoreCreateBinary();
		}
		xTaskCreate(vSystemLEDTask, "Status LED", systemledSTACK_SIZE, NULL, uxPriority, (TaskHandle_t *) NULL );
}
void vSystemLEDTask(void* pvParameters){
	//TickType_t xLastBlinkTime = xTaskGetTickCount();
	for(;;){
		xSemaphoreTake(xStatusSemaphore,portMAX_DELAY);
		size_t i;
		for(i=0; i<4;i++){
			vhToggleLED(LED_GPIO,LED10_NUMBER);
			vTaskDelay(systemledBLINK_RATE_FAST/portTICK_PERIOD_MS);
		}

//		xSemaphoreGive(xStatusSemaphore);

	}
}

