/*
 * buttonTask.c
 *
 *  Created on: 10 sty 2015
 *      Author: jakub
 */

#include "buttonTask.h"


#define buttonSTACK_SIZE		configMINIMAL_STACK_SIZE
#define buttonNO_BLOCK			( 0UL )


void vStartButtonTask( UBaseType_t uxPriority){
	if(xButtonSemaphore==NULL)
	{
		xButtonSemaphore = xSemaphoreCreateBinary();
	}
	xTaskCreate(vButtonTask, "Button", buttonSTACK_SIZE, NULL, uxPriority, (TaskHandle_t *) NULL );
}
void vButtonTask(void* pvParameters){

	xSemaphoreTake(xButtonSemaphore,buttonNO_BLOCK);
	for(;;){
		xSemaphoreTake(xButtonSemaphore, portMAX_DELAY);
		ulButtonPressCount++;
		xSemaphoreGive(xStatusSemaphore);
		if(eTaskGetState(xLedTaskHandle)== eBlocked || eTaskGetState(xLedTaskHandle)== eRunning ){
			vTaskSuspend(xLedTaskHandle);
		}
		else if(eTaskGetState(xLedTaskHandle)==eSuspended){
			vTaskResume(xLedTaskHandle);
		}

		vTaskDelay(500);
	}
}


