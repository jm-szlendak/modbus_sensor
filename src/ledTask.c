/* Demo program include files. */

#include "ledTask.h"

#define ledSTACK_SIZE		configMINIMAL_STACK_SIZE



/*-----------------------------------------------------------*/

void vStartLEDBlinkTask( UBaseType_t uxPriority)
{
	xTaskCreate(vLEDBlinkTask, "LED Blink", ledSTACK_SIZE, NULL, uxPriority, &xLedTaskHandle );
}
/*-----------------------------------------------------------*/

void vLEDBlinkTask(void* pvParameters){
	TickType_t xLastFlashTime;
	xLastFlashTime = xTaskGetTickCount();

	for(;;){
		vhToggleLED(LED_GPIO,LED3_NUMBER);
		vTaskDelayUntil(&xLastFlashTime,uxBlinkRate/portTICK_PERIOD_MS);
	}

}



