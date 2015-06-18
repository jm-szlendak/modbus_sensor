#include "usartRxTask.h"

#define usartRxSTACK_SIZE (configMINIMAL_STACK_SIZE + 2UL )
#define usartRxNO_WAIT 0UL

rx_frame_t xRxFrame;

void vStartUsartRxTask( UBaseType_t uxPriority){
    memset(&xRxFrame,0, sizeof(xRxFrame));
	xRxCompletedSemaphore = xSemaphoreCreateBinary();
	xTaskCreate(vUsartRxTask, "USART Rx", usartRxSTACK_SIZE, NULL, uxPriority, &xUsartRxTaskHandle );

}
void vUsartRxTask(void* pvParameters){ //TODO sprawdz dokladnie czy nie ma gowna

	xSemaphoreTake(xRxCompletedSemaphore, usartRxNO_WAIT);

	for(;;)
	{

		xSemaphoreTake(xRxCompletedSemaphore, portMAX_DELAY); //Wait for semafore -> wait for frame
		size_t i = 0;
		if(frame[0] != DEVICE_MODBUS_ADDRESS)   //It's not addressed to us.
		    continue;
        switch(frame[1])
        {
            case MODBUS_FUNCTION_SLEEP:
            {
                //Goodnight!
            } break;
            case MODBUS_FUNCTION_WAKEUP:
            {
                //Wakeup!
            } break;
            case MODBUS_FUNCTION_READ_FLOAT:
            {
                //Goodnight!
            } break;
        }
        for(int i=0; i<xRxFrame.size, i++)
        {

        }

	}

}
