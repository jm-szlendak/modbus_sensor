#include "usartRxTask.h"

#define usartRxSTACK_SIZE (configMINIMAL_STACK_SIZE + 2UL )
#define usartRxNO_WAIT 0UL

volatile modbus_frame_t xFrame;
float board_status[NUMBER_OF_FIELDS];
static void setResponse(unsigned char index)
{
    memcpy(&(xFrame.frame[MB_INDX_DATA]), &(board_status[index]), sizeof(float));
}
void vStartUsartRxTask( UBaseType_t uxPriority)
{
    memset(&xFrame,0, sizeof(xFrame));
	xFrameReceivedSemaphore = xSemaphoreCreateBinary();
	xTaskCreate(vUsartRxTask, "USART Rx", usartRxSTACK_SIZE, NULL, uxPriority, &xUsartRxTaskHandle );

}
void vUsartRxTask(void* pvParameters){ //TODO sprawdz dokladnie czy nie ma gowna

	xSemaphoreTake(xFrameReceivedSemaphore, usartRxNO_WAIT);

	for(;;)
	{

		xSemaphoreTake(xFrameReceivedSemaphore, portMAX_DELAY); //Wait for semafore -> wait for frame
		if(xFrame.frame[MB_INDX_DEV_ADDRESS] != DEVICE_MODBUS_ADDRESS)   //It's not addressed to us.
		    continue;
        switch(xFrame.frame[MB_INDX_FUNCTION_CODE])
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
                setResponse(xFrame.frame[MB_INDX_DATA]);
            } break;
            case MODBUS_FUNCTION_READ_REGISTER:
            {
                setResponse(xFrame.frame[MB_INDX_DATA]);
            } break;
        }



	}
}



