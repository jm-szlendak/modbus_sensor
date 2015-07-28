#include "usartRxTask.h"
#include "task.h"
#define usartRxSTACK_SIZE (configMINIMAL_STACK_SIZE + 2UL )
#define usartRxMAX_DATA_WAIT (100)
#define usartRxNO_WAIT 0UL

volatile modbus_frame_t xFrame;

static void setResponse(unsigned char index, unsigned char size)
{
    memcpy(&(xFrame.frame[MB_INDX_DATA]), &(board_status[index]), size * sizeof(float));
    xFrame.size = size * sizeof(float) + 2;
}
void vStartUsartRxTask( UBaseType_t uxPriority)
{
    memset(&xFrame,0, sizeof(xFrame));
	xFrameReceivedSemaphore = xSemaphoreCreateBinary();
	xTaskCreate(vUsartRxTask, "USART Rx", usartRxSTACK_SIZE, NULL, uxPriority, &xUsartRxTaskHandle );

}
void vUsartRxTask(void* pvParameters){

	xSemaphoreTake(xFrameReceivedSemaphore, usartRxNO_WAIT);

	for(;;)
	{
        /// Wait for semafore. It will be given when frame arrives.
		xSemaphoreTake(xFrameReceivedSemaphore, portMAX_DELAY);
		if(xFrame.frame[MB_INDX_DEV_ADDRESS] != DEVICE_MODBUS_ADDRESS)   //It's not addressed to us.
        {
            memset(&xFrame,0, sizeof(xFrame));
            continue;
        }

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
                /// Try to obtain mutex. Don't wait forever.
                if(xSemaphoreTake(xDataReadySemaphore, usartRxMAX_DATA_WAIT/portTICK_PERIOD_MS) == pdTRUE)
                {
                    char index = xFrame.frame[MB_INDX_DATA];
                    char size = xFrame.frame[MB_INDX_SIZE];
                    /// Prepare command response
                    setResponse(index, size);

                    /// Give mutex back
                   xSemaphoreGive(xDataReadySemaphore);
                }



            } break;
            case MODBUS_FUNCTION_READ_REGISTER:
            {
            } break;
        }
        vTaskDelay(5/portTICK_PERIOD_MS);
        vUsartStartTx();
        //TODO start TX here

	}
}



