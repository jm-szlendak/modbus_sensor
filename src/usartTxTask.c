#include "usartTxTask.h"

#define usartTxSTACK_SIZE (configMINIMAL_STACK_SIZE + 2UL )
#define usartTxQUEUE_SIZE 20UL
#define usartTxNO_WAIT 0UL



const char MSG_hello[] = "!Hello,it's Xplorer\r\n";

void vStartUsartTxTask( UBaseType_t uxPriority){
	xTxCompletedSemaphore = xSemaphoreCreateBinary();
	xUsartTxBuffer = xQueueCreate(usartTxQUEUE_SIZE,sizeof(portCHAR));

	size_t i =0;
	for(i=0; i<21; i++){
		xQueueSendToBack(xUsartTxBuffer,&(MSG_hello[i]),usartTxNO_WAIT);
	}
	xTaskCreate(vUsartTxTask, "USART Tx", usartTxSTACK_SIZE, NULL, uxPriority, &xUsartTxTaskHandle );

}
void vUsartTxTask(void* pvParameters){
	xSemaphoreTake(xTxCompletedSemaphore, usartTxNO_WAIT);
	vUsartStartTx();    //say hello

	for(;;)
	    ;
/*	{
        unsigned long ulItem, ulCount;
        unsigned long ulTotalPacketSize = 2;
		xSemaphoreTake(xTxCompletedSemaphore, portMAX_DELAY);           //Block until TX completed
//		xQueueReceive(xTxRequest, xMessageToSend, portMAX_DELAY);       //Block until there's something to send

        for(ulItem = 0; ulItem< g_ulNumRealTimeData; ulItem++)
        {

            for(ulCount = 0; ulCount < g_RealTimeData[ulItem].ucSize; ulCount++)
                {
                    char cTemp = *(g_RealTimeData[ulItem].pucValue+ulCount);
                    xQueueSend(xUsartTxBuffer, &cTemp, usartTxNO_WAIT);
                    ulTotalPacketSize++;
                }
        }

        char cTemp = TAG_DATA;
        xQueueSendToFront(xUsartTxBuffer, &ulTotalPacketSize, usartTxNO_WAIT);
        xQueueSendToFront(xUsartTxBuffer, &cTemp, usartTxNO_WAIT);
        cTemp = TAG_DATA_END;
        xQueueSend(xUsartTxBuffer, &cTemp, usartTxNO_WAIT);
        vUsartStartTx();
	}
*/
}

