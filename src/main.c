//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>


/**KELNER**/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "protocol.h"
#include "hardware.h"

/**TASKS**/
#include "ledTask.h"
#include "buttonTask.h"
#include "systemLedTask.h"
#include "usartTxTask.h"
#include "sensorTask.h"

/**TASK PRIORITIES**/
#define mainBLINK_TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )
#define mainBUTTON_TASK_PRIORITY	( tskIDLE_PRIORITY )
#define mainSYSTEMLED_TASK_PRIORITY	( tskIDLE_PRIORITY + 2UL )
#define mainUSARTTX_TASK_PRIORITY   ( tskIDLE_PRIORITY + 2UL )
#define mainUSARTRX_TASK_PRIORITY   ( tskIDLE_PRIORITY + 3UL )
#define mainSENSOR_TASK_PRIORITY    ( tskIDLE_PRIORITY + 4UL )


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

TickType_t uxBlinkRate = 333;
SemaphoreHandle_t xButtonSemaphore = NULL;
SemaphoreHandle_t xStatusSemaphore = NULL;
SemaphoreHandle_t xTxCompletedSemaphore = NULL;
SemaphoreHandle_t xFrameReceivedSemaphore = NULL;
TaskHandle_t xUsartRxTaskHandle;
TaskHandle_t xUsartTxTaskHandle;
TaskHandle_t xLedTaskHandle;
TaskHandle_t xSensorTaskHandle;
QueueHandle_t xUsartTxBuffer;
QueueHandle_t xUsartRxBuffer;
QueueHandle_t xTxRequest;
unsigned long ulButtonPressCount = 0;




int main(int argc, char* argv[])
{
	vhHardwareSetup();

	vStartLEDBlinkTask(mainBLINK_TASK_PRIORITY);
	vStartButtonTask(mainBUTTON_TASK_PRIORITY);
	vStartSystemLEDTask(mainSYSTEMLED_TASK_PRIORITY);
	vStartUsartRxTask(mainUSARTRX_TASK_PRIORITY);
	vTaskStartScheduler();

  while (1);
}

void vApplicationTickHook( void )
{

}
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
