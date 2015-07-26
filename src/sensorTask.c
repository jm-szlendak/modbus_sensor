#include "sensorTask.h"
#include "stdio.h"
#include "task.h"

#define sensorSTACK_SIZE (configMINIMAL_STACK_SIZE + 2UL )
#define sensorQUEUE_SIZE 20UL
#define sensorNO_WAIT 0UL
#define sensorSAMPLE_PERIOD 50UL

void vStartSensorTask( UBaseType_t uxPriority)
{
    /// Create mutex to protect boart_status array
    xDataReadySemaphore = xSemaphoreCreateMutex();
    /// Create task
    xTaskCreate(vSensorTask, "Sensor", sensorSTACK_SIZE, NULL, uxPriority, &xSensorTaskHandle );
}
void vSensorTask(void* pvParameters)
{
    TickType_t xLastSampleTime;
    /// Get system clock state
	xLastSampleTime = xTaskGetTickCount();
    for(;;)
    {
        /// Wait to take mutex protecting board_status array
        if(xSemaphoreTake(xDataReadySemaphore,sensorSAMPLE_PERIOD/portTICK_PERIOD_MS) == pdTRUE)
        {
            /// Enter critical reqion to avoid preemption during IMU read
            vTaskSuspendAll();
            imuReadAcceleration(&(board_status[ACCELERATION_X]));
            imuReadMagneticField(&(board_status[MAGNETIC_FIELD_X]));
            imuReadAngularRate(&(board_status[GYRO_ANG_RATE_X]));
            /// Exit critical reqion. Now preemption is allowed.
            xTaskResumeAll();
            /// Give mutex back so other task can obtain him
            xSemaphoreGive(xDataReadySemaphore);
        }
        else
        {
            /**< TODO: handle with this fail */
        }
        /// Sleep task until next sampling time
        vTaskDelayUntil(&xLastSampleTime, sensorSAMPLE_PERIOD/portTICK_PERIOD_MS);
    }
}


