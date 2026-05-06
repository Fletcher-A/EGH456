/*
 * fault_task.c — Centralised fault aggregator.
 *
 * Watches the fault-related event bits in xSystemEvents and logs a
 * summary line over UART when any of them assert. Keeps motor_task
 * focused on control and sensor_task focused on data.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared.h"
/*-----------------------------------------------------------*/

static void prvFaultTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        /* TODO: xEventGroupWaitBits on EVT_ESTOP_ANY | EVT_SENSOR_FAULT. */

        /* TODO: take xUARTMutex, print which bits are set, release. */

        /* TODO: cool-down delay so we don't spam if a bit stays high. */
    }
}

/*-----------------------------------------------------------*/

void vCreateFaultTask(void)
{
    xTaskCreate(prvFaultTask, "Fault",
                configMINIMAL_STACK_SIZE * 2, NULL,
                tskIDLE_PRIORITY + 1, NULL);
}
