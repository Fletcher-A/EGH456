/*
 * fault_task.c — Centralised fault aggregator.
 *
 * INTER-TASK INTERFACE
 * --------------------
 *
 * INPUTS (consumed by this task):
 *   xSystemEvents bits (waited on, NOT cleared so motor_task can
 *   still consume them in its state machine):
 *       EVT_ESTOP_POWER     — set by sensor_task (over-current limit)
 *       EVT_ESTOP_ACCEL     — set by sensor_task (crash threshold)
 *       EVT_ESTOP_DISTANCE  — set by sensor_task (too close)
 *       EVT_SENSOR_FAULT    — set by sensor_task (sensor read failed)
 *
 * OUTPUTS (produced by this task):
 *   (future) UART log lines describing which fault fired.
 *   No queues or event bits set by this task.
 *
 * PUBLIC FUNCTIONS (called from elsewhere):
 *   vCreateFaultTask()  — called once from main.c during startup
 *
 * Rationale: keeps motor_task focused on control and sensor_task
 * focused on data acquisition; all fault logging is in one place.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "shared.h"
#include "utils/uart_log.h"
/*-----------------------------------------------------------*/

static void prvLogFaultBits(EventBits_t bits)
{
    if (bits & EVT_ESTOP_POWER)
    {
        uart_log_printf("FAULT: power E-stop (limit %.0f W, measured %.0f W)\n",
                        g_thresh_power_w, g_motor_power_watts);
    }
    if (bits & EVT_ESTOP_ACCEL)
    {
        uart_log_printf("FAULT: accel E-stop (limit %.1f g)\n",
                        g_thresh_accel_g);
    }
    if (bits & EVT_ESTOP_DISTANCE)
    {
        uart_log_printf("FAULT: distance E-stop\n");
    }
    if (bits & EVT_ESTOP_DRIVER)
    {
        uart_log_printf("FAULT: DRV8323 nFAULT (driver hardware)\n");
    }
    if (bits & EVT_SENSOR_FAULT)
    {
        uart_log_printf("FAULT: sensor / MotorLib init error\n");
    }
}

static void prvFaultTask(void *pvParameters)
{
    (void)pvParameters;
    EventBits_t last_logged = 0;

    uart_log_printf("Fault task ready (waits on EVT_ESTOP_* / EVT_SENSOR_FAULT)\n");

    for (;;)
    {
        EventBits_t bits = xEventGroupWaitBits(
            xSystemEvents,
            EVT_ESTOP_ANY | EVT_SENSOR_FAULT,
            pdFALSE, pdFALSE,
            portMAX_DELAY);

        EventBits_t newly = bits & ~last_logged;
        if (newly != 0)
        {
            prvLogFaultBits(newly);
            last_logged |= newly;
        }

        if ((bits & (EVT_ESTOP_ANY | EVT_SENSOR_FAULT)) == 0)
        {
            last_logged = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*-----------------------------------------------------------*/

void vCreateFaultTask(void)
{
    xTaskCreate(prvFaultTask, "Fault",
                configMINIMAL_STACK_SIZE * 2, NULL,
                tskIDLE_PRIORITY + 1, NULL);
}
