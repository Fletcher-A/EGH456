/*
 * sensor_task.c — Sensor sampling task (assignment 2.2).
 *
 * Owns:
 *   - Periodic timing (FreeRTOS software timer or vTaskDelayUntil)
 *   - Reading each sensor via its driver (no busy-polling)
 *   - Filtering (must NOT be done in ISRs)
 *   - Publishing SensorMsgObj messages to xSensorQueue
 *   - Setting EVT_ESTOP_* bits when thresholds are crossed
 *   - Setting EVT_NIGHT_DETECTED based on lux < NIGHT_LIGHT_LUX
 *
 * Sensors (assignment 2.2.1 + 2.2.2):
 *   - OPT3001 (light, I2C, ≥2 Hz)
 *   - DRV8323 (motor power via ADC, ≥150 Hz)
 *   - Hall    (motor speed, GPIO ISR + timer)
 *   - Optional A and B: pick BMI160 / SHT31 / VL53L0X
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "shared.h"
/*-----------------------------------------------------------*/

static void prvSensorTask(void *pvParameters)
{
    (void)pvParameters;

    /* TODO: bring up sensor drivers (init / self-test). */

    for (;;)
    {
        /* TODO: periodic delay (e.g. 5 ms = 200 Hz outer loop). */

        /* TODO: read motor power (ADC) at the highest required rate. */

        /* TODO: read OPT3001 light at ≥2 Hz, set EVT_NIGHT_DETECTED. */

        /* TODO: read optional sensor A. */

        /* TODO: read optional sensor B. */

        /* TODO: filter each signal (low-pass / moving average). */

        /* TODO: compare filtered values to thresholds, set
         *       EVT_ESTOP_POWER / _ACCEL / _DISTANCE if exceeded. */

        /* TODO: publish SensorMsgObj to xSensorQueue. */
    }
}

/*-----------------------------------------------------------*/

void vCreateSensorTask(void)
{
    xTaskCreate(prvSensorTask, "Sensor",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 3, NULL);
}
