/*
 * motor_task.c — Motor control task (assignment 2.1).
 *
 * Owns:
 *   - Motor state machine (Idle, Starting, Running, E-Stop Braking, Fault Latched)
 *   - Reference-speed ramp limiter (acceleration/deceleration)
 *   - Closed-loop PI speed controller
 *   - Calls into MotorLib (initMotorLib, setDuty, updateMotor, stopMotor)
 *   - Publishes MotorMsgObj messages to xMotorQueue
 *
 * Triggered by:
 *   - Periodic 1-10 ms tick
 *   - Hall sensor ISR (in motor_driver.c) updates commutation
 *   - xCommandQueue (desired RPM from GUI)
 *   - xSystemEvents EVT_ESTOP_* and EVT_USER_* bits
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "shared.h"
/*-----------------------------------------------------------*/

static void prvMotorTask(void *pvParameters)
{
    (void)pvParameters;

    /* TODO: motor_driver_init(); */

    for (;;)
    {
        /* TODO: 1. Periodic delay (control loop period, e.g. 5 ms). */

        /* TODO: 2. Pick up new desired RPM from xCommandQueue. */

        /* TODO: 3. Poll xSystemEvents for ESTOP_* and USER_* bits. */

        /* TODO: 4. Update state machine. */

        /* TODO: 5. Ramp reference RPM toward desired RPM. */

        /* TODO: 6. PI controller update -> setDuty(). */

        /* TODO: 7. Publish MotorMsgObj to xMotorQueue. */
    }
}

/*-----------------------------------------------------------*/

void vCreateMotorTask(void)
{
    xTaskCreate(prvMotorTask, "Motor",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 4, NULL);
}
