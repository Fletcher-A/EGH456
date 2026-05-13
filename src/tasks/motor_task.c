/*
 * motor_task.c — Motor control / simulation task.
 *
 * INTER-TASK INTERFACE
 * --------------------
 *
 * INPUTS (consumed by this task):
 *   xCommandQueue (int32_t)
 *       Desired RPM, written by the GUI task when the speed slider
 *       is moved (OnSpeedSliderChange in gui_task.c). Drained every
 *       50 ms control tick.
 *   xSystemEvents bits:
 *       EVT_USER_START         set by GUI START button     -> Idle  -> Starting
 *       EVT_USER_STOP          set by GUI STOP button      -> any   -> Idle
 *       EVT_USER_ESTOP_ACK     set by GUI ACK button       -> Fault -> Idle
 *       EVT_ESTOP_POWER        set by sensor task          -> any   -> E-Stop Braking
 *       EVT_ESTOP_ACCEL        set by sensor task          -> any   -> E-Stop Braking
 *       EVT_ESTOP_DISTANCE     set by sensor task          -> any   -> E-Stop Braking
 *
 * OUTPUTS (produced by this task):
 *   xMotorQueue (MotorMsgObj) at 20 Hz, read by:
 *       gui_task.c (state text, RPM/power readouts, plot traces)
 *       fault_task.c (could check state to log fault entries)
 *
 * PUBLIC FUNCTIONS (called from elsewhere):
 *   vCreateMotorTask()   — called once from main.c during startup
 *
 * SIMULATION NOTES
 * ----------------
 *   - State machine: Idle / Starting / Running / E-Stop Braking /
 *                    Fault Latched (assignment 2.1.1 spec).
 *   - Reference RPM ramps to desired at 500 RPM/s (1000 in E-Stop).
 *   - Actual RPM lags the reference (first-order, simulating a PI).
 *   - Power = RPM * 0.04 with a low-pass filter.
 *   When the motor team wires real MotorLib + hall sensors + ADC,
 *   the simulation block (steps 3-6) gets replaced.
 */

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "shared.h"
#include "drivers/speed_sensor.h"
/*-----------------------------------------------------------*/

#define LOOP_PERIOD_MS  50          /* 20 Hz control / publish */
#define SPEED_TICK_MS   10          /* 100 Hz hall-edge-count -> RPM */
#define ACCEL_PER_TICK  ((ACCEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000)   /* 25 RPM */
#define ESTOP_PER_TICK  ((ESTOP_DECEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000) /* 50 RPM */

/* Dedicated 100 Hz speed task — converts accumulated hall-edge counts
 * into RPM at the rate the spec mandates. Kept separate from
 * motor_task so the speed estimate updates faster than the 20 Hz
 * control loop. */
static void prvSpeedTask(void *pvParameters)
{
    (void)pvParameters;
    speed_sensor_init();
    TickType_t xLastWake = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SPEED_TICK_MS));
        speed_sensor_tick(SPEED_TICK_MS);
    }
}

static void prvMotorTask(void *pvParameters)
{
    (void)pvParameters;

    MotorState_t state         = MOTOR_STATE_IDLE;
    int32_t      rpm_desired   = 0;
    int32_t      rpm_reference = 0;
    int32_t      rpm_actual    = 0;
    uint16_t     pwm_duty      = 0;
    float        power_w       = 0.0f;
    uint32_t     seq           = 0;

    TickType_t   xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));

        /* --- 1. Read commands ------------------------------------- */
        int32_t newRpm;
        if (xQueueReceive(xCommandQueue, &newRpm, 0) == pdPASS)
        {
            /* GUI can change the desired RPM at any time, but the
             * state machine decides if/when it's actually applied. */
            rpm_desired = newRpm;
        }

        EventBits_t evt = xEventGroupWaitBits(
            xSystemEvents,
            EVT_USER_START | EVT_USER_STOP | EVT_USER_ESTOP_ACK |
            EVT_ESTOP_ANY,
            pdTRUE,       /* clear on read */
            pdFALSE,      /* any bit */
            0);           /* don't block */

        /* --- 2. State transitions --------------------------------- */
        switch (state)
        {
        case MOTOR_STATE_IDLE:
            if (evt & EVT_USER_START)        state = MOTOR_STATE_STARTING;
            break;

        case MOTOR_STATE_STARTING:
            if (rpm_actual > 100)             state = MOTOR_STATE_RUNNING;
            if (evt & EVT_USER_STOP)          state = MOTOR_STATE_IDLE;
            if (evt & EVT_ESTOP_ANY)          state = MOTOR_STATE_ESTOP_BRAKING;
            break;

        case MOTOR_STATE_RUNNING:
            if (evt & EVT_USER_STOP)          state = MOTOR_STATE_IDLE;
            if (evt & EVT_ESTOP_ANY)          state = MOTOR_STATE_ESTOP_BRAKING;
            break;

        case MOTOR_STATE_ESTOP_BRAKING:
            if (rpm_actual == 0)              state = MOTOR_STATE_FAULT_LATCHED;
            break;

        case MOTOR_STATE_FAULT_LATCHED:
            if (evt & EVT_USER_ESTOP_ACK)     state = MOTOR_STATE_IDLE;
            break;
        }

        /* --- 3. Ramp reference toward target ---------------------- */
        int32_t target;
        int32_t step;
        switch (state)
        {
        case MOTOR_STATE_RUNNING:
        case MOTOR_STATE_STARTING:
            target = rpm_desired;
            step   = ACCEL_PER_TICK;
            break;
        case MOTOR_STATE_ESTOP_BRAKING:
            target = 0;
            step   = ESTOP_PER_TICK;
            break;
        default:        /* Idle / Fault */
            target = 0;
            step   = ACCEL_PER_TICK;
            break;
        }
        if (rpm_reference < target)
        {
            rpm_reference += step;
            if (rpm_reference > target) rpm_reference = target;
        }
        else if (rpm_reference > target)
        {
            rpm_reference -= step;
            if (rpm_reference < target) rpm_reference = target;
        }

        /* --- 4. "PI" — first-order lag on the reference ----------- */
        rpm_actual += (rpm_reference - rpm_actual) / 8;

        /* --- 5. Fake duty from actual RPM (0..4000 -> 0..100 %) --- */
        pwm_duty = (uint16_t)((rpm_actual * 100) / 4000);

        /* --- 6. Fake power from RPM, low-pass filtered ----------- *
         * Real version will use V*I_total from the DRV8323 ADC.    */
        float power_inst = rpm_actual * 0.04f;
        power_w = power_w + 0.2f * (power_inst - power_w);

        /* --- 7. Publish state to the GUI -------------------------- */
        MotorMsgObj msg;
        msg.seq           = ++seq;
        msg.tick          = xTaskGetTickCount();
        msg.rpm_actual    = rpm_actual;
        msg.rpm_reference = rpm_reference;
        msg.rpm_desired   = rpm_desired;
        msg.pwm_duty      = pwm_duty;
        msg.power_watts   = power_w;
        msg.state         = state;
        xQueueSend(xMotorQueue, &msg, 0);
    }
}

/*-----------------------------------------------------------*/

void vCreateMotorTask(void)
{
    /* Motor control is the highest-priority application task — when
     * the real PI loop is added it must run on a strict period. */
    xTaskCreate(prvMotorTask, "Motor",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 4, NULL);
    /* Speed task — DISABLED until the hall sensors are wired. Calling
     * speed_sensor_init() configures GPIO interrupts on PM3/PH2/PN2,
     * which will fire spuriously if the hall lines float. Re-enable
     * once the motor BoosterPack is plugged in. */
    // xTaskCreate(prvSpeedTask, "Speed",
    //             configMINIMAL_STACK_SIZE * 2, NULL,
    //             tskIDLE_PRIORITY + 5, NULL);
}
