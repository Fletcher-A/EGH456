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
 *       EVT_USER_STOP          set by GUI STOP button      -> Running/Starting -> Stopping (500 RPM/s)
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
 * CONTROL NOTES
 * -------------
 *   - State machine: Idle / Starting / Running / Stopping / E-Stop Braking /
 *                    Fault Latched (assignment 2.1.1 spec).
 *   - Reference RPM ramps to desired at 500 RPM/s (1000 in E-Stop).
 *   - Actual RPM comes from hall-sensor feedback.
 *   - Duty is adjusted from reference RPM plus measured speed error.
 *   - Power is still estimated until the DRV8323 ADC power pipeline is enabled.
 */

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "shared.h"
#include "drivers/motor_driver.h"
#include "drivers/speed_sensor.h"
#include "utils/uart_log.h"
/*-----------------------------------------------------------*/

static void prvMotorDriverOnStateChange(MotorState_t prev, MotorState_t now)
{
    if (now == prev)
    {
        return;
    }
    if (now == MOTOR_STATE_IDLE)
    {
        motor_driver_stop(false);
    }
    else if (now == MOTOR_STATE_STARTING && prev == MOTOR_STATE_IDLE)
    {
        motor_driver_start();
    }
    else if (now == MOTOR_STATE_ESTOP_BRAKING ||
             now == MOTOR_STATE_FAULT_LATCHED)
    {
        motor_driver_estop();
    }
}

#define LOOP_PERIOD_MS  10          /* 100 Hz control; assignment requires 1-10 ms */
#define SPEED_TICK_MS   10          /* 100 Hz hall-edge-count -> RPM */
#define ACCEL_PER_TICK  ((ACCEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000)   /* 25 RPM */
#define DECEL_PER_TICK  ((DECEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000)   /* 25 RPM */
#define ESTOP_PER_TICK  ((ESTOP_DECEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000) /* 50 RPM */
#define MAX_COMMAND_RPM MAX_MOTOR_RPM
/* Force Fault Latched if braking does not report 0 RPM (hall noise). */
#define ESTOP_BRAKE_FAULT_TICKS  100u   /* 1 s at 10 ms loop */

static void prvClearFaultLatch(void)
{
    xEventGroupClearBits(xSystemEvents,
                         EVT_ESTOP_ANY | EVT_USER_START | EVT_USER_STOP);
    g_motor_estop_armed = false;
}

/* DRV8323 nFAULT (red LED on green motor board) is not wired into xSystemEvents. */
static void prvHandleHardwareDriverFault(MotorState_t *pState,
                                         EventBits_t *pFaultBits,
                                         uint16_t *pBrakeTicks)
{
    if (!motor_driver_hardware_fault_active())
    {
        *pFaultBits &= ~EVT_ESTOP_DRIVER;
        return;
    }

    *pFaultBits |= EVT_ESTOP_DRIVER;
    g_motor_estop_armed = false;

    if (*pState == MOTOR_STATE_STARTING || *pState == MOTOR_STATE_RUNNING ||
        *pState == MOTOR_STATE_STOPPING)
    {
        *pBrakeTicks = 0;
        *pState = MOTOR_STATE_ESTOP_BRAKING;
    }
    /* Idle / Fault Latched / E-Stop Braking: do not re-latch here; ACK and
     * braking logic own those transitions. START is blocked while nFAULT low. */
}

static int32_t prvClampRpmCommand(int32_t rpm)
{
    if (rpm < 0)
    {
        return 0;
    }
    if (rpm > MAX_COMMAND_RPM)
    {
        return MAX_COMMAND_RPM;
    }
    return rpm;
}

/* Dedicated 100 Hz speed task — converts accumulated hall-edge counts
 * into RPM at the rate the spec mandates. Kept separate from
 * motor_task so the speed estimate updates faster than the 20 Hz
 * control loop. */
static void prvSpeedTask(void *pvParameters)
{
    (void)pvParameters;
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
    EventBits_t  fault_bits    = 0;
    uint16_t     brake_ticks   = 0;

    TickType_t   xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));

        /* --- 1. Read commands ------------------------------------- */
        int32_t newRpm;
        if (xCommandMutex != NULL)
        {
            xSemaphoreTake(xCommandMutex, portMAX_DELAY);
        }
        while (xQueueReceive(xCommandQueue, &newRpm, 0) == pdPASS)
        {
            /* GUI can change the desired RPM at any time. Drain the queue
             * so the motor always follows the most recent slider/start value;
             * state machine decides if/when it's actually applied. */
            rpm_desired = prvClampRpmCommand(newRpm);
            /* rpm_reference ramps toward rpm_desired at ACCEL_LIMIT_RPMPS. */
        }
        if (xCommandMutex != NULL)
        {
            xSemaphoreGive(xCommandMutex);
        }

        EventBits_t evt = xEventGroupWaitBits(
            xSystemEvents,
            EVT_USER_START | EVT_USER_STOP | EVT_USER_ESTOP_ACK |
            EVT_ESTOP_ANY,
            pdTRUE,       /* clear on read */
            pdFALSE,      /* any bit */
            0);           /* don't block */
        if (evt & EVT_USER_STOP)
        {
            rpm_desired = 0;
            if (state == MOTOR_STATE_STARTING || state == MOTOR_STATE_RUNNING)
            {
                state = MOTOR_STATE_STOPPING;
            }
            else if (state == MOTOR_STATE_IDLE)
            {
                rpm_reference = 0;
                pwm_duty = 0;
            }
            if (xCommandMutex != NULL)
            {
                xSemaphoreTake(xCommandMutex, portMAX_DELAY);
                xQueueReset(xCommandQueue);
                xSemaphoreGive(xCommandMutex);
            }
        }

        if (evt & EVT_USER_ESTOP_ACK)
        {
            /* ACK clears the latched fault state. Hardware nFAULT may still be
             * active (red LED): user returns to Idle but START stays blocked
             * until the driver fault clears. */
            if (state == MOTOR_STATE_ESTOP_BRAKING ||
                state == MOTOR_STATE_FAULT_LATCHED)
            {
                prvClearFaultLatch();
                fault_bits = 0;
                brake_ticks = 0;
                rpm_desired = 0;
                rpm_reference = 0;
                pwm_duty = 0;
                state = MOTOR_STATE_IDLE;
                motor_driver_estop();
                vTaskDelay(pdMS_TO_TICKS(50));
                (void)motor_driver_try_clear_hardware_fault();
            }
        }

        EventBits_t active_faults = evt & EVT_ESTOP_ANY;
        if (active_faults && !(evt & EVT_USER_ESTOP_ACK))
        {
            fault_bits |= active_faults;
        }
        rpm_actual = motor_driver_get_rpm();
        if (rpm_actual < 0)
        {
            rpm_actual = 0;
        }
        if (rpm_actual > MAX_COMMAND_RPM)
        {
            rpm_actual = MAX_COMMAND_RPM;
        }
        bool hall_a, hall_b, hall_c;
        speed_sensor_read_halls(&hall_a, &hall_b, &hall_c);
        uint8_t hall_state = (hall_a ? 4u : 0u) |
                             (hall_b ? 2u : 0u) |
                             (hall_c ? 1u : 0u);

        prvHandleHardwareDriverFault(&state, &fault_bits, &brake_ticks);

        MotorState_t prev_state = state;

        /* --- 2. State transitions --------------------------------- */
        switch (state)
        {
        case MOTOR_STATE_IDLE:
            g_motor_estop_armed = false;
            brake_ticks = 0;
            if (evt & EVT_USER_START)
            {
                if (motor_driver_hardware_fault_active())
                {
                    (void)motor_driver_try_clear_hardware_fault();
                }
                if (rpm_desired < MIN_START_RPM)
                {
                    rpm_desired = MIN_START_RPM;
                }
#if MOTOR_ENABLE_NFAULT_MONITORING && MOTOR_NFAULT_BLOCKS_START
                if (!motor_driver_hardware_fault_active() &&
                    motor_driver_is_ready())
#else
                if (motor_driver_is_ready())
#endif
                {
                    state = MOTOR_STATE_STARTING;
                }
                else if (motor_driver_hardware_fault_active())
                {
                    fault_bits |= EVT_ESTOP_DRIVER;
                }
            }
            break;

        case MOTOR_STATE_STARTING:
            g_motor_estop_armed = false;
            if (rpm_actual > 100)             state = MOTOR_STATE_RUNNING;
            if (active_faults)
            {
                brake_ticks = 0;
                state = MOTOR_STATE_ESTOP_BRAKING;
            }
            break;

        case MOTOR_STATE_RUNNING:
            g_motor_estop_armed = true;
            if (active_faults)
            {
                brake_ticks = 0;
                state = MOTOR_STATE_ESTOP_BRAKING;
            }
            break;

        case MOTOR_STATE_STOPPING:
            g_motor_estop_armed = false;
            if (rpm_reference <= 0)
            {
                state = MOTOR_STATE_IDLE;
            }
            if (active_faults)
            {
                brake_ticks = 0;
                state = MOTOR_STATE_ESTOP_BRAKING;
            }
            break;

        case MOTOR_STATE_ESTOP_BRAKING:
            g_motor_estop_armed = false;
            brake_ticks++;
            if (rpm_actual <= 0 ||
                brake_ticks >= ESTOP_BRAKE_FAULT_TICKS)
            {
                state = MOTOR_STATE_FAULT_LATCHED;
            }
            break;

        case MOTOR_STATE_FAULT_LATCHED:
            g_motor_estop_armed = false;
            break;
        }

        prvMotorDriverOnStateChange(prev_state, state);

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
        case MOTOR_STATE_STOPPING:
            target = 0;
            step   = DECEL_PER_TICK;
            break;
        case MOTOR_STATE_ESTOP_BRAKING:
            target = 0;
            step   = ESTOP_PER_TICK;
            break;
        default:        /* Idle / Fault */
            target = 0;
            step   = DECEL_PER_TICK;
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

        /* --- 4. Open-loop duty follows ramped reference. ----- */
        if (state == MOTOR_STATE_IDLE ||
            state == MOTOR_STATE_FAULT_LATCHED ||
            state == MOTOR_STATE_ESTOP_BRAKING)
        {
            motor_driver_set_speed_rpm(0);
            pwm_duty = 0;
        }
        else
        {
            motor_driver_set_speed_rpm(rpm_reference);
            pwm_duty = motor_driver_get_duty_percent();
        }

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
        msg.fault_bits    = fault_bits;
        msg.hall_state    = hall_state;
        msg.motor_ready   = motor_driver_is_ready();
        xQueueSend(xMotorQueue, &msg, pdMS_TO_TICKS(1));

        if ((seq % 100u) == 0u)
        {
            uart_log_printf("MOTOR st=%d des=%d ref=%d rpm=%d duty=%u hall=%u%u%u ready=%u fault=0x%x\n",
                            (int)state,
                            (int)rpm_desired,
                            (int)rpm_reference,
                            (int)rpm_actual,
                            (unsigned)pwm_duty,
                            hall_a ? 1u : 0u,
                            hall_b ? 1u : 0u,
                            hall_c ? 1u : 0u,
                            motor_driver_is_ready() ? 1u : 0u,
                            (unsigned)fault_bits);
        }
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
    /* 100 Hz hall edge -> RPM filter (requires DRV8323 + motor connected). */
    xTaskCreate(prvSpeedTask, "Speed",
                configMINIMAL_STACK_SIZE * 2, NULL,
                tskIDLE_PRIORITY + 5, NULL);
}
