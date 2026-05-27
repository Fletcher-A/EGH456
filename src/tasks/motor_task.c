/*
 * motor_task.c — Motor control task (assignment 2.1).
 *
 * Open-loop duty follows rpm_reference (500 RPM/s ramp). Soft-start in
 * STARTING until reference >= MIN_START_RPM; slow duty slew; power E-stop
 * armed only after soft-start completes (avoids inrush trip / PI surge).
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
#if MOTOR_ENABLE_POWER_SENSOR
#include "drivers/power_sensor.h"
#endif
#include "utils/uart_log.h"
/*-----------------------------------------------------------*/

#define LOOP_PERIOD_MS  10
#define SPEED_TICK_MS   10
/* 500 RPM/s with 10 ms loop -> 5 RPM per tick (assignment 2.1.3). */
#define ACCEL_PER_TICK  ((ACCEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000)
#define DECEL_PER_TICK  ((DECEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000)
#define ESTOP_PER_TICK  ((ESTOP_DECEL_LIMIT_RPMPS * LOOP_PERIOD_MS) / 1000)
#if (ACCEL_PER_TICK * (1000 / LOOP_PERIOD_MS)) != ACCEL_LIMIT_RPMPS
#error ACCEL_PER_TICK must give exactly ACCEL_LIMIT_RPMPS RPM/s
#endif
#define MAX_COMMAND_RPM MAX_MOTOR_RPM
#define ESTOP_BRAKE_FAULT_TICKS  100u
/* After ref=0 and drive disabled, wait for low RPM before IDLE. */
#define STOP_ZERO_DEBOUNCE_TICKS  8u
#define STOP_SPINDOWN_MAX_TICKS   400u
/* Duty slew during STOP: 500 RPM/s -> 12.5 %/s at 4000 RPM rated (milli-% units). */
#define STOP_DUTY_DECEL_MILLIPCT  \
    ((DECEL_PER_TICK * MOTOR_MAX_DUTY_PCT * 1000) / MOTOR_RATED_MAX_RPM)
#define ESTOP_DUTY_DECEL_MILLIPCT \
    ((ESTOP_PER_TICK * MOTOR_MAX_DUTY_PCT * 1000) / MOTOR_RATED_MAX_RPM)
#define STOP_OVERSPEED_RPM_MARGIN 200
#define STOP_OVERSPEED_DUTY_TRIM  500   /* extra milli-%/tick when over ref */
/* Stay in STARTING until ref reaches this; do not hand off at 100 RPM. */
#define SOFTSTART_REF_RPM          MIN_START_RPM
/* Arm power E-stop only after reference has passed this. */
#define POWER_ESTOP_ARM_REF_RPM    (MIN_START_RPM + 400)
/* Slower duty slew while reference is still climbing. */
#define START_DUTY_SLEW_PCT_PER_TICK  1u

static int32_t s_stop_duty_milli = 0;
static uint16_t s_last_pwm_duty  = 0;
static uint8_t  s_run_debounce   = 0;
static uint8_t  s_start_ol_ticks = 0;
static bool prvInputLocked(MotorState_t st)
{
    return (st == MOTOR_STATE_STOPPING ||
            st == MOTOR_STATE_ESTOP_BRAKING ||
            st == MOTOR_STATE_FAULT_LATCHED);
}

static void prvClearCommandQueue(void)
{
    if (xCommandMutex != NULL)
    {
        xSemaphoreTake(xCommandMutex, portMAX_DELAY);
    }
    xQueueReset(xCommandQueue);
    if (xCommandMutex != NULL)
    {
        xSemaphoreGive(xCommandMutex);
    }
}

static void prvMotorDriverOnStateChange(MotorState_t prev, MotorState_t now)
{
    if (now == prev)
    {
        return;
    }
        if (now == MOTOR_STATE_IDLE && prev != MOTOR_STATE_IDLE)
        {
            motor_driver_stop(false);
            speed_sensor_reset_filter();
            /* Keep power ADC zero-cal across STOP — do not reset here. */
        }
    else if (now == MOTOR_STATE_STARTING && prev == MOTOR_STATE_IDLE)
    {
        speed_sensor_reset_filter();
        s_run_debounce = 0;
        s_start_ol_ticks = 0;
        s_last_pwm_duty = 0;
        motor_driver_start();
    }
    else if (now == MOTOR_STATE_FAULT_LATCHED)
    {
        motor_driver_estop();
    }
}

static void prvClearFaultLatch(void)
{
    xEventGroupClearBits(xSystemEvents,
                         EVT_ESTOP_ANY | EVT_USER_START | EVT_USER_STOP);
    g_motor_estop_armed = false;
}

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
        prvClearCommandQueue();
    }
}

/* PI must not see open-loop hall spikes (4000) while slider/ref are 560. */
static int32_t prvRpmForControl(int32_t rpm_reference, int32_t rpm_measured)
{
    int32_t cap = rpm_reference + (int32_t)MOTOR_RUN_MAX_OVERSPEED_RPM;
    if (rpm_measured > cap)
    {
        return cap;
    }
    if (rpm_measured < 0)
    {
        return 0;
    }
    return rpm_measured;
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

static void prvPiResetIntegral(float *pIntegral)
{
    *pIntegral = 0.0f;
}

static void prvPiReset(float *pIntegral)
{
    prvPiResetIntegral(pIntegral);
    s_last_pwm_duty = 0;
}

/*-----------------------------------------------------------*/
/* Advanced feature: ACC supervisor (virtual distance).
 *
 * Slider command remains the "set speed" (rpm_set). When ACC is enabled and
 * the (virtual) distance falls below the minimum gap threshold, reduce the
 * effective command proportionally. This demonstrates ACC behavior without
 * a physical distance sensor.
 */
static int32_t prvAccLimitRpm(int32_t rpm_set)
{
    if (!g_acc_enabled)
    {
        return rpm_set;
    }

    /* Only limit while motor is active. */
    if (!(g_motor_state == MOTOR_STATE_STARTING ||
          g_motor_state == MOTOR_STATE_RUNNING))
    {
        return rpm_set;
    }

    float gap_mm = g_virtual_distance_mm;
    float min_mm = g_thresh_distance_mm;
    if (min_mm < 1.0f) min_mm = 1.0f;

    if (gap_mm >= min_mm)
    {
        return rpm_set;
    }

    /* Linear scale down: gap==0 -> 0 RPM, gap==min -> set RPM. */
    float scale = gap_mm / min_mm;
    if (scale < 0.0f) scale = 0.0f;
    if (scale > 1.0f) scale = 1.0f;

    int32_t limited = (int32_t)((float)rpm_set * scale + 0.5f);
    if (limited < 0) limited = 0;
    if (limited > MAX_COMMAND_RPM) limited = MAX_COMMAND_RPM;
    return limited;
}

/* Limit duty step per tick (slower while reference is still ramping up). */
static uint16_t prvSlewDuty(uint16_t target, int32_t rpm_reference)
{
    uint16_t step = MOTOR_DUTY_SLEW_MAX_PCT_PER_TICK;

    if (rpm_reference < POWER_ESTOP_ARM_REF_RPM)
    {
        step = START_DUTY_SLEW_PCT_PER_TICK;
    }

    int32_t delta = (int32_t)target - (int32_t)s_last_pwm_duty;

    if (delta > (int32_t)step)
    {
        target = (uint16_t)(s_last_pwm_duty + step);
    }
    else if (delta < -(int32_t)step)
    {
        target = (s_last_pwm_duty > step) ?
                 (uint16_t)(s_last_pwm_duty - step) : 0;
    }

    s_last_pwm_duty = target;
    return target;
}

static uint16_t prvDutyFromReference(int32_t rpm_reference)
{
    uint16_t duty;

    if (rpm_reference <= 0)
    {
        return 0;
    }

    duty = motor_driver_feedforward_duty_pct(rpm_reference);
    if (duty < 2u)
    {
        duty = 2u;
    }
    return prvSlewDuty(duty, rpm_reference);
}

/* STOP: ramp duty down at 500 RPM/s equivalent — do not use FF(ref) for PWM
 * (FF stays high while ref is still thousands, which caused 50% duty for seconds). */
static uint16_t prvStopDutyRamp(bool estop, int32_t rpm_reference,
                                int32_t rpm_actual)
{
    int32_t decel = estop ? ESTOP_DUTY_DECEL_MILLIPCT : STOP_DUTY_DECEL_MILLIPCT;

    if (rpm_actual > rpm_reference + STOP_OVERSPEED_RPM_MARGIN)
    {
        decel += STOP_OVERSPEED_DUTY_TRIM;
    }

    s_stop_duty_milli -= decel;
    if (s_stop_duty_milli < 0)
    {
        s_stop_duty_milli = 0;
    }

    return (uint16_t)(s_stop_duty_milli / 1000);
}

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
    float        pi_integral   = 0.0f;
    uint32_t     seq           = 0;
    EventBits_t  fault_bits    = 0;
    uint16_t     brake_ticks   = 0;
    uint16_t     stop_coast_ticks = 0;
    bool         motor_csv_hdr = false;

    TickType_t xLastWake = xTaskGetTickCount();

#if !SERIAL_PLOT_CLEAN
    uart_log_printf("Motor task: PI closed-loop, ramps 500/1000 RPM/s\n");
    uart_log_printf("Power ADC uses Timer3 (Timer0 reserved for MotorLib)\n");
#endif

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(LOOP_PERIOD_MS));

        bool locked = prvInputLocked(state);

        rpm_actual = motor_driver_get_rpm();
        if (rpm_actual < 0)
        {
            rpm_actual = 0;
        }
        if (rpm_actual > MAX_COMMAND_RPM)
        {
            rpm_actual = MAX_COMMAND_RPM;
        }

        EventBits_t evt = xEventGroupWaitBits(
            xSystemEvents,
            EVT_USER_START | EVT_USER_STOP | EVT_USER_ESTOP_ACK |
            EVT_ESTOP_ANY,
            pdTRUE,
            pdFALSE,
            0);

        if (!locked && (evt & EVT_USER_STOP))
        {
            rpm_desired = 0;
            if (state == MOTOR_STATE_STARTING || state == MOTOR_STATE_RUNNING)
            {
                prvPiReset(&pi_integral);
                /* Start decel from present PWM only (never FF step-up). */
                s_stop_duty_milli =
                    (int32_t)motor_driver_get_duty_percent() * 1000;
                /* If halls read below ref, align ref down; ignore high spikes. */
                if (rpm_actual < rpm_reference &&
                    rpm_actual <= (int32_t)MOTOR_RATED_MAX_RPM)
                {
                    rpm_reference = rpm_actual;
                }
                stop_coast_ticks = 0;
                state = MOTOR_STATE_STOPPING;
            }
            else if (state == MOTOR_STATE_IDLE)
            {
                rpm_reference = 0;
                pwm_duty = 0;
            }
            /* Avoid a queued START firing right after STOP. */
            xEventGroupClearBits(xSystemEvents, EVT_USER_START);
            prvClearCommandQueue();
        }

        /* --- 1. Commands (ignored during E-stop / fault latch) ---- */
        if (!locked)
        {
            int32_t newRpm;
            if (xCommandMutex != NULL)
            {
                xSemaphoreTake(xCommandMutex, portMAX_DELAY);
            }
            while (xQueueReceive(xCommandQueue, &newRpm, 0) == pdPASS)
            {
                rpm_desired = prvClampRpmCommand(newRpm);
            }
            if (xCommandMutex != NULL)
            {
                xSemaphoreGive(xCommandMutex);
            }
        }

        /* ACC supervisor: transform slider set-speed -> effective desired RPM. */
        int32_t rpm_desired_eff = prvAccLimitRpm(rpm_desired);

        if (evt & EVT_USER_ESTOP_ACK)
        {
            if (state == MOTOR_STATE_ESTOP_BRAKING ||
                state == MOTOR_STATE_FAULT_LATCHED)
            {
                prvClearFaultLatch();
                fault_bits = 0;
                brake_ticks = 0;
                rpm_desired = 0;
                rpm_reference = 0;
                pwm_duty = 0;
                prvPiReset(&pi_integral);
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
            if (state == MOTOR_STATE_STARTING ||
                state == MOTOR_STATE_RUNNING ||
                state == MOTOR_STATE_STOPPING)
            {
                brake_ticks = 0;
                s_stop_duty_milli =
                    (int32_t)motor_driver_get_duty_percent() * 1000;
                state = MOTOR_STATE_ESTOP_BRAKING;
                prvClearCommandQueue();
            }
        }

        bool hall_a, hall_b, hall_c;
        speed_sensor_read_halls(&hall_a, &hall_b, &hall_c);
        uint8_t hall_state = (hall_a ? 4u : 0u) |
                             (hall_b ? 2u : 0u) |
                             (hall_c ? 1u : 0u);

        prvHandleHardwareDriverFault(&state, &fault_bits, &brake_ticks);

        if (state == MOTOR_STATE_STOPPING)
        {
            rpm_desired = 0;
        }

        /* --- 2. Reference ramp (before state checks use updated ref) */
        {
            int32_t target;
            int32_t step;
            switch (state)
            {
            case MOTOR_STATE_RUNNING:
            case MOTOR_STATE_STARTING:
                /* Ramp reference at 500 RPM/s toward slider setpoint (spec 2.1.3).
                 * Do not park at MIN_START_RPM only — that paused the ramp until
                 * Running and made acceleration look wrong on the plot. */
                target = rpm_desired_eff;
                if (target < MIN_START_RPM)
                {
                    target = MIN_START_RPM;
                }
                step = ACCEL_PER_TICK;
                break;
            case MOTOR_STATE_STOPPING:
                target = 0;
                step   = DECEL_PER_TICK;
                break;
            case MOTOR_STATE_ESTOP_BRAKING:
                target = 0;
                step   = ESTOP_PER_TICK;
                break;
            default:
                target = 0;
                step   = DECEL_PER_TICK;
                break;
            }
            if (rpm_reference < target)
            {
                rpm_reference += step;
                if (rpm_reference > target)
                {
                    rpm_reference = target;
                }
            }
            else if (rpm_reference > target)
            {
                rpm_reference -= step;
                if (rpm_reference < target)
                {
                    rpm_reference = target;
                }
            }
        }

        MotorState_t prev_state = state;

        /* --- 3. State machine ------------------------------------- */
        switch (state)
        {
        case MOTOR_STATE_IDLE:
            g_motor_estop_armed = false;
            g_motor_power_estop_ok = false;
            brake_ticks = 0;
            if (!locked && (evt & EVT_USER_START))
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
                    prvPiReset(&pi_integral);
                    rpm_reference = 0;
                    s_run_debounce = 0;
                    s_start_ol_ticks = 0;
                    state = MOTOR_STATE_STARTING;
                }
                else if (motor_driver_hardware_fault_active())
                {
                    fault_bits |= EVT_ESTOP_DRIVER;
                }
                else if (!motor_driver_is_ready())
                {
                    uart_log_printf("START ignored: MotorLib not ready\n");
                }
            }
            break;

        case MOTOR_STATE_STARTING:
            g_motor_estop_armed = false;
            g_motor_power_estop_ok = false;
            {
                int32_t rpm_f = speed_sensor_get_rpm();
                int32_t rpm_hi = rpm_reference + (int32_t)MOTOR_RUN_MAX_OVERSPEED_RPM;
                if (rpm_hi < (int32_t)MOTOR_START_HALL_RPM_CAP)
                {
                    rpm_hi = (int32_t)MOTOR_START_HALL_RPM_CAP;
                }
                /* Enter RUNNING only after soft-start ref + plausible speed. */
                if (rpm_reference >= SOFTSTART_REF_RPM &&
                    rpm_f >= MOTOR_RUN_ENTER_RPM && rpm_f <= rpm_hi)
                {
                    if (s_run_debounce < 255u)
                    {
                        s_run_debounce++;
                    }
                }
                else
                {
                    s_run_debounce = 0;
                }
            }
            if (s_run_debounce >= MOTOR_RUN_DEBOUNCE_TICKS)
            {
                int32_t rpm_handoff = speed_sensor_get_rpm();
                if (rpm_handoff < 0)
                {
                    rpm_handoff = 0;
                }
                if (rpm_handoff > rpm_reference + (int32_t)MOTOR_RUN_MAX_OVERSPEED_RPM)
                {
                    rpm_handoff = rpm_reference;
                }
                speed_sensor_seed_filter(rpm_handoff);
                state = MOTOR_STATE_RUNNING;
            }
            break;

        case MOTOR_STATE_RUNNING:
            if (rpm_reference >= POWER_ESTOP_ARM_REF_RPM)
            {
                g_motor_power_estop_ok = true;
                g_motor_estop_armed = true;
            }
            else
            {
                g_motor_power_estop_ok = false;
                g_motor_estop_armed = false;
            }
            break;

        case MOTOR_STATE_STOPPING:
            g_motor_estop_armed = false;
            g_motor_power_estop_ok = false;
            if (rpm_reference <= 0)
            {
                stop_coast_ticks++;
                if ((rpm_actual <= MOTOR_HALL_RUN_RPM &&
                     stop_coast_ticks >= STOP_ZERO_DEBOUNCE_TICKS) ||
                    stop_coast_ticks >= STOP_SPINDOWN_MAX_TICKS)
                {
                    state = MOTOR_STATE_IDLE;
                    stop_coast_ticks = 0;
                }
            }
            else
            {
                stop_coast_ticks = 0;
            }
            break;

        case MOTOR_STATE_ESTOP_BRAKING:
            g_motor_estop_armed = false;
            g_motor_power_estop_ok = false;
            brake_ticks++;
            if (rpm_reference <= 0 &&
                (rpm_actual <= MOTOR_HALL_RUN_RPM ||
                 brake_ticks >= ESTOP_BRAKE_FAULT_TICKS))
            {
                state = MOTOR_STATE_FAULT_LATCHED;
            }
            break;

        case MOTOR_STATE_FAULT_LATCHED:
            g_motor_estop_armed = false;
            g_motor_power_estop_ok = false;
            break;
        }

        prvMotorDriverOnStateChange(prev_state, state);

        /* --- 4. Actuation ----------------------------------------- */
        if (state == MOTOR_STATE_IDLE || state == MOTOR_STATE_FAULT_LATCHED)
        {
            motor_driver_set_duty_percent(0);
            pwm_duty = 0;
            prvPiReset(&pi_integral);
        }
        else if (state == MOTOR_STATE_STOPPING ||
                 state == MOTOR_STATE_ESTOP_BRAKING)
        {
            pwm_duty = prvStopDutyRamp(
                state == MOTOR_STATE_ESTOP_BRAKING,
                rpm_reference, rpm_actual);

            /* Duty 0 with drive still enabled made hall ISRs commutate and
             * show fake 6000 RPM + physical surge (photo: Act 6066, Duty 0). */
            if (pwm_duty == 0 || rpm_reference <= 0)
            {
                pwm_duty = 0;
                motor_driver_disable_drive();
                if (rpm_reference > 0)
                {
                    rpm_reference = 0;
                }
            }
            else
            {
                motor_driver_set_duty_percent(pwm_duty);
            }
            prvPiReset(&pi_integral);
        }
        else if (state == MOTOR_STATE_STARTING ||
                 state == MOTOR_STATE_RUNNING)
        {
            /* Open-loop: PWM follows ramped reference only (no PI surge/cutback). */
            prvPiResetIntegral(&pi_integral);
            pwm_duty = prvDutyFromReference(rpm_reference);
            motor_driver_set_duty_percent(pwm_duty);
        }

        float power_w = g_motor_power_watts;
        g_motor_state = state;
        g_motor_pwm_duty_pct = pwm_duty;

        /* GUI: live hall RPM (only cap startup spikes, not steady-state low). */
        rpm_actual = speed_sensor_get_rpm_display();
        if (rpm_actual < 0)
        {
            rpm_actual = 0;
        }
        if (state == MOTOR_STATE_STARTING &&
            rpm_actual > (int32_t)MOTOR_START_HALL_RPM_CAP)
        {
            rpm_actual = (int32_t)MOTOR_START_HALL_RPM_CAP;
        }
        else if (state == MOTOR_STATE_RUNNING &&
                 rpm_actual > rpm_reference + (int32_t)MOTOR_RUN_MAX_OVERSPEED_RPM)
        {
            rpm_actual = rpm_reference + (int32_t)MOTOR_RUN_MAX_OVERSPEED_RPM;
        }
        if (rpm_actual > MAX_COMMAND_RPM)
        {
            rpm_actual = MAX_COMMAND_RPM;
        }

        g_plot_rpm_desired   = rpm_desired_eff;
        g_plot_rpm_reference = rpm_reference;
        g_plot_rpm_actual    = rpm_actual;

        MotorMsgObj msg;
        msg.seq           = ++seq;
        msg.tick          = xTaskGetTickCount();
        msg.rpm_actual    = rpm_actual;
        msg.rpm_reference = rpm_reference;
        msg.rpm_desired   = rpm_desired_eff;
        msg.pwm_duty      = pwm_duty;
        msg.power_watts   = power_w;
        msg.state         = state;
        msg.fault_bits    = fault_bits;
        msg.hall_state    = hall_state;
        msg.motor_ready   = motor_driver_is_ready();
        xQueueSend(xMotorQueue, &msg, pdMS_TO_TICKS(1));

#if !SERIAL_PLOT_CLEAN
        /* Legacy stream: M-prefix + 15-col sensor CSV (Tera Term / Serial Plot desktop). */
        if (!motor_csv_hdr)
        {
            uart_log_printf("# motor: M,desired,reference,actual,duty_pct\n");
            motor_csv_hdr = true;
        }
        uart_log_printf("M,%d,%d,%d,%u\n",
                        (int)rpm_desired_eff,
                        (int)rpm_reference,
                        (int)rpm_actual,
                        (unsigned)pwm_duty);
#endif
    }
}

void vCreateMotorTask(void)
{
    xTaskCreate(prvMotorTask, "Motor",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(prvSpeedTask, "Speed",
                configMINIMAL_STACK_SIZE * 2, NULL,
                tskIDLE_PRIORITY + 5, NULL);
}
