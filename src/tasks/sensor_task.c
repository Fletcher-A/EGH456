/*
 * sensor_task.c — Sensing task (assignment 2.2).
 *
 * INTER-TASK INTERFACE
 * --------------------
 *
 * INPUTS (consumed by this task):
 *   (none from other tasks — sensor task is a pure producer.)
 *   Hardware inputs:
 *     OPT3001 (ambient light)        on I2C0 @ 0x44 / 0x45
 *     BMI160  (accelerometer)        on I2C0 @ 0x68 / 0x69
 *     BME280  (temp / hum / press)   on I2C0 @ 0x76 / 0x77
 *
 * OUTPUTS (produced by this task):
 *   xSensorQueue (SensorMsgObj) at 5 Hz, read by:
 *       gui_task.c (lux + day/night LED + plot trace,
 *                   X/Y/Z accel + temp/hum/press readouts)
 *   xSystemEvents bits set/cleared by this task:
 *       EVT_NIGHT_DETECTED  set when filtered lux < NIGHT_LIGHT_LUX
 *                           cleared otherwise. Read by gui_task.c.
 *       EVT_ESTOP_ACCEL     set when peak |ax|+|ay|+|az| exceeds limit (latched).
 *                           Read by motor_task.c (E-Stop braking).
 *
 * PUBLIC FUNCTIONS:
 *   vCreateSensorTask()
 *
 * HARDWARE DRIVERS USED:
 *   initI2C / writeI2C / writeI2C1 / readI2C  (drivers/i2cOptDriver.c)
 *   sensorOpt3001Init / Read / Convert        (drivers/opt3001.c)
 *   bmi160_init / bmi160_read_accel_g         (drivers/bmi160.c)
 *   bme280_init / bme280_read                 (drivers/bme280.c)
 *   I2C0MasterIntHandler (ISR, wired in startup_gcc.c)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "FreeRTOSConfig.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driver_lib/timer.h"
#include "driver_lib/sysctl.h"
#include "driver_lib/interrupt.h"

#include "drivers/i2cOptDriver.h"
#include "drivers/opt3001.h"
#include "drivers/bmi160.h"
#include "drivers/bme280.h"
#include "drivers/sht31.h"
#include "drivers/power_sensor.h"
#include "utils/uartstdio.h"
#include "utils/uart_log.h"

#include "shared.h"
/*-----------------------------------------------------------*/

extern uint32_t g_ui32SysClock;

#define SAMPLE_PERIOD_MS     20      /* 50 Hz outer loop (serial plot rate) */
#define GUI_PUB_EVERY_N      10      /* publish to GUI every 10th tick = 5 Hz */
#define LUX_READ_EVERY_N     10      /* read OPT3001 every 200 ms (5 Hz)   */
#define BME_READ_EVERY_N     50      /* read BME280  every 1 s   (1 Hz)   */
#define MAF_WINDOW           8
#define ACCEL_MAF_WINDOW     8       /* spec 2.2.2: filtered |a| for GUI/report */

#define ACCEL_SAMPLE_HZ     200      /* spec: >= 100 Hz */

#include "drivers/speed_sensor.h"    /* for serial-plot RPM raw + filt */

/* Impact threshold on filtered |ax|+|ay|+|az| is now a runtime global
 * (g_thresh_accel_g), editable from the GUI Thresholds tab. */

/*-----------------------------------------------------------*/

static float    g_fLuxBuf[MAF_WINDOW] = {0};
static uint8_t  g_ui8LuxIdx = 0;
static bool     g_bLuxFull  = false;

/* No accel filter: impacts are sharp transients that any moving-
 * average smears out. The raw |ax|+|ay|+|az| feeds the E-Stop check
 * and the published value directly. */

/* Power filter — 32-sample MAF. At 1 kHz sampling that's a ~32 ms
 * window, fast enough to catch an overcurrent and slow enough to
 * reject PWM switching noise. Buffer holds total motor current (A). */
#define POW_MAF_WINDOW   32
#define POWER_ESTOP_DEBOUNCE_TICKS  15u  /* 15 x 20 ms: ignore inrush transients */
static float    g_fPowBuf[POW_MAF_WINDOW] = {0};
static uint8_t  g_ui8PowIdx = 0;
static bool     g_bPowFull  = false;
static uint8_t  s_pwr_estop_cnt = 0;

static void prvPowMAFReset(void)
{
    for (uint8_t i = 0; i < POW_MAF_WINDOW; i++)
    {
        g_fPowBuf[i] = 0.0f;
    }
    g_ui8PowIdx = 0;
    g_bPowFull  = false;
}

static float prvPowMAFUpdate(float newVal)
{
    g_fPowBuf[g_ui8PowIdx] = newVal;
    g_ui8PowIdx = (g_ui8PowIdx + 1) % POW_MAF_WINDOW;
    if (g_ui8PowIdx == 0) g_bPowFull = true;
    uint8_t count = g_bPowFull ? POW_MAF_WINDOW : g_ui8PowIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) sum += g_fPowBuf[i];
    return sum / count;
}

static float    g_fAccelBuf[ACCEL_MAF_WINDOW] = {0};
static uint8_t  g_ui8AccelIdx = 0;
static bool     g_bAccelFull  = false;

static float prvAccelMAFUpdate(float newVal)
{
    g_fAccelBuf[g_ui8AccelIdx] = newVal;
    g_ui8AccelIdx = (g_ui8AccelIdx + 1) % ACCEL_MAF_WINDOW;
    if (g_ui8AccelIdx == 0)
    {
        g_bAccelFull = true;
    }
    uint8_t count = g_bAccelFull ? ACCEL_MAF_WINDOW : g_ui8AccelIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++)
    {
        sum += g_fAccelBuf[i];
    }
    return sum / count;
}

static float prvLuxMAFUpdate(float newVal)
{
    g_fLuxBuf[g_ui8LuxIdx] = newVal;
    g_ui8LuxIdx = (g_ui8LuxIdx + 1) % MAF_WINDOW;
    if (g_ui8LuxIdx == 0) g_bLuxFull = true;
    uint8_t count = g_bLuxFull ? MAF_WINDOW : g_ui8LuxIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) sum += g_fLuxBuf[i];
    return sum / count;
}

static float prvAbsF(float v) { return v < 0 ? -v : v; }

/*-----------------------------------------------------------*/
/* Accelerometer sampling pipeline:
 *   Timer1A ISR @ ACCEL_SAMPLE_HZ -> gives binary semaphore
 *     -> prvAccelSamplerTask reads BMI160 over I2C
 *     -> queues raw counts into xAccelRawQueue
 *     -> sensor task drains + filters + publishes
 *
 * The I2C read happens in a task (not the ISR) so it can safely use
 * the polling I2C driver; the timer just enforces the 200 Hz cadence.
 * The semaphore-based wake guarantees the spec's mandated
 * "Raw -> ISR -> Queue -> Task" flow. */

static SemaphoreHandle_t s_xAccelTickSem = NULL;

/* NOTE: Timer1A is owned by the touchscreen driver (drivers/touch.c
 * uses it as the periodic ADC0 SS3 trigger). We use Timer2A here. */
static void prvAccelTimerInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) {}
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A,
                 g_ui32SysClock / ACCEL_SAMPLE_HZ - 1);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    /* Must be set BEFORE IntEnable. Any ISR calling FromISR-family
     * FreeRTOS APIs must run at numerically >= configMAX_SYSCALL_-
     * INTERRUPT_PRIORITY, otherwise the kernel ignores the call.
     * Without this line xSemaphoreGiveFromISR was a no-op. */
    IntPrioritySet(INT_TIMER2A, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_TIMER2A);
    TimerEnable(TIMER2_BASE, TIMER_A);
}

/* Diagnostic counters — printed by the sensor task every second. */
volatile uint32_t g_dbg_timer2a_irqs = 0;
volatile uint32_t g_dbg_accsamp_runs = 0;
volatile uint32_t g_dbg_accsamp_reads_ok = 0;
volatile uint32_t g_dbg_accsamp_reads_fail = 0;
volatile uint32_t g_dbg_queue_drained = 0;

void Timer2AIntHandler(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    g_dbg_timer2a_irqs++;
    BaseType_t hpw = pdFALSE;
    if (s_xAccelTickSem) xSemaphoreGiveFromISR(s_xAccelTickSem, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void prvAccelSamplerTask(void *pvParameters)
{
    (void)pvParameters;
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("AccSamp task started, sem=%p\n", s_xAccelTickSem);
#endif
    for (;;)
    {
        if (xSemaphoreTake(s_xAccelTickSem, portMAX_DELAY) != pdTRUE) continue;
        g_dbg_accsamp_runs++;

        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;

        int16_t ax, ay, az;
        bool ok = bmi160_read_accel_raw(&ax, &ay, &az);
        xSemaphoreGive(xI2CMutex);

        if (ok) { g_dbg_accsamp_reads_ok++; }
        else    { g_dbg_accsamp_reads_fail++; }

        if (ok)
        {
            AccelSampleRaw_t s = { ax, ay, az };
            xQueueSend(xAccelRawQueue, &s, 0);
        }
    }
}

/*-----------------------------------------------------------*/

static void prvSensorTask(void *pvParameters)
{
    (void)pvParameters;

    /* Bring up I2C0 + sensors inside the task so the scheduler is
     * already running (BMI160 / BME280 init use vTaskDelay).
     * All I2C work is serialised via xI2CMutex below — the accel
     * sampler task runs at a higher priority and would otherwise
     * pre-empt mid-transaction. */
    initI2C(g_ui32SysClock);
    initI2C2(g_ui32SysClock);
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    sensorOpt3001Init();
    xSemaphoreGive(xI2CMutex);

    /* Let the GUI task finish LCD + touch init first (it runs at higher
     * priority). Power ADC uses ADC1 only but must not run before touch
     * has configured ADC0 SS3. */
    vTaskDelay(pdMS_TO_TICKS(400));

    /* DRV8323 ADC current sensing — Timer3A triggers ADC1 SS0 at 1 kHz. */
#if MOTOR_ENABLE_POWER_SENSOR
    power_sensor_init();
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("Power sensor ON (Timer3A -> ADC1 @ %d Hz, idle zero-cal)\n",
                    POWER_SENSOR_SAMPLE_HZ);
#endif
#else
    xEventGroupClearBits(xSystemEvents, EVT_ESTOP_POWER);
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("Power sensor OFF (MOTOR_ENABLE_POWER_SENSOR=0)\n");
#endif
#endif

#if !SERIAL_PLOT_CLEAN
    uart_log_printf("BMI160 init...\n");
#endif
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bool bmi_ok = bmi160_init();
    xSemaphoreGive(xI2CMutex);
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("BMI160 %s\n", bmi_ok ? "ready" : "init FAILED");
#endif

    /* Spin up the 200 Hz accel sampling pipeline only if the IMU
     * came up. Otherwise we'd queue zeroes forever. */
    if (bmi_ok)
    {
        s_xAccelTickSem = xSemaphoreCreateBinary();
#if !SERIAL_PLOT_CLEAN
        uart_log_printf("AccSem create -> %p\n", s_xAccelTickSem);
#endif
        TaskHandle_t hAccSamp = NULL;
        BaseType_t r = xTaskCreate(prvAccelSamplerTask, "AccSamp",
                    configMINIMAL_STACK_SIZE * 2, NULL,
                    tskIDLE_PRIORITY + 4, &hAccSamp);
#if !SERIAL_PLOT_CLEAN
        uart_log_printf("AccSamp task create -> %d  handle=%p\n",
                        (int)r, hAccSamp);
#endif
        prvAccelTimerInit();
#if !SERIAL_PLOT_CLEAN
        uart_log_printf("Accel sampler (Timer2A @ %d Hz)\n", ACCEL_SAMPLE_HZ);
#endif
    }

    /* SHT31 (T+RH, spec 2.2.2 primary) — required */
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("SHT31 init...\n");
#endif
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bool sht_ok = sht31_init();
    xSemaphoreGive(xI2CMutex);
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("SHT31 %s\n", sht_ok ? "ready" : "init FAILED");
#endif

    /* BME280 (pressure only — T+H come from SHT31). */
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("BME280 init...\n");
#endif
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bool bme_ok = bme280_init();
    xSemaphoreGive(xI2CMutex);
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("BME280 %s\n", bme_ok ? "ready" : "init FAILED");
#endif

    float    light_lux  = 0.0f;
    float    accel_mag  = 0.0f;
    float    ax_g = 0, ay_g = 0, az_g = 0;
    float    temp_c = 0, hum_pct = 0, pres_hpa = 0;
    float    power_w   = 0.0f;     /* latest filtered motor power */
    uint32_t seq        = 0;

    /* Latest unfiltered values, kept across iterations so the serial
     * plot stream always has a fresh "raw" column even on ticks where
     * a particular sensor wasn't read. */
    float   i_total_raw = 0.0f;    /* last raw amps from ADC */
    float   raw_lux     = 0.0f;
    float   raw_total_g = 0.0f;
    uint32_t tick_count = 0;
    /* CSV header for the serial-plot tool (spec 2.2.5). All values
     * are integer-scaled to keep printf simple:
     *   p_raw_mw, p_filt_mw   power in milliwatts (W * 1000)
     *   lux_raw, lux_filt     lux (integer)
     *   a*_mg                 acceleration in milli-g  (g * 1000)
     *   t_cc                  temperature * 100        (deg C)
     *   h_cp                  humidity * 100           (%RH)
     *   p_dhpa                pressure * 10            (hPa)
     *   rpm_raw, rpm_filt     RPM (integer)
     */
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("# sensor CSV @ 50 Hz: t,p_raw_mw,p_filt_mw,lux,acc...\n");
    uart_log_printf("t,p_raw_mw,p_filt_mw,lux_raw,lux_filt,"
                    "ax_mg,ay_mg,az_mg,acc_raw_mg,acc_filt_mg,"
                    "t_cc,h_cp,p_dhpa,rpm_raw,rpm_filt\n");
#endif

#if MOTOR_ENABLE_POWER_SENSOR
    g_motor_power_watts = 0.0f;
    prvPowMAFReset();
    power_sensor_drain_raw_queue();
    power_sensor_adc_start();
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("Power ADC started; hold Idle ~0.3 s for zero-cal\n");
#endif
#endif

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        tick_count++;

        int32_t rpm_now = speed_sensor_get_rpm();
        if (rpm_now < 0)
        {
            rpm_now = 0;
        }

        /* ---- Power: average ADC samples since last wake (1 kHz ISR). */
        PowerSampleRaw_t psraw;
        float   i_accum = 0.0f;
        uint32_t i_samples = 0;
        bool    motor_idle = (g_motor_state == MOTOR_STATE_IDLE ||
                              g_motor_state == MOTOR_STATE_FAULT_LATCHED);

        while (xQueueReceive(xPowerRawQueue, &psraw, 0) == pdPASS)
        {
#if MOTOR_ENABLE_POWER_SENSOR
            if (motor_idle)
            {
                power_sensor_note_idle_sample(psraw.ia_counts, psraw.ib_counts);
            }
#endif
            float i_inst = power_sensor_counts_to_amps(psraw.ia_counts,
                                                     psraw.ib_counts);
            i_accum += i_inst;
            i_samples++;
        }

        if (!power_sensor_zero_ready() || motor_idle ||
            !g_motor_power_estop_ok ||
            rpm_now < POWER_ESTOP_MIN_RPM)
        {
            power_w = 0.0f;
            prvPowMAFReset();
            s_pwr_estop_cnt = 0;
        }
        else if (i_samples > 0u)
        {
            i_total_raw = i_accum / (float)i_samples;
            power_w = power_sensor_amps_to_watts(prvPowMAFUpdate(i_total_raw));
        }
        else
        {
            power_w = 0.0f;
        }

        g_motor_power_watts = power_w;
        g_serial_plot_lux       = (int32_t)light_lux;
        g_serial_plot_accel_mg  = (int32_t)(accel_mag * 1000.0f);
#if MOTOR_ENABLE_POWER_SENSOR
        if (g_motor_power_estop_ok &&
            power_sensor_zero_ready() &&
            rpm_now >= POWER_ESTOP_MIN_RPM &&
            power_w > g_thresh_power_w)
        {
            if (s_pwr_estop_cnt < 255u)
            {
                s_pwr_estop_cnt++;
            }
        }
        else
        {
            s_pwr_estop_cnt = 0;
        }

        if (s_pwr_estop_cnt >= POWER_ESTOP_DEBOUNCE_TICKS)
        {
            xEventGroupSetBits(xSystemEvents, EVT_ESTOP_POWER);
        }
        else
        {
            xEventGroupClearBits(xSystemEvents, EVT_ESTOP_POWER);
        }
#else
        xEventGroupClearBits(xSystemEvents, EVT_ESTOP_POWER);
#endif

        /* ---- Lux: poll OPT3001 at 5 Hz (every 10th tick). */
        if ((tick_count % LUX_READ_EVERY_N) == 0)
        {
            uint16_t opt_raw;
            xSemaphoreTake(xI2CMutex, portMAX_DELAY);
            bool got = sensorOpt3001Read(&opt_raw);
            xSemaphoreGive(xI2CMutex);
            if (got)
            {
                sensorOpt3001Convert(opt_raw, &raw_lux);
                light_lux = prvLuxMAFUpdate(raw_lux);
            }

            if (light_lux < NIGHT_LIGHT_LUX)
                xEventGroupSetBits(xSystemEvents, EVT_NIGHT_DETECTED);
            else
                xEventGroupClearBits(xSystemEvents, EVT_NIGHT_DETECTED);
        }

        /* ---- Accel: drain Timer2A-fed queue. */
        AccelSampleRaw_t araw;
        const float lsb_per_g = bmi160_lsb_per_g();
        float       peak_total_g = 0.0f;
        bool        accel_estop_active = g_motor_estop_armed;

        while (xQueueReceive(xAccelRawQueue, &araw, 0) == pdPASS)
        {
            g_dbg_queue_drained++;
            ax_g = (float)araw.ax_raw / lsb_per_g;
            ay_g = (float)araw.ay_raw / lsb_per_g;
            az_g = (float)araw.az_raw / lsb_per_g;
            raw_total_g = prvAbsF(ax_g) + prvAbsF(ay_g) + prvAbsF(az_g);
            accel_mag = prvAccelMAFUpdate(raw_total_g);
            if (raw_total_g > peak_total_g)
            {
                peak_total_g = raw_total_g;
            }
        }
        /* E-stop: peak |ax|+|ay|+|az| (no MAF) so firm shakes are not averaged away.
         * Latch the event bit until motor ACK — do not clear when below. */
        if (accel_estop_active && peak_total_g > g_thresh_accel_g)
        {
            xEventGroupSetBits(xSystemEvents, EVT_ESTOP_ACCEL);
        }

        /* ---- Environment: 1 Hz read.
         * SHT31 -> temp_c + hum_pct (spec-required T/H sensor).
         * BME280 -> pressure only; its own T/H is discarded so we
         * don't disagree with the SHT31. Each gated independently
         * so one missing sensor doesn't take down the other. */
        if ((tick_count % BME_READ_EVERY_N) == 0)
        {
            /* SHT31: spec-required source for temperature + humidity.
             * If absent, temp_c / hum_pct stay zero and the GUI
             * displays "--- C" / "--- %". */
            if (sht_ok)
            {
                xSemaphoreTake(xI2CMutex, portMAX_DELAY);
                bool sok = sht31_read(&temp_c, &hum_pct);
                xSemaphoreGive(xI2CMutex);
                if (!sok) sht_ok = false;
            }

            /* BME280: read for pressure only. Its own T+H values are
             * discarded so we don't disagree with the SHT31. */
            if (bme_ok)
            {
                float bme_t_unused, bme_h_unused;
                xSemaphoreTake(xI2CMutex, portMAX_DELAY);
                bme280_read(&bme_t_unused, &bme_h_unused, &pres_hpa);
                xSemaphoreGive(xI2CMutex);
                (void)bme_t_unused; (void)bme_h_unused;
            }
        }

        /* ---- Publish to GUI at 5 Hz. */
        if ((tick_count % GUI_PUB_EVERY_N) == 0)
        {
            SensorMsgObj msg;
            memset(&msg, 0, sizeof(msg));
            msg.seq           = ++seq;
            msg.tick          = xTaskGetTickCount();
            msg.power_watts   = power_w;
            msg.light_lux     = light_lux;
            msg.accel_x_g     = ax_g;
            msg.accel_y_g     = ay_g;
            msg.accel_z_g     = az_g;
            msg.accel_total_g = accel_mag;
            msg.temp_c        = temp_c;
            msg.humidity_pct  = hum_pct;
            msg.pressure_hpa  = pres_hpa;
            msg.sht_ok        = sht_ok;
            msg.bme_ok        = bme_ok;
            xQueueSend(xSensorQueue, &msg, 0);
        }

#if !SERIAL_PLOT_CLEAN
        /* Diagnostic dump every 50 ticks (1 s). */
        if ((tick_count % 50) == 0)
        {
            uart_log_printf("DBG  T2A=%u  I_mA=%d  P_mW=%d  AccRun=%u\n",
                            (unsigned)g_dbg_timer2a_irqs,
                            (int)(i_total_raw * 1000.0f),
                            (int)(power_w * 1000.0f),
                            (unsigned)g_dbg_accsamp_runs);
        }

        /* Verbose sensor CSV @ 50 Hz (15 columns). */
        uart_log_printf("%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                        (unsigned)tick_count,
                        (int)(i_total_raw * POWER_SENSOR_VOLTAGE_V * 1000),
                        (int)(power_w * 1000),
                        (int)raw_lux,
                        (int)light_lux,
                        (int)(ax_g * 1000),
                        (int)(ay_g * 1000),
                        (int)(az_g * 1000),
                        (int)(raw_total_g * 1000),
                        (int)(accel_mag * 1000),
                        (int)(temp_c * 100),
                        (int)(hum_pct * 100),
                        (int)(pres_hpa * 10),
                        (int)speed_sensor_get_rpm_raw(),
                        (int)speed_sensor_get_rpm());
#endif
    }
}

/*-----------------------------------------------------------*/

void vCreateSensorTask(void)
{
    xTaskCreate(prvSensorTask, "Sensor",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 3, NULL);
}
