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
 *       EVT_ESTOP_ACCEL     set when filtered total |a| exceeds limit.
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

#include "shared.h"
/*-----------------------------------------------------------*/

extern uint32_t g_ui32SysClock;

#define SAMPLE_PERIOD_MS     20      /* 50 Hz outer loop (serial plot rate) */
#define GUI_PUB_EVERY_N      10      /* publish to GUI every 10th tick = 5 Hz */
#define LUX_READ_EVERY_N     10      /* read OPT3001 every 200 ms (5 Hz)   */
#define BME_READ_EVERY_N     50      /* read BME280  every 1 s   (1 Hz)   */
#define MAF_WINDOW           8

#define ACCEL_SAMPLE_HZ     200      /* spec: >= 100 Hz */

#include "drivers/speed_sensor.h"    /* for serial-plot RPM raw + filt */

/* Impact threshold on filtered |ax|+|ay|+|az| is now a runtime global
 * (g_thresh_accel_g), editable from the GUI Thresholds tab. */

/*-----------------------------------------------------------*/

static float    g_fLuxBuf[MAF_WINDOW] = {0};
static uint8_t  g_ui8LuxIdx = 0;
static bool     g_bLuxFull  = false;

static float    g_fAccBuf[MAF_WINDOW] = {0};
static uint8_t  g_ui8AccIdx = 0;
static bool     g_bAccFull  = false;

/* Power filter — 32-sample MAF. At 1 kHz sampling that's a ~32 ms
 * window, fast enough to catch an overcurrent and slow enough to
 * reject PWM switching noise. Buffer holds total motor current (A). */
#define POW_MAF_WINDOW   32
static float    g_fPowBuf[POW_MAF_WINDOW] = {0};
static uint8_t  g_ui8PowIdx = 0;
static bool     g_bPowFull  = false;

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

static float prvAccMAFUpdate(float newVal)
{
    g_fAccBuf[g_ui8AccIdx] = newVal;
    g_ui8AccIdx = (g_ui8AccIdx + 1) % MAF_WINDOW;
    if (g_ui8AccIdx == 0) g_bAccFull = true;
    uint8_t count = g_bAccFull ? MAF_WINDOW : g_ui8AccIdx;
    float sum = 0.0f;
    for (uint8_t i = 0; i < count; i++) sum += g_fAccBuf[i];
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
    UARTprintf("AccSamp task started, sem=%p\n", s_xAccelTickSem);
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
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    sensorOpt3001Init();
    xSemaphoreGive(xI2CMutex);

    /* DRV8323 ADC current sensing — Timer0A triggers ADC1 SS0 at
     * 1 kHz; the ISR queues raw counts; we drain + filter here.
     * DISABLED until the motor BoosterPack is wired — the power
     * pipeline (ISR, queue, MAF, EVT_ESTOP_POWER) is implemented
     * but won't run. To re-enable, uncomment the init below. */
    // power_sensor_init();
    // UARTprintf("Power sensor init (Timer0A -> ADC1 SS0 @ %d Hz)\n",
    //            POWER_SENSOR_SAMPLE_HZ);

    UARTprintf("BMI160 init...\n");
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bool bmi_ok = bmi160_init();
    xSemaphoreGive(xI2CMutex);
    UARTprintf("BMI160 %s\n", bmi_ok ? "ready" : "init FAILED");

    /* Spin up the 200 Hz accel sampling pipeline only if the IMU
     * came up. Otherwise we'd queue zeroes forever. */
    if (bmi_ok)
    {
        s_xAccelTickSem = xSemaphoreCreateBinary();
        UARTprintf("AccSem create -> %p\n", s_xAccelTickSem);
        TaskHandle_t hAccSamp = NULL;
        BaseType_t r = xTaskCreate(prvAccelSamplerTask, "AccSamp",
                    configMINIMAL_STACK_SIZE * 2, NULL,
                    tskIDLE_PRIORITY + 5, &hAccSamp);
        UARTprintf("AccSamp task create -> %d  handle=%p\n",
                   (int)r, hAccSamp);
        prvAccelTimerInit();
        UARTprintf("Accel sampler (Timer2A @ %d Hz)\n", ACCEL_SAMPLE_HZ);
    }

    UARTprintf("BME280 init...\n");
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bool bme_ok = bme280_init();
    xSemaphoreGive(xI2CMutex);
    UARTprintf("BME280 %s\n", bme_ok ? "ready" : "init FAILED");

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
    UARTprintf("t,p_raw_mw,p_filt_mw,lux_raw,lux_filt,"
               "ax_mg,ay_mg,az_mg,acc_raw_mg,acc_filt_mg,"
               "t_cc,h_cp,p_dhpa,rpm_raw,rpm_filt\n");

    TickType_t xLastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        tick_count++;

        /* ---- Power: drain every raw ADC sample since last wake. */
        PowerSampleRaw_t psraw;
        while (xQueueReceive(xPowerRawQueue, &psraw, 0) == pdPASS)
        {
            i_total_raw = power_sensor_counts_to_amps(psraw.ia_counts,
                                                     psraw.ib_counts);
            float i_filt = prvPowMAFUpdate(i_total_raw);
            power_w = POWER_SENSOR_VOLTAGE_V * i_filt;
        }
        if (power_w > g_thresh_power_w)
            xEventGroupSetBits(xSystemEvents, EVT_ESTOP_POWER);

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

        /* ---- Accel: drain Timer1A-fed queue. */
        AccelSampleRaw_t araw;
        const float lsb_per_g = bmi160_lsb_per_g();
        while (xQueueReceive(xAccelRawQueue, &araw, 0) == pdPASS)
        {
            g_dbg_queue_drained++;
            ax_g = (float)araw.ax_raw / lsb_per_g;
            ay_g = (float)araw.ay_raw / lsb_per_g;
            az_g = (float)araw.az_raw / lsb_per_g;
            raw_total_g = prvAbsF(ax_g) + prvAbsF(ay_g) + prvAbsF(az_g);
            accel_mag = prvAccMAFUpdate(raw_total_g);

            if (accel_mag > g_thresh_accel_g)
                xEventGroupSetBits(xSystemEvents, EVT_ESTOP_ACCEL);
        }

        /* ---- BME280: 1 Hz read (its own internal oversampling). */
        if (bme_ok && (tick_count % BME_READ_EVERY_N) == 0)
        {
            xSemaphoreTake(xI2CMutex, portMAX_DELAY);
            //bme280_read(&temp_c, &hum_pct, &pres_hpa);
            sht31_read(&temp_c, &hum_pct);
            xSemaphoreGive(xI2CMutex);
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
            msg.bme_ok        = bme_ok;
            xQueueSend(xSensorQueue, &msg, 0);
        }

        /* Diagnostic dump every 50 ticks (1 s). */
        if ((tick_count % 50) == 0)
        {
            UARTprintf("DBG  T2A=%u  AccRun=%u  RdOK=%u  RdFail=%u  Drained=%u\n",
                       (unsigned)g_dbg_timer2a_irqs,
                       (unsigned)g_dbg_accsamp_runs,
                       (unsigned)g_dbg_accsamp_reads_ok,
                       (unsigned)g_dbg_accsamp_reads_fail,
                       (unsigned)g_dbg_queue_drained);
        }

        /* ---- Serial plot: one CSV line per tick (50 Hz). Each value
         * is integer-scaled to avoid pulling in libc's %f. Units are
         * documented in the CSV header above. */
        if (xSemaphoreTake(xUARTMutex, 0) == pdTRUE)
        {
            /* Print in two halves to keep the per-call printf
             * argument count modest. */
            UARTprintf("%u,%d,%d,%d,%d,%d,%d,%d,",
                       (unsigned)tick_count,
                       (int)(i_total_raw * POWER_SENSOR_VOLTAGE_V * 1000),
                       (int)(power_w * 1000),
                       (int)raw_lux,
                       (int)light_lux,
                       (int)(ax_g * 1000),
                       (int)(ay_g * 1000),
                       (int)(az_g * 1000));
            UARTprintf("%d,%d,%d,%d,%d,%d,%d\n",
                       (int)(raw_total_g * 1000),
                       (int)(accel_mag   * 1000),
                       (int)(temp_c   * 100),
                       (int)(hum_pct  * 100),
                       (int)(pres_hpa * 10),
                       (int)speed_sensor_get_rpm_raw(),
                       (int)speed_sensor_get_rpm());
            xSemaphoreGive(xUARTMutex);
        }
    }
}

/*-----------------------------------------------------------*/

void vCreateSensorTask(void)
{
    xTaskCreate(prvSensorTask, "Sensor",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 3, NULL);
}
