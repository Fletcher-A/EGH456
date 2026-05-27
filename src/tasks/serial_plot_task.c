/*
 * serial_plot_task.c — Dedicated 50 Hz CSV stream for serialplotter.io.
 *
 * Runs independently of sensor I2C init so the browser sees numeric lines
 * within ~100 ms of boot. Values are updated by motor_task / sensor_task.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "shared.h"
#include "drivers/speed_sensor.h"
#include "utils/uart_log.h"
#include "utils/uartstdio.h"

#if SERIAL_PLOT_CLEAN

#define PLOT_PERIOD_MS  20

static void prvSerialPlotTask(void *pvParameters)
{
    TickType_t xLastWake = xTaskGetTickCount();
    uint32_t   line = 0;

    (void)pvParameters;

    /* Channel names once at boot. Do NOT repeat — atomic14 / SerialPlotster
     * treat a new "# ..." line as a device restart and wipe the chart (looks
     * like traces stuck on the right forever). Use \\n only (uartstdio adds \\r). */
    uart_plot_printf("# sample_number desired_rpm reference_rpm actual_rpm "
                     "duty_percent power_milliwatts light_lux "
                     "acceleration_millig hall_display_rpm\n");

    for (;;)
    {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PLOT_PERIOD_MS));
        line++;

        uart_plot_printf("%u,%d,%d,%d,%u,%d,%d,%d,%d\n",
                         (unsigned)line,
                         (int)g_plot_rpm_desired,
                         (int)g_plot_rpm_reference,
                         (int)g_plot_rpm_actual,
                         (unsigned)g_motor_pwm_duty_pct,
                         (int)(g_motor_power_watts * 1000.0f),
                         (int)g_serial_plot_lux,
                         (int)g_serial_plot_accel_mg,
                         (int)speed_sensor_get_rpm_display());
    }
}

void vCreateSerialPlotTask(void)
{
    BaseType_t ok = xTaskCreate(prvSerialPlotTask, "Plot",
                                configMINIMAL_STACK_SIZE * 4, NULL,
                                tskIDLE_PRIORITY + 5, NULL);
    if (ok != pdPASS)
    {
        UARTprintf("Plot task create FAILED (heap?)\n");
    }
}

#else

void vCreateSerialPlotTask(void) {}

#endif
