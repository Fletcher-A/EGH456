/*
 * gui_task.c — Touchscreen GUI task (assignment 2.3).
 *
 * Single task that owns BOTH GUI pages (control panel + sensor plots)
 * and the touch input. No separate gui_*.c files — keep all the LCD
 * code in one place so the page-switching logic is obvious.
 *
 * Page 1 — Control panel (assignment 2.3.1):
 *   - Start / Stop / E-Stop ACK buttons
 *   - Speed slider (writes desired RPM to xCommandQueue)
 *   - Motor state text + colour-coded indicator (green/orange/red)
 *   - System clock (HH:MM:SS)
 *   - Threshold inputs (power, accel, distance)
 *   - Day/night LED indicator
 *
 * Page 2 — Plots (assignment 2.3.3):
 *   - ≥5 s scrolling time window
 *   - Channels: speed, power, light, optional A, optional B
 *   - Per-channel visibility toggle
 *
 * Inputs:
 *   - xMotorQueue   - latest MotorMsgObj
 *   - xSensorQueue  - latest SensorMsgObj
 *   - xSystemEvents - night flag, fault flags
 *   - Touch screen  - taps and drags
 *
 * Outputs:
 *   - LCD via grlib (Kentec320x240x16_ssd2119_spi driver)
 *   - xCommandQueue (desired RPM)
 *   - EVT_USER_START / _STOP / _ESTOP_ACK / _SPEED_CHANGED /
 *     _THRESHOLD_CHANGED in xSystemEvents
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "shared.h"

#include "grlib.h"
#include "drivers/Kentec320x240x16_ssd2119_spi.h"
/*-----------------------------------------------------------*/

/* Which page is currently shown. Toggled by a tab/button. */
typedef enum { GUI_PAGE_CONTROL = 0, GUI_PAGE_PLOTS } GuiPage_t;

/*-----------------------------------------------------------*/

static void prvGuiTask(void *pvParameters)
{
    (void)pvParameters;

    /* TODO: Kentec320x240x16_SSD2119Init(g_ui32SysClock);
     * TODO: GrContextInit(&context, &g_sKentec320x240x16_SSD2119);
     * TODO: clear screen, draw static frame for the default page.
     * TODO: TouchScreenInit + TouchScreenCallbackSet for input.
     */

    for (;;)
    {
        /* TODO: xQueueReceive(xMotorQueue,  ...) with short timeout. */

        /* TODO: xQueueReceive(xSensorQueue, ..., 0) non-blocking. */

        /* TODO: xEventGroupWaitBits non-blocking on
         *       EVT_NIGHT_DETECTED | EVT_ESTOP_ANY | EVT_SENSOR_FAULT.
         */

        /* TODO: switch on current page:
         *   - PAGE_CONTROL: redraw clock, state, slider, buttons,
         *     thresholds, day/night LED.
         *   - PAGE_PLOTS: push samples into circular buffers,
         *     redraw line traces for each visible channel.
         */
    }
}

/*-----------------------------------------------------------*/
/* Touch callback — called by the touch driver when a tap/drag
 * occurs. Translate (x, y) into a UI action. */

void gui_handle_touch(int32_t x, int32_t y)
{
    (void)x; (void)y;
    /* TODO:
     *   - START button hit -> xEventGroupSetBits(EVT_USER_START)
     *   - STOP  button hit -> xEventGroupSetBits(EVT_USER_STOP)
     *   - E-Stop ACK hit   -> xEventGroupSetBits(EVT_USER_ESTOP_ACK)
     *   - Slider drag      -> compute new RPM, xQueueSend(xCommandQueue, ...)
     *   - Threshold +/-    -> update the local threshold, set
     *                         EVT_USER_THRESHOLD_CHANGED.
     *   - Tab tap          -> swap GUI_PAGE_CONTROL <-> GUI_PAGE_PLOTS.
     */
}

/*-----------------------------------------------------------*/

void vCreateGuiTask(void)
{
    xTaskCreate(prvGuiTask, "GUI",
                configMINIMAL_STACK_SIZE * 8, NULL,
                tskIDLE_PRIORITY + 2, NULL);
}
