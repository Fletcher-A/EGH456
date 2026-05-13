/*
 * gui_task.c — Touchscreen GUI task (assignment 2.3).
 *
 * INTER-TASK INTERFACE
 * --------------------
 *
 * INPUTS (consumed by this task):
 *   xMotorQueue (MotorMsgObj)    — produced by motor_task.c at 20 Hz.
 *                                  Provides state, rpm_actual, pwm_duty,
 *                                  power_watts. Drained every loop iter.
 *   xSensorQueue (SensorMsgObj)  — produced by sensor_task.c at 5 Hz.
 *                                  Provides light_lux (OPT3001) and
 *                                  optional_a = total accel in g
 *                                  (BMI160, |ax|+|ay|+|az|).
 *   xSystemEvents bits read (non-blocking):
 *       EVT_NIGHT_DETECTED       — currently re-checked via hysteresis
 *                                  on the lux value here.
 *
 * OUTPUTS (produced by this task):
 *   xCommandQueue (int32_t)      — desired RPM, written when the
 *                                  speed slider moves. Read by
 *                                  motor_task.c.
 *   xSystemEvents bits set:
 *       EVT_USER_START           — set on START button press,
 *                                  read by motor_task.c (Idle->Starting)
 *       EVT_USER_STOP            — set on STOP button press,
 *                                  read by motor_task.c (any->Idle)
 *       EVT_USER_ESTOP_ACK       — set on ACK button press,
 *                                  read by motor_task.c (Fault->Idle)
 *       (future) EVT_USER_THRESHOLD_CHANGED — when threshold +/- added.
 *
 * PUBLIC FUNCTIONS (called from elsewhere):
 *   vCreateGuiTask()             — called once from main.c
 *
 * HARDWARE DRIVERS USED (private to this task):
 *   Kentec320x240x16_SSD2119Init  (drivers/Kentec320x240x16_ssd2119_spi.c)
 *   TouchScreenInit / Callback    (drivers/touch.c)
 *   TouchScreenIntHandler         (ISR, wired via startup_gcc.c -> ADC SS3)
 *   grlib (canvas, slider, pushbutton, widget, image)
 *
 * PAGES
 * -----
 *   Page 0 — Control: state text, status indicator, RPM/power/lux
 *                     readouts, speed slider, START/STOP/ACK buttons,
 *                     clock, day/night text + LED, threshold label.
 *   Page 1 — Plots:   scrolling RPM/Power/Lux traces with axis labels.
 *
 *   Prev/Next tab buttons set a deferred g_pendingPanel flag — the
 *   actual widget-tree swap happens in the main loop outside the
 *   touch callback, so the buttons never get stuck in pressed state.
 */

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "driver_lib/fpu.h"
#include "driver_lib/sysctl.h"
#include "driver_lib/udma.h"
#include "driver_lib/rom_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "grlib.h"
#include "widget.h"
#include "canvas.h"
#include "pushbutton.h"
#include "slider.h"
#include "utils/ustdlib.h"

#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"
#include "images.h"

#include "shared.h"
/*-----------------------------------------------------------*/

extern uint32_t g_ui32SysClock;

/* uDMA control table required by the LCD driver. */
#ifdef ewarm
#pragma data_alignment=1024
static tDMAControlTable s_DMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(s_DMAControlTable, 1024)
static tDMAControlTable s_DMAControlTable[64];
#else
static tDMAControlTable s_DMAControlTable[64] __attribute__ ((aligned(1024)));
#endif

/*-----------------------------------------------------------*/
/* Model — last received values from the inter-task queues. */

static MotorState_t g_state           = MOTOR_STATE_IDLE;
static int32_t      g_rpm_actual      = 0;
static uint16_t     g_pwm_duty        = 0;
static float        g_power_w         = 0.0f;
static float        g_light_lux       = 0.0f;
static float        g_accel_g         = 0.0f;   /* filtered Y, used by plot */
static float        g_accel_x_g       = 0.0f;
static float        g_accel_y_g       = 0.0f;
static float        g_accel_z_g       = 0.0f;
static float        g_temp_c          = 0.0f;
static float        g_humidity_pct    = 0.0f;
static float        g_pressure_hpa    = 0.0f;
static bool         g_bme_ok          = false;

/* Tab state. */
static volatile int32_t g_pendingPanel = -1;
static uint32_t         g_ui32Panel    = 0;

/* TODO: threshold values shown on screen — currently a static label.
 * When +/- buttons are added, these globals will be the editable
 * source of truth, posted to the sensor task via a mailbox queue. */

/*-----------------------------------------------------------*/
/* Forward declarations. */

static void OnStartPressed(tWidget *psWidget);
static void OnStopPressed(tWidget *psWidget);
static void OnEStopAckPressed(tWidget *psWidget);
static void OnSpeedSliderChange(tWidget *psWidget, int32_t i32Value);
static void OnPrevTab(tWidget *psWidget);
static void OnNextTab(tWidget *psWidget);
static void OnPlotCanvasPaint(tWidget *psWidget, tContext *psContext);
static void OnPowerInc(tWidget *psWidget);
static void OnPowerDec(tWidget *psWidget);
static void OnAccelInc(tWidget *psWidget);
static void OnAccelDec(tWidget *psWidget);
static void OnDistInc (tWidget *psWidget);
static void OnDistDec (tWidget *psWidget);
static void prvRefreshPowerLabel(void);
static void prvRefreshAccelLabel(void);
static void prvRefreshDistLabel(void);
static void prvRefreshControlThreshLine(void);

extern tCanvasWidget g_psPanels[];
extern tCanvasWidget g_sPlotCanvas;
extern tCanvasWidget g_sSensPresText;
extern tPushButtonWidget g_sThreshDistInc;

/*-----------------------------------------------------------*/
/* Control-panel widgets. */

Canvas(g_sStateText, g_psPanels, 0, 0, &g_sKentec320x240x16_SSD2119,
       10, 30, 180, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrSilver, &g_sFontCm22, "Idle", 0, 0);

Canvas(g_sStatusIndicator, g_psPanels, &g_sStateText, 0,
       &g_sKentec320x240x16_SSD2119,
       250, 30, 60, 24,
       CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE,
       ClrOrange, ClrGray, 0, 0, 0, 0, 0);

Canvas(g_sRpmText, g_psPanels, &g_sStatusIndicator, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 60, 200, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrSilver, &g_sFontCm18, "0 RPM", 0, 0);

Canvas(g_sPowerText, g_psPanels, &g_sRpmText, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 82, 200, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrSilver, &g_sFontCm18, "0 W", 0, 0);

Canvas(g_sDayNightText, g_psPanels, &g_sPowerText, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 104, 120, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrSilver, &g_sFontCm18, "Day", 0, 0);

Canvas(g_sDayNightLed, g_psPanels, &g_sDayNightText, 0,
       &g_sKentec320x240x16_SSD2119,
       140, 104, 20, 20,
       CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0);

/* Lux + accel readouts now live on the Sensors tab (panel 2). */

Canvas(g_sPowerLimitText, g_psPanels, &g_sDayNightLed, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 128, 300, 18,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrGray, &g_sFontCm14,
       "Power 150W  Accel 2.0g  Dist 200mm", 0, 0);

Canvas(g_sClockText, g_psPanels, &g_sPowerLimitText, 0,
       &g_sKentec320x240x16_SSD2119,
       220, 60, 90, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_OPAQUE,
       ClrBlack, 0, ClrWhite, &g_sFontCm18, "00:00:00", 0, 0);

tSliderWidget g_sSpeedSlider =
    SliderStruct(g_psPanels, &g_sClockText, 0,
                 &g_sKentec320x240x16_SSD2119,
                 10, 150, 300, 24, 0, 100, 0,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrBlue, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm16, "Speed 0%", 0, 0, OnSpeedSliderChange);

RectangularButton(g_sStartBtn, g_psPanels, &g_sSpeedSlider, 0,
                  &g_sKentec320x240x16_SSD2119,
                  10, 180, 90, 30,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGreen, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm18, "START", 0, 0, 0, 0, OnStartPressed);

RectangularButton(g_sStopBtn, g_psPanels, &g_sStartBtn, 0,
                  &g_sKentec320x240x16_SSD2119,
                  110, 180, 90, 30,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGoldenrod, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm18, "STOP", 0, 0, 0, 0, OnStopPressed);

RectangularButton(g_sAckBtn, g_psPanels, &g_sStopBtn, 0,
                  &g_sKentec320x240x16_SSD2119,
                  210, 180, 100, 30,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkRed, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm18, "ACK", 0, 0, 0, 0, OnEStopAckPressed);

/*-----------------------------------------------------------*/
/* Panels and tabs. */

tCanvasWidget g_psPanels[] =
{
    /* Page 0 — Control. */
    CanvasStruct(0, 0, &g_sAckBtn, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 216, CANVAS_STYLE_FILL,
                 ClrBlack, 0, 0, 0, 0, 0, 0),

    /* Page 1 — Plots (child = g_sPlotCanvas). */
    CanvasStruct(0, 0, &g_sPlotCanvas, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 216, CANVAS_STYLE_FILL,
                 ClrBlack, 0, 0, 0, 0, 0, 0),

    /* Page 2 — Sensors (scalar readouts: lux, ax/ay/az, BME280). */
    CanvasStruct(0, 0, &g_sSensPresText, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 216, CANVAS_STYLE_FILL,
                 ClrBlack, 0, 0, 0, 0, 0, 0),

    /* Page 3 — Thresholds (runtime-editable safety limits). */
    CanvasStruct(0, 0, &g_sThreshDistInc, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 216, CANVAS_STYLE_FILL,
                 ClrBlack, 0, 0, 0, 0, 0, 0),
};

#define NUM_PANELS  (sizeof(g_psPanels) / sizeof(g_psPanels[0]))

static const char *g_pcPanelNames[NUM_PANELS] = {
    "Control", "Plots", "Sensors", "Thresholds"
};

/*-----------------------------------------------------------*/
/* Plot canvas. */

Canvas(g_sPlotCanvas, g_psPanels + 1, 0, 0, &g_sKentec320x240x16_SSD2119,
       10, 30, 300, 200,
       CANVAS_STYLE_OUTLINE | CANVAS_STYLE_APP_DRAWN,
       0, ClrGray, 0, 0, 0, 0, OnPlotCanvasPaint);

#define PLOT_X        12
#define PLOT_Y        32
#define PLOT_W        296
#define PLOT_H        196
#define PLOT_SAMPLES  40
#define PLOT_RPM_MAX    4000.0f
#define PLOT_POWER_MAX   200.0f
#define PLOT_LUX_MAX     500.0f
#define PLOT_ACCEL_MAX     2.0f       /* +-2 g full scale, centred on plot */

static float    g_plot_rpm  [PLOT_SAMPLES];
static float    g_plot_power[PLOT_SAMPLES];
static float    g_plot_lux  [PLOT_SAMPLES];
static float    g_plot_accel[PLOT_SAMPLES];
static float    g_rpm_flat  [PLOT_SAMPLES];
static float    g_pow_flat  [PLOT_SAMPLES];
static float    g_lux_flat  [PLOT_SAMPLES];
static float    g_acc_flat  [PLOT_SAMPLES];
static float    g_prev_rpm  [PLOT_SAMPLES];
static float    g_prev_pow  [PLOT_SAMPLES];
static float    g_prev_lux  [PLOT_SAMPLES];
static float    g_prev_acc  [PLOT_SAMPLES];
static uint32_t g_prev_count = 0;
static uint32_t g_plot_head  = 0;
static uint32_t g_plot_count = 0;

static void prvPlotPush(float rpm, float power, float lux, float accel)
{
    g_plot_rpm  [g_plot_head] = rpm;
    g_plot_power[g_plot_head] = power;
    g_plot_lux  [g_plot_head] = lux;
    g_plot_accel[g_plot_head] = accel;
    g_plot_head = (g_plot_head + 1) % PLOT_SAMPLES;
    if (g_plot_count < PLOT_SAMPLES) g_plot_count++;
}

static int prvPlotY(float v, float vmax)
{
    if (v < 0)    v = 0;
    if (v > vmax) v = vmax;
    return (int)(PLOT_Y + PLOT_H - (v / vmax) * PLOT_H);
}

/* Centred Y mapping for signed traces (accel-Y). [-vmax, +vmax] ->
 * [bottom, top], 0 at the vertical midline. */
static int prvPlotYSigned(float v, float vmax)
{
    if (v >  vmax) v =  vmax;
    if (v < -vmax) v = -vmax;
    float norm = (v + vmax) / (2.0f * vmax);
    return (int)(PLOT_Y + PLOT_H - norm * PLOT_H);
}

static int prvPlotXAt(uint32_t i)
{
    return PLOT_X + (int)((i * (PLOT_W - 1)) / (PLOT_SAMPLES - 1));
}

static void prvDrawTrace(tContext *ctx, const float *flat, uint32_t count,
                         float vmax, uint32_t color, bool signed_axis)
{
    if (count < 2) return;
    GrContextForegroundSet(ctx, color);
    for (uint32_t i = 1; i < count; i++)
    {
        int y0 = signed_axis ? prvPlotYSigned(flat[i - 1], vmax)
                             : prvPlotY      (flat[i - 1], vmax);
        int y1 = signed_axis ? prvPlotYSigned(flat[i],     vmax)
                             : prvPlotY      (flat[i],     vmax);
        GrLineDraw(ctx, prvPlotXAt(i - 1), y0, prvPlotXAt(i), y1);
    }
}

static void OnPlotCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    (void)psWidget;

    /* Erase previous frame in black. */
    if (g_prev_count >= 2)
    {
        prvDrawTrace(psContext, g_prev_rpm, g_prev_count, PLOT_RPM_MAX,   ClrBlack, false);
        prvDrawTrace(psContext, g_prev_pow, g_prev_count, PLOT_POWER_MAX, ClrBlack, false);
        prvDrawTrace(psContext, g_prev_lux, g_prev_count, PLOT_LUX_MAX,   ClrBlack, false);
        prvDrawTrace(psContext, g_prev_acc, g_prev_count, PLOT_ACCEL_MAX, ClrBlack, true);
    }

    /* Unroll circular buffer. */
    uint32_t start = (g_plot_head + PLOT_SAMPLES - g_plot_count) % PLOT_SAMPLES;
    for (uint32_t i = 0; i < g_plot_count; i++)
    {
        uint32_t idx = (start + i) % PLOT_SAMPLES;
        g_rpm_flat[i] = g_plot_rpm  [idx];
        g_pow_flat[i] = g_plot_power[idx];
        g_lux_flat[i] = g_plot_lux  [idx];
        g_acc_flat[i] = g_plot_accel[idx];
    }

    /* Draw current frame. */
    prvDrawTrace(psContext, g_rpm_flat, g_plot_count, PLOT_RPM_MAX,   ClrGoldenrod, false);
    prvDrawTrace(psContext, g_pow_flat, g_plot_count, PLOT_POWER_MAX, ClrCyan,      false);
    prvDrawTrace(psContext, g_lux_flat, g_plot_count, PLOT_LUX_MAX,   ClrWhite,     false);
    prvDrawTrace(psContext, g_acc_flat, g_plot_count, PLOT_ACCEL_MAX, ClrMagenta,   true);

    /* Snapshot for next erase. */
    for (uint32_t i = 0; i < g_plot_count; i++)
    {
        g_prev_rpm[i] = g_rpm_flat[i];
        g_prev_pow[i] = g_pow_flat[i];
        g_prev_lux[i] = g_lux_flat[i];
        g_prev_acc[i] = g_acc_flat[i];
    }
    g_prev_count = g_plot_count;

    /* Y-axis labels. */
    GrContextFontSet(psContext, &g_sFontCm12);
    GrContextBackgroundSet(psContext, ClrBlack);
    GrContextForegroundSet(psContext, ClrGoldenrod);
    GrStringDraw(psContext, "4000 RPM", -1, PLOT_X + 4, PLOT_Y + 2,  1);
    GrContextForegroundSet(psContext, ClrCyan);
    GrStringDraw(psContext, "200 W",    -1, PLOT_X + 4, PLOT_Y + 16, 1);
    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDraw(psContext, "500 lux",  -1, PLOT_X + 4, PLOT_Y + 30, 1);
    GrContextForegroundSet(psContext, ClrMagenta);
    GrStringDraw(psContext, "+2 g",     -1, PLOT_X + 4, PLOT_Y + 44, 1);
    GrStringDraw(psContext, "0 g",      -1, PLOT_X + 4,
                 PLOT_Y + (PLOT_H / 2) - 6, 1);
    GrStringDraw(psContext, "-2 g",     -1, PLOT_X + 4,
                 PLOT_Y + PLOT_H - 28, 1);
    GrContextForegroundSet(psContext, ClrSilver);
    GrStringDraw(psContext, "0",        -1, PLOT_X + 4, PLOT_Y + PLOT_H - 14, 1);
}

/*-----------------------------------------------------------*/
/* Page 2 — Sensors panel widgets. Each one has its OWN static text
 * buffer (CanvasTextSet stores the pointer, not the string — sharing
 * one buffer would alias all widgets together). Declared in reverse
 * draw order so each Canvas can use the previous one as its sibling. */

Canvas(g_sSensLuxText, g_psPanels + 2, 0, 0, &g_sKentec320x240x16_SSD2119,
       10, 32, 300, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrWhite, &g_sFontCm22, "Lux:   ---", 0, 0);

Canvas(g_sSensAxText, g_psPanels + 2, &g_sSensLuxText, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 62, 100, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm18, "X: ---", 0, 0);

Canvas(g_sSensAyText, g_psPanels + 2, &g_sSensAxText, 0,
       &g_sKentec320x240x16_SSD2119,
       115, 62, 100, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm18, "Y: ---", 0, 0);

Canvas(g_sSensAzText, g_psPanels + 2, &g_sSensAyText, 0,
       &g_sKentec320x240x16_SSD2119,
       220, 62, 100, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm18, "Z: ---", 0, 0);

Canvas(g_sSensTempText, g_psPanels + 2, &g_sSensAzText, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 92, 145, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrOrange, &g_sFontCm18, "Temp:  --- C", 0, 0);

Canvas(g_sSensHumText, g_psPanels + 2, &g_sSensTempText, 0,
       &g_sKentec320x240x16_SSD2119,
       165, 92, 150, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrCyan, &g_sFontCm18, "Hum:   --- %", 0, 0);

Canvas(g_sSensPresText, g_psPanels + 2, &g_sSensHumText, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 122, 300, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrLimeGreen, &g_sFontCm18, "Press: --- hPa", 0, 0);

/*-----------------------------------------------------------*/
/* Page 3 — Thresholds tab.
 *
 * Three rows, each "label + value + [-] + [+]":
 *   Power     150 W       step  10 W       range  50 .. 300
 *   Accel     2.0 g       step 0.1 g       range 0.5 .. 4.0
 *   Distance  200 mm      step  50 mm      range  50 .. 1000
 *
 * The +/- callbacks mutate the runtime threshold globals and refresh
 * the value-text widgets in place. Sensor task picks up the new
 * values automatically on its next cycle. */

#define THR_ROW_Y(n)   (40 + 50*(n))     /* row spacing */

/* Value text widgets (also act as the chain head for each row). */
Canvas(g_sThreshPowerText, g_psPanels + 3, 0, 0, &g_sKentec320x240x16_SSD2119,
       110, THR_ROW_Y(0), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrWhite, &g_sFontCm22, "150 W", 0, 0);

Canvas(g_sThreshAccelText, g_psPanels + 3, &g_sThreshPowerText, 0,
       &g_sKentec320x240x16_SSD2119,
       110, THR_ROW_Y(1), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrWhite, &g_sFontCm22, "2.0 g", 0, 0);

Canvas(g_sThreshDistVal, g_psPanels + 3, &g_sThreshAccelText, 0,
       &g_sKentec320x240x16_SSD2119,
       110, THR_ROW_Y(2), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrWhite, &g_sFontCm22, "200 mm", 0, 0);

/* Row labels. */
Canvas(g_sThreshPowerLabel, g_psPanels + 3, &g_sThreshDistVal, 0,
       &g_sKentec320x240x16_SSD2119,
       10, THR_ROW_Y(0), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrCyan, &g_sFontCm18, "Power", 0, 0);

Canvas(g_sThreshAccelLabel, g_psPanels + 3, &g_sThreshPowerLabel, 0,
       &g_sKentec320x240x16_SSD2119,
       10, THR_ROW_Y(1), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm18, "Accel", 0, 0);

Canvas(g_sThreshDistLabel, g_psPanels + 3, &g_sThreshAccelLabel, 0,
       &g_sKentec320x240x16_SSD2119,
       10, THR_ROW_Y(2), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrLimeGreen, &g_sFontCm18, "Distance", 0, 0);

/* +/- buttons (40 px wide each, 32 px tall, right-hand side). */
RectangularButton(g_sThreshPowerDec, g_psPanels + 3, &g_sThreshDistLabel, 0,
                  &g_sKentec320x240x16_SSD2119,
                  215, THR_ROW_Y(0)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkRed, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "-", 0, 0, 0, 0, OnPowerDec);
RectangularButton(g_sThreshPowerInc, g_psPanels + 3, &g_sThreshPowerDec, 0,
                  &g_sKentec320x240x16_SSD2119,
                  265, THR_ROW_Y(0)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGreen, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "+", 0, 0, 0, 0, OnPowerInc);

RectangularButton(g_sThreshAccelDec, g_psPanels + 3, &g_sThreshPowerInc, 0,
                  &g_sKentec320x240x16_SSD2119,
                  215, THR_ROW_Y(1)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkRed, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "-", 0, 0, 0, 0, OnAccelDec);
RectangularButton(g_sThreshAccelInc, g_psPanels + 3, &g_sThreshAccelDec, 0,
                  &g_sKentec320x240x16_SSD2119,
                  265, THR_ROW_Y(1)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGreen, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "+", 0, 0, 0, 0, OnAccelInc);

RectangularButton(g_sThreshDistDec, g_psPanels + 3, &g_sThreshAccelInc, 0,
                  &g_sKentec320x240x16_SSD2119,
                  215, THR_ROW_Y(2)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkRed, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "-", 0, 0, 0, 0, OnDistDec);
RectangularButton(g_sThreshDistInc, g_psPanels + 3, &g_sThreshDistDec, 0,
                  &g_sKentec320x240x16_SSD2119,
                  265, THR_ROW_Y(2)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGreen, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "+", 0, 0, 0, 0, OnDistInc);

/* g_sThreshDistInc is the panel's first_child (highest in the
 * sibling chain — see CanvasStruct entry for panel 3 above). */

/*-----------------------------------------------------------*/
/* Title-bar tabs. */

RectangularButton(g_sPrevBtn, 0, 0, 0, &g_sKentec320x240x16_SSD2119,
                  2, 2, 60, 22,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkBlue, ClrSteelBlue, ClrGray, ClrWhite,
                  &g_sFontCm16, "Prev", 0, 0, 0, 0, OnPrevTab);

Canvas(g_sTitleText, 0, 0, 0, &g_sKentec320x240x16_SSD2119,
       64, 2, 192, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE,
       ClrDarkBlue, ClrBlack, ClrWhite, &g_sFontCm18, "Control", 0, 0);

RectangularButton(g_sNextBtn, 0, 0, 0, &g_sKentec320x240x16_SSD2119,
                  258, 2, 60, 22,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkBlue, ClrSteelBlue, ClrGray, ClrWhite,
                  &g_sFontCm16, "Next", 0, 0, 0, 0, OnNextTab);

/*-----------------------------------------------------------*/
/* Tab callbacks — deferred so the widget tree isn't mutated inside
 * the touch handler. */

static void OnPrevTab(tWidget *psWidget)
{
    (void)psWidget;
    if (g_ui32Panel > 0) g_pendingPanel = (int32_t)g_ui32Panel - 1;
}

static void OnNextTab(tWidget *psWidget)
{
    (void)psWidget;
    if (g_ui32Panel < NUM_PANELS - 1) g_pendingPanel = (int32_t)g_ui32Panel + 1;
}

/* Force the next pass of prvRedrawWidgets to repaint every widget
 * (otherwise the trackers might think nothing has changed and skip
 * the redraw over the freshly cleared panel). Set on tab swaps. */
static volatile bool g_force_refresh = false;

static void prvSwapPanel(uint32_t idx)
{
    WidgetRemove((tWidget *)(g_psPanels + g_ui32Panel));
    g_ui32Panel = idx;
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    if (idx == 1) g_prev_count = 0;
    if (idx == 0 || idx == 2) g_force_refresh = true;
    WidgetPaint((tWidget *)(g_psPanels + g_ui32Panel));
    CanvasTextSet(&g_sTitleText, (char *)g_pcPanelNames[g_ui32Panel]);
    WidgetPaint((tWidget *)&g_sTitleText);

    /* Render the current threshold values on entry to the tab. The
     * +/- callbacks repaint them in-place, but the first display
     * needs a one-shot redraw. */
    if (idx == 3)
    {
        prvRefreshPowerLabel();
        prvRefreshAccelLabel();
        prvRefreshDistLabel();
    }

    /* Returning to Control: re-render the summary line so it shows
     * the current (possibly edited) threshold values. */
    if (idx == 0)
        prvRefreshControlThreshLine();
}

/*-----------------------------------------------------------*/
/* Button callbacks — send signals to the motor task. */

static void OnStartPressed(tWidget *psWidget)
{
    (void)psWidget;
    xEventGroupSetBits(xSystemEvents, EVT_USER_START);
}

static void OnStopPressed(tWidget *psWidget)
{
    (void)psWidget;
    xEventGroupSetBits(xSystemEvents, EVT_USER_STOP);
}

static void OnEStopAckPressed(tWidget *psWidget)
{
    (void)psWidget;
    xEventGroupSetBits(xSystemEvents, EVT_USER_ESTOP_ACK);
}

static void OnSpeedSliderChange(tWidget *psWidget, int32_t i32Value)
{
    (void)psWidget;
    /* Slider 0..100 % -> 0..4000 RPM. The motor task picks up the
     * new desired RPM via the command queue and ramps to it. */
    int32_t rpm = (i32Value * 4000) / 100;
    xQueueSend(xCommandQueue, &rpm, 0);

    static char buf[20];
    usprintf(buf, "Speed %d%%", i32Value);
    SliderTextSet(&g_sSpeedSlider, buf);
    WidgetPaint((tWidget *)&g_sSpeedSlider);
}

/*-----------------------------------------------------------*/
/* Threshold-adjustment buttons. Each callback clamps the runtime
 * global, repaints its value widget, and pings the sensor task via
 * an event-group bit so it can re-read promptly (not strictly
 * required — sensor task re-reads every cycle — but documents the
 * dataflow per the assignment event-group spec). */

static char s_thresh_power_buf[16];
static char s_thresh_accel_buf[16];
static char s_thresh_dist_buf [16];

static void prvRefreshPowerLabel(void)
{
    usprintf(s_thresh_power_buf, "%d W", (int)g_thresh_power_w);
    CanvasTextSet(&g_sThreshPowerText, s_thresh_power_buf);
    WidgetPaint((tWidget *)&g_sThreshPowerText);
}
static void prvRefreshAccelLabel(void)
{
    int g10 = (int)(g_thresh_accel_g * 10.0f + 0.5f);
    usprintf(s_thresh_accel_buf, "%d.%d g", g10 / 10, g10 % 10);
    CanvasTextSet(&g_sThreshAccelText, s_thresh_accel_buf);
    WidgetPaint((tWidget *)&g_sThreshAccelText);
}
static void prvRefreshDistLabel(void)
{
    usprintf(s_thresh_dist_buf, "%d mm", (int)g_thresh_distance_mm);
    CanvasTextSet(&g_sThreshDistVal, s_thresh_dist_buf);
    WidgetPaint((tWidget *)&g_sThreshDistVal);
}

/* Summary line on the Control tab — mirrors the runtime thresholds
 * so the user can see them without leaving Control. */
static char s_control_thresh_buf[40];
static void prvRefreshControlThreshLine(void)
{
    int g10 = (int)(g_thresh_accel_g * 10.0f + 0.5f);
    usprintf(s_control_thresh_buf, "Power %dW  Accel %d.%dg  Dist %dmm",
             (int)g_thresh_power_w,
             g10 / 10, g10 % 10,
             (int)g_thresh_distance_mm);
    CanvasTextSet(&g_sPowerLimitText, s_control_thresh_buf);
    /* Only paint if Control is the active tab — otherwise the paint
     * would land on whatever panel is currently mounted. */
    if (g_ui32Panel == 0)
        WidgetPaint((tWidget *)&g_sPowerLimitText);
}

static void OnPowerInc(tWidget *w) { (void)w;
    g_thresh_power_w += 10.0f;
    if (g_thresh_power_w > 300.0f) g_thresh_power_w = 300.0f;
    prvRefreshPowerLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnPowerDec(tWidget *w) { (void)w;
    g_thresh_power_w -= 10.0f;
    if (g_thresh_power_w < 50.0f) g_thresh_power_w = 50.0f;
    prvRefreshPowerLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnAccelInc(tWidget *w) { (void)w;
    g_thresh_accel_g += 0.1f;
    if (g_thresh_accel_g > 4.0f) g_thresh_accel_g = 4.0f;
    prvRefreshAccelLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnAccelDec(tWidget *w) { (void)w;
    g_thresh_accel_g -= 0.1f;
    if (g_thresh_accel_g < 0.5f) g_thresh_accel_g = 0.5f;
    prvRefreshAccelLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnDistInc(tWidget *w) { (void)w;
    g_thresh_distance_mm += 50.0f;
    if (g_thresh_distance_mm > 1000.0f) g_thresh_distance_mm = 1000.0f;
    prvRefreshDistLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnDistDec(tWidget *w) { (void)w;
    g_thresh_distance_mm -= 50.0f;
    if (g_thresh_distance_mm < 50.0f) g_thresh_distance_mm = 50.0f;
    prvRefreshDistLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}

/*-----------------------------------------------------------*/
/* Widget update. Called from the GUI task whenever a model value
 * has changed. Only redraws widgets whose value actually moved, and
 * only while their page is on screen. */

static void prvRedrawWidgets(void)
{
    /* Cache of last drawn values — invalidated on tab swap so the new
     * panel paints over the cleared area. */
    static MotorState_t last_state    = (MotorState_t)-1;
    static int32_t      last_rpm      = -1;
    static int32_t      last_pwr_int  = -1;
    static bool         last_is_night = false;
    static int          last_lux_int  = -10000;
    static uint32_t     last_sec      = 0xFFFFFFFFu;
    static int          last_ax_100   = -100000;
    static int          last_ay_100   = -100000;
    static int          last_az_100   = -100000;
    static int          last_temp_10  = -100000;
    static int          last_hum_10   = -100000;
    static int          last_pres_10  = -100000;

    /* Plots tab repaints itself via OnPlotCanvasPaint. */
    if (g_ui32Panel == 1) return;

    if (g_force_refresh)
    {
        last_state    = (MotorState_t)-1;
        last_rpm      = -1;
        last_pwr_int  = -1;
        last_is_night = !last_is_night;
        last_lux_int  = -10000;
        last_sec      = 0xFFFFFFFFu;
        last_ax_100   = -100000;
        last_ay_100   = -100000;
        last_az_100   = -100000;
        last_temp_10  = -100000;
        last_hum_10   = -100000;
        last_pres_10  = -100000;
        g_force_refresh = false;
    }

    char buf[24];

    if (g_ui32Panel == 0)
    {
        /* ---- Control tab: motor + day/night + clock. ---- */

        if (g_state != last_state)
        {
            const char *name = "Idle";
            uint32_t    col  = ClrOrange;
            switch (g_state)
            {
            case MOTOR_STATE_IDLE:          name = "Idle";          col = ClrOrange;    break;
            case MOTOR_STATE_STARTING:      name = "Starting";      col = ClrOrange;    break;
            case MOTOR_STATE_RUNNING:       name = "Running";       col = ClrLimeGreen; break;
            case MOTOR_STATE_ESTOP_BRAKING: name = "E-Stop Brake";  col = ClrRed;       break;
            case MOTOR_STATE_FAULT_LATCHED: name = "Fault Latched"; col = ClrRed;       break;
            }
            CanvasTextSet(&g_sStateText, (char *)name);
            WidgetPaint((tWidget *)&g_sStateText);
            g_sStatusIndicator.ui32FillColor = col;
            WidgetPaint((tWidget *)&g_sStatusIndicator);
            last_state = g_state;
        }

        if (g_rpm_actual != last_rpm)
        {
            usprintf(buf, "%d RPM", (int)g_rpm_actual);
            CanvasTextSet(&g_sRpmText, buf);
            WidgetPaint((tWidget *)&g_sRpmText);
            last_rpm = g_rpm_actual;
        }

        int p = (int)(g_power_w + 0.5f);
        int diff = (p > last_pwr_int) ? (p - last_pwr_int) : (last_pwr_int - p);
        if (last_pwr_int < 0 || diff >= 2)
        {
            usprintf(buf, "%d W", p);
            CanvasTextSet(&g_sPowerText, buf);
            WidgetPaint((tWidget *)&g_sPowerText);
            last_pwr_int = p;
        }

        /* Day/night hysteresis. Spec uses < 5 lux; we use a tiny
         * hysteresis around it so the LED doesn't flicker right at
         * the threshold. Define DAYNIGHT_DESK_TEST to 1 if you need
         * the relaxed indoor-testing thresholds back. */
        #ifdef DAYNIGHT_DESK_TEST
        const float kNight = 100.0f, kDay = 220.0f;
        #else
        const float kNight = (float)NIGHT_LIGHT_LUX;       /* 5 lux */
        const float kDay   = (float)NIGHT_LIGHT_LUX * 2.0f;/* 10 lux */
        #endif
        bool is_night = last_is_night;
        if (g_light_lux < kNight)    is_night = true;
        else if (g_light_lux > kDay) is_night = false;
        if (is_night != last_is_night)
        {
            CanvasTextSet(&g_sDayNightText, is_night ? "Night" : "Day");
            WidgetPaint((tWidget *)&g_sDayNightText);
            CanvasImageSet(&g_sDayNightLed, is_night ? g_pui8LightOn : g_pui8LightOff);
            WidgetPaint((tWidget *)&g_sDayNightLed);
            last_is_night = is_night;
        }

        uint32_t sec = xTaskGetTickCount() / configTICK_RATE_HZ;
        if (sec != last_sec)
        {
            uint32_t h = (sec / 3600) % 24;
            uint32_t m = (sec / 60) % 60;
            uint32_t s =  sec % 60;
            usprintf(buf, "%02d:%02d:%02d", h, m, s);
            CanvasTextSet(&g_sClockText, buf);
            WidgetPaint((tWidget *)&g_sClockText);
            last_sec = sec;
        }
    }
    else if (g_ui32Panel == 2)
    {
        /* ---- Sensors tab: raw scalar readouts. ---- *
         * Each widget needs its OWN persistent buffer — CanvasTextSet
         * stores the pointer, not the string. */
        static char s_lux[24], s_ax[16], s_ay[16], s_az[16];
        static char s_temp[24], s_hum[24], s_pres[24];

        int lux_i = (int)(g_light_lux + 0.5f);
        if (lux_i != last_lux_int)
        {
            usprintf(s_lux, "Lux:   %d", lux_i);
            CanvasTextSet(&g_sSensLuxText, s_lux);
            WidgetPaint((tWidget *)&g_sSensLuxText);
            last_lux_int = lux_i;
        }

        #define UI_R100(v) ((int)((v) >= 0 ? (v)*100.0f + 0.5f : (v)*100.0f - 0.5f))
        int ax100 = UI_R100(g_accel_x_g);
        int ay100 = UI_R100(g_accel_y_g);
        int az100 = UI_R100(g_accel_z_g);

        if (ax100 != last_ax_100)
        {
            int w = ax100 / 100, f = ax100 % 100; if (f < 0) f = -f;
            const char *s = (ax100 < 0 && w == 0) ? "-" : "";
            usprintf(s_ax, "X:%s%d.%02d", s, w, f);
            CanvasTextSet(&g_sSensAxText, s_ax);
            WidgetPaint((tWidget *)&g_sSensAxText);
            last_ax_100 = ax100;
        }
        if (ay100 != last_ay_100)
        {
            int w = ay100 / 100, f = ay100 % 100; if (f < 0) f = -f;
            const char *s = (ay100 < 0 && w == 0) ? "-" : "";
            usprintf(s_ay, "Y:%s%d.%02d", s, w, f);
            CanvasTextSet(&g_sSensAyText, s_ay);
            WidgetPaint((tWidget *)&g_sSensAyText);
            last_ay_100 = ay100;
        }
        if (az100 != last_az_100)
        {
            int w = az100 / 100, f = az100 % 100; if (f < 0) f = -f;
            const char *s = (az100 < 0 && w == 0) ? "-" : "";
            usprintf(s_az, "Z:%s%d.%02d", s, w, f);
            CanvasTextSet(&g_sSensAzText, s_az);
            WidgetPaint((tWidget *)&g_sSensAzText);
            last_az_100 = az100;
        }
        #undef UI_R100

        int temp_10 = (int)(g_temp_c       * 10.0f + 0.5f);
        int hum_10  = (int)(g_humidity_pct * 10.0f + 0.5f);
        int pres_10 = (int)(g_pressure_hpa * 10.0f + 0.5f);
        if (temp_10 != last_temp_10)
        {
            if (g_bme_ok) usprintf(s_temp, "Temp:  %d.%d C",
                                   temp_10 / 10, temp_10 % 10);
            else          usprintf(s_temp, "Temp:  --- C");
            CanvasTextSet(&g_sSensTempText, s_temp);
            WidgetPaint((tWidget *)&g_sSensTempText);
            last_temp_10 = temp_10;
        }
        if (hum_10 != last_hum_10)
        {
            if (g_bme_ok) usprintf(s_hum, "Hum:   %d.%d %%",
                                   hum_10 / 10, hum_10 % 10);
            else          usprintf(s_hum, "Hum:   --- %%");
            CanvasTextSet(&g_sSensHumText, s_hum);
            WidgetPaint((tWidget *)&g_sSensHumText);
            last_hum_10 = hum_10;
        }
        if (pres_10 != last_pres_10)
        {
            if (g_bme_ok) usprintf(s_pres, "Press: %d.%d hPa",
                                   pres_10 / 10, pres_10 % 10);
            else          usprintf(s_pres, "Press: --- hPa");
            CanvasTextSet(&g_sSensPresText, s_pres);
            WidgetPaint((tWidget *)&g_sSensPresText);
            last_pres_10 = pres_10;
        }
    }
}

/*-----------------------------------------------------------*/

static void prvGuiTask(void *pvParameters)
{
    (void)pvParameters;
    tContext sContext;
    MotorMsgObj  motor_msg;
    SensorMsgObj sensor_msg;

    /* The FPU was enabled in main() but ensure it stays enabled
     * inside this task too — grlib does float math. */
    FPUEnable();
    FPULazyStackingEnable();

    /* uDMA for the LCD. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlDelay(10);
    uDMAControlBaseSet(&s_DMAControlTable[0]);
    uDMAEnable();

    /* LCD + grlib + touch. */
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);
    TouchScreenInit(g_ui32SysClock);
    TouchScreenCallbackSet(WidgetPointerMessage);

    /* Initial widget tree. */
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sPrevBtn);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sTitleText);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sNextBtn);
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ui32Panel));
    WidgetPaint(WIDGET_ROOT);

    TickType_t last_sample_tick = 0;
    TickType_t last_plot_paint  = 0;

    for (;;)
    {
        /* 1. Process widget message queue (touches + paints). */
        WidgetMessageQueueProcess();

        /* 2. Handle a deferred tab switch. */
        if (g_pendingPanel >= 0)
        {
            uint32_t target = (uint32_t)g_pendingPanel;
            g_pendingPanel = -1;
            prvSwapPanel(target);
        }

        /* 3. Drain the queues — keep only the latest of each.
         *    Motor task supplies state + RPM + power. Sensor task
         *    supplies lux. */
        while (xQueueReceive(xMotorQueue, &motor_msg, 0) == pdPASS)
        {
            g_state      = motor_msg.state;
            g_rpm_actual = motor_msg.rpm_actual;
            g_pwm_duty   = motor_msg.pwm_duty;
            /* Motor task supplies simulated power until the DRV8323
             * is wired up and the sensor-task ADC pipeline goes live;
             * switch this back to sensor_msg.power_watts then. */
            g_power_w    = motor_msg.power_watts;
        }
        while (xQueueReceive(xSensorQueue, &sensor_msg, 0) == pdPASS)
        {
            g_light_lux    = sensor_msg.light_lux;
            g_accel_x_g    = sensor_msg.accel_x_g;
            g_accel_y_g    = sensor_msg.accel_y_g;
            g_accel_z_g    = sensor_msg.accel_z_g;
            g_accel_g      = sensor_msg.accel_y_g;   /* plot uses Y, signed */
            g_temp_c       = sensor_msg.temp_c;
            g_humidity_pct = sensor_msg.humidity_pct;
            g_pressure_hpa = sensor_msg.pressure_hpa;
            g_bme_ok       = sensor_msg.bme_ok;
        }

        /* 4. Refresh whichever readout tab is currently showing. */
        prvRedrawWidgets();

        /* 5. Push a plot sample at 5 Hz regardless of which tab is
         *    showing, so the trace is fresh when the user switches. */
        TickType_t now = xTaskGetTickCount();
        if (now - last_sample_tick >= pdMS_TO_TICKS(200))
        {
            last_sample_tick = now;
            prvPlotPush((float)g_rpm_actual, g_power_w, g_light_lux, g_accel_g);
        }

        /* 6. Repaint the plot at ~5 Hz only while the Plots tab is up. */
        if (g_ui32Panel == 1 && (now - last_plot_paint >= pdMS_TO_TICKS(200)))
        {
            last_plot_paint = now;
            WidgetPaint((tWidget *)&g_sPlotCanvas);
        }

        /* Small task delay so we don't peg the CPU. */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/*-----------------------------------------------------------*/

void vCreateGuiTask(void)
{
    xTaskCreate(prvGuiTask, "GUI",
                configMINIMAL_STACK_SIZE * 8, NULL,
                tskIDLE_PRIORITY + 2, NULL);
}
