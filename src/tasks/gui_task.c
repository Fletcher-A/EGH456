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
 *   Page 0 — Control: state text, status indicator (green/orange/yellow/red),
 *                     RPM/power/lux
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
#if MOTOR_ENABLE_POWER_SENSOR
#include "drivers/power_sensor.h"
#endif
#include "drivers/speed_sensor.h"
#include "utils/uart_log.h"
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
static int32_t      g_rpm_reference   = 0;
static int32_t      g_rpm_desired     = 0;
static uint16_t     g_pwm_duty        = 0;
static float        g_power_w         = 0.0f;
static EventBits_t  g_fault_bits      = 0;
static uint8_t      g_hall_state      = 0;
static bool         g_motor_ready     = false;
static float        g_light_lux       = 0.0f;
static float        g_accel_g         = 0.0f;   /* filtered Y, used by plot */
static float        g_accel_x_g       = 0.0f;
static float        g_accel_y_g       = 0.0f;
static float        g_accel_z_g       = 0.0f;
static float        g_accel_total_g   = 0.0f;   /* |ax|+|ay|+|az| for status LED */
static float        g_temp_c          = 0.0f;
static float        g_humidity_pct    = 0.0f;
static float        g_pressure_hpa    = 0.0f;
static bool         g_bme_ok          = false;
static bool         g_sht_ok          = false;

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
static void OnToggleLux (tWidget *psWidget);
static void OnToggleAcc (tWidget *psWidget);
static void OnToggleTemp(tWidget *psWidget);
static void OnToggleHum (tWidget *psWidget);
static void OnTogglePres(tWidget *psWidget);
static void OnToggleRpm (tWidget *psWidget);
static void OnTogglePow (tWidget *psWidget);
static void OnPowerInc(tWidget *psWidget);
static void OnPowerDec(tWidget *psWidget);
static void OnAccelInc(tWidget *psWidget);
static void OnAccelDec(tWidget *psWidget);
static void OnNightInc (tWidget *psWidget);
static void OnNightDec (tWidget *psWidget);
static void OnCoolInc  (tWidget *psWidget);
static void OnCoolDec  (tWidget *psWidget);
static void OnAccToggle(tWidget *psWidget);
static void OnVDistInc(tWidget *psWidget);
static void OnVDistDec(tWidget *psWidget);
static void prvRefreshPowerLabel(void);
static void prvRefreshAccelLabel(void);
static void prvRefreshNightLabel(void);
static void prvRefreshCoolLabel(void);
static void prvRefreshControlThreshLine(void);
static void prvRefreshVDistLabel(void);
static void prvRepaintClockDate(void);
static void prvProcessTouchMessages(void);

extern tCanvasWidget g_psPanels[];
extern tCanvasWidget g_sPlotCanvas;
extern tPushButtonWidget g_sBtnTogPow;     /* last child in Sensors panel chain */
extern tPushButtonWidget g_sThreshCoolInc;  /* last child in Thresholds panel chain */

/*-----------------------------------------------------------*/
/* Control-panel widgets. */

/* Inset from 320 px wide LCD so the slider track clears the bezel. */
#define CTRL_SLIDER_W   180
#define CTRL_SLIDER_X   ((320 - CTRL_SLIDER_W) / 2)   /* centred, ~70 px bezel each side */

Canvas(g_sStateText, g_psPanels, 0, 0, &g_sKentec320x240x16_SSD2119,
       10, 30, 180, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrSilver, &g_sFontCm22, "Idle", 0, 0);

Canvas(g_sStatusIndicator, g_psPanels, &g_sStateText, 0,
       &g_sKentec320x240x16_SSD2119,
       250, 30, 60, 24,
       CANVAS_STYLE_FILL | CANVAS_STYLE_OUTLINE,
       ClrOrange, ClrGray, 0, 0, 0, 0, 0);

/* Width stops before the clock column (x=258) — a full-width repaint was
 * erasing the clock every time RPM updated while the motor was running. */
Canvas(g_sRpmText, g_psPanels, &g_sStatusIndicator, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 60, 242, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrSilver, &g_sFontCm14, "0/0/0 RPM", 0, 0);

Canvas(g_sPowerText, g_psPanels, &g_sRpmText, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 82, 168, 20,
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

/* Cooling indicator: "Cool" label + a coloured LED-style rectangle.
 * Fill colour is updated each redraw based on temp vs g_thresh_cool_c
 * (cyan = cooling active, dark grey = off). */
Canvas(g_sCoolText, g_psPanels, &g_sDayNightLed, 0,
       &g_sKentec320x240x16_SSD2119,
       180, 104, 90, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrSilver, &g_sFontCm18, "Cooling", 0, 0);

Canvas(g_sCoolLed, g_psPanels, &g_sCoolText, 0,
       &g_sKentec320x240x16_SSD2119,
       280, 104, 20, 20,
       CANVAS_STYLE_IMG, 0, 0, 0, 0, 0, g_pui8LightOff, 0);

/* Lux + accel readouts now live on the Sensors tab (panel 2). */

Canvas(g_sPowerLimitText, g_psPanels, &g_sCoolLed, 0,
       &g_sKentec320x240x16_SSD2119,
       10, 128, 300, 18,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrGray, &g_sFontCm14,
       "Pwr/Acc/Nt/Cool thresholds on Thresholds tab", 0, 0);

Canvas(g_sClockText, g_psPanels, &g_sPowerLimitText, 0,
       &g_sKentec320x240x16_SSD2119,
       258, 60, 62, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_OPAQUE,
       ClrBlack, 0, ClrWhite, &g_sFontCm18, "00:00:00", 0, 0);

tSliderWidget g_sSpeedSlider =
    SliderStruct(g_psPanels, &g_sClockText, 0,
                 &g_sKentec320x240x16_SSD2119,
                 CTRL_SLIDER_X, 150, CTRL_SLIDER_W, 24, 0, MAX_MOTOR_RPM, 0,
                 (SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE |
                  SL_STYLE_TEXT | SL_STYLE_BACKG_TEXT),
                 ClrBlue, ClrBlack, ClrSilver, ClrWhite, ClrWhite,
                 &g_sFontCm16, "0 RPM", 0, 0, OnSpeedSliderChange);

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

/* Date readout: under clock (right column). ACC button is separate — do not
 * overlap (220,82) or the opaque date canvas hides the button. */
Canvas(g_sDateText, g_psPanels, &g_sAckBtn, 0, &g_sKentec320x240x16_SSD2119,
       258, 82, 62, 20,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_OPAQUE,
       ClrBlack, 0, ClrSilver, &g_sFontCm14, "2026-05-26", 0, 0);

/* ACC toggle: left of clock/date column; last in widget chain = on top for touch. */
RectangularButton(g_sAccBtn, g_psPanels, &g_sDateText, 0,
                  &g_sKentec320x240x16_SSD2119,
                  182, 82, 72, 20,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGray, ClrBlack, ClrGray, ClrSilver,
                  &g_sFontCm14, "ACC OFF", 0, 0, 0, 0, OnAccToggle);

/*-----------------------------------------------------------*/
/* Panels and tabs. */

tCanvasWidget g_psPanels[] =
{
    /* Page 0 — Control. */
    CanvasStruct(0, 0, &g_sAccBtn, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 216, CANVAS_STYLE_FILL,
                 ClrBlack, 0, 0, 0, 0, 0, 0),

    /* Page 1 — Plots (child = g_sPlotCanvas). */
    CanvasStruct(0, 0, &g_sPlotCanvas, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 216, CANVAS_STYLE_FILL,
                 ClrBlack, 0, 0, 0, 0, 0, 0),

    /* Page 2 — Sensors (scalar readouts + per-trace plot toggles).
     * first_child points to the LAST widget declared in source order
     * (g_sBtnTogPow); siblings chain backwards via their .next fields. */
    CanvasStruct(0, 0, &g_sBtnTogPow, &g_sKentec320x240x16_SSD2119,
                 0, 24, 320, 216, CANVAS_STYLE_FILL,
                 ClrBlack, 0, 0, 0, 0, 0, 0),

    /* Page 3 — Thresholds (runtime-editable safety limits). */
    CanvasStruct(0, 0, &g_sThreshCoolInc, &g_sKentec320x240x16_SSD2119,
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

/* Screen coords; leave left/bottom margin for axis titles (canvas y=30, h=200). */
#define PLOT_X        26
#define PLOT_Y        34
#define PLOT_W        282
#define PLOT_H        178
#define PLOT_WINDOW_S 5.0f        /* 25 samples x 200 ms sample period */
#define PLOT_SAMPLES  25
#define PLOT_RPM_MAX    ((float)MAX_MOTOR_RPM)
#define PLOT_POWER_MAX   350.0f
#define PLOT_LUX_MAX     500.0f
#define PLOT_ACCEL_MAX     2.0f       /* +-2 g full scale, centred on plot */
#define PLOT_TEMP_MIN     10.0f       /* deg C baseline (subtracted before plotting) */
#define PLOT_TEMP_SPAN    20.0f       /* 10..30 deg C visible range */
#define PLOT_HUM_MAX     100.0f       /* 0..100 %RH */
#define PLOT_PRESS_MIN  1000.0f       /* hPa baseline (subtracted before plotting) */
#define PLOT_PRESS_SPAN   25.0f       /* 1000..1025 hPa visible range */

static float    g_plot_rpm  [PLOT_SAMPLES];
static float    g_plot_power[PLOT_SAMPLES];
static float    g_plot_lux  [PLOT_SAMPLES];
static float    g_plot_accel[PLOT_SAMPLES];
static float    g_plot_temp [PLOT_SAMPLES];
static float    g_plot_hum  [PLOT_SAMPLES];
static float    g_plot_press[PLOT_SAMPLES];
static float    g_rpm_flat  [PLOT_SAMPLES];
static float    g_pow_flat  [PLOT_SAMPLES];
static float    g_lux_flat  [PLOT_SAMPLES];
static float    g_acc_flat  [PLOT_SAMPLES];
static float    g_temp_flat [PLOT_SAMPLES];
static float    g_hum_flat  [PLOT_SAMPLES];
static float    g_press_flat[PLOT_SAMPLES];
static float    g_prev_rpm  [PLOT_SAMPLES];
static float    g_prev_pow  [PLOT_SAMPLES];
static float    g_prev_lux  [PLOT_SAMPLES];
static float    g_prev_acc  [PLOT_SAMPLES];
static float    g_prev_temp [PLOT_SAMPLES];
static float    g_prev_hum  [PLOT_SAMPLES];
static float    g_prev_press[PLOT_SAMPLES];
static uint32_t g_prev_count = 0;
static uint32_t g_plot_head  = 0;
static uint32_t g_plot_count = 0;

/* Per-trace enable, controlled from the Sensors tab toggle buttons.
 * Order: 0=RPM, 1=Power, 2=Lux, 3=Accel, 4=Temp, 5=Humidity, 6=Pressure. */
enum {
    TRACE_RPM = 0, TRACE_POW, TRACE_LUX, TRACE_ACC,
    TRACE_TEMP, TRACE_HUM, TRACE_PRES, NUM_TRACES
};
static volatile bool g_trace_enabled[NUM_TRACES] = {
    true, true, true, true, true, true, true
};
/* True if the trace was actually drawn last frame, so the next paint
 * knows whether to erase its old pixels in black. */
static bool g_prev_drawn[NUM_TRACES] = { false };

static void prvPlotPush(float rpm, float power, float lux, float accel,
                        float temp_c, float hum_pct, float press_hpa)
{
    g_plot_rpm  [g_plot_head] = rpm;
    g_plot_power[g_plot_head] = power;
    g_plot_lux  [g_plot_head] = lux;
    g_plot_accel[g_plot_head] = accel;
    /* Offset before storing so the 10..30 deg C range maps onto 0..span. */
    g_plot_temp [g_plot_head] = temp_c - PLOT_TEMP_MIN;
    g_plot_hum  [g_plot_head] = hum_pct;
    /* Subtract baseline so the 950..1050 hPa range maps onto 0..span. */
    g_plot_press[g_plot_head] = press_hpa - PLOT_PRESS_MIN;
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

static void prvDrawPlotAxesAndTitles(tContext *psContext)
{
    const int yAxisX = PLOT_Y + PLOT_H;

    GrContextForegroundSet(psContext, ClrSilver);
    GrLineDrawV(psContext, PLOT_X, PLOT_Y, yAxisX);
    GrLineDrawH(psContext, PLOT_X, PLOT_X + PLOT_W, yAxisX);

    /* No single Y-axis label: each trace has its own colour-coded
     * top-row max and bottom-row min in the plot legend. */

    /* X-axis title and rolling time labels (25 samples @ 5 Hz -> 5 s
     * window). Right tick = current clock time, left tick = now-5s. */
    const int yXLabel = yAxisX + 8;
    GrContextFontSet(psContext, &g_sFontCm14);
    GrStringDrawCentered(psContext, "Time", -1,
                         PLOT_X + (PLOT_W / 2), yXLabel, 1);
    GrContextFontSet(psContext, &g_sFontCm12);

    uint32_t now  = xTaskGetTickCount() / configTICK_RATE_HZ;
    uint32_t past = (now >= (uint32_t)PLOT_WINDOW_S) ? (now - (uint32_t)PLOT_WINDOW_S) : 0;
    char lbl_left[12], lbl_right[12];
    usprintf(lbl_left,  "%02d:%02d:%02d",
             (int)((past / 3600) % 24), (int)((past / 60) % 60), (int)(past % 60));
    usprintf(lbl_right, "%02d:%02d:%02d",
             (int)((now  / 3600) % 24), (int)((now  / 60) % 60), (int)(now  % 60));
    GrStringDraw(psContext, lbl_left,  -1, PLOT_X, yXLabel, 1);
    GrStringDraw(psContext, lbl_right, -1, PLOT_X + PLOT_W - 48, yXLabel, 1);
}

static void OnPlotCanvasPaint(tWidget *psWidget, tContext *psContext)
{
    (void)psWidget;

    prvDrawPlotAxesAndTitles(psContext);

    /* Erase previous frame in black — but only the traces that were
     * actually drawn last frame. Skipping disabled traces avoids
     * drawing spurious zero-lines along the axis. */
    if (g_prev_count >= 2)
    {
        if (g_prev_drawn[TRACE_RPM])  prvDrawTrace(psContext, g_prev_rpm,   g_prev_count, PLOT_RPM_MAX,    ClrBlack, false);
        if (g_prev_drawn[TRACE_POW])  prvDrawTrace(psContext, g_prev_pow,   g_prev_count, PLOT_POWER_MAX,  ClrBlack, false);
        if (g_prev_drawn[TRACE_LUX])  prvDrawTrace(psContext, g_prev_lux,   g_prev_count, PLOT_LUX_MAX,    ClrBlack, false);
        if (g_prev_drawn[TRACE_ACC])  prvDrawTrace(psContext, g_prev_acc,   g_prev_count, PLOT_ACCEL_MAX,  ClrBlack, true);
        if (g_prev_drawn[TRACE_TEMP]) prvDrawTrace(psContext, g_prev_temp,  g_prev_count, PLOT_TEMP_SPAN,  ClrBlack, false);
        if (g_prev_drawn[TRACE_HUM])  prvDrawTrace(psContext, g_prev_hum,   g_prev_count, PLOT_HUM_MAX,    ClrBlack, false);
        if (g_prev_drawn[TRACE_PRES]) prvDrawTrace(psContext, g_prev_press, g_prev_count, PLOT_PRESS_SPAN, ClrBlack, false);
    }

    /* Unroll circular buffer. */
    uint32_t start = (g_plot_head + PLOT_SAMPLES - g_plot_count) % PLOT_SAMPLES;
    for (uint32_t i = 0; i < g_plot_count; i++)
    {
        uint32_t idx = (start + i) % PLOT_SAMPLES;
        g_rpm_flat  [i] = g_plot_rpm  [idx];
        g_pow_flat  [i] = g_plot_power[idx];
        g_lux_flat  [i] = g_plot_lux  [idx];
        g_acc_flat  [i] = g_plot_accel[idx];
        g_temp_flat [i] = g_plot_temp [idx];
        g_hum_flat  [i] = g_plot_hum  [idx];
        g_press_flat[i] = g_plot_press[idx];
    }

    /* Draw current frame — only enabled traces. */
    if (g_trace_enabled[TRACE_RPM])  prvDrawTrace(psContext, g_rpm_flat,   g_plot_count, PLOT_RPM_MAX,    ClrGoldenrod, false);
    if (g_trace_enabled[TRACE_POW])  prvDrawTrace(psContext, g_pow_flat,   g_plot_count, PLOT_POWER_MAX,  ClrCyan,      false);
    if (g_trace_enabled[TRACE_LUX])  prvDrawTrace(psContext, g_lux_flat,   g_plot_count, PLOT_LUX_MAX,    ClrWhite,     false);
    if (g_trace_enabled[TRACE_ACC])  prvDrawTrace(psContext, g_acc_flat,   g_plot_count, PLOT_ACCEL_MAX,  ClrMagenta,   true);
    if (g_trace_enabled[TRACE_TEMP]) prvDrawTrace(psContext, g_temp_flat,  g_plot_count, PLOT_TEMP_SPAN,  ClrOrange,    false);
    if (g_trace_enabled[TRACE_HUM])  prvDrawTrace(psContext, g_hum_flat,   g_plot_count, PLOT_HUM_MAX,    ClrTurquoise, false);
    if (g_trace_enabled[TRACE_PRES]) prvDrawTrace(psContext, g_press_flat, g_plot_count, PLOT_PRESS_SPAN, ClrLimeGreen, false);

    /* Snapshot for next erase. Only copy the buffers that were drawn
     * (so we don't try to erase a trace that was never on screen). */
    for (uint32_t i = 0; i < g_plot_count; i++)
    {
        if (g_trace_enabled[TRACE_RPM])  g_prev_rpm  [i] = g_rpm_flat  [i];
        if (g_trace_enabled[TRACE_POW])  g_prev_pow  [i] = g_pow_flat  [i];
        if (g_trace_enabled[TRACE_LUX])  g_prev_lux  [i] = g_lux_flat  [i];
        if (g_trace_enabled[TRACE_ACC])  g_prev_acc  [i] = g_acc_flat  [i];
        if (g_trace_enabled[TRACE_TEMP]) g_prev_temp [i] = g_temp_flat [i];
        if (g_trace_enabled[TRACE_HUM])  g_prev_hum  [i] = g_hum_flat  [i];
        if (g_trace_enabled[TRACE_PRES]) g_prev_press[i] = g_press_flat[i];
    }
    for (int t = 0; t < NUM_TRACES; t++) g_prev_drawn[t] = g_trace_enabled[t];
    g_prev_count = g_plot_count;

    /* Per-trace scale hints (colour-matched); axes titled separately. */
    GrContextFontSet(psContext, &g_sFontCm12);
    GrContextBackgroundSet(psContext, ClrBlack);

    const int yTop = PLOT_Y + 10;
    const int yBot = PLOT_Y + PLOT_H - 18;

    /* Column X offsets within the plot. Tuned to leave a couple of
     * pixels gap between adjacent labels at 12-pt CM. */
    const int xRPM   = PLOT_X + 4;
    const int xPow   = PLOT_X + 50;
    const int xLux   = PLOT_X + 84;
    const int xAcc   = PLOT_X + 122;
    const int xTemp  = PLOT_X + 150;
    const int xHum   = PLOT_X + 180;
    const int xPres  = PLOT_X + 232;

    GrContextForegroundSet(psContext, ClrGoldenrod);
    GrStringDraw(psContext, "4000",     -1, xRPM,  yTop, 1);
    GrStringDraw(psContext, "0",        -1, xRPM,  yBot, 1);

    GrContextForegroundSet(psContext, ClrCyan);
    GrStringDraw(psContext, "350W",     -1, xPow,  yTop, 1);
    GrStringDraw(psContext, "0W",       -1, xPow,  yBot, 1);

    GrContextForegroundSet(psContext, ClrWhite);
    GrStringDraw(psContext, "500lx",    -1, xLux,  yTop, 1);
    GrStringDraw(psContext, "0lx",      -1, xLux,  yBot, 1);

    GrContextForegroundSet(psContext, ClrMagenta);
    GrStringDraw(psContext, "+2g",      -1, xAcc,  yTop, 1);
    GrStringDraw(psContext, "-2g",      -1, xAcc,  yBot, 1);

    GrContextForegroundSet(psContext, ClrOrange);
    GrStringDraw(psContext, "30C",      -1, xTemp, yTop, 1);
    GrStringDraw(psContext, "10C",      -1, xTemp, yBot, 1);

    GrContextForegroundSet(psContext, ClrTurquoise);
    GrStringDraw(psContext, "100%RH",   -1, xHum,  yTop, 1);
    GrStringDraw(psContext, "0%RH",     -1, xHum,  yBot, 1);

    GrContextForegroundSet(psContext, ClrLimeGreen);
    GrStringDraw(psContext, "1025hPa",  -1, xPres, yTop, 1);
    GrStringDraw(psContext, "1000hPa",  -1, xPres, yBot, 1);
}

/*-----------------------------------------------------------*/
/* Page 2 — Sensors panel widgets. Each one has its OWN static text
 * buffer (CanvasTextSet stores the pointer, not the string — sharing
 * one buffer would alias all widgets together). Declared in reverse
 * draw order so each Canvas can use the previous one as its sibling. */

/* Each row: text canvas (full-width value) on the left, a small
 * toggle button on the right that enables / disables the corresponding
 * plot trace. Rows are stacked at SROW_DY vertical spacing.
 *
 *   y=32 + 0*SROW_DY  Lux            [ON]
 *   y=32 + 1*SROW_DY  X / Y / Z      [ON]   (toggle controls accel-Y trace)
 *   y=32 + 2*SROW_DY  Temp           [ON]
 *   y=32 + 3*SROW_DY  Hum            [ON]
 *   y=32 + 4*SROW_DY  Press          [ON]
 *   y=32 + 5*SROW_DY  RPM            [ON]
 *   y=32 + 6*SROW_DY  Power          [ON]
 */
#define SROW_Y0    32
#define SROW_DY    23
#define SROW_TX    8         /* text x */
#define SROW_TW    230       /* text width */
#define SROW_BX    248       /* toggle button x */
#define SROW_BW    62        /* toggle button width */
#define SROW_BH    22        /* toggle button height */

/* Row 0 - Lux. */
Canvas(g_sSensLuxText, g_psPanels + 2, 0, 0, &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 0*SROW_DY, SROW_TW, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrWhite, &g_sFontCm18, "Lux:   ---", 0, 0);

/* Row 1 - X / Y / Z accel (three small canvases sharing one row). */
Canvas(g_sSensAxText, g_psPanels + 2, &g_sSensLuxText, 0,
       &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 1*SROW_DY, 78, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm16, "X:---", 0, 0);

Canvas(g_sSensAyText, g_psPanels + 2, &g_sSensAxText, 0,
       &g_sKentec320x240x16_SSD2119,
       88, SROW_Y0 + 1*SROW_DY, 78, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm16, "Y:---", 0, 0);

Canvas(g_sSensAzText, g_psPanels + 2, &g_sSensAyText, 0,
       &g_sKentec320x240x16_SSD2119,
       168, SROW_Y0 + 1*SROW_DY, 78, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm16, "Z:---", 0, 0);

/* Row 2 - Temp. */
Canvas(g_sSensTempText, g_psPanels + 2, &g_sSensAzText, 0,
       &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 2*SROW_DY, SROW_TW, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrOrange, &g_sFontCm18, "Temp:  --- C", 0, 0);

/* Row 3 - Humidity. */
Canvas(g_sSensHumText, g_psPanels + 2, &g_sSensTempText, 0,
       &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 3*SROW_DY, SROW_TW, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrTurquoise, &g_sFontCm18, "Hum:   --- %", 0, 0);

/* Row 4 - Pressure. */
Canvas(g_sSensPresText, g_psPanels + 2, &g_sSensHumText, 0,
       &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 4*SROW_DY, SROW_TW, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrLimeGreen, &g_sFontCm18, "Press: --- hPa", 0, 0);

/* Row 5 - Motor RPM (new readout on Sensors tab). */
Canvas(g_sSensRpmText, g_psPanels + 2, &g_sSensPresText, 0,
       &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 5*SROW_DY, SROW_TW, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrGoldenrod, &g_sFontCm18, "RPM:   ---", 0, 0);

/* Row 6 - Motor power (new readout on Sensors tab). */
Canvas(g_sSensPowText, g_psPanels + 2, &g_sSensRpmText, 0,
       &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 6*SROW_DY, SROW_TW, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrCyan, &g_sFontCm18, "Power: --- W", 0, 0);

/* Row 7 - Virtual distance (ACC demo input). */
Canvas(g_sSensDistText, g_psPanels + 2, &g_sSensPowText, 0,
       &g_sKentec320x240x16_SSD2119,
       SROW_TX, SROW_Y0 + 7*SROW_DY, SROW_TW, 22,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrWhite, &g_sFontCm18, "Dist:  --- mm", 0, 0);

RectangularButton(g_sVDistDecBtn, g_psPanels + 2, &g_sSensDistText, 0,
                  &g_sKentec320x240x16_SSD2119,
                  248, SROW_Y0 + 7*SROW_DY, 30, 22,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrGray, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "-", 0, 0, 0, 0, OnVDistDec);

RectangularButton(g_sVDistIncBtn, g_psPanels + 2, &g_sVDistDecBtn, 0,
                  &g_sKentec320x240x16_SSD2119,
                  280, SROW_Y0 + 7*SROW_DY, 30, 22,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrGray, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "+", 0, 0, 0, 0, OnVDistInc);

/* Seven plot-toggle buttons. Each toggles g_trace_enabled[t] for its
 * trace; the button's fill colour and text track the state (trace
 * colour + "ON" when enabled, dark grey + "OFF" when disabled). */
RectangularButton(g_sBtnTogLux, g_psPanels + 2, &g_sVDistIncBtn, 0,
                  &g_sKentec320x240x16_SSD2119,
                  SROW_BX, SROW_Y0 + 0*SROW_DY, SROW_BW, SROW_BH,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrWhite, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "ON", 0, 0, 0, 0, OnToggleLux);

RectangularButton(g_sBtnTogAcc, g_psPanels + 2, &g_sBtnTogLux, 0,
                  &g_sKentec320x240x16_SSD2119,
                  SROW_BX, SROW_Y0 + 1*SROW_DY, SROW_BW, SROW_BH,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrMagenta, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "ON", 0, 0, 0, 0, OnToggleAcc);

RectangularButton(g_sBtnTogTemp, g_psPanels + 2, &g_sBtnTogAcc, 0,
                  &g_sKentec320x240x16_SSD2119,
                  SROW_BX, SROW_Y0 + 2*SROW_DY, SROW_BW, SROW_BH,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrOrange, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "ON", 0, 0, 0, 0, OnToggleTemp);

RectangularButton(g_sBtnTogHum, g_psPanels + 2, &g_sBtnTogTemp, 0,
                  &g_sKentec320x240x16_SSD2119,
                  SROW_BX, SROW_Y0 + 3*SROW_DY, SROW_BW, SROW_BH,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrTurquoise, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "ON", 0, 0, 0, 0, OnToggleHum);

RectangularButton(g_sBtnTogPres, g_psPanels + 2, &g_sBtnTogHum, 0,
                  &g_sKentec320x240x16_SSD2119,
                  SROW_BX, SROW_Y0 + 4*SROW_DY, SROW_BW, SROW_BH,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrLimeGreen, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "ON", 0, 0, 0, 0, OnTogglePres);

RectangularButton(g_sBtnTogRpm, g_psPanels + 2, &g_sBtnTogPres, 0,
                  &g_sKentec320x240x16_SSD2119,
                  SROW_BX, SROW_Y0 + 5*SROW_DY, SROW_BW, SROW_BH,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrGoldenrod, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "ON", 0, 0, 0, 0, OnToggleRpm);

RectangularButton(g_sBtnTogPow, g_psPanels + 2, &g_sBtnTogRpm, 0,
                  &g_sKentec320x240x16_SSD2119,
                  SROW_BX, SROW_Y0 + 6*SROW_DY, SROW_BW, SROW_BH,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrCyan, ClrBlack, ClrGray, ClrBlack,
                  &g_sFontCm16, "ON", 0, 0, 0, 0, OnTogglePow);

/* Cooling status moved to a Control-tab LED indicator. */

/*-----------------------------------------------------------*/
/* Page 3 — Thresholds tab.
 *
 * Three rows, each "label + value + [-] + [+]":
 *   Power     300 W       step  10 W       range  50 .. 300
 *   Accel     2.0 g       step 0.1 g       range 0.5 .. 4.0
 *   Night     5 lux       step  1 lux      range  1 .. 50
 *   Cool      25 C        step  1 C        range 10 .. 40
 *   (Distance/ToF not used — optional sensor: BMI160 + SHT31)
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

Canvas(g_sThreshNightVal, g_psPanels + 3, &g_sThreshAccelText, 0,
       &g_sKentec320x240x16_SSD2119,
       110, THR_ROW_Y(2), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrWhite, &g_sFontCm22, "5 lux", 0, 0);

/* Row labels. */
Canvas(g_sThreshPowerLabel, g_psPanels + 3, &g_sThreshNightVal, 0,
       &g_sKentec320x240x16_SSD2119,
       10, THR_ROW_Y(0), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrCyan, &g_sFontCm18, "Power", 0, 0);

Canvas(g_sThreshAccelLabel, g_psPanels + 3, &g_sThreshPowerLabel, 0,
       &g_sKentec320x240x16_SSD2119,
       10, THR_ROW_Y(1), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrMagenta, &g_sFontCm18, "Accel", 0, 0);

Canvas(g_sThreshNightLabel, g_psPanels + 3, &g_sThreshAccelLabel, 0,
       &g_sKentec320x240x16_SSD2119,
       10, THR_ROW_Y(2), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrLimeGreen, &g_sFontCm18, "Night", 0, 0);

/* +/- buttons (40 px wide each, 32 px tall, right-hand side). */
RectangularButton(g_sThreshPowerDec, g_psPanels + 3, &g_sThreshNightLabel, 0,
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

RectangularButton(g_sThreshNightDec, g_psPanels + 3, &g_sThreshAccelInc, 0,
                  &g_sKentec320x240x16_SSD2119,
                  215, THR_ROW_Y(2)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkRed, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "-", 0, 0, 0, 0, OnNightDec);
RectangularButton(g_sThreshNightInc, g_psPanels + 3, &g_sThreshNightDec, 0,
                  &g_sKentec320x240x16_SSD2119,
                  265, THR_ROW_Y(2)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGreen, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "+", 0, 0, 0, 0, OnNightInc);

/* Row 3 - Cool threshold (deg C). Sensors tab shows "Cooling On/Off"
 * by comparing g_temp_c against g_thresh_cool_c. */
Canvas(g_sThreshCoolVal, g_psPanels + 3, &g_sThreshNightInc, 0,
       &g_sKentec320x240x16_SSD2119,
       110, THR_ROW_Y(3), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL,
       ClrBlack, 0, ClrWhite, &g_sFontCm22, "25 C", 0, 0);

Canvas(g_sThreshCoolLabel, g_psPanels + 3, &g_sThreshCoolVal, 0,
       &g_sKentec320x240x16_SSD2119,
       10, THR_ROW_Y(3), 100, 24,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrCyan, &g_sFontCm18, "Cool", 0, 0);

RectangularButton(g_sThreshCoolDec, g_psPanels + 3, &g_sThreshCoolLabel, 0,
                  &g_sKentec320x240x16_SSD2119,
                  215, THR_ROW_Y(3)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkRed, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "-", 0, 0, 0, 0, OnCoolDec);
RectangularButton(g_sThreshCoolInc, g_psPanels + 3, &g_sThreshCoolDec, 0,
                  &g_sKentec320x240x16_SSD2119,
                  265, THR_ROW_Y(3)-4, 40, 32,
                  PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT,
                  ClrDarkGreen, ClrBlack, ClrGray, ClrWhite,
                  &g_sFontCm22, "+", 0, 0, 0, 0, OnCoolInc);

/* g_sThreshCoolInc is the new panel-3 first_child (last in source). */

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
        prvRefreshNightLabel();
        prvRefreshCoolLabel();
    }

    /* Returning to Control: re-render the summary line so it shows
     * the current (possibly edited) threshold values. */
    if (idx == 0)
        prvRefreshControlThreshLine();
}

/*-----------------------------------------------------------*/
/* Button callbacks — send signals to the motor task. */

static int32_t prvClampSliderRpm(int32_t rpm)
{
    if (rpm < 0)
    {
        return 0;
    }
    if (rpm > MAX_MOTOR_RPM)
    {
        return MAX_MOTOR_RPM;
    }
    return rpm;
}

/* GrLib maps touch X to RPM with integer division, so the rightmost pixel
 * is usually below i32Max (worse on a short track — e.g. ~9945 at 180 px). */
static int32_t prvSliderTrackPixels(void)
{
    int16_t span = (g_sSpeedSlider.sBase.sPosition.i16XMax -
                    g_sSpeedSlider.sBase.sPosition.i16XMin) + 1;
    if (g_sSpeedSlider.ui32Style & SL_STYLE_OUTLINE)
    {
        span -= 2;
    }
    return (span > 0) ? (int32_t)span : 1;
}

static int32_t prvNormalizeSliderRpm(int32_t rpm)
{
    int32_t track = prvSliderTrackPixels();
    int32_t max_gap = (MAX_MOTOR_RPM + track - 1) / track;

    rpm = prvClampSliderRpm(rpm);
    if (rpm > MAX_MOTOR_RPM - max_gap)
    {
        return MAX_MOTOR_RPM;
    }
    return rpm;
}

static void prvRefreshSpeedSliderLabel(int32_t rpm)
{
    static char buf[24];

    rpm = prvClampSliderRpm(rpm);
    usprintf(buf, "%d RPM", (int)rpm);
    SliderTextSet(&g_sSpeedSlider, buf);
}

static void prvSendLatestRpmCommand(int32_t rpm)
{
    /* Slider callbacks can generate several commands before the motor task
     * drains the queue. Keep only the newest command so START cannot be
     * paired with an old 0 RPM command. */
    if (xCommandMutex != NULL)
    {
        xSemaphoreTake(xCommandMutex, portMAX_DELAY);
    }
    xQueueReset(xCommandQueue);
    xQueueSend(xCommandQueue, &rpm, 0);
    if (xCommandMutex != NULL)
    {
        xSemaphoreGive(xCommandMutex);
    }
}

static void OnStartPressed(tWidget *psWidget)
{
    (void)psWidget;
    int32_t rpm = prvNormalizeSliderRpm(g_sSpeedSlider.i32Value);
    if (rpm < MIN_START_RPM)
    {
        rpm = MIN_START_RPM;
        SliderValueSet(&g_sSpeedSlider, rpm);
        prvRefreshSpeedSliderLabel(rpm);
        WidgetPaint((tWidget *)&g_sSpeedSlider);
    }
    prvSendLatestRpmCommand(rpm);
    speed_sensor_reset_filter();
    xEventGroupClearBits(xSystemEvents, EVT_USER_STOP);
    xEventGroupSetBits(xSystemEvents, EVT_USER_START);
#if MOTOR_ENABLE_NFAULT_MONITORING
    if (g_ui32Panel == 0 && (g_fault_bits & EVT_ESTOP_DRIVER))
    {
        CanvasTextSet(&g_sPowerLimitText,
                      (char *)"Drv fault - power-cycle motor board");
        WidgetPaint((tWidget *)&g_sPowerLimitText);
    }
#endif
}

static void OnStopPressed(tWidget *psWidget)
{
    (void)psWidget;
    prvSendLatestRpmCommand(0);
    SliderValueSet(&g_sSpeedSlider, 0);
    prvRefreshSpeedSliderLabel(0);
    WidgetPaint((tWidget *)&g_sSpeedSlider);
    xEventGroupClearBits(xSystemEvents, EVT_USER_START);
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
    int32_t rpm = prvNormalizeSliderRpm(i32Value);
    if (rpm == MAX_MOTOR_RPM && i32Value != MAX_MOTOR_RPM)
    {
        SliderValueSet(&g_sSpeedSlider, MAX_MOTOR_RPM);
    }

    prvSendLatestRpmCommand(rpm);
    prvRefreshSpeedSliderLabel(rpm);
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
static char s_thresh_night_buf [16];
static char s_thresh_cool_buf  [16];

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
static void prvRefreshNightLabel(void)
{
    usprintf(s_thresh_night_buf, "%d lux", (int)g_thresh_night_lux);
    CanvasTextSet(&g_sThreshNightVal, s_thresh_night_buf);
    WidgetPaint((tWidget *)&g_sThreshNightVal);
}
static void prvRefreshCoolLabel(void)
{
    usprintf(s_thresh_cool_buf, "%d C", (int)g_thresh_cool_c);
    CanvasTextSet(&g_sThreshCoolVal, s_thresh_cool_buf);
    WidgetPaint((tWidget *)&g_sThreshCoolVal);
}

/* Summary line on the Control tab — mirrors the runtime thresholds
 * so the user can see them without leaving Control. */
static char s_control_thresh_buf[40];
static void prvRefreshControlThreshLine(void)
{
    int g10 = (int)(g_thresh_accel_g * 10.0f + 0.5f);
    usprintf(s_control_thresh_buf,
             "Pwr %dW  Acc %d.%dg  Nt %dlx  Cool %dC",
             (int)g_thresh_power_w,
             g10 / 10, g10 % 10,
             (int)g_thresh_night_lux,
             (int)g_thresh_cool_c);
    CanvasTextSet(&g_sPowerLimitText, s_control_thresh_buf);
    /* Only paint if Control is the active tab — otherwise the paint
     * would land on whatever panel is currently mounted. */
    if (g_ui32Panel == 0)
        WidgetPaint((tWidget *)&g_sPowerLimitText);
}

/* Clock/date sit above the RPM line; partial RPM repaint can erase them. */
static void prvRepaintClockDate(void)
{
    if (g_ui32Panel != 0)
    {
        return;
    }
    WidgetPaint((tWidget *)&g_sClockText);
    WidgetPaint((tWidget *)&g_sDateText);
}

/* Drain touch/paint messages — call often so the LCD stays responsive. */
static void prvProcessTouchMessages(void)
{
    for (uint8_t n = 0; n < 4u; n++)
    {
        WidgetMessageQueueProcess();
    }
}

/* Control-tab status indicator (rectangle at x=250): green=Running,
 * orange=Idle/Starting/Stopping, yellow=high |a|, red=fault/E-stop. */
static uint32_t prvStatusIndicatorColor(void)
{
    if (g_state == MOTOR_STATE_ESTOP_BRAKING ||
        g_state == MOTOR_STATE_FAULT_LATCHED)
    {
        return ClrRed;
    }
    if (g_accel_total_g >= g_thresh_accel_g)
    {
        return ClrYellow;
    }
    if (g_state == MOTOR_STATE_RUNNING)
    {
        return ClrLimeGreen;
    }
    return ClrOrange;
}

static const char *prvStatusStateName(void)
{
    switch (g_state)
    {
    case MOTOR_STATE_IDLE:          return "Idle";
    case MOTOR_STATE_STARTING:      return "Starting";
    case MOTOR_STATE_RUNNING:       return "Running";
    case MOTOR_STATE_STOPPING:      return "Stopping";
    case MOTOR_STATE_ESTOP_BRAKING: return "E-Stop Brake";
    case MOTOR_STATE_FAULT_LATCHED: return "Fault Latched";
    default:                        return "Idle";
    }
}

static void prvRefreshStatusIndicator(void)
{
    uint32_t col = prvStatusIndicatorColor();

    CanvasTextSet(&g_sStateText, (char *)prvStatusStateName());
    WidgetPaint((tWidget *)&g_sStateText);
    g_sStatusIndicator.ui32FillColor = col;
    WidgetPaint((tWidget *)&g_sStatusIndicator);
}

/* Control-tab status line under thresholds (fault text, duty debug, etc.). */
static char s_status_line_buf[48];

static void prvFormatEstopReasons(char *buf, EventBits_t bits)
{
    usprintf(buf, "EStop:%s%s%s%s  Press ACK",
             (bits & EVT_ESTOP_POWER)    ? " Pwr" : "",
             (bits & EVT_ESTOP_ACCEL)    ? " Acc" : "",
             (bits & EVT_ESTOP_DISTANCE) ? " Dst" : "",
             (bits & EVT_ESTOP_DRIVER)   ? " Drv" : "");
}

static void prvRefreshControlStatusLine(void)
{
    if (g_ui32Panel != 0)
    {
        return;
    }

    if (g_state == MOTOR_STATE_FAULT_LATCHED)
    {
        if (g_fault_bits & EVT_ESTOP_ANY)
        {
            prvFormatEstopReasons(s_status_line_buf, g_fault_bits);
        }
        else
        {
            usprintf(s_status_line_buf, "Fault latched - press ACK");
        }
    }
    else if (g_state == MOTOR_STATE_ESTOP_BRAKING)
    {
        if (g_fault_bits & EVT_ESTOP_ANY)
        {
            prvFormatEstopReasons(s_status_line_buf, g_fault_bits);
        }
        else
        {
            usprintf(s_status_line_buf, "E-Stop braking - press ACK");
        }
    }
    else if (g_fault_bits & EVT_SENSOR_FAULT)
    {
        usprintf(s_status_line_buf, "Fault: MotorLib init failed");
    }
    else if (g_fault_bits & EVT_ESTOP_ANY)
    {
        if (g_state == MOTOR_STATE_IDLE && (g_fault_bits & EVT_ESTOP_DRIVER))
        {
            usprintf(s_status_line_buf,
                     "Idle - Drv fault: cycle motor power");
        }
        else
        {
            prvFormatEstopReasons(s_status_line_buf, g_fault_bits);
        }
    }
    else if (g_state == MOTOR_STATE_STARTING ||
             g_state == MOTOR_STATE_RUNNING ||
             g_state == MOTOR_STATE_STOPPING)
    {
        usprintf(s_status_line_buf, "Duty %d%% Ready %d Hall %d%d%d",
                 (int)g_pwm_duty,
                 g_motor_ready ? 1 : 0,
                 (g_hall_state & 4u) ? 1 : 0,
                 (g_hall_state & 2u) ? 1 : 0,
                 (g_hall_state & 1u) ? 1 : 0);
    }
    else
    {
        prvRefreshControlThreshLine();
        return;
    }

    CanvasTextSet(&g_sPowerLimitText, s_status_line_buf);
    WidgetPaint((tWidget *)&g_sPowerLimitText);
}

static void OnPowerInc(tWidget *w) { (void)w;
    g_thresh_power_w += POWER_THRESH_GUI_STEP_W;
    if (g_thresh_power_w > POWER_THRESH_GUI_MAX_W)
    {
        g_thresh_power_w = POWER_THRESH_GUI_MAX_W;
    }
    prvRefreshPowerLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnPowerDec(tWidget *w) { (void)w;
    g_thresh_power_w -= POWER_THRESH_GUI_STEP_W;
    if (g_thresh_power_w < POWER_THRESH_GUI_MIN_W)
    {
        g_thresh_power_w = POWER_THRESH_GUI_MIN_W;
    }
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
static void OnNightInc(tWidget *w) { (void)w;
    g_thresh_night_lux += 1.0f;
    if (g_thresh_night_lux > 50.0f) g_thresh_night_lux = 50.0f;
    prvRefreshNightLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnNightDec(tWidget *w) { (void)w;
    g_thresh_night_lux -= 1.0f;
    if (g_thresh_night_lux < 1.0f) g_thresh_night_lux = 1.0f;
    prvRefreshNightLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnCoolInc(tWidget *w) { (void)w;
    g_thresh_cool_c += 1.0f;
    if (g_thresh_cool_c > 40.0f) g_thresh_cool_c = 40.0f;
    prvRefreshCoolLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}
static void OnCoolDec(tWidget *w) { (void)w;
    g_thresh_cool_c -= 1.0f;
    if (g_thresh_cool_c < 10.0f) g_thresh_cool_c = 10.0f;
    prvRefreshCoolLabel();
    xEventGroupSetBits(xSystemEvents, EVT_USER_THRESHOLD_CHANGED);
    prvRefreshControlThreshLine();
}

/*-----------------------------------------------------------*/
/* Advanced feature: ACC (virtual distance) UI. */

static void prvAccPaint(void)
{
    bool on = g_acc_enabled;
    g_sAccBtn.ui32FillColor = on ? ClrLimeGreen : ClrDarkGray;
    g_sAccBtn.ui32TextColor = on ? ClrBlack     : ClrSilver;
    PushButtonTextSet(&g_sAccBtn, (char *)(on ? "ACC ON" : "ACC OFF"));
    WidgetPaint((tWidget *)&g_sAccBtn);
}

static void prvRefreshVDistLabel(void)
{
    static char s_dist[32];
    int d = (int)(g_virtual_distance_mm + 0.5f);
    int min = (int)(g_thresh_distance_mm + 0.5f);
    usprintf(s_dist, "Dist:  %d mm  (min %d)", d, min);
    CanvasTextSet(&g_sSensDistText, s_dist);
    if (g_ui32Panel == 2)
    {
        WidgetPaint((tWidget *)&g_sSensDistText);
    }
}

static void OnAccToggle(tWidget *w)
{
    (void)w;
    g_acc_enabled = !g_acc_enabled;
    prvAccPaint();
}

static void OnVDistInc(tWidget *w)
{
    (void)w;
    g_virtual_distance_mm += VDIST_GUI_STEP_MM;
    if (g_virtual_distance_mm > VDIST_GUI_MAX_MM) g_virtual_distance_mm = VDIST_GUI_MAX_MM;
    prvRefreshVDistLabel();
}

static void OnVDistDec(tWidget *w)
{
    (void)w;
    g_virtual_distance_mm -= VDIST_GUI_STEP_MM;
    if (g_virtual_distance_mm < VDIST_GUI_MIN_MM) g_virtual_distance_mm = VDIST_GUI_MIN_MM;
    prvRefreshVDistLabel();
}

/*-----------------------------------------------------------*/
/* Plot-trace toggle buttons on the Sensors tab.
 *
 * Each button flips its g_trace_enabled[] entry and updates its own
 * fill colour + text so the operator sees the new state instantly:
 *   enabled  -> trace colour fill,  "ON"  text
 *   disabled -> dark grey fill,     "OFF" text */

static void prvTogglePaint(tPushButtonWidget *btn, bool enabled,
                           uint32_t on_color)
{
    btn->ui32FillColor = enabled ? on_color    : ClrDarkGray;
    btn->ui32TextColor = enabled ? ClrBlack    : ClrSilver;
    PushButtonTextSet(btn, (char *)(enabled ? "ON" : "OFF"));
    WidgetPaint((tWidget *)btn);
}

static void OnToggleLux(tWidget *w) { (void)w;
    g_trace_enabled[TRACE_LUX] = !g_trace_enabled[TRACE_LUX];
    prvTogglePaint(&g_sBtnTogLux, g_trace_enabled[TRACE_LUX], ClrWhite);
}
static void OnToggleAcc(tWidget *w) { (void)w;
    g_trace_enabled[TRACE_ACC] = !g_trace_enabled[TRACE_ACC];
    prvTogglePaint(&g_sBtnTogAcc, g_trace_enabled[TRACE_ACC], ClrMagenta);
}
static void OnToggleTemp(tWidget *w) { (void)w;
    g_trace_enabled[TRACE_TEMP] = !g_trace_enabled[TRACE_TEMP];
    prvTogglePaint(&g_sBtnTogTemp, g_trace_enabled[TRACE_TEMP], ClrOrange);
}
static void OnToggleHum(tWidget *w) { (void)w;
    g_trace_enabled[TRACE_HUM] = !g_trace_enabled[TRACE_HUM];
    prvTogglePaint(&g_sBtnTogHum, g_trace_enabled[TRACE_HUM], ClrTurquoise);
}
static void OnTogglePres(tWidget *w) { (void)w;
    g_trace_enabled[TRACE_PRES] = !g_trace_enabled[TRACE_PRES];
    prvTogglePaint(&g_sBtnTogPres, g_trace_enabled[TRACE_PRES], ClrLimeGreen);
}
static void OnToggleRpm(tWidget *w) { (void)w;
    g_trace_enabled[TRACE_RPM] = !g_trace_enabled[TRACE_RPM];
    prvTogglePaint(&g_sBtnTogRpm, g_trace_enabled[TRACE_RPM], ClrGoldenrod);
}
static void OnTogglePow(tWidget *w) { (void)w;
    g_trace_enabled[TRACE_POW] = !g_trace_enabled[TRACE_POW];
    prvTogglePaint(&g_sBtnTogPow, g_trace_enabled[TRACE_POW], ClrCyan);
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
    static int32_t      last_rpm_ref  = -1;
    static int32_t      last_rpm_des  = -1;
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
    static EventBits_t  last_fault_bits = (EventBits_t)-1;

    /* Plots tab repaints itself via OnPlotCanvasPaint. */
    if (g_ui32Panel == 1) return;

    if (g_force_refresh)
    {
        last_state    = (MotorState_t)-1;
        last_rpm      = -1;
        last_rpm_ref  = -1;
        last_rpm_des  = -1;
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
        last_fault_bits = (EventBits_t)-1;
        g_force_refresh = false;
    }

    if (g_ui32Panel == 0)
    {
        /* ---- Control tab: motor + day/night + clock. ---- */
        static char s_rpm_buf[24];
        static char s_power_buf[24];
        static char s_clock_buf[24];
        static bool last_acc_on = (bool)-1;

        static uint32_t last_status_col = 0xFFFFFFFFu;
        static int      last_accel_100  = -1;

        {
            uint32_t col = prvStatusIndicatorColor();
            int      a100 = (int)(g_accel_total_g * 100.0f + 0.5f);

            if (g_state != last_state || col != last_status_col ||
                a100 != last_accel_100)
            {
                prvRefreshStatusIndicator();
                last_state = g_state;
                last_status_col = col;
                last_accel_100 = a100;
                last_fault_bits = (EventBits_t)-1;
            }
        }

        if (g_fault_bits != last_fault_bits)
        {
            prvRefreshControlStatusLine();
            last_fault_bits = g_fault_bits;
        }
        else if (g_state == MOTOR_STATE_FAULT_LATCHED ||
                 g_state == MOTOR_STATE_ESTOP_BRAKING)
        {
            /* Keep fault text visible while latched (not only on bit change). */
            prvRefreshControlStatusLine();
        }
        else if (g_state == MOTOR_STATE_IDLE &&
                 (g_fault_bits & EVT_ESTOP_DRIVER))
        {
            prvRefreshControlStatusLine();
        }
        else if (g_state == MOTOR_STATE_STARTING ||
                 g_state == MOTOR_STATE_RUNNING ||
                 g_state == MOTOR_STATE_STOPPING)
        {
            /* Duty/Hall debug line used to repaint on every PI tick (~100 Hz)
             * and blocked touch for tens of ms per frame. Throttle heavily. */
            static uint16_t    last_pwm_stat = 0xFFFFu;
            static TickType_t last_stat_paint = 0;
            TickType_t now_stat = xTaskGetTickCount();
            int pwm_diff = (int)g_pwm_duty - (int)last_pwm_stat;
            if (pwm_diff < 0)
            {
                pwm_diff = -pwm_diff;
            }
            bool stat_due = (now_stat - last_stat_paint >= pdMS_TO_TICKS(500));
            if (last_pwm_stat == 0xFFFFu || pwm_diff >= 5 || stat_due)
            {
                prvRefreshControlStatusLine();
                last_pwm_stat = g_pwm_duty;
                last_stat_paint = now_stat;
            }
        }

        if (g_rpm_actual != last_rpm || g_rpm_reference != last_rpm_ref ||
            g_rpm_desired != last_rpm_des)
        {
            int32_t d_act = g_rpm_actual - last_rpm;
            int32_t d_ref = g_rpm_reference - last_rpm_ref;
            int32_t d_des = g_rpm_desired - last_rpm_des;
            if (d_act < 0) d_act = -d_act;
            if (d_ref < 0) d_ref = -d_ref;
            if (d_des < 0) d_des = -d_des;

            /* Throttle repaint while running — hall display still updates
             * in the model but we only redraw when change is visible. */
            static TickType_t last_rpm_paint = 0;
            TickType_t now_rpm = xTaskGetTickCount();
            bool big_change = (d_act >= 8 || d_ref >= 8 || d_des >= 8);
            bool paint_rpm = (last_rpm < 0) || big_change ||
                             (now_rpm - last_rpm_paint >= pdMS_TO_TICKS(100));

            last_rpm     = g_rpm_actual;
            last_rpm_ref = g_rpm_reference;
            last_rpm_des = g_rpm_desired;

            if (paint_rpm)
            {
                usprintf(s_rpm_buf, "%d/%d/%d RPM",
                         (int)g_rpm_actual, (int)g_rpm_reference,
                         (int)g_rpm_desired);
                CanvasTextSet(&g_sRpmText, s_rpm_buf);
                WidgetPaint((tWidget *)&g_sRpmText);
                prvRepaintClockDate();
                last_rpm_paint = now_rpm;
            }
        }

        int p = (int)(g_power_w + 0.5f);
        int diff = (p > last_pwr_int) ? (p - last_pwr_int) : (last_pwr_int - p);
        if (last_pwr_int < 0 || diff >= 2)
        {
            usprintf(s_power_buf, "%d W", p);
            CanvasTextSet(&g_sPowerText, s_power_buf);
            WidgetPaint((tWidget *)&g_sPowerText);
            last_pwr_int = p;
        }

        if (g_acc_enabled != last_acc_on)
        {
            prvAccPaint();
            last_acc_on = g_acc_enabled;
        }

        /* Simple threshold (no hysteresis): below the user-set night
         * level is night, at-or-above is day. Editable from the
         * Thresholds tab via g_thresh_night_lux. */
        bool is_night = (g_light_lux < g_thresh_night_lux);
        if (is_night != last_is_night)
        {
            CanvasTextSet(&g_sDayNightText, is_night ? "Night" : "Day");
            WidgetPaint((tWidget *)&g_sDayNightText);
            CanvasImageSet(&g_sDayNightLed, is_night ? g_pui8LightOn : g_pui8LightOff);
            WidgetPaint((tWidget *)&g_sDayNightLed);
            last_is_night = is_night;
        }

        /* Cooling LED on the Control tab — same bulb image as the
         * Day/Night LED. On when SHT31 temp exceeds the cool threshold. */
        static bool last_cool_on_ctl = (bool)-1;
        bool cool_on_ctl = (g_sht_ok && g_temp_c > g_thresh_cool_c);
        if (cool_on_ctl != last_cool_on_ctl)
        {
            CanvasImageSet(&g_sCoolLed,
                           cool_on_ctl ? g_pui8LightOn : g_pui8LightOff);
            WidgetPaint((tWidget *)&g_sCoolLed);
            last_cool_on_ctl = cool_on_ctl;
        }

        uint32_t sec = xTaskGetTickCount() / configTICK_RATE_HZ;
        if (sec != last_sec)
        {
            uint32_t h = (sec / 3600) % 24;
            uint32_t m = (sec / 60) % 60;
            uint32_t s =  sec % 60;
            usprintf(s_clock_buf, "%02d:%02d:%02d", h, m, s);
            CanvasTextSet(&g_sClockText, s_clock_buf);

            /* Date: walk forward from the demo epoch by elapsed days.
             * Cheap O(days) loop; demo never runs long enough to feel it. */
            #define BASE_YEAR  2026
            #define BASE_MONTH 5
            #define BASE_DAY   26
            static const int days_in_month[12] =
                { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
            uint32_t days = sec / 86400u;
            int y = BASE_YEAR, mo = BASE_MONTH, d = BASE_DAY;
            while (days > 0)
            {
                d++;
                int dim = days_in_month[mo - 1];
                /* Leap year adjustment for February. */
                if (mo == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0))
                    dim = 29;
                if (d > dim) { d = 1; mo++; if (mo > 12) { mo = 1; y++; } }
                days--;
            }
            static char s_date_buf[16];
            static int last_day = -1;
            if (d != last_day)
            {
                usprintf(s_date_buf, "%04d-%02d-%02d", y, mo, d);
                CanvasTextSet(&g_sDateText, s_date_buf);
                last_day = d;
            }

            prvRepaintClockDate();
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
        static int  last_vdist_i = -1;

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
        /* T/H come from the SHT31 only (spec). BME280's T/H is
         * intentionally discarded in sensor_task. */
        if (temp_10 != last_temp_10)
        {
            if (g_sht_ok) usprintf(s_temp, "Temp:  %d.%d C",
                                   temp_10 / 10, temp_10 % 10);
            else          usprintf(s_temp, "Temp:  --- C");
            CanvasTextSet(&g_sSensTempText, s_temp);
            WidgetPaint((tWidget *)&g_sSensTempText);
            last_temp_10 = temp_10;
        }
        if (hum_10 != last_hum_10)
        {
            if (g_sht_ok) usprintf(s_hum, "Hum:   %d.%d %%",
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

        /* RPM + Power readouts on the Sensors tab. */
        static char     s_sens_rpm[24], s_sens_pow[24];
        static int32_t  last_sens_rpm   = -1;
        static int      last_sens_pow_i = -1;
        if (g_rpm_actual != last_sens_rpm)
        {
            usprintf(s_sens_rpm, "RPM:   %d", (int)g_rpm_actual);
            CanvasTextSet(&g_sSensRpmText, s_sens_rpm);
            WidgetPaint((tWidget *)&g_sSensRpmText);
            last_sens_rpm = g_rpm_actual;
        }
        int p_i = (int)(g_power_w + 0.5f);
        if (p_i != last_sens_pow_i)
        {
            usprintf(s_sens_pow, "Power: %d W", p_i);
            CanvasTextSet(&g_sSensPowText, s_sens_pow);
            WidgetPaint((tWidget *)&g_sSensPowText);
            last_sens_pow_i = p_i;
        }

        int vdist_i = (int)(g_virtual_distance_mm + 0.5f);
        if (vdist_i != last_vdist_i)
        {
            prvRefreshVDistLabel();
            last_vdist_i = vdist_i;
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

#if !SERIAL_PLOT_CLEAN
    uart_log_printf("GUI: LCD init...\n");
#endif

    /* LCD + grlib + touch. */
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);
    TouchScreenInit(g_ui32SysClock);
#if !SERIAL_PLOT_CLEAN
    uart_log_printf("GUI: LCD + touch ready\n");
#endif
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
        /* 1. Touch first — do not let SPI readouts starve the queue. */
        prvProcessTouchMessages();

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
            g_rpm_actual    = motor_msg.rpm_actual;
            g_rpm_reference = motor_msg.rpm_reference;
            g_rpm_desired   = motor_msg.rpm_desired;
            g_pwm_duty   = motor_msg.pwm_duty;
            g_fault_bits = motor_msg.fault_bits;
            g_hall_state = motor_msg.hall_state;
            g_motor_ready = motor_msg.motor_ready;
        }
        while (xQueueReceive(xSensorQueue, &sensor_msg, 0) == pdPASS)
        {
            g_light_lux    = sensor_msg.light_lux;
            g_accel_x_g    = sensor_msg.accel_x_g;
            g_accel_y_g    = sensor_msg.accel_y_g;
            g_accel_z_g    = sensor_msg.accel_z_g;
            g_accel_total_g = sensor_msg.accel_total_g;
            g_accel_g      = sensor_msg.accel_y_g;   /* plot uses Y, signed */
            g_temp_c       = sensor_msg.temp_c;
            g_humidity_pct = sensor_msg.humidity_pct;
            g_pressure_hpa = sensor_msg.pressure_hpa;
            g_bme_ok       = sensor_msg.bme_ok;
            g_sht_ok       = sensor_msg.sht_ok;
        }

        /* Power comes only from the ADC pipeline (sensor task), not motor queue. */
#if MOTOR_ENABLE_POWER_SENSOR
        g_power_w = power_sensor_zero_ready() ? g_motor_power_watts : 0.0f;
#else
        g_power_w = g_motor_power_watts;
#endif

        /* 4. Refresh whichever readout tab is currently showing. */
        prvRedrawWidgets();

        /* 5. Push a plot sample at 5 Hz regardless of which tab is
         *    showing, so the trace is fresh when the user switches. */
        TickType_t now = xTaskGetTickCount();
        if (now - last_sample_tick >= pdMS_TO_TICKS(200))
        {
            last_sample_tick = now;
            /* Plot reference (500 RPM/s ramp) — actual hall speed is noisier. */
            prvPlotPush((float)g_rpm_reference, g_power_w, g_light_lux, g_accel_g,
                        g_temp_c, g_humidity_pct, g_pressure_hpa);
        }

        /* 6. Repaint the plot at ~2 Hz on Plots tab (full canvas is costly). */
        if (g_ui32Panel == 1 && (now - last_plot_paint >= pdMS_TO_TICKS(500)))
        {
            last_plot_paint = now;
            WidgetPaint((tWidget *)&g_sPlotCanvas);
        }

        prvProcessTouchMessages();

        /* 5 ms loop -> touch polled ~200 Hz; was 20 ms (~50 Hz). */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/*-----------------------------------------------------------*/

void vCreateGuiTask(void)
{
    /* Above sensor (+3) during boot so Kentec/touch init is not starved by
     * I2C sensor bring-up or the 1 kHz power ADC ISR load. */
    xTaskCreate(prvGuiTask, "GUI",
                configMINIMAL_STACK_SIZE * 8, NULL,
                tskIDLE_PRIORITY + 5, NULL);
}
