/*
 * shared.h — Assignment-wide RTOS object handles, message structs,
 *            and event-group bit assignments.
 *
 * Every task includes this so they all agree on:
 *   - which queues exist and what they carry
 *   - which event-group bits mean what
 *   - shared types like motor state and sensor messages
 *
 * The ACTUAL handles are defined once in main.c.
 */
#ifndef SHARED_H
#define SHARED_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/*-----------------------------------------------------------*/
/* Motor state machine (assignment 2.1.1) */

typedef enum
{
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_STARTING,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_STOPPING,       /* user STOP: ramp down at DECEL_LIMIT_RPMPS */
    MOTOR_STATE_ESTOP_BRAKING,
    MOTOR_STATE_FAULT_LATCHED
} MotorState_t;

/*-----------------------------------------------------------*/
/* Inter-task message types */

typedef struct
{
    uint32_t   seq;
    TickType_t tick;
    int32_t    rpm_actual;
    int32_t    rpm_reference;
    int32_t    rpm_desired;
    uint16_t   pwm_duty;
    float      power_watts;        /* filtered motor power (W) */
    MotorState_t state;
    EventBits_t fault_bits;        /* latched EVT_ESTOP_* reason bits */
    uint8_t    hall_state;         /* bit2=A, bit1=B, bit0=C */
    bool       motor_ready;
} MotorMsgObj;

typedef struct
{
    uint32_t   seq;
    TickType_t tick;
    float      power_watts;        /* filtered motor power */
    float      light_lux;          /* filtered ambient light (OPT3001) */

    /* BMI160 accelerometer — all three axes in g. */
    float      accel_x_g;
    float      accel_y_g;
    float      accel_z_g;
    float      accel_total_g;      /* filtered |ax|+|ay|+|az|, for E-stop */

    /* Environmental sensing. SHT31 provides T+RH (spec 2.2.2 #2);
     * BME280 lives on the same bus and contributes pressure only. */
    float      temp_c;             /* deg C  (SHT31) */
    float      humidity_pct;       /* %RH    (SHT31) */
    float      pressure_hpa;       /* hPa    (BME280) */
    bool       sht_ok;             /* SHT31 init succeeded */
    bool       bme_ok;             /* BME280 init succeeded (pressure only) */
} SensorMsgObj;

/*-----------------------------------------------------------*/
/* Shared RTOS object handles (defined in main.c) */

extern QueueHandle_t      xMotorQueue;     /* motor task -> gui */
extern QueueHandle_t      xSensorQueue;    /* sensor task -> gui */
extern QueueHandle_t      xCommandQueue;   /* gui -> motor task */
extern QueueHandle_t      xPowerRawQueue;  /* ADC ISR -> sensor task (raw counts) */
extern QueueHandle_t      xAccelRawQueue;  /* Timer ISR -> sensor task (raw axes) */

/* One sample = (ia_counts, ib_counts) captured at the same instant. */
typedef struct
{
    uint16_t ia_counts;
    uint16_t ib_counts;
} PowerSampleRaw_t;

/* Raw 16-bit accel counts (BMI160 LSBs). The task converts to g. */
typedef struct
{
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
} AccelSampleRaw_t;
extern EventGroupHandle_t xSystemEvents;
extern SemaphoreHandle_t  xUARTMutex;
extern SemaphoreHandle_t  xI2CMutex;       /* shared I2C bus mutex */
extern SemaphoreHandle_t  xCommandMutex; /* serialises xCommandQueue access */

/*-----------------------------------------------------------*/
/* Task priorities (FreeRTOS: higher number = higher priority)
 *
 *   Speed + AccSamp  idle+5  100 Hz hall RPM + 200 Hz accel I2C
 *   Motor            idle+4  100 Hz control / state machine
 *   Sensor           idle+3  50 Hz fusion + serial CSV
 *   GUI              idle+2  touch + display (~50 Hz effective)
 *   Fault            idle+1  blocks on fault bits (logging only)
 *
 * Inter-task data paths (all preemptive scheduler):
 *   GUI  --xCommandQueue--> Motor
 *   GUI  --xEventGroup-----> Motor, Sensor (estop bits)
 *   Motor--xMotorQueue----> GUI
 *   Sensor-xSensorQueue---> GUI
 *   ISRs --queues/sems----> AccSamp, Sensor (no MotorLib in ISRs except halls)
 */

/*-----------------------------------------------------------*/
/* Event-group bits */

/* Faults / E-Stop triggers */
#define EVT_ESTOP_POWER          (1 << 0)   /* motor power threshold */
#define EVT_ESTOP_ACCEL          (1 << 1)   /* IMU accel threshold */
#define EVT_ESTOP_DISTANCE       (1 << 2)   /* ToF threshold */
#define EVT_ESTOP_DRIVER         (1 << 3)   /* DRV8323 nFAULT (red LED on motor board) */
#define EVT_ESTOP_ANY            (EVT_ESTOP_POWER | EVT_ESTOP_ACCEL | \
                                  EVT_ESTOP_DISTANCE | EVT_ESTOP_DRIVER)

/* Status / sensor flags */
#define EVT_NIGHT_DETECTED       (1 << 4)
#define EVT_SENSOR_FAULT         (1 << 5)

/* User-interface signals */
#define EVT_USER_START           (1 << 8)
#define EVT_USER_STOP            (1 << 9)
#define EVT_USER_ESTOP_ACK       (1 << 10)  /* fault latched -> idle */
#define EVT_USER_SPEED_CHANGED   (1 << 11)
#define EVT_USER_THRESHOLD_CHANGED (1 << 12)

/*-----------------------------------------------------------*/
/* Tunable thresholds (initial values; GUI can override at runtime) */

#define DEFAULT_POWER_LIMIT_W      150.0f
#define POWER_THRESH_GUI_MIN_W     50.0f
#define POWER_THRESH_GUI_MAX_W     300.0f
#define POWER_THRESH_GUI_STEP_W    10.0f
#define DEFAULT_ACCEL_LIMIT_G      2.0f       /* total |a| in g */
#define DEFAULT_DISTANCE_LIMIT_MM  200.0f
#define DEFAULT_COOL_ON_TEMP_C     30.0f   /* SHT31: Cooling On above this (spec 2.3.2) */
#define NIGHT_LIGHT_LUX            5.0f

/* Runtime-editable thresholds (GUI Thresholds tab writes; sensor task
 * reads). Defined once in main.c. Single-word writes are atomic on
 * Cortex-M4, so no mutex is needed. */
extern volatile float g_thresh_power_w;
extern volatile float g_thresh_accel_g;
extern volatile float g_thresh_distance_mm;
extern volatile float g_thresh_night_lux;     /* day/night cut-off */
extern volatile float g_thresh_cool_c;        /* cooling-on threshold (deg C) */

/* Filtered motor power (W) from sensor task ADC pipeline. */
extern volatile float g_motor_power_watts;
extern volatile uint16_t g_motor_pwm_duty_pct;
/* Current motor state for motor_driver_get_state() / other readers. */
extern volatile MotorState_t g_motor_state;

/* True while motor may run (Starting/Running). Sensor task only asserts
 * EVT_ESTOP_* when this is set so bench vibration cannot latch a fault
 * while Idle. Cleared on STOP/ACK. */
extern volatile bool g_motor_estop_armed;
/* False during soft-start; sensor power E-stop waits for this. */
extern volatile bool g_motor_power_estop_ok;

/*-----------------------------------------------------------*/
/* Advanced feature: "ACC" (adaptive cruise control) supervisor.
 *
 * Slider command remains the cruise set-speed (RPM). When ACC is enabled,
 * motor_task reduces the effective RPM target if the (virtual) following
 * distance drops below g_thresh_distance_mm.
 *
 * Distance is a GUI-controlled "virtual ToF" value so the behavior can be
 * demonstrated even without a VL53 sensor module installed.
 */
extern volatile bool  g_acc_enabled;            /* GUI toggle (Control tab) */
extern volatile float g_virtual_distance_mm;    /* GUI +/- on Sensors tab */

#define VDIST_GUI_MIN_MM   50.0f
#define VDIST_GUI_MAX_MM   1000.0f
#define VDIST_GUI_STEP_MM  50.0f

/* Bench motor mechanical rating (BLY172S-24V-4000). */
#define MOTOR_RATED_MAX_RPM        4000
/* Slider and RPM commands: 0 .. rated max (achievable speed on this motor). */
#define MAX_MOTOR_RPM              MOTOR_RATED_MAX_RPM
/* Minimum speed after START (14% of slider max). */
#define MIN_START_RPM              ((MAX_MOTOR_RPM * 14) / 100)

/* Poll DRV8323 nFAULT on PL0 (active low). Set to 1 only after the motor-board
 * red LED / nFAULT line is confirmed on your adapter — see motorlib_example
 * motor_pinmap.h (default there is 0 to avoid a floating pin blocking START). */
#ifndef MOTOR_ENABLE_NFAULT_MONITORING
#define MOTOR_ENABLE_NFAULT_MONITORING  0
#endif

/* When monitoring is on: if 1, START is refused while nFAULT is low. */
#ifndef MOTOR_NFAULT_BLOCKS_START
#define MOTOR_NFAULT_BLOCKS_START  1
#endif

/* Set to 1 when DRV8323 phase-current ADC (PE0/PE1) is wired. Leave 0 on
 * bench setups — init can interfere and floating ADC triggers false E-stop. */
#ifndef MOTOR_ENABLE_POWER_SENSOR
#define MOTOR_ENABLE_POWER_SENSOR  1
#endif

/* Motor ramp limits (assignment 2.1.3) */
#define ACCEL_LIMIT_RPMPS          500
#define DECEL_LIMIT_RPMPS          500
#define ESTOP_DECEL_LIMIT_RPMPS    1000

/* PI speed loop (assignment 2.1.2): duty% = FF(rpm_ref) + Kp*e + Ki*integral. */
#define MOTOR_PI_KP                0.07f
#define MOTOR_PI_KI                0.012f
#define MOTOR_PI_DT_S              0.01f
#define MOTOR_PI_INTEGRAL_MAX      400.0f
#define MOTOR_MAX_DUTY_PCT         100u
#define MOTOR_START_DUTY_PCT       18u   /* open-loop kickstart duty (%) */
#define MOTOR_START_OL_STEP_TICKS  2u    /* advance commutation every N motor ticks */
#define MOTOR_HALL_RUN_RPM         100   /* valid hall speed -> leave open-loop kickstart */
#define MOTOR_RUN_ENTER_RPM        MOTOR_HALL_RUN_RPM
#define MOTOR_RUN_DEBOUNCE_TICKS   5     /* 5 x 10 ms; reject one-shot hall spikes */
/* Reject open-loop hall bursts before closed-loop (slider can be 560, halls read 4000). */
#define MOTOR_START_HALL_RPM_CAP   700
#define MOTOR_RUN_MAX_OVERSPEED_RPM 200
/* Max duty change per 10 ms tick (Starting->Running handoff + PI). */
#define MOTOR_DUTY_SLEW_MAX_PCT_PER_TICK  3u
/* Distance E-stop (VL53L0X): not used — team sensors are BMI160 + SHT31. */
#ifndef MOTOR_ENABLE_DISTANCE_ESTOP
#define MOTOR_ENABLE_DISTANCE_ESTOP  0
#endif

/* Ignore power E-stop below this RPM (floating ADC when motor is idle). */
#define POWER_ESTOP_MIN_RPM        150

#endif /* SHARED_H */
