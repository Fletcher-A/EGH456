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
#define DEFAULT_ACCEL_LIMIT_G      2.0f       /* total |a| in g */
#define DEFAULT_DISTANCE_LIMIT_MM  200.0f
#define NIGHT_LIGHT_LUX            5.0f

/* Runtime-editable thresholds (GUI Thresholds tab writes; sensor task
 * reads). Defined once in main.c. Single-word writes are atomic on
 * Cortex-M4, so no mutex is needed. */
extern volatile float g_thresh_power_w;
extern volatile float g_thresh_accel_g;
extern volatile float g_thresh_distance_mm;

/* True while motor may run (Starting/Running). Sensor task only asserts
 * EVT_ESTOP_* when this is set so bench vibration cannot latch a fault
 * while Idle. Cleared on STOP/ACK. */
extern volatile bool g_motor_estop_armed;

/* Minimum RPM applied when START is pressed with the slider at 0%. */
#define MIN_START_RPM              500

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

/* Motor ramp limits (assignment 2.1.3) */
#define ACCEL_LIMIT_RPMPS          500
#define DECEL_LIMIT_RPMPS          500
#define ESTOP_DECEL_LIMIT_RPMPS    1000

#endif /* SHARED_H */
