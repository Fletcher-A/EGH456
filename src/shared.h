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
    MotorState_t state;
} MotorMsgObj;

typedef struct
{
    uint32_t   seq;
    TickType_t tick;
    float      power_watts;        /* filtered motor power */
    float      light_lux;          /* filtered ambient light */
    /* TODO: add fields for the two optional sensors you choose */
    float      optional_a;
    float      optional_b;
} SensorMsgObj;

/*-----------------------------------------------------------*/
/* Shared RTOS object handles (defined in main.c) */

extern QueueHandle_t      xMotorQueue;     /* motor task -> gui */
extern QueueHandle_t      xSensorQueue;    /* sensor task -> gui */
extern QueueHandle_t      xCommandQueue;   /* gui -> motor task */
extern EventGroupHandle_t xSystemEvents;
extern SemaphoreHandle_t  xUARTMutex;
extern SemaphoreHandle_t  xI2CMutex;       /* shared I2C bus mutex */

/*-----------------------------------------------------------*/
/* Event-group bits */

/* Faults / E-Stop triggers */
#define EVT_ESTOP_POWER          (1 << 0)   /* motor power threshold */
#define EVT_ESTOP_ACCEL          (1 << 1)   /* IMU accel threshold */
#define EVT_ESTOP_DISTANCE       (1 << 2)   /* ToF threshold */
#define EVT_ESTOP_ANY            (EVT_ESTOP_POWER | EVT_ESTOP_ACCEL | EVT_ESTOP_DISTANCE)

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
#define DEFAULT_ACCEL_LIMIT_MS2    20.0f
#define DEFAULT_DISTANCE_LIMIT_MM  200
#define NIGHT_LIGHT_LUX            5.0f

/* Motor ramp limits (assignment 2.1.3) */
#define ACCEL_LIMIT_RPMPS          500
#define DECEL_LIMIT_RPMPS          500
#define ESTOP_DECEL_LIMIT_RPMPS    1000

#endif /* SHARED_H */
