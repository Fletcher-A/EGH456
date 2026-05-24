/*
 * speed_sensor.c — Hall-edge counter + 100 Hz RPM calculation.
 *
 * See speed_sensor.h for the pipeline diagram and pin assumptions.
 *
 * Atomicity: g_edges is 32-bit, accessed by ISRs and the tick. The
 * Cortex-M4F's 32-bit aligned loads/stores are atomic, so the tick
 * does a disable-IRQ around the snapshot+clear to make it an atomic
 * read-modify-write. The ISRs use a plain post-increment because
 * they're the only writers and they can't pre-empt each other (NVIC
 * priority is the same across the three port vectors by default).
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driver_lib/gpio.h"
#include "driver_lib/sysctl.h"
#include "driver_lib/interrupt.h"
#include "FreeRTOSConfig.h"

#include "drivers/speed_sensor.h"
#include "drivers/motor_driver.h"
#include "shared.h"

/*-----------------------------------------------------------*/
/* Pin assignments. Tune for the BoosterPack you have. */
#define HALL_A_PORT       GPIO_PORTM_BASE
#define HALL_A_PIN        GPIO_PIN_3
#define HALL_A_PERIPH     SYSCTL_PERIPH_GPIOM
#define HALL_A_INT_VEC    INT_GPIOM

#define HALL_B_PORT       GPIO_PORTH_BASE
#define HALL_B_PIN        GPIO_PIN_2
#define HALL_B_PERIPH     SYSCTL_PERIPH_GPIOH
#define HALL_B_INT_VEC    INT_GPIOH

#define HALL_C_PORT       GPIO_PORTN_BASE
#define HALL_C_PIN        GPIO_PIN_2
#define HALL_C_PERIPH     SYSCTL_PERIPH_GPION
#define HALL_C_INT_VEC    INT_GPION

/*-----------------------------------------------------------*/

static volatile uint32_t g_edges = 0;     /* hall sector changes per tick window */
static volatile int32_t  g_rpm_raw  = 0;
static volatile int32_t  g_rpm_filt = 0;
static uint8_t           s_hall_prev = 0xFF;  /* force first sample to count */

/*-----------------------------------------------------------*/

/* Count one edge per valid commutation step (6 per electrical revolution),
 * not every GPIO toggle. BOTH_EDGE on three lines over-counts and inflates
 * RPM (e.g. fixed ~6000+ on the GUI while the slider targets 0..4000). */
static void prvHallSectorChanged(void)
{
    bool ha, hb, hc;
    bool count_edge = false;

    IntMasterDisable();
    speed_sensor_read_halls(&ha, &hb, &hc);
    uint8_t hall = (uint8_t)((ha ? 4u : 0u) | (hb ? 2u : 0u) | (hc ? 1u : 0u));

    if (hall != s_hall_prev && hall != 0u && hall != 7u)
    {
        g_edges++;
        s_hall_prev = hall;
        count_edge = true;
    }
    IntMasterEnable();

    (void)count_edge;
    motor_driver_update_commutation();
}

/*-----------------------------------------------------------*/

void speed_sensor_init(void)
{
    /* Enable each hall port. */
    SysCtlPeripheralEnable(HALL_A_PERIPH);
    SysCtlPeripheralEnable(HALL_B_PERIPH);
    SysCtlPeripheralEnable(HALL_C_PERIPH);
    while (!SysCtlPeripheralReady(HALL_A_PERIPH)) {}
    while (!SysCtlPeripheralReady(HALL_B_PERIPH)) {}
    while (!SysCtlPeripheralReady(HALL_C_PERIPH)) {}

    /* Pin direction = input with weak pull-up (hall sensors are
     * open-drain on most BoosterPacks). */
    GPIODirModeSet(HALL_A_PORT, HALL_A_PIN, GPIO_DIR_MODE_IN);
    GPIODirModeSet(HALL_B_PORT, HALL_B_PIN, GPIO_DIR_MODE_IN);
    GPIODirModeSet(HALL_C_PORT, HALL_C_PIN, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(HALL_A_PORT, HALL_A_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALL_B_PORT, HALL_B_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(HALL_C_PORT, HALL_C_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    /* Any hall transition runs prvHallSectorChanged (counts valid 1..6 only). */
    GPIOIntTypeSet(HALL_A_PORT, HALL_A_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(HALL_B_PORT, HALL_B_PIN, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(HALL_C_PORT, HALL_C_PIN, GPIO_BOTH_EDGES);

    GPIOIntClear(HALL_A_PORT, HALL_A_PIN);
    GPIOIntClear(HALL_B_PORT, HALL_B_PIN);
    GPIOIntClear(HALL_C_PORT, HALL_C_PIN);

    GPIOIntEnable(HALL_A_PORT, HALL_A_PIN);
    GPIOIntEnable(HALL_B_PORT, HALL_B_PIN);
    GPIOIntEnable(HALL_C_PORT, HALL_C_PIN);

    /* Priority must be >= configMAX_SYSCALL_INTERRUPT_PRIORITY (5 on this
     * port) if ISRs ever call FromISR APIs.  Use 6–7 so hall ISRs do not
     * preempt MotorLib calls in motor_task that run with IntMasterEnable. */
    IntPrioritySet(HALL_A_INT_VEC, configMAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    IntPrioritySet(HALL_B_INT_VEC, configMAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    IntPrioritySet(HALL_C_INT_VEC, configMAX_SYSCALL_INTERRUPT_PRIORITY + 1);

    IntEnable(HALL_A_INT_VEC);
    IntEnable(HALL_B_INT_VEC);
    IntEnable(HALL_C_INT_VEC);

    s_hall_prev = 0xFF;
}

void speed_sensor_read_halls(bool *ha, bool *hb, bool *hc)
{
    if (ha)
        *ha = (GPIOPinRead(HALL_A_PORT, HALL_A_PIN) & HALL_A_PIN) != 0;
    if (hb)
        *hb = (GPIOPinRead(HALL_B_PORT, HALL_B_PIN) & HALL_B_PIN) != 0;
    if (hc)
        *hc = (GPIOPinRead(HALL_C_PORT, HALL_C_PIN) & HALL_C_PIN) != 0;
}

/*-----------------------------------------------------------*/
/* Per-port ISRs. Each only handles its own hall pin. */

void HallPortMIntHandler(void)
{
    GPIOIntClear(HALL_A_PORT, HALL_A_PIN);
    prvHallSectorChanged();
}

void HallPortHIntHandler(void)
{
    GPIOIntClear(HALL_B_PORT, HALL_B_PIN);
    prvHallSectorChanged();
}

void HallPortNIntHandler(void)
{
    GPIOIntClear(HALL_C_PORT, HALL_C_PIN);
    prvHallSectorChanged();
}

/*-----------------------------------------------------------*/
/* Periodic tick. Called from a 100 Hz software timer or task. */

void speed_sensor_tick(uint32_t period_ms)
{
    /* Atomic snapshot-and-clear of the edge counter. */
    uint32_t edges;
    IntMasterDisable();
    edges    = g_edges;
    g_edges  = 0;
    IntMasterEnable();

    /* RPM = (edges / period_s) / edges_per_rev * 60
     *     = edges * 60_000 / (period_ms * edges_per_rev) */
    int32_t rpm_raw = (int32_t)((edges * 60000UL) /
                                (period_ms * SPEED_EDGES_PER_REV));

    /* Exponential low-pass: y[n] = y[n-1] + alpha*(x - y[n-1]).
     * alpha = 1/4 gives a ~40 ms time constant at 100 Hz —
     * smooths quantisation noise on slow rotation without
     * adding noticeable lag. */
    if (rpm_raw > MAX_MOTOR_RPM)
    {
        rpm_raw = MAX_MOTOR_RPM;
    }

    g_rpm_filt = g_rpm_filt + ((rpm_raw - g_rpm_filt) >> 2);
    if (g_rpm_filt > MAX_MOTOR_RPM)
    {
        g_rpm_filt = MAX_MOTOR_RPM;
    }
    else if (g_rpm_filt < 0)
    {
        g_rpm_filt = 0;
    }
    g_rpm_raw  = rpm_raw;
}

int32_t speed_sensor_get_rpm(void)      { return g_rpm_filt; }
int32_t speed_sensor_get_rpm_raw(void)  { return g_rpm_raw;  }
