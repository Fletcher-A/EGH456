/*
 * speed_sensor.c — Convert hall-edge counts to RPM.
 *
 * Algorithm:
 *   - speed_sensor_tick() called periodically (e.g. 10 ms = 100 Hz).
 *   - Snapshot the hall edge counter, reset it, compute RPM.
 *   - Optionally apply a low-pass filter to the result.
 */

#include "drivers/speed_sensor.h"
/*-----------------------------------------------------------*/

void speed_sensor_init(void)
{
    /* TODO: zero internal RPM state. */
}

void speed_sensor_tick(uint32_t period_ms)
{
    (void)period_ms;
    /* TODO: 1. Atomically read and reset the hall edge counter.
     * TODO: 2. Compute RPM = (edges / EDGES_PER_REV) * (60_000 / period_ms).
     * TODO: 3. Filter and store.
     */
}

int32_t speed_sensor_get_rpm(void)
{
    /* TODO: return last filtered RPM. */
    return 0;
}
