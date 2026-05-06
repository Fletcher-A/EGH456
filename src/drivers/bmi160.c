/*
 * bmi160.c — BMI160 accelerometer driver scaffold.
 *
 * Init sequence:
 *   - Soft-reset (CMD 0xB6 to register 0x7E)
 *   - Set accel power mode normal (CMD 0x11 to 0x7E)
 *   - Configure accel range / output rate via 0x40, 0x41
 *
 * Read:
 *   - 6 bytes from 0x12 (ACC_X_L .. ACC_Z_H) as little-endian int16
 *   - Convert to g using the configured range scale.
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/i2cOptDriver.h"
#include "drivers/bmi160.h"

#define BMI160_ADDR    0x68

bool bmi160_init(void)
{
    /* TODO: write CMD register 0x7E with 0xB6 (reset), wait 1 ms,
     *       write 0x11 (accel normal mode), wait 5 ms,
     *       write 0x40 (accel ODR/bandwidth) and 0x41 (range = +-4 g). */
    return true;
}

bool bmi160_read_accel_g(float *ax, float *ay, float *az)
{
    /* TODO: read 6 bytes from register 0x12, decode int16 LE, scale.
     *       Return false on I2C failure. */
    (void)ax; (void)ay; (void)az;
    return false;
}
