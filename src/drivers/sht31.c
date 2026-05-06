/*
 * sht31.c — SHT31 driver (optional sensor, assignment 2.2.2).
 *
 * Wire format:
 *   Trigger a single-shot measurement with command 0x2400 (high
 *   repeatability, no clock stretching). Wait ~15 ms. Read 6 bytes:
 *     [T_hi, T_lo, T_crc, RH_hi, RH_lo, RH_crc]
 *
 *   Temperature_C = -45 + 175 * (T_raw / 65535)
 *   Humidity_RH   =       100 * (RH_raw / 65535)
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/i2cOptDriver.h"
#include "drivers/sht31.h"
/*-----------------------------------------------------------*/

#define SHT31_ADDR                  0x44

bool sht31_init(void)
{
    /* TODO: optional soft-reset command 0x30A2. */
    return true;
}

bool sht31_read(float *temp_c, float *humidity_rh)
{
    /* TODO:
     *   - Send command 0x2400 over I2C.
     *   - vTaskDelay(pdMS_TO_TICKS(20)).
     *   - Read 6 bytes.
     *   - Validate CRC8 on each pair.
     *   - Convert to °C and %RH.
     */
    (void)temp_c; (void)humidity_rh;
    return false;
}
