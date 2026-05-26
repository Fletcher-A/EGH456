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
 *
 * Init probes the bus by issuing a soft-reset write — if no chip
 * ACKs at 0x44 the write fails and we return false so the GUI shows
 * "--- C / --- %" instead of garbage.
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/i2cSHT31Driver.h"
#include "drivers/sht31.h"

/*-----------------------------------------------------------*/

#define SHT31_ADDR        0x44
#define OUTPUT_LENGTH     6
#define SHT31_FIRST_BIT   0x24    /* meas high-rep, no clock stretching MSB */
#define SHT31_SECOND_BIT  0x00    /* meas high-rep, no clock stretching LSB */

/* Soft reset command 0x30A2 — used as a "does anyone ACK at 0x44?" probe. */
#define SHT31_RESET_HI    0x30
#define SHT31_RESET_LO    0xA2

bool sht31_init(void)
{
    /* writeI2C1 returns false if the slave NACKs the address byte —
     * exactly what we want when the SHT31 isn't fitted. */
    return writeI2C2Single(SHT31_ADDR, SHT31_RESET_HI, SHT31_RESET_LO);
}

bool sht31_read(float *temp_c, float *humidity_rh)
{
    uint8_t d[OUTPUT_LENGTH];

    /* Kick off a single-shot conversion. If the chip isn't on the bus
     * the write NACKs and we abort — the caller's temp_c/humidity_rh
     * stay untouched so the GUI shows the previous value (or zero on
     * first call), not random noise from an unrelated transaction. */
    if (!writeI2C2Single(SHT31_ADDR, SHT31_FIRST_BIT, SHT31_SECOND_BIT))
        return false;

    /* Chip needs ~15 ms in high-rep mode before data is valid. */
    if (!readSHT31(SHT31_ADDR, d))
        return false;

    uint16_t temp     = (d[0] << 8) | d[1];
    uint16_t humidity = (d[3] << 8) | d[4];

    *temp_c      = -45.0f + (175.0f * ((float)temp     / 65535.0f));
    *humidity_rh =          (100.0f * ((float)humidity / 65535.0f));
    return true;
}
