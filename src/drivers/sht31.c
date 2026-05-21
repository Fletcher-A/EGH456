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

#define SHT31_ADDR 0x44
#define OUTPUT_LENGTH 6
#define SHT31_FIRST_BIT 0x24
#define SHT31_SECOND_BIT 0x00



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
    // uint8_t buffer[OUTPUT_LENGTH];
    //  if (!prvReadN(SHT31_MEAS_HIGHREP, buffer, OUTPUT_LENGTH)) return false;

    uint8_t d[OUTPUT_LENGTH];

    writeI2C1(SHT31_ADDR, SHT31_FIRST_BIT, SHT31_SECOND_BIT);
    // There should be a 20-ish milisecond delay to wait for the sensor to sense here, but nothing happens if there isn't
    // Also, there might technically be a race condition or something here because the system doesn't wait for the I2C
    // to finish before converting and returning the values. TODO fix if it becomes a problem, but right now it doesn't seem to be 
    readSHT31(SHT31_ADDR, d);

    uint16_t temp = (d[0] << 8) | (d[1]);
    uint16_t humidity = (d[3] << 8) | (d[4]);

    *temp_c = -45.0f + (175.0f * ((float)temp / 65535.0f));
    *humidity_rh = 100.0f * ((float)humidity / 65535.0f);

    return true;
}
