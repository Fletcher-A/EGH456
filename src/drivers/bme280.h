/*
 * bme280.h — Bosch BME280 environmental sensor (T / RH / P).
 *
 * Shares the I2C0 bus with OPT3001 and BMI160. Default address 0x77
 * (SDO high); some modules tie SDO low giving 0x76. bme280_init
 * probes both.
 *
 * Output values are SI units:
 *   temp  in degrees C
 *   hum   in %RH
 *   press in hPa (millibar)
 */
#ifndef BME280_H
#define BME280_H

#include <stdbool.h>

bool bme280_init(void);
bool bme280_read(float *temp_c, float *hum_pct, float *press_hpa);

#endif /* BME280_H */
