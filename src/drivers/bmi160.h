/*
 * bmi160.h — Bosch BMI160 6-axis IMU (optional sensor).
 *
 * Used here only for accelerometer. Default I2C address: 0x68 or 0x69.
 */
#ifndef BMI160_H
#define BMI160_H

#include <stdbool.h>

#include <stdint.h>

bool bmi160_init(void);
bool bmi160_read_accel_g(float *ax, float *ay, float *az);
bool bmi160_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);
bool bmi160_read_y_g(float *ay);
float bmi160_lsb_per_g(void);      /* For converting raw -> g */

#endif /* BMI160_H */
