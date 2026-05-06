/*
 * bmi160.h — Bosch BMI160 6-axis IMU (optional sensor).
 *
 * Used here only for accelerometer. Default I2C address: 0x68 or 0x69.
 */
#ifndef BMI160_H
#define BMI160_H

#include <stdbool.h>

bool bmi160_init(void);
bool bmi160_read_accel_g(float *ax, float *ay, float *az);

#endif /* BMI160_H */
