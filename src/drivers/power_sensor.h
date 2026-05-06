/*
 * power_sensor.h — Motor power estimation via DRV8323 phase-current ADC
 *                   (assignment 2.2.1, sensor 1).
 *
 *   Power = V * I_total,  V = 24V,  I_total estimated from two phase
 *   currents (the third is inferred).
 */
#ifndef POWER_SENSOR_H
#define POWER_SENSOR_H

#include <stdint.h>

#define POWER_SENSOR_VOLTAGE_V   24.0f

void  power_sensor_init(void);
float power_sensor_read_w(void);     /* one new instantaneous sample */
float power_sensor_read_current_a(void);

#endif /* POWER_SENSOR_H */
