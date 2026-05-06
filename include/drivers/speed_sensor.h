/*
 * speed_sensor.h — RPM measurement from hall-sensor edges
 *                   (assignment 2.2.1, sensor 2).
 *
 * The hall ISR (in motor_driver.c) increments an edge counter on each
 * commutation. This module periodically samples that counter and turns
 * it into RPM.
 */
#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <stdint.h>

void    speed_sensor_init(void);
int32_t speed_sensor_get_rpm(void);   /* latest filtered RPM */
void    speed_sensor_tick(uint32_t period_ms);  /* call from a periodic task */

#endif /* SPEED_SENSOR_H */
