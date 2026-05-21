/*
 * motor_driver.h — Modular motor API (assignment 2.1.7).
 *
 * Wraps MotorLib (initMotorLib/setDuty/updateMotor/stopMotor) and the
 * hall-sensor GPIO used for speed and commutation updates.
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "shared.h"

void          motor_driver_init(void);
void          motor_driver_start(void);
void          motor_driver_stop(bool brakeHard);
void          motor_driver_estop(void);
void          motor_driver_update_commutation(void);
void          motor_driver_set_speed_rpm(int32_t rpm);
uint16_t      motor_driver_get_duty_percent(void);
int32_t       motor_driver_get_rpm(void);
MotorState_t  motor_driver_get_state(void);
bool          motor_driver_is_ready(void);
bool          motor_driver_hardware_fault_active(void);

#endif /* MOTOR_DRIVER_H */
