/*
 * motor_driver.h — Modular motor API (assignment 2.1.7).
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "shared.h"

void          motor_driver_init(void);
void          motor_driver_start(void);
void          motor_driver_stop(bool brakeHard);
void          motor_driver_disable_drive(void);
void          motor_driver_estop(void);
void          motor_driver_update_commutation(void);
uint16_t      motor_driver_feedforward_duty_pct(int32_t rpm);
void          motor_driver_set_duty_percent(uint16_t duty_pct);
void          motor_driver_kickstart(void);
void          motor_driver_openloop_step(void);
void          motor_driver_openloop_reset(void);
void          motor_driver_set_speed_rpm(int32_t rpm);
uint16_t      motor_driver_get_duty_percent(void);
int32_t       motor_driver_get_rpm(void);
MotorState_t  motor_driver_get_state(void);
bool          motor_driver_is_ready(void);
bool          motor_driver_hardware_fault_active(void);
bool          motor_driver_try_clear_hardware_fault(void);

#endif /* MOTOR_DRIVER_H */
