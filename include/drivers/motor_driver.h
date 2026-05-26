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
/* Cut PWM + disable DRV8323 without stopMotor() (safe at end of STOP ramp). */
void          motor_driver_disable_drive(void);
void          motor_driver_estop(void);
void          motor_driver_update_commutation(void);
/* Feedforward duty from RPM setpoint (0..MOTOR_MAX_DUTY_PCT). */
uint16_t      motor_driver_feedforward_duty_pct(int32_t rpm);
/* Apply duty 0..100 % (closed-loop output). */
void          motor_driver_set_duty_percent(uint16_t duty_pct);
/* Minimum duty for Starting (14% — spec startup torque). */
void          motor_driver_kickstart(void);
/* Step through 6 hall states while rotor is stationary (MotorLib needs this). */
void          motor_driver_openloop_step(void);
void          motor_driver_openloop_reset(void);
/* Open-loop map: ramped RPM reference -> duty (primary actuator). */
void          motor_driver_set_speed_rpm(int32_t rpm);
/* Returns PWM duty 0..100 % (for GUI / debug). */
uint16_t      motor_driver_get_duty_percent(void);
int32_t       motor_driver_get_rpm(void);
MotorState_t  motor_driver_get_state(void);
bool          motor_driver_is_ready(void);
/* True when DRV8323 nFAULT is asserted (active low; red LED on motor board). */
bool          motor_driver_hardware_fault_active(void);
/* Toggle disable/enable; some DRV8323 faults clear when nFAULT releases. */
bool          motor_driver_try_clear_hardware_fault(void);

#endif /* MOTOR_DRIVER_H */
