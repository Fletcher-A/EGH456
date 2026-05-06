/*
 * motor_driver.h — Modular motor API (assignment 2.1.7).
 *
 * Wraps MotorLib (initMotorLib/setDuty/updateMotor/stopMotor) and the
 * hall-sensor speed logic into a clean interface that motor_task uses.
 */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "shared.h"     /* for MotorState_t */

void          motor_driver_init(void);          /* sets up MotorLib + halls */
void          motor_driver_start(void);         /* IDLE -> STARTING kickstart */
void          motor_driver_stop(bool brakeHard);
void          motor_driver_estop(void);         /* immediate fault */
void          motor_driver_set_duty(uint16_t duty);
int32_t       motor_driver_get_rpm(void);       /* latest filtered speed */
MotorState_t  motor_driver_get_state(void);

/* Hall ISR — invoked from the GPIO Port handler in startup_gcc.c. */
void HallSensorHandler(void);

#endif /* MOTOR_DRIVER_H */
