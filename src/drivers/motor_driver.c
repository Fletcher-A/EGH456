/*
 * motor_driver.c — MotorLib wrapper + hall-edge speed measurement.
 *
 * MotorLib API (from motorlib.h):
 *   bool     initMotorLib(uint16_t pwm_period);
 *   void     setDuty(uint16_t duty);
 *   void     updateMotor(bool ha, bool hb, bool hc);
 *   void     stopMotor(bool brakeType);
 *   void     enableMotor(void);
 *   void     disableMotor(void);
 *   uint16_t getMotorPWMPeriod(void);
 */

#include "drivers/motor_driver.h"
#include "shared.h"
#include "motorlib.h"
/*-----------------------------------------------------------*/

void motor_driver_init(void)
{
    /* TODO: enable hall-sensor GPIO ports as inputs.
     * TODO: configure GPIO interrupts (both edges) on hall A/B/C.
     * TODO: initMotorLib(PWM_PERIOD); setDuty(0);
     */
}

void motor_driver_start(void)
{
    /* TODO: enableMotor();
     * TODO: read hall A/B/C, call updateMotor(...) once for kickstart.
     * TODO: setDuty(initial); set state to STARTING.
     */
}

void motor_driver_stop(bool brakeHard)
{
    (void)brakeHard;
    /* TODO: setDuty(0); stopMotor(brakeHard); set state to IDLE. */
}

void motor_driver_estop(void)
{
    /* TODO: disableMotor(); set state to FAULT_LATCHED. */
}

void motor_driver_set_duty(uint16_t duty)
{
    (void)duty;
    /* TODO: setDuty(duty); */
}

int32_t motor_driver_get_rpm(void)
{
    /* TODO: return latest filtered RPM from speed_sensor. */
    return 0;
}

MotorState_t motor_driver_get_state(void)
{
    /* TODO: return cached state. */
    return MOTOR_STATE_IDLE;
}

/* HallSensorHandler is defined in tasks/motor_task.c (matches the
 * supplied motorlib example). */
