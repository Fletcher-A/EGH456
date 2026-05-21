/*
 * motor_driver.c — MotorLib integration + DRV8323 hall lines (PM3, PH2, PN2).
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "drivers/motor_driver.h"
#include "drivers/speed_sensor.h"
#include "utils/uartstdio.h"
#include "driver_lib/gpio.h"
#include "driver_lib/sysctl.h"
#include "driver_lib/interrupt.h"
#include "motorlib.h"

#define MOTOR_PWM_PERIOD_US  50u
#define MOTOR_MAX_COMMAND_RPM 4000

/* EGH456 Motor Adapter v2.2: DRV8323 nFAULT -> PL0 (active low, red LED). */
#define DRV_NFAULT_PERIPH   SYSCTL_PERIPH_GPIOL
#define DRV_NFAULT_PORT     GPIO_PORTL_BASE
#define DRV_NFAULT_PIN      GPIO_PIN_0

static bool     s_motorlib_ok = false;
static uint16_t s_duty_percent = 0;
static bool     s_nfault_gpio_ok = false;

/* MotorLib is not re-entrant: hall ISRs call updateMotor() while this task
 * calls setDuty(). Mask interrupts briefly for task-context MotorLib calls. */
static void prvMotorLibTaskEnter(void)
{
    IntMasterDisable();
}

static void prvMotorLibTaskExit(void)
{
    IntMasterEnable();
}

static void prvInitDrvNfaultInput(void)
{
    SysCtlPeripheralEnable(DRV_NFAULT_PERIPH);
    while (!SysCtlPeripheralReady(DRV_NFAULT_PERIPH)) {}
    GPIODirModeSet(DRV_NFAULT_PORT, DRV_NFAULT_PIN, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(DRV_NFAULT_PORT, DRV_NFAULT_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    s_nfault_gpio_ok = true;
}

bool motor_driver_hardware_fault_active(void)
{
    if (!s_nfault_gpio_ok)
    {
        return false;
    }
    return (GPIOPinRead(DRV_NFAULT_PORT, DRV_NFAULT_PIN) & DRV_NFAULT_PIN) == 0;
}

void motor_driver_init(void)
{
    prvInitDrvNfaultInput();
    s_motorlib_ok = initMotorLib(MOTOR_PWM_PERIOD_US);
    UARTprintf("MotorLib init: %s\n", s_motorlib_ok ? "OK" : "FAILED");
    setDuty(0);
    disableMotor();
    speed_sensor_init();
}

void motor_driver_start(void)
{
    bool ha, hb, hc;

    prvMotorLibTaskEnter();
    setDuty(0);
    speed_sensor_read_halls(&ha, &hb, &hc);
    updateMotor(ha, hb, hc);
    enableMotor();
    prvMotorLibTaskExit();
}

void motor_driver_stop(bool brakeHard)
{
    prvMotorLibTaskEnter();
    setDuty(0);
    stopMotor(brakeHard);
    disableMotor();
    prvMotorLibTaskExit();
}

void motor_driver_estop(void)
{
    prvMotorLibTaskEnter();
    setDuty(0);
    disableMotor();
    prvMotorLibTaskExit();
}

void motor_driver_set_speed_rpm(int32_t rpm)
{
    uint16_t period = MOTOR_PWM_PERIOD_US;
    uint32_t us = 0;

    if (rpm > 0)
    {
        if (rpm > MOTOR_MAX_COMMAND_RPM)
        {
            rpm = MOTOR_MAX_COMMAND_RPM;
        }
        us = ((uint32_t)rpm * (uint32_t)period) / (uint32_t)MOTOR_MAX_COMMAND_RPM;
        if (us > period)
        {
            us = period;
        }
        /* Minimum breakdown torque (~10 % PWM) so low slider % still spins. */
        if (us < 5u)
        {
            us = 5u;
        }
    }

    prvMotorLibTaskEnter();
    setDuty((uint16_t)us);
    s_duty_percent = (uint16_t)((us * 100u) / period);
    motor_driver_update_commutation();
    prvMotorLibTaskExit();
}

uint16_t motor_driver_get_duty_percent(void)
{
    return s_duty_percent;
}

void motor_driver_update_commutation(void)
{
    bool ha, hb, hc;
    speed_sensor_read_halls(&ha, &hb, &hc);
    updateMotor(ha, hb, hc);
}

int32_t motor_driver_get_rpm(void)
{
    return speed_sensor_get_rpm();
}

MotorState_t motor_driver_get_state(void)
{
    return MOTOR_STATE_IDLE;
}

bool motor_driver_is_ready(void)
{
    return s_motorlib_ok;
}
