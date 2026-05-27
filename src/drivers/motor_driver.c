/*
 * motor_driver.c — MotorLib integration + DRV8323 hall lines (PM3, PH2, PN2).
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "drivers/motor_driver.h"
#include "drivers/speed_sensor.h"
#include "shared.h"
#include "utils/uartstdio.h"
#include "driver_lib/gpio.h"
#include "driver_lib/sysctl.h"
#include "driver_lib/interrupt.h"
#include "motorlib.h"

#define MOTOR_PWM_PERIOD_US  50u
#define MOTOR_MAX_COMMAND_RPM MAX_MOTOR_RPM

/* EGH456 Motor Adapter v2.2: DRV8323 nFAULT -> PL0 (active low, red LED). */
#define DRV_NFAULT_PERIPH   SYSCTL_PERIPH_GPIOL
#define DRV_NFAULT_PORT     GPIO_PORTL_BASE
#define DRV_NFAULT_PIN      GPIO_PIN_0

static bool     s_motorlib_ok = false;
static uint16_t s_duty_percent = 0;
static bool     s_nfault_gpio_ok = false;
static bool     s_drive_active = false;
static bool     s_commutation_enabled = false;
static uint8_t  s_openloop_idx        = 0;

/* Forward 6-step sequence (H3:H2:H1 = ha:hb:hc). */
static const uint8_t s_openloop_seq[6] = {5u, 1u, 3u, 2u, 6u, 4u};

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
#if MOTOR_ENABLE_NFAULT_MONITORING
    SysCtlPeripheralEnable(DRV_NFAULT_PERIPH);
    while (!SysCtlPeripheralReady(DRV_NFAULT_PERIPH)) {}
    GPIODirModeSet(DRV_NFAULT_PORT, DRV_NFAULT_PIN, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(DRV_NFAULT_PORT, DRV_NFAULT_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    s_nfault_gpio_ok = true;
#else
    s_nfault_gpio_ok = false;
#endif
}

bool motor_driver_hardware_fault_active(void)
{
#if !MOTOR_ENABLE_NFAULT_MONITORING
    return false;
#else
    if (!s_nfault_gpio_ok)
    {
        return false;
    }
    return (GPIOPinRead(DRV_NFAULT_PORT, DRV_NFAULT_PIN) & DRV_NFAULT_PIN) == 0;
#endif
}

bool motor_driver_try_clear_hardware_fault(void)
{
    s_commutation_enabled = false;
    s_drive_active = false;
    prvMotorLibTaskEnter();
    disableMotor();
    prvMotorLibTaskExit();
    prvMotorLibTaskEnter();
    enableMotor();
    disableMotor();
    prvMotorLibTaskExit();
    return !motor_driver_hardware_fault_active();
}

void motor_driver_init(void)
{
    prvInitDrvNfaultInput();
    s_motorlib_ok = initMotorLib(MOTOR_PWM_PERIOD_US);
#if !SERIAL_PLOT_CLEAN
    UARTprintf("MotorLib init: %s\n", s_motorlib_ok ? "OK" : "FAILED");
#endif
    setDuty(0);
    disableMotor();
    speed_sensor_init();
}

void motor_driver_openloop_reset(void)
{
    s_openloop_idx = 0;
}

void motor_driver_openloop_step(void)
{
    if (!s_drive_active || !s_commutation_enabled)
    {
        return;
    }

    uint8_t hall = s_openloop_seq[s_openloop_idx];
    bool ha = (hall & 4u) != 0;
    bool hb = (hall & 2u) != 0;
    bool hc = (hall & 1u) != 0;

    prvMotorLibTaskEnter();
    updateMotor(ha, hb, hc);
    prvMotorLibTaskExit();

    s_openloop_idx = (uint8_t)((s_openloop_idx + 1u) % 6u);
}

void motor_driver_start(void)
{
    bool ha, hb, hc;

    motor_driver_openloop_reset();
    speed_sensor_hall_irq_enable(true);
    prvMotorLibTaskEnter();
    speed_sensor_read_halls(&ha, &hb, &hc);
    updateMotor(ha, hb, hc);
    enableMotor();
    s_drive_active = true;
    s_commutation_enabled = true;
    prvMotorLibTaskExit();
    motor_driver_set_duty_percent(MOTOR_START_DUTY_PCT);
}

void motor_driver_disable_drive(void)
{
    s_commutation_enabled = false;
    speed_sensor_hall_irq_enable(false);

    if (!s_drive_active)
    {
        return;
    }

    prvMotorLibTaskEnter();
    setDuty(0);
    disableMotor();
    prvMotorLibTaskExit();
    s_duty_percent = 0;
    s_drive_active = false;
}

void motor_driver_stop(bool brakeHard)
{
    s_commutation_enabled = false;
    speed_sensor_hall_irq_enable(false);

    if (!s_drive_active)
    {
        return;
    }

    prvMotorLibTaskEnter();
    setDuty(0);
    stopMotor(brakeHard);
    disableMotor();
    prvMotorLibTaskExit();
    s_duty_percent = 0;
    s_drive_active = false;
}

void motor_driver_estop(void)
{
    motor_driver_disable_drive();
}

uint16_t motor_driver_feedforward_duty_pct(int32_t rpm)
{
    if (rpm <= 0)
    {
        return 0;
    }
    if (rpm > MOTOR_MAX_COMMAND_RPM)
    {
        rpm = MOTOR_MAX_COMMAND_RPM;
    }
    /* Scale to rated motor RPM so 4000 ref ≈ 100% duty (not 10000). */
    uint32_t pct = ((uint32_t)rpm * (uint32_t)MOTOR_MAX_DUTY_PCT) /
                   (uint32_t)MOTOR_RATED_MAX_RPM;
    if (pct > MOTOR_MAX_DUTY_PCT)
    {
        pct = MOTOR_MAX_DUTY_PCT;
    }
    return (uint16_t)pct;
}

void motor_driver_set_duty_percent(uint16_t duty_pct)
{
    uint16_t period = MOTOR_PWM_PERIOD_US;
    uint32_t us = 0;

    if (duty_pct > MOTOR_MAX_DUTY_PCT)
    {
        duty_pct = MOTOR_MAX_DUTY_PCT;
    }
    if (duty_pct > 0)
    {
        us = ((uint32_t)duty_pct * (uint32_t)period) / 100u;
        /* One timer tick (~2% at 50 us period), not 5 us (~10%). A 5 us
         * floor made low STOP ramp duties jump to ~10% and re-accelerate. */
        if (us == 0u)
        {
            us = 1u;
        }
    }

    prvMotorLibTaskEnter();
    setDuty((uint16_t)us);
    s_duty_percent = (uint16_t)((us * 100u) / period);
    if (s_commutation_enabled)
    {
        bool ha, hb, hc;
        speed_sensor_read_halls(&ha, &hb, &hc);
        updateMotor(ha, hb, hc);
    }
    prvMotorLibTaskExit();
}

void motor_driver_kickstart(void)
{
    motor_driver_set_duty_percent(MOTOR_START_DUTY_PCT);
}

void motor_driver_set_speed_rpm(int32_t rpm)
{
    motor_driver_set_duty_percent(motor_driver_feedforward_duty_pct(rpm));
}

uint16_t motor_driver_get_duty_percent(void)
{
    return s_duty_percent;
}

void motor_driver_update_commutation(void)
{
    if (!s_commutation_enabled)
    {
        return;
    }

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
    return g_motor_state;
}

bool motor_driver_is_ready(void)
{
    return s_motorlib_ok;
}
