/*
 * power_sensor.c — DRV8323 phase-current ADC capture.
 *
 * ADC1 SS0 on PE0/PE1 (SOA/SOB). Timer3A triggers ADC (Timer0 = MotorLib PWM).
 * P = 24 V * Idc. Idle zero-cal on raw counts removes SOA/SOB offset.
 */

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "task.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driver_lib/adc.h"
#include "driver_lib/gpio.h"
#include "driver_lib/sysctl.h"
#include "driver_lib/timer.h"
#include "driver_lib/interrupt.h"

#include "drivers/power_sensor.h"
#include "shared.h"

extern uint32_t g_ui32SysClock;

static uint32_t s_ia_zero      = 2048u;
static uint32_t s_ib_zero      = 2048u;
static uint16_t s_zero_samples = 0;
static bool     s_zero_ready   = false;

/*-----------------------------------------------------------*/

void power_sensor_init(void)
{
    power_sensor_reset_zero_calibration();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1)) {}

    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0,
                             ADC_CTL_CH3 | ADC_CTL_SHOLD_32);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 1,
                             ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END |
                             ADC_CTL_SHOLD_32);
    ADCSequenceEnable(ADC1_BASE, 0);
    ADCIntClear(ADC1_BASE, 0);
    ADCIntEnable(ADC1_BASE, 0);
    IntPrioritySet(INT_ADC1SS0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_ADC1SS0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3)) {}
    TimerDisable(TIMER3_BASE, TIMER_A);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A,
                 g_ui32SysClock / POWER_SENSOR_SAMPLE_HZ - 1);
    TimerADCEventSet(TIMER3_BASE, TIMER_ADC_TIMEOUT_A);
    TimerControlTrigger(TIMER3_BASE, TIMER_A, true);
    /* Timer left off until sensor task finishes boot init (see adc_start). */
}

void power_sensor_adc_start(void)
{
    TimerEnable(TIMER3_BASE, TIMER_A);
}

void power_sensor_drain_raw_queue(void)
{
    PowerSampleRaw_t discard;

    if (xPowerRawQueue == NULL)
    {
        return;
    }
    while (xQueueReceive(xPowerRawQueue, &discard, 0) == pdPASS)
    {
    }
}

void power_sensor_reset_zero_calibration(void)
{
    s_ia_zero       = 2048u;
    s_ib_zero       = 2048u;
    s_zero_samples  = 0;
    s_zero_ready    = false;
}

bool power_sensor_zero_ready(void)
{
    return s_zero_ready;
}

void power_sensor_note_idle_sample(uint16_t ia_counts, uint16_t ib_counts)
{
    s_ia_zero = (s_ia_zero * 31u + (uint32_t)(ia_counts & 0xFFFu)) / 32u;
    s_ib_zero = (s_ib_zero * 31u + (uint32_t)(ib_counts & 0xFFFu)) / 32u;
    if (s_zero_samples < 0xFFFFu)
    {
        s_zero_samples++;
    }
    if (s_zero_samples >= POWER_SENSOR_ZERO_SAMPLES)
    {
        s_zero_ready = true;
    }
}

/*-----------------------------------------------------------*/

void ADC1Seq0IntHandler(void)
{
    ADCIntClear(ADC1_BASE, 0);

    uint32_t buf[2];
    if (ADCSequenceDataGet(ADC1_BASE, 0, buf) != 2)
    {
        return;
    }

    PowerSampleRaw_t s = {
        .ia_counts = (uint16_t)(buf[0] & 0xFFFu),
        .ib_counts = (uint16_t)(buf[1] & 0xFFFu)
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xPowerRawQueue != NULL)
    {
        xQueueSendFromISR(xPowerRawQueue, &s, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*-----------------------------------------------------------*/

static float prvAbsF(float v) { return v < 0.0f ? -v : v; }

static float prvCountsToSignedAmps(uint16_t counts, uint32_t zero_counts)
{
    float v = ((float)counts * POWER_SENSOR_VREF_V) /
              (float)POWER_SENSOR_ADC_MAX_COUNTS;
    float v0;

    if (s_zero_ready)
    {
        v0 = ((float)zero_counts * POWER_SENSOR_VREF_V) /
             (float)POWER_SENSOR_ADC_MAX_COUNTS;
    }
    else
    {
        v0 = POWER_SENSOR_BIAS_V;
    }

    return (v - v0) / (POWER_SENSOR_SHUNT_OHM * POWER_SENSOR_CSA_GAIN);
}

float power_sensor_counts_to_amps(uint16_t ia_counts, uint16_t ib_counts)
{
    if (!s_zero_ready)
    {
        return 0.0f;
    }

    float ia = prvCountsToSignedAmps(ia_counts & 0xFFFu, s_ia_zero);
    float ib = prvCountsToSignedAmps(ib_counts & 0xFFFu, s_ib_zero);
    float i_dc = 0.5f * (prvAbsF(ia) + prvAbsF(ib));
    return i_dc * POWER_SENSOR_I_SCALE;
}

float power_sensor_amps_to_watts(float i_dc_a)
{
    float p = POWER_SENSOR_VOLTAGE_V * i_dc_a;
    if (p < POWER_SENSOR_W_DISPLAY_FLOOR)
    {
        p = 0.0f;
    }
    if (p > POWER_SENSOR_W_MAX)
    {
        p = POWER_SENSOR_W_MAX;
    }
    return p;
}
