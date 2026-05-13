/*
 * power_sensor.c — DRV8323 phase-current ADC capture.
 *
 * See power_sensor.h for the pipeline diagram and hardware assumptions.
 *
 * Why ADC1 (not ADC0): the touchscreen driver in gui_task.c owns
 * ADC0 sample-sequencer 3 for the resistive touch reads. Using ADC1
 * keeps the two subsystems independent so their interrupts and
 * sequencers can never collide.
 *
 * Why Timer0A as the ADC trigger: spec mandates raw samples captured
 * by an ISR (not polled inside a task). Driving the sequencer from a
 * hardware timer keeps the sampling jitter-free and means we never
 * need a separate timer ISR — Timer0A fires straight into the ADC
 * and the ADC1SS0 ISR is the only one that wakes the CPU.
 */

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
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

/*-----------------------------------------------------------*/

void power_sensor_init(void)
{
    /* ---- 1. GPIO PE0 / PE1 as analog inputs ---------------- */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* ---- 2. ADC1 SS0: 2 steps, triggered by Timer0A -------- */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1)) {}

    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 0);

    /* Step 0: SOA  -> AIN3 (PE0) */
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3);
    /* Step 1: SOB  -> AIN2 (PE1), last step + raise interrupt */
    ADCSequenceStepConfigure(ADC1_BASE, 0, 1,
                             ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC1_BASE, 0);
    ADCIntClear(ADC1_BASE, 0);
    ADCIntEnable(ADC1_BASE, 0);
    IntEnable(INT_ADC1SS0);

    /* ---- 3. Timer0A: periodic, drives ADC1 SS0 -------------- */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A,
                 g_ui32SysClock / POWER_SENSOR_SAMPLE_HZ - 1);
    /* Output a trigger pulse to the ADC at the end of each timer period. */
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

/*-----------------------------------------------------------*/
/* ADC1 SS0 ISR — drains the two ADC FIFO samples and pushes a
 * raw-counts struct onto the queue. No floating point or filtering
 * here — that lives in the sensor task. */
void ADC1Seq0IntHandler(void)
{
    ADCIntClear(ADC1_BASE, 0);

    uint32_t buf[2];
    if (ADCSequenceDataGet(ADC1_BASE, 0, buf) != 2) return;

    PowerSampleRaw_t s = {
        .ia_counts = (uint16_t)buf[0],
        .ib_counts = (uint16_t)buf[1]
    };

    /* xQueueSendFromISR drops the sample if the queue is full —
     * we'd rather lose one than block in interrupt context. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xPowerRawQueue != NULL)
    {
        xQueueSendFromISR(xPowerRawQueue, &s, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

static float prvCountsToAmps(uint16_t counts)
{
    /* counts -> volts at the ADC pin */
    float v = ((float)counts * POWER_SENSOR_VREF_V) /
              (float)POWER_SENSOR_ADC_MAX_COUNTS;
    /* CSA output is (I * R_shunt * gain) + bias. Solve for I. */
    return (v - POWER_SENSOR_BIAS_V) /
           (POWER_SENSOR_SHUNT_OHM * POWER_SENSOR_CSA_GAIN);
}

static float prvAbsF(float v) { return v < 0 ? -v : v; }

float power_sensor_counts_to_amps(uint16_t ia_counts, uint16_t ib_counts)
{
    float ia = prvCountsToAmps(ia_counts);
    float ib = prvCountsToAmps(ib_counts);
    float ic = -(ia + ib);                     /* Kirchhoff */
    return prvAbsF(ia) + prvAbsF(ib) + prvAbsF(ic);
}
