/*
 * power_sensor.c — Read DRV8323 phase-current ADC outputs and compute
 *                   motor current and power.
 *
 * Hardware:
 *   - DRV8323 SOA / SOB pins drive analog values proportional to
 *     phase A/B currents.
 *   - Only two phases are wired to the LaunchPad ADC; infer the third
 *     using Kirchhoff:  I_a + I_b + I_c = 0.
 *
 * Sampling:
 *   - Sensor task calls power_sensor_read_w() at ≥150 Hz.
 *   - Recommended: trigger ADC on the PWM mid-cycle for clean samples.
 */

#include "drivers/power_sensor.h"
/*-----------------------------------------------------------*/

void power_sensor_init(void)
{
    /* TODO: enable ADC peripheral.
     * TODO: configure ADC sequencer + steps for the two phase channels.
     * TODO: set up trigger source (processor or PWM trigger).
     */
}

float power_sensor_read_current_a(void)
{
    /* TODO: trigger ADC, wait for sequence completion, read both
     *       samples, convert ADC counts -> amps using the sense
     *       resistor and amp gain, return |Ia| + |Ib| + |Ic|.
     */
    return 0.0f;
}

float power_sensor_read_w(void)
{
    /* TODO: return POWER_SENSOR_VOLTAGE_V * power_sensor_read_current_a(); */
    return 0.0f;
}
