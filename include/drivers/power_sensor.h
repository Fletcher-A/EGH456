/*
 * power_sensor.h — DRV8323 phase-current sensing -> motor power (W).
 *
 *  Power = V * I_total,  V = 24 V,  I_total = |Ia| + |Ib| + |Ic|
 *  Only SOA / SOB are wired; the third phase is inferred via
 *  Kirchhoff:  Ic = -(Ia + Ib).
 *
 * Pipeline (matches spec 2.2 architecture):
 *   Timer0A  -> ADC1 SS0 trigger -> ADC1SS0 ISR
 *       -> ISR pushes PowerSampleRaw_t into xPowerRawQueue
 *       -> sensor_task drains, converts, filters, publishes
 *
 * Hardware assumptions (verify against the BoosterPack schematic):
 *   SOA  -> PE0  (ADC1 channel AIN3)
 *   SOB  -> PE1  (ADC1 channel AIN2)
 *   Shunt resistor      = 7 mOhm
 *   CSA gain            = 10 V/V    (DRV8323 default after reset)
 *   Bias (midrail)      = 1.65 V    (3.3 V / 2)
 *   ADC                 = 12 bits, VREF = 3.3 V
 *      -> 1 count = 0.806 mV  -> ~11.5 mA at the motor wire
 */
#ifndef POWER_SENSOR_H
#define POWER_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define POWER_SENSOR_VOLTAGE_V          24.0f
#define POWER_SENSOR_SAMPLE_HZ          1000        /* >= spec's 150 Hz */
#define POWER_SENSOR_SHUNT_OHM          0.007f
#define POWER_SENSOR_CSA_GAIN           10.0f
#define POWER_SENSOR_VREF_V             3.3f
#define POWER_SENSOR_ADC_MAX_COUNTS     4095
#define POWER_SENSOR_BIAS_V             1.65f

void  power_sensor_init(void);

/* Helper exposed for the sensor task — converts a (ia, ib) ADC sample
 * pair into total current magnitude in amps. The third phase is
 * inferred. */
float power_sensor_counts_to_amps(uint16_t ia_counts, uint16_t ib_counts);

#endif /* POWER_SENSOR_H */
