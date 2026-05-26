/*
 * power_sensor.h — DRV8323 phase-current sensing -> motor power (W).
 */
#ifndef POWER_SENSOR_H
#define POWER_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define POWER_SENSOR_VOLTAGE_V          24.0f
#define POWER_SENSOR_SAMPLE_HZ          1000
#define POWER_SENSOR_SHUNT_OHM          0.007f
#define POWER_SENSOR_CSA_GAIN           10.0f
#define POWER_SENSOR_VREF_V             3.3f
#define POWER_SENSOR_ADC_MAX_COUNTS     4095
#define POWER_SENSOR_BIAS_V             1.65f
#define POWER_SENSOR_I_SCALE            0.55f
#define POWER_SENSOR_ZERO_SAMPLES       256u
#define POWER_SENSOR_W_DISPLAY_FLOOR    8.0f
#define POWER_SENSOR_W_MAX              350.0f

void  power_sensor_init(void);
void  power_sensor_adc_start(void);
void  power_sensor_drain_raw_queue(void);
void  power_sensor_reset_zero_calibration(void);
bool  power_sensor_zero_ready(void);
void  power_sensor_note_idle_sample(uint16_t ia_counts, uint16_t ib_counts);
float power_sensor_counts_to_amps(uint16_t ia_counts, uint16_t ib_counts);
float power_sensor_amps_to_watts(float i_dc_a);

#endif /* POWER_SENSOR_H */
