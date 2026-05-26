/*
 * speed_sensor.h — Hall-effect RPM measurement.
 *
 * Pipeline (matches spec 2.2 architecture):
 *   Hall transition  -> GPIO ISR (per port)
 *       -> ISR atomically increments edge counter (zero buffering)
 *       -> 100 Hz software timer / task calls speed_sensor_tick(10)
 *       -> tick snapshots + zeros the counter, divides into RPM,
 *          applies a low-pass filter
 *
 * Hardware assumptions (verify against the BoosterPack schematic;
 * change the pin defines in speed_sensor.c if your board differs):
 *   Hall A  -> PM3   (GPIO Port M)
 *   Hall B  -> PH2   (GPIO Port H)
 *   Hall C  -> PN2   (GPIO Port N)
 *
 * Edges per mechanical revolution = 6 hall transitions per electrical
 * revolution * pole pairs of the motor. The default below matches the
 * Anaheim BLY172S-24V-4000 (4 pole pairs -> 24 edges/rev). Tune the
 * SPEED_EDGES_PER_REV macro for your motor.
 */
#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stdbool.h>

#define SPEED_EDGES_PER_REV     24      /* 6 * pole_pairs */
#define SPEED_RPM_DISPLAY_INTEGRATE_TICKS  5
#define SPEED_RPM_CTRL_INTEGRATE_TICKS     3

void    speed_sensor_init(void);
void    speed_sensor_read_halls(bool *ha, bool *hb, bool *hc);
int32_t speed_sensor_get_rpm(void);
int32_t speed_sensor_get_rpm_raw(void);
int32_t speed_sensor_get_rpm_display(void);
void    speed_sensor_tick(uint32_t period_ms);
void    speed_sensor_reset_filter(void);
void    speed_sensor_seed_filter(int32_t rpm);
void    speed_sensor_hall_irq_enable(bool enable);
bool    speed_sensor_hall_irq_enabled(void);

/* Three port ISRs (wired in startup_gcc.c). Each one just increments
 * the shared edge counter — extremely short, no FreeRTOS calls. */
void HallPortMIntHandler(void);
void HallPortHIntHandler(void);
void HallPortNIntHandler(void);

#endif /* SPEED_SENSOR_H */
