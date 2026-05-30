# EGH456 Electric Vehicle Embedded System

FreeRTOS firmware for TM4C1294 + DRV8323 + EGH456 motor connector (Assignment 2).

**Optional sensors implemented:** BMI160 (acceleration E-stop) + SHT31 (temperature/humidity). **VL53 distance not used** — Night lux threshold replaces distance on the Thresholds tab (document in report).

Layout: `src/tasks/` (motor / sensor / gui / fault), `src/drivers/`, `src/main.c`, `lib/motorlib/`, `lib/grlib/`.

---

## Assignment compliance summary

| Section | Status |
|---------|--------|
| 2.1 Motor control (state machine, PI, ramps, E-stop, RTOS, API) | Done |
| 2.1.8 E-stop: power + accel | Done |
| 2.1.8 E-stop: distance | N/A (no ToF) — note in report |
| 2.1.9 Motor UART plot (`desired, reference, actual, duty`) | Done @ 100 Hz (`M,...` lines) |
| 2.2 Sensing (power, speed, light, BMI160, SHT31, filters) | Done |
| 2.2.5 Sensor UART CSV @ 50 Hz | Done (`t,...` lines) |
| 2.3 GUI (control, status, plots, thresholds, ACK, day/night, cooling) | Done |
| 2.4 Advanced (BME280 pressure, nFAULT, plot toggles, fault logging) | Done |

---

## Demonstration procedure

1. **Build / flash:** `pio run -t upload`
2. **Serial plot** ([atomic14 Web Serial Plotter](https://web-serial-plotter.atomic14.com/) or monitor): **115200 baud**, close other serial tools first.
   - With `SERIAL_PLOT_CLEAN=1` (default): `#` header once, then CSV @ 50 Hz. Columns:
     `sample_number`, `desired_rpm`, `reference_rpm`, `actual_rpm`, `duty_percent`, `power_milliwatts`, `light_lux`, `acceleration_millig`, `hall_display_rpm`, `estop_active` (0 idle, 4000 when E-stop brake / fault — visible step on plot)
   - Reset board after connect so the plotter catches the `#` header (do not repeat the header in firmware).
   - Legacy mode (`-DSERIAL_PLOT_CLEAN=0`): `M,...` motor + 15-column sensor CSV (desktop Serial Plot).
3. **Power zero-cal:** leave **Idle** ~0.5 s before **START** (SOA/SOB ADC offset).
4. **Thresholds tab:** Power **300 W** default (range 50–300 W) for full-speed bench test; use **150 W** for safety demo if required.
5. **START** → **Running** → raise slider to 4000 RPM; verify ramps (500 RPM/s) on motor plot.
6. **E-stop tests:** lower power threshold while loaded; bump accelerometer threshold test on bench.
7. **ACK** clears **Fault Latched** after E-stop.

---

## Motor control (2.1)

- States: Idle, Starting, Running, Stopping, E-Stop Braking, Fault Latched
- PI closed-loop in Running; open-loop commutation + kickstart in Starting
- Ramps: 500 / 500 / 1000 RPM/s (accel / stop / E-stop)
- API: `motor_driver_init/start/stop/set_duty_percent/set_speed_rpm/estop/get_rpm/get_state`
- **Timer0 = MotorLib PWM only** — power ADC uses **Timer3A → ADC1**

---

## Sensing (2.2)

| Sensor | Rate | Filter | E-stop |
|--------|------|--------|--------|
| Power (DRV8323 SOA/SOB) | 1 kHz ADC | 32-tap MAF | `EVT_ESTOP_POWER` if W > threshold & RPM ≥ 150 |
| Speed (halls) | 100 Hz | exp LPF | (control + stop detect) |
| OPT3001 lux | 5 Hz | 8-tap MAF | Night LED < threshold |
| BMI160 | 200 Hz | 8-tap MAF on \|a\| | `EVT_ESTOP_ACCEL` |
| SHT31 T/RH | 1 Hz | — | Cooling LED |
| BME280 pressure | 1 Hz | — | plot only |

**Power:** `P = 24 V × max(|Ia|,|Ib|,|Ic|)` after idle zero-cal (not sum of three phases).

---

## GUI (2.3)

- **Control:** START / STOP / ACK, state, RPM, power, day/night + cooling LEDs, clock/date
- **Plots:** 5 s window — RPM, power (0–350 W), lux, accel, T, RH, pressure
- **Thresholds:** Power 50–300 W, Accel, Night lux, Cool °C
- **Sensors:** live readouts + plot trace toggles

---

## Build

```bash
pio run
pio run -t upload
pio device monitor
```

Bench without current sense (motor/GUI only):

```ini
-DMOTOR_ENABLE_POWER_SENSOR=0
```

in `platformio.ini`.

---

## Report pointers

- RTOS diagram: tasks Motor(+Speed), Sensor(+AccSamp), GUI, Fault; queues `xMotorQueue`, `xSensorQueue`, `xPowerRawQueue`, `xAccelRawQueue`; `xSystemEvents`
- Filter justification: MAF on power/lux/accel; exp LPF on RPM; no ISR filtering
- Distance E-stop: not implemented (sensor choice BMI160 + SHT31)
- Include Serial Plot captures of `M,...` and `t,...` streams

---

## Reference documents

See assignment PDF and TI / Bosch / Sensirion datasheets listed in course materials.
