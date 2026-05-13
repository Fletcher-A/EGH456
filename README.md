# EGH456 — Electric Vehicle Embedded System

Group assignment build. Task files in `src/tasks/`, sensor + I/O drivers in
`src/drivers/`, RTOS objects defined in `src/main.c`, inter-task contracts
in `src/shared.h`.

---

## Status

| Subsystem | Status |
|---|---|
| GUI (spec 2.3) | **Complete** — 4 tabs, threshold editor, day/night LED, plots |
| Sensing — Light (OPT3001) | **Done** — 5 Hz polled, 8-tap MAF |
| Sensing — Accel (BMI160) | **Done** — 200 Hz ISR pipeline, 8-tap MAF, E-Stop wired |
| Sensing — T/RH/P (BME280) | **Done** — 1 Hz polled (substituted for spec-listed SHT31) |
| Sensing — Power (DRV8323 ADC) | **Code complete, init commented out** — waits on motor BoosterPack |
| Sensing — Speed (hall) | **Code complete, init commented out** — waits on hall wiring |
| Motor control | **Simulation only** — state machine + ramped RPM; real MotorLib commutation TODO |
| Fault task | Skeleton — logs faults, no actions yet |

---

## What's still to do

1. **Plug in the motor BoosterPack**, then in `src/tasks/sensor_task.c` uncomment
   `power_sensor_init()`, and in `src/tasks/motor_task.c` uncomment the
   `prvSpeedTask` xTaskCreate.
2. **Real motor commutation** — replace the simulation in `prvMotorTask()`
   steps 3–6 with MotorLib (`setDuty`, `updateMotor`, hall reads).
3. **GUI** → consume sensor-task power instead of motor-task simulated power
   (`gui_task.c`: change `g_power_w = motor_msg.power_watts` back to
   `sensor_msg.power_watts`).
4. **Tighten thresholds** for the final demo. Day/night already at `<5 lux`
   per spec when `DAYNIGHT_DESK_TEST` is undefined.
5. **Pinout verify on hardware** — `SOA/SOB` ADC channels, hall ports,
   `SPEED_EDGES_PER_REV` (pole pairs of the motor).
6. **Report**: justify BME280-instead-of-SHT31 substitution and filter
   choices (MAF for accel/light/power, exponential LPF for RPM).

---

## Build / Flash / Monitor

```
pio run                # build
pio run -t upload      # flash via ICDI
pio device monitor     # opens at 115200 (pinned in platformio.ini)
```

Serial output is a CSV plot stream — pipe into Tera Term Plotter, Serial
Plotter, or `pandas.read_csv()` for the report figures.

---

## Inter-task data contract

Each task block below lists **what flows into it** (with the originator)
and **what flows out** (with the consumer). RTOS object names match
`shared.h` exactly.

### `prvMotorTask` (priority 4) — `tasks/motor_task.c`

**Reads in:**
| Source | Object | Carries |
|---|---|---|
| `prvGuiTask` (slider) | `xCommandQueue` (int32_t) | Desired RPM 0..4000 |
| `prvGuiTask` (Start button) | `xSystemEvents` bit `EVT_USER_START` | Idle → Starting |
| `prvGuiTask` (Stop button) | `xSystemEvents` bit `EVT_USER_STOP` | any → Idle |
| `prvGuiTask` (ACK button) | `xSystemEvents` bit `EVT_USER_ESTOP_ACK` | Fault Latched → Idle |
| `prvSensorTask` (power threshold) | `xSystemEvents` bit `EVT_ESTOP_POWER` | any → E-Stop Braking |
| `prvSensorTask` (accel threshold) | `xSystemEvents` bit `EVT_ESTOP_ACCEL` | any → E-Stop Braking |
| `prvSensorTask` (distance threshold) | `xSystemEvents` bit `EVT_ESTOP_DISTANCE` | any → E-Stop Braking |

**Publishes out:**
| Object | Field | Consumer |
|---|---|---|
| `xMotorQueue` (MotorMsgObj) | `state` (MotorState_t) | `prvGuiTask` — state-text canvas + status indicator colour |
| `xMotorQueue` | `rpm_actual` | `prvGuiTask` — RPM readout + RPM plot trace |
| `xMotorQueue` | `pwm_duty` | (currently unused; reserved for real motor) |
| `xMotorQueue` | `power_watts` | `prvGuiTask` — power readout (simulated; will move to sensor msg) |

---

### `prvSensorTask` (priority 3) — `tasks/sensor_task.c`

**Reads in:**
| Source | Object | Carries |
|---|---|---|
| ADC1 SS0 ISR (Timer0A-triggered) | `xPowerRawQueue` (PowerSampleRaw_t) | Raw `ia/ib` counts at 1 kHz — *currently dormant; init disabled* |
| `prvAccelSamplerTask` (Timer2A-driven) | `xAccelRawQueue` (AccelSampleRaw_t) | Raw BMI160 X/Y/Z LSBs at 200 Hz |
| GUI Thresholds tab (volatiles) | `g_thresh_power_w` | Power E-Stop limit (W) |
| GUI Thresholds tab | `g_thresh_accel_g` | Accel E-Stop limit (g) |
| GUI Thresholds tab | `g_thresh_distance_mm` | Distance E-Stop limit (mm) |

**Publishes out:**
| Object | Fields | Consumer |
|---|---|---|
| `xSensorQueue` (SensorMsgObj) | `light_lux` | `prvGuiTask` — Sensors lux text, day/night LED, lux plot |
| `xSensorQueue` | `accel_x_g`, `accel_y_g`, `accel_z_g` | `prvGuiTask` — Sensors X/Y/Z text, accel plot trace |
| `xSensorQueue` | `accel_total_g` | (filtered magnitude — currently logged only) |
| `xSensorQueue` | `temp_c`, `humidity_pct`, `pressure_hpa`, `bme_ok` | `prvGuiTask` — Sensors T/H/P text |
| `xSensorQueue` | `power_watts` | `prvGuiTask` (will be the canonical source once motor is real) |
| `xSystemEvents` bit `EVT_NIGHT_DETECTED` | — | `prvGuiTask` (currently re-computed locally with hysteresis) |
| `xSystemEvents` bit `EVT_ESTOP_POWER` | — | `prvMotorTask` |
| `xSystemEvents` bit `EVT_ESTOP_ACCEL` | — | `prvMotorTask` |
| UART CSV stream | t,p_raw,p_filt,lux_raw,lux_filt,ax,ay,az,acc_raw,acc_filt,t,h,p,rpm_raw,rpm_filt | Serial-plot host |

---

### `prvAccelSamplerTask` (priority 5) — `tasks/sensor_task.c`

**Reads in:**
| Source | Object | Carries |
|---|---|---|
| Timer2A ISR @ 200 Hz | `s_xAccelTickSem` (binary semaphore) | "Sample now" tick |

**Publishes out:**
| Object | Field | Consumer |
|---|---|---|
| `xAccelRawQueue` (AccelSampleRaw_t) | `ax_raw`, `ay_raw`, `az_raw` (int16) | `prvSensorTask` |

---

### `prvSpeedTask` (priority 5, **currently disabled**) — `tasks/motor_task.c`

Will run once hall sensors are physically wired (see "What's still to do").

**Reads in:** hall edge counter `g_edges` (incremented by `HallPortMIntHandler` /
`HallPortHIntHandler` / `HallPortNIntHandler` in `drivers/speed_sensor.c`).

**Publishes out:** filtered RPM accessible via `speed_sensor_get_rpm()` —
consumed by `prvMotorTask` (closed-loop control) and the CSV stream.

---

### `prvGuiTask` (priority 2) — `tasks/gui_task.c`

**Reads in:**
| Source | Object | Carries |
|---|---|---|
| `prvMotorTask` | `xMotorQueue` (MotorMsgObj) | `state`, `rpm_actual`, `power_watts` |
| `prvSensorTask` | `xSensorQueue` (SensorMsgObj) | `light_lux`, `accel_*_g`, `temp_c`, `humidity_pct`, `pressure_hpa`, `bme_ok` |

**Publishes out:**
| Object | Trigger | Consumer |
|---|---|---|
| `xCommandQueue` (int32_t RPM) | Slider drag | `prvMotorTask` (desired RPM) |
| `xSystemEvents` bit `EVT_USER_START` | START button | `prvMotorTask` |
| `xSystemEvents` bit `EVT_USER_STOP` | STOP button | `prvMotorTask` |
| `xSystemEvents` bit `EVT_USER_ESTOP_ACK` | ACK button | `prvMotorTask` |
| `xSystemEvents` bit `EVT_USER_THRESHOLD_CHANGED` | +/- on Thresholds tab | `prvSensorTask` (advisory; values are read every cycle regardless) |
| `g_thresh_power_w` / `g_thresh_accel_g` / `g_thresh_distance_mm` | +/- on Thresholds tab | `prvSensorTask` (used in E-Stop comparisons) |

---

### `prvFaultTask` (priority TBD) — `tasks/fault_task.c`

Currently a skeleton — intended to watch `xSystemEvents` for `EVT_ESTOP_*`
bits and journal a fault record. No outputs yet.

---

## Shared RTOS objects (defined once in `main.c`)

```
xMotorQueue       motor    -> gui      (8 × MotorMsgObj)
xSensorQueue      sensor   -> gui      (8 × SensorMsgObj)
xCommandQueue     gui      -> motor    (4 × int32_t  RPM)
xPowerRawQueue    ADC ISR  -> sensor   (64 × PowerSampleRaw_t)
xAccelRawQueue    Timer ISR-> sampler  (32 × AccelSampleRaw_t)
xSystemEvents     all      <-> all     (event group, bits below)
xUARTMutex        all                   (serialises UARTprintf calls)
xI2CMutex         sensor + sampler      (serialises every I2C0 transaction)
```

Event-group bits live in `shared.h`:

```
EVT_ESTOP_POWER / EVT_ESTOP_ACCEL / EVT_ESTOP_DISTANCE
EVT_ESTOP_ANY = (POWER | ACCEL | DISTANCE)
EVT_NIGHT_DETECTED  EVT_SENSOR_FAULT
EVT_USER_START      EVT_USER_STOP        EVT_USER_ESTOP_ACK
EVT_USER_SPEED_CHANGED                   EVT_USER_THRESHOLD_CHANGED
```

---

## ISR -> task data flow (spec 2.2 pipeline)

```
Power:   Timer0A ----trigger----> ADC1 SS0  -> ISR ----> xPowerRawQueue ----> sensor task MAF ----> SensorMsgObj.power_watts + EVT_ESTOP_POWER
Accel:   Timer2A IRQ ----sem----> AccSamp task -> BMI160 I2C read -> xAccelRawQueue ----> sensor task MAF ----> SensorMsgObj.accel_* + EVT_ESTOP_ACCEL
Hall:    GPIO edge ----IRQ-----> g_edges++  -> 100 Hz speed_sensor_tick (in prvSpeedTask) -> exp LPF -> speed_sensor_get_rpm()
Light:   sensor task polls OPT3001 @ 5 Hz -> MAF -> SensorMsgObj.light_lux + EVT_NIGHT_DETECTED
T/H/P:   sensor task polls BME280  @ 1 Hz -> SensorMsgObj.temp_c / humidity_pct / pressure_hpa
```
