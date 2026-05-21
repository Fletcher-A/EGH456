# EGH456 — Electric Vehicle Embedded System

Group assignment build. Task files in `src/tasks/`, sensor + I/O drivers in
`src/drivers/`, RTOS objects defined in `src/main.c`, inter-task contracts
in `src/shared.h`.

---

## Status

| Subsystem | Status |
|---|---|
| GUI (spec 2.3) | **Complete** — 4 tabs, threshold editor, day/night LED, plots, DRV fault display |
| Sensing — Light (OPT3001) | **Done** — 5 Hz polled, 8-tap MAF |
| Sensing — Accel (BMI160) | **Done** — 200 Hz ISR pipeline, 8-tap MAF, E-Stop wired |
| Sensing — T/RH/P (BME280) | **Done** — 1 Hz polled (substituted for spec-listed SHT31) |
| Sensing — Power (DRV8323 ADC) | **Code complete** — `power_sensor_init()` still commented out in `sensor_task.c` |
| Sensing — Speed (hall) | **Done** — `prvSpeedTask` @ 100 Hz, sector-valid edge counting, exp LPF |
| Motor control | **Done** — MotorLib commutation, open-loop duty from slider, state machine, nFAULT (PL0) |
| RTOS / concurrency | **Documented + hardened** — see [RTOS and concurrency](#rtos-and-concurrency) |
| Fault task | **Skeleton** — waits on `EVT_ESTOP_*`, UART logging TODO |

---

## What's still to do

1. **Uncomment `power_sensor_init()`** in `src/tasks/sensor_task.c` when the motor
   BoosterPack ADC path is wired; point GUI power readout at `sensor_msg.power_watts`
   if that becomes the canonical source.
2. **Tighten thresholds** for the final demo. Day/night already at `<5 lux`
   per spec when `DAYNIGHT_DESK_TEST` is undefined.
3. **Pinout verify on hardware** — `SOA/SOB` ADC channels, hall ports (PM3/PH2/PN2),
   `SPEED_EDGES_PER_REV` (pole pairs), nFAULT on PL0 (red LED on motor board).
4. **Report**: justify BME280-instead-of-SHT31 substitution and filter
   choices (MAF for accel/light/power, exponential LPF for RPM).
5. **RTOS polish (optional)** — migrate remaining `UARTprintf` in BMI160/BME280/OPT3001
   init and add logging in `fault_task.c`; see concurrency notes below.

---

## Build / Flash / Monitor

```
pio run                # build
pio run -t upload      # flash via ICDI
pio device monitor     # opens at 115200 (pinned in platformio.ini)
```

Serial output is a CSV plot stream — pipe into Tera Term Plotter, Serial
Plotter, or `pandas.read_csv()` for the report figures.

Tasks that log after the scheduler starts should use `uart_log_printf()`
(`src/utils/uart_log.c`) so lines are not interleaved on the shared UART.

---

## RTOS and concurrency

FreeRTOS runs **five preemptive tasks** (no coroutines). Priorities are
documented in `include/shared.h` / `src/shared.h` (higher number = higher
priority on this port):

| Task | Priority | Rate / role |
|------|----------|-------------|
| Speed | `idle+5` | 100 Hz hall RPM (`speed_sensor_tick`) |
| AccSamp | `idle+5` | 200 Hz BMI160 I2C (Timer2A → semaphore) |
| Motor | `idle+4` | 100 Hz state machine, PWM, nFAULT poll |
| Sensor | `idle+3` | 50 Hz fusion, E-stop checks, UART CSV |
| GUI | `idle+2` | Touch + grlib display (~50 Hz effective) |
| Fault | `idle+1` | Blocks on fault bits (logging stub) |

Speed and AccSamp share priority 5 and **time-slice** when both are ready.

### Inter-thread communication

```
GUI  --xCommandQueue + xCommandMutex--> Motor
GUI  --xSystemEvents (START/STOP/ACK)--> Motor, Sensor
Motor--xMotorQueue--------------------> GUI
Sensor-xSensorQueue-------------------> GUI
ADC ISR --xPowerRawQueue-------------> Sensor
Timer2A --xAccelRawQueue + sem-------> AccSamp
Sensor --xSystemEvents (E-stop bits)--> Motor
```

### Synchronisation primitives (`main.c`)

| Object | Purpose |
|--------|---------|
| `xUARTMutex` | Serialises UART output via `uart_log_printf()` |
| `xI2CMutex` | One transaction at a time on shared I2C0 |
| `xCommandMutex` | GUI `xQueueReset`/`xQueueSend` vs motor `xQueueReceive` |
| `xSystemEvents` | START/STOP/ACK, E-stops, night mode, threshold advisory bit |

### Concurrent access (design choices)

| Resource | Protection |
|----------|------------|
| **I2C bus** | `xI2CMutex` in sensor task and AccSamp task |
| **UART** | `xUARTMutex` in `uart_log_printf()` (motor + sensor tasks migrated) |
| **Command queue** | `xCommandMutex` around send/receive/drain on both GUI and motor |
| **MotorLib** | Not re-entrant: task calls use `IntMasterDisable()` in `motor_driver.c`; hall ISRs call `updateMotor()` at NVIC priority `configMAX_SYSCALL_INTERRUPT_PRIORITY + 1` |
| **Hall edge counter `g_edges`** | ISR writers; `speed_sensor_tick()` snapshots with `IntMasterDisable()` |
| **Thresholds `g_thresh_*`** | `volatile`; single-word writes on Cortex-M4 — no mutex |
| **`g_motor_estop_armed`** | `volatile bool`; sensor only asserts accel/power E-stop while motor may run |
| **grlib / LCD** | Only touched from GUI task — no display mutex |

### ISR ↔ FreeRTOS rules

- **ADC (power):** `xQueueSendFromISR` on `xPowerRawQueue` (drops sample if full).
- **Timer2A (accel):** priority set to `configMAX_SYSCALL_INTERRUPT_PRIORITY`;
  `xSemaphoreGiveFromISR` on `s_xAccelTickSem`.
- **Hall GPIO:** no FromISR APIs; commutation only. Priority above syscall threshold
  so hall ISRs do not call FreeRTOS FromISR helpers incorrectly.

### Scheduling and contention notes

- Motor → GUI: `xMotorQueue` depth **16**; `xQueueSend` uses a **1 ms** timeout so a
  slow GUI cannot block the motor loop indefinitely.
- Motor task clears `EVT_ESTOP_*` / user bits on read (`pdTRUE` auto-clear); sensor
  task sets/clears power/accel E-stop bits; fault task waits **without** clearing.
- **Residual risks:** BMI160/BME280/OPT3001 init still use raw `UARTprintf`;
  `fault_task` does not log yet; ISR-side `updateMotor()` is not masked when the
  motor task has interrupts enabled between MotorLib calls (task side masks globally).

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
| DRV8323 nFAULT (PL0, polled) | `motor_driver_hardware_fault_active()` | any → E-Stop Braking / Fault Latched (`EVT_ESTOP_DRIVER`) |

**Publishes out:**
| Object | Field | Consumer |
|---|---|---|
| `xMotorQueue` (MotorMsgObj) | `state` (MotorState_t) | `prvGuiTask` — state-text canvas + status indicator colour |
| `xMotorQueue` | `rpm_actual`, `rpm_desired`, `rpm_reference` | `prvGuiTask` — RPM readout + plot |
| `xMotorQueue` | `pwm_duty`, `hall_state`, `fault_bits`, `motor_ready` | `prvGuiTask` — duty %, status line, fault text |
| `xMotorQueue` | `power_watts` | `prvGuiTask` — power readout (until sensor ADC path is primary) |

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

### `prvSpeedTask` (priority 5) — `tasks/motor_task.c`

**Reads in:** hall edge counter `g_edges` (valid sector changes only; ISRs in
`drivers/speed_sensor.c` on PM3/PH2/PN2).

**Publishes out:** filtered RPM via `speed_sensor_get_rpm()` — display and CSV;
motor control uses open-loop duty from slider (`motor_driver_set_speed_rpm`).

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
| `xCommandQueue` (int32_t RPM) | Slider drag / START min RPM | `prvMotorTask` (under `xCommandMutex`) |
| `xSystemEvents` bit `EVT_USER_START` | START button | `prvMotorTask` |
| `xSystemEvents` bit `EVT_USER_STOP` | STOP button | `prvMotorTask` |
| `xSystemEvents` bit `EVT_USER_ESTOP_ACK` | ACK button | `prvMotorTask` |
| `xSystemEvents` bit `EVT_USER_THRESHOLD_CHANGED` | +/- on Thresholds tab | `prvSensorTask` (advisory; values are read every cycle regardless) |
| `g_thresh_power_w` / `g_thresh_accel_g` / `g_thresh_distance_mm` | +/- on Thresholds tab | `prvSensorTask` (used in E-Stop comparisons) |

---

### `prvFaultTask` (priority 1) — `tasks/fault_task.c`

Waits on `EVT_ESTOP_ANY | EVT_SENSOR_FAULT` without clearing (motor task
consumes E-stop bits in its state machine). UART fault journal not implemented yet.

---

## Shared RTOS objects (defined once in `main.c`)

```
xMotorQueue       motor    -> gui      (16 × MotorMsgObj)
xSensorQueue      sensor   -> gui      (8 × SensorMsgObj)
xCommandQueue     gui      -> motor    (4 × int32_t RPM)
xPowerRawQueue    ADC ISR  -> sensor   (64 × PowerSampleRaw_t)
xAccelRawQueue    Timer ISR-> sampler  (32 × AccelSampleRaw_t)
xSystemEvents     all      <-> all     (event group, bits below)
xUARTMutex        uart_log              (thread-safe logging after scheduler)
xI2CMutex         sensor + AccSamp      (every I2C0 transaction)
xCommandMutex     gui + motor           (xCommandQueue send/receive/reset)
```

Event-group bits live in `shared.h`:

```
EVT_ESTOP_POWER / EVT_ESTOP_ACCEL / EVT_ESTOP_DISTANCE / EVT_ESTOP_DRIVER
EVT_ESTOP_ANY = (POWER | ACCEL | DISTANCE | DRIVER)
EVT_NIGHT_DETECTED  EVT_SENSOR_FAULT
EVT_USER_START      EVT_USER_STOP        EVT_USER_ESTOP_ACK
EVT_USER_SPEED_CHANGED                   EVT_USER_THRESHOLD_CHANGED
```

Runtime flags in `main.c` (no mutex — single-word / bool writes):

```
g_thresh_power_w   g_thresh_accel_g   g_thresh_distance_mm   (GUI writes, sensor reads)
g_motor_estop_armed                         (motor sets; sensor gates E-stop asserts)
```

---

## ISR -> task data flow (spec 2.2 pipeline)

```
Power:   Timer0A ----trigger----> ADC1 SS0  -> ISR ----> xPowerRawQueue ----> sensor task MAF ----> SensorMsgObj.power_watts + EVT_ESTOP_POWER
Accel:   Timer2A IRQ ----sem----> AccSamp task -> BMI160 I2C read -> xAccelRawQueue ----> sensor task MAF ----> SensorMsgObj.accel_* + EVT_ESTOP_ACCEL
Hall:    GPIO edge ----IRQ-----> valid sector -> g_edges++ -> 100 Hz prvSpeedTask tick -> exp LPF -> speed_sensor_get_rpm() + MotorLib commutation in ISR
Light:   sensor task polls OPT3001 @ 5 Hz -> MAF -> SensorMsgObj.light_lux + EVT_NIGHT_DETECTED
T/H/P:   sensor task polls BME280  @ 1 Hz -> SensorMsgObj.temp_c / humidity_pct / pressure_hpa
```
