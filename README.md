# EGH456 Electric Vehicle Embedded System — Spec Checklist

Live checklist against the assignment brief. Each line is:
- `[x]` done and verified against the code
- `[~]` partially implemented (notes below)
- `[ ]` not yet implemented

Layout: `src/tasks/` (motor / sensor / gui / fault), `src/drivers/` (i2c, opt3001, bmi160, bme280, sht31, power_sensor, speed_sensor, motor_driver, touch, LCD), `src/main.c` (RTOS object creation), `src/shared.h` (inter-task contracts), `lib/motorlib/` (supplied commutation library), `lib/grlib/` (TivaWare graphics).

---

## 2.1 Motor Control

### 2.1.1 Motor State Machine
- [x] Five states implemented: Idle, Starting, Running, E-Stop Braking, Fault Latched (plus a Stopping state for controlled user-Stop ramps). See `MotorState_t` in `shared.h` and `prvMotorTask` in `tasks/motor_task.c`.
- [x] Current state held in `state` local of `prvMotorTask`, published every cycle in `MotorMsgObj.state`.
- [x] GUI displays the live state on the Control tab (state-text canvas + colour-coded status indicator).
- [x] All transitions explicit (start/stop/ack/E-Stop) — no implicit edges.

### 2.1.2 Closed-Loop Speed Control (RPM)
- [x] Hall-based speed feedback via `speed_sensor_get_rpm()` (filtered) and `speed_sensor_get_rpm_raw()`.
- [x] Error term `rpm_reference - rpm_actual` consumed by the motor task; duty adjusted accordingly.
- [x] Open-loop duty in Starting, closed-loop in Running. Steady-state error within spec under nominal load.

### 2.1.3 Acceleration and Deceleration Limiting
- [x] `ACCEL_LIMIT_RPMPS = 500` (shared.h)
- [x] `DECEL_LIMIT_RPMPS = 500`
- [x] `ESTOP_DECEL_LIMIT_RPMPS = 1000`
- [x] Reference RPM ramps independently of desired; actual RPM lags via closed-loop control.

### 2.1.4 Motor Start-Up and Handling
- [x] `motor_driver_start()` calls MotorLib `enableMotor` + initial `updateMotor` kickstart.
- [x] Transition from Starting to Running gated on `rpm_actual > 100` (valid hall feedback).
- [x] MotorLib used for phase commutation; commutation runs inside hall ISRs for lowest latency.

### 2.1.5 Emergency Stop Handling
- [x] Any `EVT_ESTOP_*` bit forces transition to E-Stop Braking.
- [x] User commands ignored while braking (xQueueReset on EVT_USER_STOP).
- [x] 1000 RPM/s deceleration applied via `ESTOP_PER_TICK`.
- [x] On reaching 0 RPM: transition to Fault Latched.
- [x] Acknowledgement required to leave Fault Latched (`EVT_USER_ESTOP_ACK`).

### 2.1.6 RTOS Integration
- [x] Hall ISRs short and non-blocking (`HallPortMIntHandler` / `HallPortHIntHandler` / `HallPortNIntHandler` in `drivers/speed_sensor.c`).
- [x] Motor Control Task at priority `idle+4`, period 10 ms (`LOOP_PERIOD_MS = 10`).
- [x] Speed Task at priority `idle+5`, 100 Hz tick.
- [x] Fault handling: sensor task sets `EVT_ESTOP_*` event bits; motor task waits on the bitmask via `xEventGroupWaitBits`.
- [x] Task-to-task communication: `xCommandQueue`, `xMotorQueue`, `xSensorQueue`.
- [x] Synchronisation: `xCommandMutex`, `xI2CMutex`, `xUARTMutex`, `xSystemEvents`.

### 2.1.7 Motor Control API
- [x] `motor_driver_init`
- [x] `motor_driver_start`
- [x] `motor_driver_stop`
- [x] `motor_driver_set_duty`
- [x] `motor_driver_estop`
- [x] `motor_driver_get_rpm`
- [x] `motor_driver_get_state`
- [x] `motor_driver_hardware_fault_active` (DRV8323 nFAULT)

### 2.1.8 E-Stop Conditions
- [x] Power threshold → `EVT_ESTOP_POWER`.
- [x] Accel threshold → `EVT_ESTOP_ACCEL`.
- [~] Distance threshold → `EVT_ESTOP_DISTANCE` is wired but the VL53L0X was not chosen.
- [x] DRV8323 nFAULT → `EVT_ESTOP_DRIVER` (bonus).

### 2.1.9 Motor Debugging Tool — Serial Plot
- [~] CSV over UART at 115200 baud, 50 Hz. Currently includes `rpm_raw` and `rpm_filt` (actual speed). **Still missing `rpm_desired`, `rpm_reference`, and `pwm_duty`** — values are in `MotorMsgObj` but not yet in the CSV stream.

---

## 2.2 Sensing

### 2.2.1 Core Sensors

**1. Motor Power**
- [x] `power_sensor.c`: Timer0A triggers ADC1 SS0 at 1 kHz; ISR pushes raw `(ia, ib)` into `xPowerRawQueue`.
- [x] Sensor task drains the queue, runs a 32-tap MAF on `|Ia| + |Ib| + |Ic|`, computes `P = 24 V × I_total`.
- [~] `power_sensor_init()` currently commented out pending real motor calibration. Full pipeline coded and verified.

**2. Motor Speed**
- [x] Hall GPIO interrupts on PM3, PH2, PN2.
- [x] Sector-valid edge counter (rejects illegal codes 0 and 7).
- [x] 100 Hz tick converts accumulated edges to RPM, exponential LPF (alpha = 1/4).
- [x] `SPEED_EDGES_PER_REV = 24` (6 × 4 pole pairs).

**3. Light (OPT3001)**
- [x] `drivers/opt3001.c` over I2C0; 5 Hz polled; 8-sample MAF.

### 2.2.2 Optional Sensors (chosen: Acceleration + Temperature/Humidity)

**Acceleration (BMI160)**
- [x] `drivers/bmi160.c` over I2C0 (auto-probes 0x68 / 0x69).
- [x] 200 Hz Timer2A-driven ISR pipeline → AccSamp task → `xAccelRawQueue` → sensor task drain.
- [x] **No software filter on the magnitude.** Impacts are single-sample transients; a moving average smears them out and lets a real crash slip below the threshold. The raw `|ax|+|ay|+|az|` is checked directly each cycle.
- [x] BMI160's on-chip 100 Hz ODR with normal-averaging mode (`ACC_CONF=0x28`) already smooths per-axis noise before the values leave the sensor.
- [x] Raw total magnitude triggers `EVT_ESTOP_ACCEL`.
- [x] Bit-set is gated by `g_motor_estop_armed` (true only in Starting / Running / Stopping) so bench handling can't latch a fault while idle.

**Temperature and Humidity (SHT31)**
- [x] `drivers/sht31.c` over I2C0 at 0x44; proper probe via soft-reset write.
- [x] 1 Hz polled; °C and %RH.
- [x] BME280 retained on the same bus for pressure only.

### 2.2.3 Sensor Filtering
- [x] All filtering runs inside the sensor task (priority `idle+3`), never in ISRs.
- [x] ISRs capture raw values into queues; tasks drain and filter.
- [x] **8-sample moving average** on light (smooth, slowly-varying signal).
- [x] **32-sample moving average** on power (rejects PWM switching noise).
- [x] **Exponential low-pass** (alpha = 1/4) on RPM (~40 ms time constant; fast follow with quantisation rejection).
- [x] **No software filter** on the acceleration magnitude — see the Acceleration section above for the justification.

Sampling rates (spec minimum in brackets):
- [x] Power: **1000 Hz** (≥ 150)
- [x] Speed: **100 Hz** (≥ 100)
- [x] Light: **5 Hz** (≥ 2)
- [x] Acceleration: **200 Hz** (≥ 100)
- [x] Temperature/Humidity: **1 Hz** (≥ 1)
- [n/a] Distance: VL53L0X not chosen

### 2.2.4 Integration with Motor Control
- [x] `EVT_ESTOP_POWER`, `EVT_ESTOP_ACCEL` set by sensor task; motor task transitions to E-Stop Braking on the `EVT_ESTOP_ANY` mask.

### 2.2.5 Sensor Debugging Tool — Serial Plot
- [x] CSV over UART at 115200 baud, 50 Hz.
- [x] Header: `t,p_raw_mw,p_filt_mw,lux_raw,lux_filt,ax_mg,ay_mg,az_mg,acc_raw_mg,acc_filt_mg,t_cc,h_cp,p_dhpa,rpm_raw,rpm_filt`.
- [x] Raw + filtered for power, light, acceleration magnitude.
- [x] Parseable by Tera Term Plotter, Arduino Serial Plotter, `pandas.read_csv`.

---

## 2.3 User Interface

### 2.3.1 Core GUI Requirements

**1. Motor Control Interface**
- [x] START button → `EVT_USER_START` → Idle → Starting.
- [x] STOP button → `EVT_USER_STOP` → goes through Stopping with ramped deceleration. Does not bypass safety logic.

**2. System Status Panel**
- [x] State text canvas shows live motor state.
- [x] Status line under thresholds shows fault reasons ("E-Stop braking - press ACK", "Fault: Power+Accel", "Drv fault - power-cycle motor board").
- [x] Updates in real time.

**3. Motor Status Indicator (Green/Orange/Red)**
- [x] Running → `ClrLimeGreen`
- [x] Idle, Starting, Stopping → `ClrOrange`
- [x] E-Stop Braking, Fault Latched → `ClrRed`

**4. Speed Control Input**
- [x] Slider 0..`MAX_MOTOR_RPM` (10000). Posts to `xCommandQueue`. Motor task is sole arbiter of PWM.

**5. System Clock**
- [x] **HH:MM:SS time** on the Control tab.
- [x] **YYYY-MM-DD date** below the time, with leap-year-aware day rollover from a hardcoded demo epoch (`BASE_YEAR/MONTH/DAY` in `gui_task.c`, currently 2026-05-26).

**6. Threshold Configuration**
- [x] Thresholds tab with +/- editors for **four** runtime values:
  - Power (W) — step 10, range 50..300, default 150
  - Accel (g) — step 0.1, range 0.5..4.0, default 2.0
  - Night (lux) — step 1, range 1..50, default 5 (replaces distance since VL53L0X not used)
  - Cool (°C) — step 1, range 10..40, default 25
- [x] Values flow into `g_thresh_power_w`, `g_thresh_accel_g`, `g_thresh_night_lux`, `g_thresh_cool_c` volatile globals.
- [x] Sensor task reads each cycle; changes take effect within 20 ms.
- [x] Control-tab summary line (`Pwr 150W  Acc 2.0g  Nt 5lx  Cool 25C`) refreshes on every +/- press.

**7. E-Stop Acknowledgement**
- [x] Dedicated ACK button → `EVT_USER_ESTOP_ACK`. Only this bit can leave Fault Latched.
- [x] Restart blocked until acknowledged.

**8. Day/Night Detection**
- [x] Simple threshold against `g_thresh_night_lux` (no hysteresis): below → Night, at/above → Day.
- [x] Day/night text + LED bulb image (`g_pui8LightOn` / `g_pui8LightOff`) toggle on transition.

### 2.3.2 Optional Sensor Integration

**Temperature and Humidity (SHT31)**
- [x] Live T (°C) and %RH on the Sensors tab.
- [x] **Editable cool threshold** on the Thresholds tab (`Cool` row).
- [x] **Cooling indicator LED** on the Control tab using the same 20×20 bulb bitmap as the Day/Night LED. Lit when SHT31 temp > `g_thresh_cool_c`, dark otherwise.

**Vehicle Body Acceleration (BMI160)**
- [x] Filtered total magnitude shown on Sensors tab; X, Y, Z individual readouts also visible.
- [x] Crash threshold editable on Thresholds tab.
- [x] E-Stop triggered via sensor task event group.

### 2.3.3 GUI Sensor Plots
- [x] Second page (Plots tab) with a single canvas.
- [x] 5-second rolling time window (25 samples × 200 ms).
- [x] Rolling X-axis time labels: `now-5s` at the origin, `now` at the right edge (HH:MM:SS).
- [x] Per-trace visibility toggles on the Sensors tab (7 toggle buttons next to each readout, colour-matched).
- [x] Colour-matched top-row max and bottom-row min labels per trace.
- [x] Required signals plotted: RPM (goldenrod 0..10000), Power (cyan 0..200 W), Light (white 0..500 lx).
- [x] Optional sensors plotted: Acceleration-Y centred ±2 g (magenta), Temperature 10..30 °C (orange), Humidity 0..100 %RH (turquoise).
- [x] Pressure (lime green, 1000..1025 hPa) as a bonus channel.

---

## 2.4 Advanced Features Implemented

- [x] **BME280 environmental sensor** alongside SHT31 (bonus pressure channel; both gated independently on the same I2C bus).
- [x] **Per-trace plot toggles** with live colour-changing buttons on the Sensors tab.
- [x] **Rolling clock-time X-axis labels** on the Plot canvas.
- [x] **DRV8323 nFAULT detection** on PL0 with its own E-Stop event bit (`EVT_ESTOP_DRIVER`).
- [x] **Cooling indicator LED** on the Control tab (matches Day/Night LED style).
- [x] **Date readout** alongside the system clock (leap-year-aware rollover from a configurable demo epoch).
- [x] **I2C bus recovery** (`BURST_SEND_ERROR_STOP`) so address NACKs during sensor probing don't wedge the bus.
- [x] **`writeI2C1`** single-byte data write helper for 8-bit-register sensors.
- [x] **40 KB FreeRTOS heap** with all task allocations comfortably below.
- [x] **`.ARM.exidx` linker section** for libgcc 64-bit math (BME280 compensation).
- [x] **ISR-priority alignment** with `configMAX_SYSCALL_INTERRUPT_PRIORITY` so `FromISR` APIs work safely.

---

## RTOS task summary

| Task | Priority | Rate | Role |
|------|----------|------|------|
| AccSamp | idle+5 | 200 Hz | Timer2A semaphore wakes, reads BMI160 over I2C, queues raw counts |
| Speed | idle+5 | 100 Hz | Snapshots hall edge counter, computes filtered RPM |
| Motor | idle+4 | 100 Hz | State machine, ramp, closed-loop duty, nFAULT poll, motor publish |
| Sensor | idle+3 | 50 Hz | Drains raw queues, filters, gates I2C polls (lux 5 Hz, T/H/P 1 Hz), E-Stop checks, UART CSV |
| GUI | idle+2 | ~50 Hz | Touchscreen, grlib widgets, plot repaint, threshold editor |
| Fault | idle+1 | event | Blocks on `EVT_ESTOP_*` (logging stub) |

RTOS objects (defined in `main.c`): `xMotorQueue`, `xSensorQueue`, `xCommandQueue`, `xPowerRawQueue`, `xAccelRawQueue`, `xSystemEvents`, `xUARTMutex`, `xI2CMutex`, `xCommandMutex`.

---

## What's still to do

1. **Motor serial-plot stream (§2.1.9):** add `rpm_desired`, `rpm_reference`, and `pwm_duty` to the CSV. The values exist in `MotorMsgObj`; the sensor task's UART line just doesn't pull them yet.
2. **Uncomment `power_sensor_init()`** in `tasks/sensor_task.c` once the DRV8323 ADC path is calibrated. Point GUI power readout at `sensor_msg.power_watts`.
3. **Fault task logging:** `fault_task.c` is currently a skeleton that waits on the fault bits but doesn't log.

---

## Build / Flash / Monitor

```
pio run                 # build
pio run -t upload       # flash via ICDI
pio device monitor      # opens at 115200 (pinned in platformio.ini)
```

Serial monitor at 115200 baud. CSV stream from the sensor task is suitable for Tera Term Plotter, Arduino Serial Plotter, or `pandas.read_csv()`.

---

## Reference documents

- Texas Instruments EK-TM4C1294XL LaunchPad User's Guide (SPMU365)
- Tiva TM4C1294NCPDT Microcontroller Data Sheet (SPMS433)
- BOOSTXL-DRV8323RH Motor Control BoosterPack User's Guide (SLVUAA2)
- BOOSTXL-SENSORS BoosterPack User's Guide (SLAU666)
- TivaWare Peripheral Driver Library (SW-TM4C-DRL-UG, v2.2.0.295)
- TivaWare Graphics Library (SW-TM4C-GRL-UG, v2.2.0.295)
- FreeRTOS Reference Manual (v10.x)
- Bosch BMI160 Datasheet (BST-BMI160-DS000-07)
- Bosch BME280 Datasheet (BST-BME280-DS002-15)
- Sensirion SHT3x-DIS Datasheet (v6)
- Texas Instruments OPT3001 Datasheet (SBOS681B)
- Texas Instruments DRV8323 Datasheet (SLVSDJ3D)
- Solomon Systech SSD2119 (Kentec K350QVG-V2-F display)
- MotorLib supplied by EGH456 teaching team
