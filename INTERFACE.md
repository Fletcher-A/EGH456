# Assignment Data Interface

This document is the **contract** between the three subsystems. Each
variable below is consumed by the GUI. Motor and Sensors teams: please
make sure your tasks publish these values via the existing queues and
event group in `src/shared.h`.

All transfer happens through:
- `xMotorQueue`   — Motor task posts `MotorMsgObj` (see shared.h)
- `xSensorQueue`  — Sensor task posts `SensorMsgObj`
- `xCommandQueue` — GUI posts desired RPM as `int32_t`
- `xSystemEvents` — bit flags for events (start, stop, e-stop, faults)

---

## 1. Motor → GUI  (published in `MotorMsgObj`)

The motor team must populate every field of `MotorMsgObj` every control
tick (recommended ~50 Hz to the GUI, even if the control loop runs
faster) and send it to `xMotorQueue`.

| Variable | Type | Units | What it means | When updated |
|---|---|---|---|---|
| `state` | `MotorState_t` enum | — | Current state machine state: `MOTOR_STATE_IDLE`, `_STARTING`, `_RUNNING`, `_ESTOP_BRAKING`, `_FAULT_LATCHED` | Every transition |
| `rpm_actual` | `int32_t` | RPM | Measured motor speed from hall sensors (filtered) | Every control tick |
| `rpm_reference` | `int32_t` | RPM | Ramped reference the PI controller is tracking | Every control tick |
| `rpm_desired` | `int32_t` | RPM | The latest desired RPM (echo of what GUI sent — useful for plotting) | When user changes slider |
| `pwm_duty` | `uint16_t` | 0 .. PWM_PERIOD | Current PWM duty cycle applied to motor | Every control tick |
| `seq` | `uint32_t` | — | Increments on every send (debug / dropped-message detection) | Every send |
| `tick` | `TickType_t` | ticks | `xTaskGetTickCount()` at moment of send | Every send |

---

## 2. Sensors → GUI  (published in `SensorMsgObj`)

The sensor team must populate this struct and send to `xSensorQueue` at
~20 Hz (faster than the GUI needs to repaint).

| Variable | Type | Units | What it means | Required rate |
|---|---|---|---|---|
| `power_watts` | `float` | W | Motor electrical power (V × I_total, filtered) | ≥150 Hz internally, ~20 Hz to queue |
| `light_lux` | `float` | lux | Filtered ambient light from OPT3001 | ≥2 Hz |
| `optional_a` | `float` | depends | First chosen optional sensor (see §4) | per sensor spec |
| `optional_b` | `float` | depends | Second chosen optional sensor (see §4) | per sensor spec |
| `seq` | `uint32_t` | — | Increments on every send | every send |
| `tick` | `TickType_t` | ticks | Send timestamp | every send |

Speed RPM is NOT in `SensorMsgObj` because the motor task already
publishes it in `MotorMsgObj`. The GUI plots `motor.rpm_actual` for
the speed channel.

---

## 3. Sensors / Motor → GUI  (event-group flags, `xSystemEvents`)

Set by the sensor or motor task. The GUI polls these every loop
iteration with `xEventGroupWaitBits(..., timeout=0)` and displays
warning text / changes the status colour.

| Bit | Meaning | Set by | Cleared by |
|---|---|---|---|
| `EVT_ESTOP_POWER` | Motor power exceeded user threshold | Sensor task | Motor task on ACK |
| `EVT_ESTOP_ACCEL` | Body acceleration exceeded threshold | Sensor task | Motor task on ACK |
| `EVT_ESTOP_DISTANCE` | Distance under safe minimum | Sensor task | Motor task on ACK |
| `EVT_NIGHT_DETECTED` | `light_lux < 5` | Sensor task | Sensor task when daylight returns |
| `EVT_SENSOR_FAULT` | A sensor read failed / not communicating | Sensor task | Sensor task when recovered |

---

## 4. Optional sensor selection (pick TWO of three)

Whatever the sensor team picks goes into `optional_a` / `optional_b` in
`SensorMsgObj`. Document which is which in the code AND tell the GUI
team. Suggested:

| Sensor | Field name suggestion | Units | What GUI shows |
|---|---|---|---|
| BMI160 (accel) | `accel_total_g` | g | "Body accel: X.XX g" + crash threshold |
| SHT31 (temp + RH) | needs TWO fields | °C / %RH | "Temp: XX.X °C  Hum: XX %RH" + cooling on/off |
| VL53L0X (ToF) | `distance_mm` | mm | "Distance: XXXX mm" + safe minimum |

If SHT31 is chosen, the team needs to add a second float to
`SensorMsgObj` (`temp_c` AND `humidity_rh`) since one field isn't enough.

---

## 5. GUI → Motor  (commands and signals)

| What | How | Payload | When |
|---|---|---|---|
| Set desired RPM | `xQueueSend(xCommandQueue, &rpm, 0)` | `int32_t` RPM (0–max) | When slider moves |
| Start motor | `xEventGroupSetBits(xSystemEvents, EVT_USER_START)` | none | START button tap |
| Stop motor | `xEventGroupSetBits(xSystemEvents, EVT_USER_STOP)` | none | STOP button tap |
| Acknowledge fault | `xEventGroupSetBits(xSystemEvents, EVT_USER_ESTOP_ACK)` | none | E-Stop ACK button tap |
| Speed changed (optional ping) | `xEventGroupSetBits(xSystemEvents, EVT_USER_SPEED_CHANGED)` | none | After slider drag ends |

---

## 6. GUI → Sensors  (threshold configuration)

The GUI lets the user edit thresholds at runtime. There are two valid
implementations — pick ONE and tell everyone:

**Option A — Shared globals + mutex** (simplest):

```c
extern volatile float    g_threshold_power_w;
extern volatile float    g_threshold_accel_g;
extern volatile uint32_t g_threshold_distance_mm;
```
GUI writes; sensor task reads. Wrap each access in `xUARTMutex` (or
make a dedicated mutex) only if you see torn writes — single floats
are usually atomic on Cortex-M4.

**Option B — Threshold mailbox queue**:

```c
typedef struct {
    enum { THRESH_POWER, THRESH_ACCEL, THRESH_DIST } which;
    float value;
} ThresholdMsg;
QueueHandle_t xThresholdQueue;
```
GUI sends a `ThresholdMsg`; sensor task drains on every loop and updates
its internal copy.

After updating any threshold the GUI sets
`EVT_USER_THRESHOLD_CHANGED` so the sensor task can log the change.

---

## Minimum publishing rates

| Producer | Required Hz | To |
|---|---|---|
| Motor → `xMotorQueue` | ≥20 Hz | GUI |
| Sensor → `xSensorQueue` | ≥20 Hz | GUI |
| Hall ISR (commutation) | edge-triggered | Motor task |
| ADC current sampling | ≥150 Hz | Sensor task internal |
| Light sample | ≥2 Hz | Sensor task internal |
| Speed sample | ≥100 Hz | Motor task internal |

---

## Summary "to fill in" for the team

**Motor team — please publish into `MotorMsgObj`:**
- `state`, `rpm_actual`, `rpm_reference`, `rpm_desired`, `pwm_duty`

**Sensor team — please publish into `SensorMsgObj`:**
- `power_watts`, `light_lux`, `optional_a`, `optional_b`
- Set `EVT_ESTOP_POWER`/`_ACCEL`/`_DISTANCE` on threshold breach
- Set `EVT_NIGHT_DETECTED` when `light_lux < 5`

**GUI team (me) — will consume those and send back:**
- `int32_t` desired RPM to `xCommandQueue`
- `EVT_USER_START` / `_STOP` / `_ESTOP_ACK` event bits
- Threshold updates (via shared globals OR threshold queue, pick one)
