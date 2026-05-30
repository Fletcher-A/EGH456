# RTOS Integration — slide content & speaker notes (2.1.6)

Use with slides added to `EGH456_Motor_3min_Presentation.pptx` (slides 3–4).

---

## Slide 3: RTOS Integration (2.1.6)

### On-slide bullets

**ISRs — hall speed + sensor sampling**
- HallPortM / H / N: GPIO IRQ → `g_edges` + commutation (no blocking RTOS calls)
- ADC1Seq0 → `xPowerRawQueue` (FromISR) · Timer2A → accel semaphore (FromISR)
- RPM calculated in **Speed** task (100 Hz), not inside hall ISR

**Motor task — 10 ms, priority idle+4**
- State machine · Ramp (500 / 1000 RPM/s) · Duty from reference

**Fault / E-stop — event group `xSystemEvents`**
- Sensor: `EVT_ESTOP_POWER`, `EVT_ESTOP_ACCEL` → Motor: `ESTOP_BRAKING` → `FAULT`
- GUI: START/STOP/ACK bits + `xCommandQueue` (mutex)
- Fault task (idle+1): UART logging only

**Priorities:** Speed/GUI/Plot (+5) > Motor/AccSamp (+4) > Sensor (+3)

### Speaker notes (~45 s)

> “Section 2.1.6 requires FreeRTOS with short ISRs and a periodic motor task. We use three hall GPIO interrupts on ports M, H, and N. Each ISR only clears the interrupt, counts a valid hall sector into `g_edges`, and updates commutation — it does not call queues or delays, so it stays non-blocking for the RTOS.
>
> RPM is computed in a separate Speed task at 100 Hz. The Motor task runs every 10 milliseconds at priority idle plus four. Each cycle it updates the state machine, ramps the reference at 500 RPM per second or 1000 on E-stop, and sets PWM duty from that reference.
>
> For faults, the sensor task sets bits in a shared event group when power or acceleration limits are exceeded. The motor task reads those bits and transitions to E-stop braking, then fault latched. The GUI uses the same event group for start and stop, and a command queue protected by a mutex for speed setpoints. A low-priority fault task only logs to UART so it never blocks control.”

---

## Slide 4: RTOS Communication

### On-slide diagram

```
Hall ISR  →  Speed task  →  Motor task (control loop)
ADC ISR   →  queue       →  Sensor task  →  event bits  →  Motor
Timer ISR →  semaphore   →  AccSamp task →  Sensor
GUI       →  queue + events  →  Motor  →  queue  →  GUI
```

### Speaker notes (~30 s)

> “This diagram summarises how data moves. Hall interrupts feed the speed task, which feeds the motor control loop. The ADC interrupt sends raw current samples through a queue to the sensor task, which can raise an E-stop event to the motor task. The accelerometer uses a timer interrupt and a binary semaphore into a dedicated sampler task. The GUI sends RPM commands on a queue and reads motor status back on another queue. That shows both ISR-to-task and task-to-task communication using queues, semaphores, mutexes, and event groups.”

---

## Code references (if asked)

| Topic | File |
|-------|------|
| Hall ISRs | `src/drivers/speed_sensor.c` |
| ISR vector table | `src/startup_gcc.c` |
| Motor task 10 ms | `src/tasks/motor_task.c` |
| E-stop events | `include/shared.h`, `sensor_task.c`, `motor_task.c` |
| RTOS objects created | `src/main.c` |
