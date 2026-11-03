# Motion Control (Subsystem 6) – Robo Pico

Owner: Buddy 4 (Lim Zi Ying) · FR-37 to FR-42

This drives the 4 mecanum wheels (2 × L298N), the arm and the claw. It runs a
fixed 20 ms update on the Robo Pico that handles ramping, start boost, heading
hold (IMU), active braking and the safety stops.

---

## 1. Autonomous moves (for mapping / DFS)

Publish on the MQTT topic **`robot/command`**. One command starts the move and
the rover **keeps doing it on its own**, so there is no need to resend it.

| Command | What it does |
|---|---|
| `MOVE_FORWARD:30` | Drive forward at 30 % until stopped, holding a straight heading. Number optional (default 30). |
| `MOVE_BACKWARD:30` | Same, backward |
| `TURN_BY:90` | Rotate on the spot by 90° (**+ = right, − = left**), then stop by itself (±3°) |
| `TURN_TO:180` | Rotate to heading 180°, then stop by itself (±3°). 0° = the way the rover faced at power-on, clockwise positive. |
| `MOVE_STOP` | Normal stop (smooth ramp down) |
| `STOP` | Operator stop: instant, PWM 0 + brake |

### Replies on `robot/status`

| Reply | Meaning |
|---|---|
| `AUTO_MOVE: <command>` | Move started |
| `AUTO_REFUSED: <command> (reason)` | Not started: no IMU heading, safety lock / SAFE STOP / busy, or a bad value |
| `AUTO_RESULT: HEADING_REACHED` | Turn finished |
| `AUTO_RESULT: DONE` | Stopped after `MOVE_STOP` |
| `AUTO_RESULT: STOPPED` | Ended by something else (see below) |

Typical step: `TURN_BY:90` → wait for `HEADING_REACHED` → `MOVE_FORWARD:30` →
`MOVE_STOP` when the next cell / marker is reached → wait for `DONE`.

### A move also ends by itself (`STOPPED`) when

- the operator presses **STOP** on the dashboard
- Collision Detection sends a stop (obstacle in front)
- the laptop link drops (no dashboard heartbeat for 1.5 s, FR-08)
- the IMU heading stops arriving (older than 0.6 s)

After a safety stop the rover is in **SAFE STOP** and refuses every move until
the operator presses **Reset Safety** on the dashboard. The rover publishes
`SAFE STOP: <reason> - press Reset Safety` once when this happens.

### Requirements

- The **Sensor Pico must send IMU heading** (vision branch firmware). Without
  it every auto move is refused, so the rover never drives blind.
- **Keep the dashboard connected**: its heartbeat keeps the link alive.
- **Dashboard movement keys are ignored while an auto move runs** (FR-40).
  Send `MOVE_STOP` or press STOP first.

### Calling from C (code on the Robo Pico)

```c
#include "motor_driver.h"

motor_auto_rotate_by(90);     // turn right 90°, stops by itself
motor_auto_drive(30);         // forward 30 % (negative = backward), keeps going
motor_auto_stop();            // normal stop
motor_auto_rotate_to(180);    // turn to an absolute heading

motor_auto_status_t s = motor_auto_status();
// MOTOR_AUTO_RUNNING, _HEADING_REACHED, _DONE, _STOPPED, _REJECTED
```

Return values: `MOTOR_OK`, `MOTOR_ERR_NO_HEADING` (no fresh IMU heading),
`MOTOR_ERR_SAFETY` (safety lock / SAFE STOP / busy), `MOTOR_ERR_PIN` (bad value).

---

## 2. Dashboard keys (manual driving)

| Key | Move | Speed (`motor_config.h`) |
|---|---|---|
| W / S | Forward / backward | `MC_DASH_STRAIGHT_PCT` = 30 |
| A / D | Turn left / right on the spot | `MC_DASH_ROTATE_PCT` = 50 |
| Q / E | Strafe left / right (zigzag) | `MC_DASH_STRAFE_PCT` = 100 |
| R / T / F / G | Diagonals | `MC_DASH_DIAGONAL_PCT` = 60 |
| O / C | Claw open / close | on/off |
| I / K | Arm up / down | on/off |
| X / Space | Stop | |

Hold a key to move; release to stop (smooth ramp down).

---

## 3. Calibration (floor test, 30 Oct)

All in `include/motor_config.h`. Wheel order: 0 = front-left, 1 = front-right,
2 = back-left, 3 = back-right.

| Setting | Value | Why |
|---|---|---|
| `MC_DEADBAND_PCT` | 40, 40, 40, 40 | every wheel starts turning at 40 % |
| `MC_TRIM_FWD_PCT` | 90, 100, 90, 100 | forward curved right → left side slowed |
| `MC_TRIM_REV_PCT` | 100, 90, 100, 90 | backward curved left → right side slowed |
| `MC_KICK_PCT` / `MC_KICK_MS` | 90 % / 150 ms | start boost to get the rover moving |
| `MC_STRAFE_ZIGZAG_MS` | 200 ms | 4-wheel strafe stalls on the floor; alternating diagonals works (150–300 ok) |
| `MC_ROT_FAST_PCT` / `MC_ROT_SLOW_PCT` | 60 / 40 | auto turns; slows within 25° of target |

Speeds are "above the minimum": 1 % ≈ 40 % real power, 100 % = full.

---

## 4. Tuning with the bench app

`motor_bench/` is a separate firmware that runs only the motors and reads
commands typed in a serial monitor (115200, line ending LF). Only the 12 V
battery is needed. Start with `h 0` then `ready 1`.

| Command | What it does |
|---|---|
| `t 60` | Each wheel alone, forward then back (raw power, no minimum/boost) |
| `m vx vy w` | Move, e.g. `m 30 0 0` forward, `m 0 100 0` strafe right, `m 0 0 50` turn right, `m 30 30 0` diagonal |
| `s` | Stop |
| `db 40` / `db 1 45` | Minimum power, all wheels / one wheel |
| `trim W FWD REV` | Scale one wheel, e.g. `trim 0 90 100` = front-left 90 % forward only |
| `kick 90 150` | Start boost (`kick 0 0` = off) |
| `zz 200` | Zigzag strafe period (`zz 0` = normal strafe) |
| `r 90` + `h 89` | Test an auto turn (fake heading) |
| `q` | Status line with all current values: copy them into `motor_config.h` |

Bench values are lost on restart. Copy them into `motor_config.h` and rebuild.

---

## 5. Build and flash

In VS Code (Raspberry Pi Pico extension), open the `integrated-rover-ii`
folder and click **Compile**. Outputs:

```
build\firmware\robo_pico\mecanum_robot.uf2     → Robo Pico
build\firmware\sensor_pico\sensor_pico.uf2     → Sensor Pico
```

Bench app (needs the FreeRTOS kernel, `FREERTOS_KERNEL_PATH`):

```
cmake -S firmware\robo_pico\motor_bench -B build_bench -G Ninja
cmake --build build_bench          → build_bench\motor_bench.uf2
```

Flash: hold **BOOTSEL**, plug in USB, drag the `.uf2` onto **RPI-RP2**.

Before building, set your WiFi / broker IP in `include/config.h`, but **don't
commit your own password**.

Laptop unit tests (no Pico needed): `cd firmware/robo_pico/test && make`.

---

## 6. Files

| File | What |
|---|---|
| `include/motor_driver.h`, `src/motor_driver.c` | API used by `main.c` (keys, auto moves, safety) |
| `include/motor_control.h`, `src/motor_control.c` | State machine (statechart 6.6), ramp, heading hold; pure C, unit-tested |
| `include/motor_config.h` | All tuning numbers |
| `include/motor_hw.h`, `src/motor_hw.c`, `include/motor_pins.h` | L298N pins, PWM, arm/claw |
| `include/motor_types.h` | Packed command / heading / result formats |
| `src/motor_task.c`, `include/motor_task.h` | FreeRTOS version (used by the bench app) |
| `motor_bench/` | Bench tuning app |
| `test/` | Laptop unit tests |
