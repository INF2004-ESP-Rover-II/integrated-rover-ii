# Rover II: tests without a video

These tests are checked with code and output, not a video, as the prof said on 26 Nov: "if your test case is just a function... you don't need a video at all. Show me the code for the test case, this is the evidence." The IDs match **Rover II test cases and results**. The videos are listed in `Rover_II_Test_Videos.md`.

Each test shows the code that does the test, how to run it, and the output. The output files are in this folder.

| ID | Req | Owner | Result | Date | Commit | Output |
|---|---|---|---|---|---|---|
| M8 | FR-40 | Zi Ying | Pass | 2026-11-26 | `99da84f` (main) | `M8_M14_host_tests_main.txt` |
| M14 | FR-42 | Zi Ying | Pass | 2026-11-26 | `99da84f` (main) | `M8_M14_host_tests_main.txt` |
| M11 | FR-41, FR-42 | Zi Ying | Pass | 2026-11-26 | `633a942` (main) | `M11a_stale_heading.txt`, `M11b_manual_no_heading.txt` |
| M12 | FR-42 | Zi Ying | Pass | 2026-11-26 | `633a942` (main) | `M12_timeout.txt` |
| P1 (dry run) | FR-27, FR-29 | Jes | Pass (host tests) | 2026-11-26 | `7e16df7` (feature/goto-on-pico) | `P1_goto_host_tests.txt` |
| V3 | FR-33 | Jeremy | Pass | 2026-11-24 | `02ef2fa` (loopback build) | `V3_uart_loopback/` |
| V4 | FR-36 | Jeremy | Pass | 2026-11-06 | `6b2bcbb` (ESP32-LogiCam-Test) | `V4_camera_health/` |
| V5 | FR-32 | Jeremy | Not run | | | S3 serial log to add |
| C2 | FR-06, FR-08 | Jia Hui | Partial (rover) | 2026-10-30 | `1e57d7c` (JiaHui) | `C2_link_loss.txt` |
| C3 | FR-07 | Jia Hui | Partial (fake rover) | 2026-11-27 | `031c901` (main) | `C3_dashboard_log.txt` |
| C4 | FR-12 | Jia Hui | Pass | 2026-11-27 | `031c901` (main) | `C4_binary_host_tests.txt` |
| C5 | FR-12 | Jia Hui | Pass (fake rover) | 2026-11-27 | `031c901` (main) | `C5_binary_capture_off.txt`, `C5_binary_capture_on.txt` |
| C6 | FR-10 | Jia Hui | Pass (fake rover) | 2026-11-27 | `031c901` (main) | `C6_dashboard_log.txt` |
| C7 | FR-13 | Jia Hui | Pass (rover) | 2026-10-30 | `1e57d7c` (JiaHui) | `C7_uart_packets.txt` |
| C8 | FR-15 | Jia Hui | Pass | 2026-11-27 | `031c901` (main) | `C8_dashboard_log.txt` |
| C9 | FR-11 | Jia Hui | Pass (fake rover) | 2026-11-27 | `031c901` (main) | `C9_stale_data.png`, `C9_dashboard_log.txt` |

---

## M8 and M14: motor host tests (FR-40, FR-42)

**Test code:** `firmware/robo_pico/test/test_motor_control.c` and `test_motor_front_stop.c`. They build the real motor source files with stub hardware.

**How to run:** in `firmware/robo_pico/test`, run `make`.

**Expected:**
- **M8:** the lines `FR-40 source rules` and `ZERO GYRO -> IDLE, FR-40 manual only in READY`. Autonomous mode can't strafe, go diagonal, arc turn or park, and manual commands are refused until the mission is READY.
- **M14:** the line `FR-42 ramp up, normal stop ramps down to IDLE`. Power goes up 5% every 20 ms, and after a normal stop it comes down 5% every 20 ms to 0, ending in IDLE.
- At the end: `96 passed, 0 failed`.

**Output (excerpt):**
```
ZERO GYRO -> IDLE, FR-40 manual only in READY
FR-40 source rules
FR-42 ramp up, normal stop ramps down to IDLE
FR-42 command timeout -> IDLE, refresh keeps it alive
e-stop -> SAFE STOP latched, operator reset -> IDLE
...
96 passed, 0 failed
Front range, timeout and manual E-STOP tests passed
```

## M11 and M12: stale heading and command timeout (FR-41, FR-42)

**How to run:** use the motor bench firmware on the Robo Pico, over USB serial.
- Log the serial output with timestamps: `python tools/bench_logger.py COM3 --log m11a.txt` (type the commands below into that window).
- **M11(a):** `ready 1`, `h 0`, `d 30 0`, wait 2 s, then `h off`. The rover should stop within about 0.6 s and flag a stale heading (`faults=0x04`).
- **M12:** `ready 1`, `m 20 0 0`, then `nr` (stops the bench refreshing the command, like a silent dashboard). The rover should go to IDLE within about 0.3 to 0.5 s (`faults=0x02`).

**Result (26 Nov 2026, motor bench on the Robo Pico, wheels off the ground, logs timestamped by `tools/bench_logger.py`):**
- **M11(a), `M11a_stale_heading.txt`** (terminal copy of the run, as the saved log was overwritten): `d 30 0` ran in AUTO_DRIVE. After `h off` the move ended `STOPPED` and the state went to IDLE with `faults=0x04` (stale heading) 666 ms after `h off` (about 0.6 s, includes about 40 ms serial delay).
- **M11(b), `M11b_manual_no_heading.txt`:** with `h off`, `m 15 0 0` was accepted and the state stayed MANUAL (wheels driving) until `s`.
- **M12, `M12_timeout.txt`:** after `m 20 0 0` and `nr` (no more refreshes), the move ended `STOPPED` about 0.3 s later and the state went to IDLE with `faults=0x02` (command timeout).

The host test output above also has `FR-42 command timeout -> IDLE` and `stale heading -> IDLE`. M11(b) with the dashboard (hold W with the Sensor Pico heading missing) is also in video 2.

## P1: go-to route planning, dry run (FR-27, FR-29)

The go-to is on `feature/goto-on-pico` and isn't merged into main yet, so this is the dry-run evidence the test doc asks for.

**Test code:** `firmware/robo_pico/test/test_route_planner.c`, `test_goto_ctrl.c`, `test_goto_dry.c` and `test_dfs_backtrack.c` (on the branch).

**How to run:** in `firmware/robo_pico/test`, run `make`.

**Output (excerpt):**
```
random: 3439 queries routed, 14561 not routed or trivial, all equal to brute force
test_route_planner: 124353 checks OK
test_goto_ctrl: 91 checks OK
test_goto_dry: 13 checks OK
current layout       returns=4 shortcuts=1 ... total hops stack-run=53 planner-run=51
S layout             returns=11 shortcuts=1 ... total hops stack-run=85 planner-run=81
loop and dead ends   returns=9 shortcuts=0 ... total hops stack-run=75 planner-run=75
test_dfs_backtrack: 2809 checks OK
```

Every route is checked against a brute-force search. "Not routed" covers the unreachable targets.

## V3: UART frames S3 to Pico (FR-33)

**Test code:**
- `firmware/esp32_s3_vision/main/app_main.c`, the `VISION_LOOPBACK_TEST` block. A second UART on GPIO18 reads back what really left GPIO17 and logs it.
- `firmware/esp32_s3_vision/tools/check_uart_log.py`, which checks that every frame sent was read back unchanged.

**How to run:**
1. Build with `VISION_LOOPBACK_TEST` turned on and jumper GPIO17 to GPIO18.
2. Capture the serial log.
3. Run `python tools/check_uart_log.py <log>`.

**Output:**
```
$ python tools/check_uart_log.py loopback_clean.log
sent 59, read 59, mismatched 0, by status {'2': 11, '1': 48}
$ python tools/check_uart_log.py marker_clean.log
sent 98, read 98, mismatched 0, by status {'2': 11, '0': 35, '1': 52}
```

157 frames were sent and all were read back unchanged. The second log had marker ID 10 in view (status 0, 567 mm, -5.75°), so frames carrying marker data were checked too. Status 2 (STALLED) frames only appear at boot, before the first webcam frame.

## V4: camera health status (FR-36)

**Test code:** `firmware/esp32_s3_vision/main/app_main.c`. `VISION_STALL_US` (500 ms) and `VISION_LOST_US` (5000 ms) are on lines 65–66, and the health check is on lines 566–569. The build that was tested (`6b2bcbb` in ESP32-LogiCam-Test) has the same values and the same check.

**How to run:** S3 on the bench with the loopback build, recording the serial log for 60 s. Unplug the webcam, wait more than 5 s, then plug it back in. Then list each status change from the `uart read` lines.

**Expected:** status 2 (STALLED) about 0.5 s after the last good frame, status 3 (CAMERA_LOST) after 5 s, and back to status 0 or 1 once the webcam is plugged back in.

**Output** (`status_transitions.txt`, from `uart_failures_2026-11-06.log`):
```
t=12.62s status 0 seq 45 age 246 N 1  (last status 1 at t=12.43s)
t=20.40s status 2 seq 83 age 776 N 0  (last status 0 at t=19.90s)
t=24.82s status 3 seq 83 age 5196 N 0  (last status 2 at t=24.62s)
t=32.62s status 0 seq 84 age 188 N 1  (last status 3 at t=32.44s)
```

- **STALLED:** 0.50 s after the last good frame (19.90 s to 20.40 s). `age_ms` reads 776 because it counts from the last image that was processed, not the last frame sent.
- **CAMERA_LOST:** `age_ms` 5196, just past the 5 s limit. `seq` stays at 83 the whole time the webcam is out.
- **Recovery:** the webcam was found again without a reset, and status 0 came back at seq 84.
- **UART:** over the whole run, 296 of 297 frames were read back with 0 mismatched (the last frame was still sending when the recording ended).
- The status 2 at 1.84 s is the boot, before the first webcam frame.

The rover version is in video 7.

## V5: marker rejection (FR-32)

**How to run:** S3 on the bench with the serial monitor open. Hold a marker beyond 1.5 m, then tilted steeply.

**Expected:** the marker is reported as TOO_FAR or BAD_SHAPE, not OK, and is never used as a target.

**Evidence:** the S3 serial log with the quality codes.

## C2: link loss (FR-06, FR-08)

**Test code:** `LINK_TIMEOUT_MS` (1500 ms) in `firmware/robo_pico/include/config.h` and the link check in `firmware/robo_pico/src/main.c`; the dashboard sends a heartbeat every 500 ms.

**How it was run:** real rover, router. The link was cut (laptop WiFi off, dashboard closed or killed) while the rover was connected; the time is from the rover's serial log, t(LINK LOST) minus t(last laptop message).

**Expected:** the rover stops by itself within about 1.5 s of the last laptop message, and the mission goes to PAUSED.

**Result (29 to 30 Oct, real rover):** 13 link-lost stops, all between 1501 and 1510 ms, each logged as `[LINK] LOST ... MOTORS STOPPED`:
```
[LINK] LOST at t=97559 ms (last laptop msg at t=96051 ms), MOTORS STOPPED
[LINK] LOST at t=120730 ms (last laptop msg at t=119222 ms), MOTORS STOPPED
...
Stop times (ms): 1508, 1507, 1507, 1508, 1510, 1510, 1510, 1503, 1503, 1509, 1509, 1508, 1501
```
Not yet checked: the mission going to PAUSED, because mission states were added after this run. Rerun on main.

**Output:** `C2_link_loss.txt`.

## C3: E-STOP stays locked until Reset Safety (FR-07)

**Test code:** `trigger_estop()`, `on_key_press()` and `handle_safety_status()` in `dashboard/robot_dashboard.py`. The E-STOP lock is cleared only when the rover confirms `SAFETY_READY` after the operator presses Reset Safety; there is no timer (the old automatic Reset Safety after 3 s was removed, Tribunal 2 row #33).

**How to run:** fake rover and dashboard. Press E-STOP, wait 10 s, press W; then press Reset Safety and press W again.

**Expected:** W is still refused after 10 s and no RESET_SAFETY is sent by itself; movement works only after Reset Safety. (Mission FAULT needs the real rover; the fake rover has no mission states.)

**Result (27 Nov, fake rover):**
```
06:39:07.335  [06:39:07] → ESTOP
06:39:07.343  ⚠ E-STOP ACTIVATED at 06:39:07 - movement locked until Reset Safety
06:39:17.823  Movement blocked - E-STOP active. Press Reset Safety first.
06:39:20.305  [06:39:20] → RESET_SAFETY
06:39:20.314  Manual E-Stop released. Press a movement key to drive.
06:39:20.955  [06:39:20] → FORWARD
```

**Output:** `C3_dashboard_log.txt`.

## C4: binary message format (FR-12)

**Test code:** `tests/test_mqtt_binary.py`. It tests `dashboard/mqtt_binary.py`, the same format the Robo Pico decodes in `firmware/robo_pico/src/mqtt_binary.c`.

**How to run:** from the repository root, run `python tests/test_mqtt_binary.py -v`.

**Expected:** every command encodes to 1 byte (TURN_TO 2 bytes) and decodes back; invalid bytes (0x00, unknown codes, wrong lengths) are rejected; the heartbeat is 1 byte. At the end: `Ran 22 tests` and `OK`.

**Output (excerpt):**
```
test_bad_commands_rejected ... ok
test_every_plain_command_is_one_byte_and_round_trips ... ok
test_heartbeat_one_byte ... ok
test_text_commands_are_never_mistaken_for_binary ... ok
test_turn_to_all_headings ... ok
...
Ran 22 tests in 0.031s
OK
```

## C5: commands on the wire, text vs binary (FR-12)

**Test code:** `send_command()` and `link_heartbeat_tick()` in `dashboard/robot_dashboard.py`, switched by `BINARY_COMMANDS`.

**How to run:** start Mosquitto, `python dashboard/fake_rover.py` and the dashboard, then in another window run `mosquitto_sub -h localhost -t "robot/command" -t "robot/heartbeat" -F "%t  %l bytes  %x"`. Press W, S, D, E-STOP and Reset Safety, once with `BINARY_COMMANDS = False` and once with `True`.

**Expected:** off: each command is sent as text (4 to 12 bytes). On: each command and heartbeat is 1 byte, commands with no binary code (`MAP_GET`) stay text, and the fake rover receives every command correctly.

**Output (excerpt, off, 27 Nov):**
```
robot/command  7 bytes  464f5257415244            (FORWARD)
robot/command  8 bytes  4241434b57415244          (BACKWARD)
robot/command  12 bytes  524f544154455f5249474854  (ROTATE_RIGHT)
robot/command  4 bytes  53544f50                  (STOP)
robot/command  5 bytes  4553544f50                (ESTOP)
robot/command  12 bytes  52455345545f534146455459  (RESET_SAFETY)
robot/heartbeat  2 bytes  3135                    ("15")
```

**Output (excerpt, on, 27 Nov):**
```
robot/command  1 bytes  04      (FORWARD)
robot/command  1 bytes  09      (ROTATE_RIGHT)
robot/command  1 bytes  05      (BACKWARD)
robot/command  1 bytes  08      (ROTATE_LEFT)
robot/command  1 bytes  01      (STOP)
robot/command  1 bytes  02      (ESTOP)
robot/command  1 bytes  03      (RESET_SAFETY)
robot/heartbeat  1 bytes  0d
robot/command  7 bytes  4d41505f474554   (MAP_GET, still text)
```

**Output:** `C5_binary_capture_off.txt` and `C5_binary_capture_on.txt`.

## C6: front obstacle disables only W (FR-10)

**Test code:** the W lock in `navigation_tick()` in `dashboard/robot_dashboard.py`. `FRONT_CLEAR_MARGIN_MM = 50` keeps W locked until the front reading is 50 mm past the stop line, so it can't flicker.

**How to run:** fake rover and dashboard. Hold W until the front obstacle stops the rover, then hold S to back away.

**Expected:** one `Front obstacle: W disabled` message at 150 mm while other keys still drive; one `Front clear: W enabled` message once past 200 mm, with no on/off messages in between.

**Result (27 Nov, fake rover):**
```
06:44:30.781  Front obstacle: W disabled; other keys remain available.
06:44:31.992  [06:44:31] → BACKWARD
06:44:32.610  Front clear: W enabled. Press W again to drive forward.
```
One disable and one enable message, no flicker in between, and BACKWARD still drove while W was disabled.

**Output:** `C6_dashboard_log.txt`.

## C7: Sensor Pico to Robo Pico UART (FR-13)

**Test code:** the packet layout in `uart_protocol.h` (59-byte `sensor_packet_t`, the same on both Picos); the Robo Pico counts total, valid and error packets and prints them as `[STATS]`.

**How it was run:** Robo Pico serial monitor while the Sensor Pico streams packets.

**Expected:** 0 checksum or sync errors.

**Result (real rover):** before the two Picos used the same layout (the Robo Pico expected 67 bytes, the Sensor Pico sent 59), almost every packet failed. After matching the layout, every session had 0 errors:
```
[STATS] Packets: 197 total, 1 valid, 196 errors      (before the fix)
[STATS] Packets: 818 total, 818 valid, 0 errors      (after the fix)
[STATS] Packets: 2079 total, 2079 valid, 0 errors    (30 Oct, 14:07 session)
[STATS] Packets: 1486 total, 1486 valid, 0 errors    (30 Oct, 14:14 session)
```

**Output:** `C7_uart_packets.txt`.

## C8: dashboard session log (FR-15)

**Test code:** `log()` in `dashboard/robot_dashboard.py` writes every log line to a new file per session in `dashboard/logs/`.

**How to run:** connect the dashboard, drive, press E-STOP and Reset Safety, then disconnect.

**Expected:** a new `dashboard_<date>_<time>.txt` with each command, rover status and connection change, each with a millisecond timestamp.

**Result (27 Nov):** `dashboard_..._064356.txt` recorded the connection, every command (FORWARD, STOP, BACKWARD, ESTOP, RESET_SAFETY), the rover's replies and the disconnect, each with a millisecond timestamp.

**Output:** `C8_dashboard_log.txt` (a copy of that session's file).

## C9: stale data is not shown as live (FR-11)

**Test code:** the watchdog in `dashboard/robot_dashboard.py`: `TOF_UART_STALE_MS = 500`, `ROBOT_LOST_S = 2.0`, checked every 250 ms.

**How to run:** fake rover and dashboard connected. In the fake rover window, type `p` and Enter (telemetry pauses for 3 s).

**Expected:** the ToF readings show "Telemetry stale" within 0.5 s, Robot shows Lost after 2 s and movement is blocked; everything returns to normal when telemetry resumes.

**Result (27 Nov, fake rover):** the ToF panel showed "Telemetry stale" for left, front and right instead of distances, Robot showed Lost, and the log recorded:
```
[06:46:44] ⚠ Robot: Lost - no telemetry for 2.1s. Movement blocked.
[06:46:45] Robot link restored - movement enabled
```

**Output:** `C9_stale_data.png` (screenshot during the pause) and `C9_dashboard_log.txt`.
