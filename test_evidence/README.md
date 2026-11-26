# Rover II: tests without a video

These tests are checked with code and output, not a video, as the prof said on 26 Nov: "if your test case is just a function... you don't need a video at all. Show me the code for the test case, this is the evidence." The IDs match **Rover II test cases and results**. The videos are listed in `Rover_II_Test_Videos.md`.

Each test shows the code that does the test, how to run it, and the output. The output files are in this folder.

| ID | Req | Owner | Result | Date | Commit | Output |
|---|---|---|---|---|---|---|
| M8 | FR-40 | Zi Ying | Pass | 2026-11-26 | `99da84f` (main) | `M8_M14_host_tests_main.txt` |
| M14 | FR-42 | Zi Ying | Pass | 2026-11-26 | `99da84f` (main) | `M8_M14_host_tests_main.txt` |
| M11 | FR-41, FR-42 | Zi Ying | Not run | | | serial log to add |
| M12 | FR-42 | Zi Ying | Not run | | | serial log to add |
| P1 (dry run) | FR-27, FR-29 | Jes | Pass (host tests) | 2026-11-26 | `7e16df7` (feature/goto-on-pico) | `P1_goto_host_tests.txt` |
| V3 | FR-33 | Jeremy | Pass | 2026-11-24 | `02ef2fa` (loopback build) | `V3_uart_loopback/` |
| V4 | FR-36 | Jeremy | Not run | | | S3 serial log to add |
| V5 | FR-32 | Jeremy | Not run | | | S3 serial log to add |

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
- **M11(a):** `h 0`, `ready 1`, `d 30 0`, then `h off`. The rover should stop within about 0.6 s and flag a stale heading.
- **M12:** `m 20 0 0`, then stop sending commands. The rover should go to IDLE within about 0.3 to 0.5 s.

**Evidence:** the serial log showing the state change and the fault flag. The host test output above also has `FR-42 command timeout -> IDLE` and `stale heading -> IDLE`. M11(b), manual driving with no heading, shows the rover moving, so it goes in video 2.

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

**How to run:** S3 on the bench with the serial monitor open, then unplug the webcam.

**Expected:** status 1 (NO_MARKER) changes to 2 (STALLED) within about 0.5 s, then to 3 (CAMERA_LOST) after about 5 s.

**Evidence:** the S3 serial log. The rover version is in video 7.

## V5: marker rejection (FR-32)

**How to run:** S3 on the bench with the serial monitor open. Hold a marker beyond 1.5 m, then tilted steeply.

**Expected:** the marker is reported as TOO_FAR or BAD_SHAPE, not OK, and is never used as a target.

**Evidence:** the S3 serial log with the quality codes.
