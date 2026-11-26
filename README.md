# Rover II (integrated-rover-ii)

Firmware, vision code and dashboard for the INF2004 Rover II project (group P1-SE10). The rover explores a grid of ArUco markers on its own and can drive to a chosen marker. See the [organisation page](https://github.com/INF2004-ESP-Rover-II) for the overview, requirements and owners.

## Repository layout

| Path | What it is |
|---|---|
| `firmware/robo_pico/` | Robo Pico: mission control, DFS, motion control, MQTT. Builds `mecanum_robot` |
| `firmware/robo_pico/motor_bench/` | Separate motor bench app with serial commands (needs the FreeRTOS kernel) |
| `firmware/robo_pico/test/` | Host tests for motion control (run on a laptop) |
| `firmware/sensor_pico/` | Sensor Pico: ToF sensors and gyro. Builds `sensor_pico` |
| `firmware/esp32_s3_vision/` | ESP32-S3 ArUco detector (ESP-IDF project) |
| `firmware/common/` | Headers shared by the Picos |
| `dashboard/` | Laptop dashboard (`robot_dashboard.py`) and a fake rover for offline testing |
| `vision/` | Laptop-side camera publisher and calibration |
| `rover_mapping/` | Mapping and route-planning reference code and tests |
| `tests/` | Python and C tests for the dashboard, DFS and sensor code |
| `mosquitto-rover.conf` | MQTT broker config for the rover network |

## What you need

- Raspberry Pi Pico SDK 2.3.1 with the ARM toolchain 15_2_Rel1. The easiest way is the Raspberry Pi Pico extension for VS Code, which `CMakeLists.txt` is set up for.
- CMake and Ninja.
- Python 3 with `paho-mqtt`, `matplotlib` and `numpy` for the dashboard, plus `opencv-python` for the vision publisher.
- Mosquitto (MQTT broker) on the laptop.
- ESP-IDF for the ESP32-S3 vision firmware (the component versions in `firmware/esp32_s3_vision/dependencies.lock` allow ESP-IDF 4.4.1 up to 5.5).
- A host C compiler (gcc) and `make` for the host tests.

## First-time setup

1. Clone the repository and open the top `integrated-rover-ii` folder in VS Code.
2. Set the WiFi name and password, and the broker IP, in `firmware/robo_pico/include/config.h`. The broker IP is the laptop's address on the rover network (`ipconfig` shows it). Do not commit your own WiFi password.
3. For the ESP32-S3, put the WiFi name and password in `firmware/esp32_s3_vision/main/wifi_secrets.h` (see `wifi_secrets.example.h`). Do not commit it.
4. Install the Python packages: `pip install paho-mqtt matplotlib numpy opencv-python`.

## Build and flash the Picos

In VS Code, press Compile. Or from a terminal in the repo root:

```
cmake -S . -B build -G Ninja
cmake --build build --target mecanum_robot
cmake --build build --target sensor_pico
```

The outputs are `build/firmware/robo_pico/mecanum_robot.uf2` and `build/firmware/sensor_pico/sensor_pico.uf2`. To flash a board, hold BOOTSEL while plugging it in over USB, then copy the matching `.uf2` onto the drive that appears. If a build fails strangely after a big pull, delete the `build` folder and compile again.

### Motor bench (optional)

The bench needs the FreeRTOS kernel (`raspberrypi/FreeRTOS-Kernel`). Set `FREERTOS_KERNEL_PATH` to it, then:

```
cmake -S firmware/robo_pico/motor_bench -B build_bench -G Ninja
cmake --build build_bench
```

Flash `build_bench/motor_bench.uf2`. Open the USB serial port at 115200 baud and type `h 0` then `ready 1`; the commands are listed at the top of `firmware/robo_pico/motor_bench/main.c`.

## ESP32-S3 vision

From `firmware/esp32_s3_vision`, with ESP-IDF set up: `idf.py build flash monitor`.

## Run the system

1. Connect the laptop to the rover WiFi (the broker address is `192.168.50.11`).
2. Start the broker: `mosquitto -c mosquitto-rover.conf -v`. If it will not start, another Mosquitto may already be running; stop it first.
3. Start the dashboard: `python dashboard/robot_dashboard.py`, then click Connect.
4. Power the rover. The dashboard shows telemetry once the Robo Pico connects.
5. Set the start node, send Start DFS, or drive manually.

Manual keys: W/S forward/back, A/D turn, Q/E strafe, R/T/F/G diagonals, X or Space E-STOP. Movement is only accepted when the mission is READY.

`start_rover_mapping.ps1` (Windows PowerShell) starts the vision publisher and dashboard together.

To try the dashboard without a rover, run `python dashboard/fake_rover.py` against a local broker.

## Finding the next DFS target

Set **Target nodes (0 = all)** on the dashboard before starting DFS. Values
1–64 finish mapping after that many unique confirmed (green) nodes are in the
onboard map, including the start node and any previously confirmed nodes.
Revisits and blue camera estimates do not add to the count. Zero keeps full DFS
exploration; reaching actual DFS exhaustion can also finish before a larger
requested limit is reached.

At the limit, RoboPico stays at the last confirmed node, finishes a settled exit
scan and stops with mission `COMPLETE`, reason/detail `NODE_LIMIT_REACHED`.
Still-unknown directions each get a bounded scan attempt; unavailable readings
remain unknown. It does not start another journey or backtrack to the origin.
Open unexplored exits and blue estimates are retained, so this completion means
the requested mapping limit was reached, not that the whole maze was exhausted.
Normal exhaustion still reports `DFS_COMPLETE` / `DFS_EXHAUSTED`.

The dashboard sends `START_DFS:<opening_mm>:<node_limit>` (for example,
`START_DFS:700:4`) and displays the confirmed count from RoboPico's `DFS_STATE`.
Legacy `START_DFS` and `START_DFS:<opening_mm>` retain the previous settings;
explicitly send a zero limit to return to unlimited exploration. Invalid
settings are rejected atomically; stop DFS before changing them. Re-initialising
the onboard DFS context clears the limit; the next dashboard Start sends its
current form settings again. The dashboard never marks arrivals or finishes DFS
locally from the count.

At a confirmed node, RoboPico keeps its chosen exit and waits for an eligible
camera marker. After five seconds without a locked target, it scans 10 degrees
left, centre, 10 degrees right, centre, repeating around that exit. Each stop
settles for 300 ms, then allows two seconds for target confirmation. Only fresh
images captured after stopping can confirm a target; moving-image candidates
are discarded. Selection, map compatibility and arrival checks still apply.

After a full unsuccessful left/centre/right/centre pass, it can reverse for
150 ms at a 15% motor command, stop, settle and confirm new camera images before
starting another pass. Reverse is allowed only with fresh camera health, fresh
sensor/heading/link inputs and no eligible locked target. Camera outages stop
motion and wait. Accepted reverse command durations total at most one second
per confirmed-node visit; the budget survives retries and outages. At the cap,
scanning continues in place (`SEARCHING_TARGET_REVERSE_LIMIT`). The reverse
uses an independent short motor lease and does not renew it. Existing motor
calibration and startup boost still apply to the 15% command.

Missing markers no longer trigger `DFS_TARGET_NOT_VISIBLE` or pause exploration.
`DFS_STATE` reports `WAIT_TARGET` with scan/wait details on the dashboard. Camera
outages stop scanning until fresh observations return. Sensor, heading, link and
lease waits retain DFS; operator Stop and E-stop remain effective. Each scan turn
still stops and retries after three seconds, with a five-second turn limit;
reaching that limit stops that scan attempt and continues searching. Ordinary DFS
heading turns have a 60-second total limit and retain the same three-second
stop/retry cycle. Searching does not mark a node visited or drive forward before
a target is locked. The confirmed node is retained as the map anchor during the
bounded positioning adjustment; no odometry-based arrival is inferred. Forward-first exit priority remains unchanged.

Reference-search driving now has a 20-second budget. The current controller
still charges subsequent tracked approach to that same budget; the separately
agreed search-only/overall journey timer redesign remains pending.

### Stop a target scan when a marker appears

During next-target recovery, any fresh good-quality marker sighting immediately
stops the preset turn or reverse, including off-centre sightings. A moving
image is only a stop hint. After settling, new stopped images must confirm an
eligible target. While a compatible candidate remains visible, confirmation
is not interrupted by the next timed scan step.

This stopped acquisition allows candidates within 45 degrees of camera centre,
then uses the existing alignment controller to get within 10 degrees before
forward departure. Negative bearings turn left; positive bearings turn right.
Three distinct target-confirmation images and the existing fresh alignment
checks remain required. Map/exit compatibility, marker quality and arrival
checks still apply; sighting a marker never marks it explored.

An incompatible, ambiguous, lost or unusable sighting gets a bounded stationary
recheck, then scanning resumes. Camera outages freeze the accumulated recheck and
scan-dwell progress instead of restarting it. Motion stays stopped throughout
the outage; fresh observations resume the remaining wait. Overlapping sensor or
link waits are excluded once. Interrupted movement still requires a new settled
pose and post-stop images. This lets unsuccessful checks progress to the existing
bounded scan reverse without reversing on stale camera data.
A continuously visible rejected marker does not
repeatedly restart the same stop; a new sighting or newly eligible candidate can
interrupt scanning again. Stale camera data stops motion and waits as before.
Relevant DFS details: `MARKER_SEEN_STOPPING_SCAN`, `CONFIRMING_VISIBLE_MARKER`,
`CHECKING_VISIBLE_MARKER` and `MARKER_UNUSABLE_RESUMING_SCAN`.

## Recovering unresolved DFS exits

If next-leg validation reports `GRID_DIRECTION_NOT_ALIGNED`, DFS now stops,
retains the chosen exit and destination identity, and turns to that exit's grid
bearing (`RECOVERING_GRID_HEADING`). It keeps the confirmed current node, map,
stack/backtracking state and accumulated scan-reverse budget. After the turn it
settles and requires fresh target confirmation before departure; cached images
or another marker cannot replace the selected destination. Motor/input waits
retain the correction. Existing turn retries, the 60-second turn limit and
Stop/E-stop checks remain. The 15-degree grid validation is unchanged.

After arrival, exits that remain unknown no longer pause DFS with
`DFS_EXITS_UNRESOLVED`. RoboPico repeats its settled ToF measurements and cycles
retry headings through the grid directions, letting different sensors inspect
an uncertain side. The dashboard reports `RESCANNING_UNRESOLVED_EXITS` during
these retries. Each measurement window still requires two fresh, consecutive
matching classifications; unavailable, faulted or inconsistent readings remain
unknown. Existing confirmed nodes, traversed edges and the DFS stack are kept.

When readings resolve, the normal planner continues exploring, backtracks, or
finishes only if the entire map is resolved. Unvisited camera estimates still
prevent completion. Sensor/link/heading waits retain recovery; Stop/E-stop and
existing turn/motor checks remain effective. Forward-first priority is unchanged.

## Front stop while waiting for the camera

During a configured journey that has already departed, a front stop now
supersedes `WAITING_CAMERA` and enters stationary obstacle-arrival verification.
The controller clears the saved camera-resume state so camera recovery cannot
restart forward motion while it verifies the obstacle. A pending stop event is
handled even if the current range has already cleared by the next main-loop tick.

After 300 ms settling, two distinct fresh ToF samples below the 400 mm front
threshold confirm arrival. Stale/cached ranges, zero/unavailable readings, a
single transient pulse or an urgent flag without close measurements do not mark
the target explored. Camera observations are not required for this ToF arrival
path. On success RoboPico updates the current node/map and resumes DFS exit
scanning and planning. Restored sensor/link/heading inputs restart confirmation;
operator Stop/E-stop, faults, map changes and existing journey limits still apply.
Predeparture alignment and reverse recovery cannot claim obstacle arrival.

## Motor outputs during startup

RoboPico sets all wheel, arm and claw motor control pins low as its first
operation in `main`, before USB startup delays and Wi-Fi connection attempts.
They remain low while networking starts, including when Wi-Fi connection fails.
Normal motor/PWM initialisation follows later; startup does not request motion.

## Front obstacle stopping distance

DFS exit classification defaults to 700 mm in both firmware and dashboard:
valid ranges below 700 mm are blocked; 700 mm or farther are open. The dashboard
can configure this separately when starting DFS. Zero/faulted returns remain
unknown. This classification applies to settled node scans, not motor stopping.

The shared front stopping threshold is 400 mm: valid readings below 400 mm
stop forward movement. Flash both RoboPico and Sensor Pico builds to keep their
obstacle handling consistent. Side urgency remains 150 mm; map exit thresholds
are separate. The dashboard uses the threshold reported by the rover, with a
400 mm fallback.

Starting or resuming reference search no longer requires 600 mm front clearance.
A healthy reading of 400 mm or farther permits slow search, including after a
camera wait or alignment correction. No usable return still permits bounded
search when the sensor is healthy. Existing camera, sensor, heading, motor and
journey-budget checks remain. Below 400 mm the front guard stops forward motion;
an eligible departed journey must settle and receive two fresh close samples
before confirming obstacle arrival. Removing the clearance gate does not mark
the target visited or reset the accumulated search time. The 600 mm visual
reference-arrival range is unchanged.

## Tests

- Motion control host tests: `cd firmware/robo_pico/test` then `make`. It should end with `96 passed, 0 failed`.
- Python tests: from the repo root, `python -m unittest discover -s tests -p "test_*.py"`. Some dashboard tests (ToF, manual turn, heading zero, DFS mission) are out of date with the current dashboard and are being updated, so a full run currently reports failures.

## Code style and known deviations from BARR-C (motor code)

The motor code (`firmware/robo_pico/src/motor_*.c`, `include/motor_*.h`, `motor_bench/`) follows BARR-C:2018 as the team applies it: Allman braces, braces on every `if`/`else`/loop, 80 columns, fixed-width types, no `//` comments, a `default` in every `switch`, and `volatile` on state shared with interrupts. These were checked by hand and with a script, not with a BARR-C or MISRA tool. Magic numbers and function complexity have not been audited. The clang-format config is `firmware/esp32_s3_vision/.clang-format`.

Two things deliberately do not follow the rules or the formatter:

1. **Plain `unsigned` bit-fields in `mc_t` and `mc_out_t`** (`include/motor_control.h`). BARR-C asks for fixed-width types, but a bit-field cannot portably be declared as `uint8_t` or `uint32_t`. These fields pack the private Motion Control state, which only one task writes. They are never sent over a wire or stored: the command word, heading word, direction byte, result byte and fault mask are built with explicit unsigned shifts and masks (BARR-C 5.3.b), so their layout does not depend on the compiler. The order of the bit-fields inside `mc_t` is compiler-defined, and no code depends on it.
2. **Four files are not formatted with clang-format:** `include/motor_config.h`, `include/motor_types.h`, `include/motor_control.h` and `motor_bench/FreeRTOSConfig.h`. Their aligned defines and multi-line macros stay within 80 columns, but clang-format rewrites them into backslash-continued lines that are harder to read. Do not run format-on-save or clang-format on these four files; edit them by hand and keep lines within 80 columns. Every other motor file is clang-formatted.
