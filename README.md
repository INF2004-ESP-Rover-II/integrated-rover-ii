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

## Tests

- Motion control host tests: `cd firmware/robo_pico/test` then `make`. It should end with `96 passed, 0 failed`.
- Python tests: from the repo root, `python -m unittest discover -s tests -p "test_*.py"`. Some dashboard tests (ToF, manual turn, heading zero, DFS mission) are out of date with the current dashboard and are being updated, so a full run currently reports failures.

## Code style and known deviations from BARR-C (motor code)

The motor code (`firmware/robo_pico/src/motor_*.c`, `include/motor_*.h`, `motor_bench/`) follows BARR-C:2018 as the team applies it: Allman braces, braces on every `if`/`else`/loop, 80 columns, fixed-width types, no `//` comments, a `default` in every `switch`, and `volatile` on state shared with interrupts. These were checked by hand and with a script, not with a BARR-C or MISRA tool. Magic numbers and function complexity have not been audited. The clang-format config is `firmware/esp32_s3_vision/.clang-format`.

Two things deliberately do not follow the rules or the formatter:

1. **Plain `unsigned` bit-fields in `mc_t` and `mc_out_t`** (`include/motor_control.h`). BARR-C asks for fixed-width types, but a bit-field cannot portably be declared as `uint8_t` or `uint32_t`. These fields pack the private Motion Control state, which only one task writes. They are never sent over a wire or stored: the command word, heading word, direction byte, result byte and fault mask are built with explicit unsigned shifts and masks (BARR-C 5.3.b), so their layout does not depend on the compiler. The order of the bit-fields inside `mc_t` is compiler-defined, and no code depends on it.
2. **Four files are not formatted with clang-format:** `include/motor_config.h`, `include/motor_types.h`, `include/motor_control.h` and `motor_bench/FreeRTOSConfig.h`. Their aligned defines and multi-line macros stay within 80 columns, but clang-format rewrites them into backslash-continued lines that are harder to read. Do not run format-on-save or clang-format on these four files; edit them by hand and keep lines within 80 columns. Every other motor file is clang-formatted.
