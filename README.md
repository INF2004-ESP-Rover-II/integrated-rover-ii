# INF2004 - Embedded Mecanum Robot Project

An embedded systems project using Pico W and Robo Pico to control a mecanum-wheeled robot with LIDAR, IMU, and ultrasonic sensors. The system features autonomous navigation and remote control via MQTT.

See [MPU6500 integration](MPU6500.md) for current IMU wiring, calibration and firmware compatibility.

See [ESP32-S3 On-board Vision](#-esp32-s3-on-board-vision-updated-2026-11-06) for the latest vision changes and the UART link to the Robo Pico.

---

## 📁 Project Structure

```
INF2004-Embedded/
├── firmware/
│   ├── robo_pico/          # Robot controller (Pico W)
│   ├── sensor_pico/        # Sensor processor (Pico)
│   ├── esp32_s3_vision/    # On-board marker detection (ESP-IDF, separate build)
│   └── CMakeLists.txt
├── dashboard/              # Python control interface
├── build/                  # Build output (generated)
└── CMakeLists.txt
```

---

## 📂 Folder and File Descriptions

### `firmware/robo_pico/` (Robo Pico)
Controls motors and handles WiFi/MQTT communication.

**src/**
- `main.c` - Main program loop, processes UART sensor data and MQTT commands
- `motor_driver.c` - Controls 6 motors (4 wheels, 1 arm, 1 claw) via PWM
- `wifi_mqtt.c` - WiFi connection and MQTT client for remote commands
- `uart_protocol.c` - Receives binary sensor packets from sensor_pico via UART

**include/**
- `config.h` - Pin assignments and motor tuning parameters
- `motor_driver.h` - Motor control function declarations
- `wifi_mqtt.h` - WiFi/MQTT function declarations
- `uart_protocol.h` - Binary packet structure definitions
- `lwipopts.h` - lwIP network stack configuration

**CMakeLists.txt** - Build configuration for robo_pico firmware

---

### `firmware/sensor_pico/` (Pico W)
Reads sensors and transmits data to robo_pico.

**src/**
- `main.c` - Reads all sensors and sends data packets via UART
- `rplidar.c` - RPLIDAR A1 driver using interrupt-based UART reading
- `mpu6500.c` - MPU6500 accelerometer/gyroscope driver and startup calibration via I2C
- `imu_filter.c` - Complementary pitch/roll filter and gravity-aligned relative heading
- `imu_sampler.c` - Independent core 1 IMU sampling/fusion at a target of 100 Hz
- `ultrasonic.c` - HC-SR04 ultrasonic interface (front/back/right active; legacy left disabled)
- `sd_card.c` - SD card file operations for data logging
- `sd_config.c` - SD card hardware configuration
- `uart_protocol.c` - Sends binary sensor packets to robo_pico via UART

**include/**
- `rplidar.h` - RPLIDAR function declarations
- `mpu6500.h` / `imu_filter.h` / `imu_sampler.h` - IMU driver, filter and snapshot declarations
- `ultrasonic.h` - Ultrasonic sensor function declarations
- `sd_card.h` - SD card function declarations
- `sensor_config.h` - Pin assignments and timing constants
- `uart_protocol.h` - Binary packet structure definitions

**lib/fatfs/** - FatFS library for SD card filesystem support (third-party)

**CMakeLists.txt** - Build configuration for sensor_pico firmware

---

### `dashboard/`
Python GUI for robot control and visualization.

- `robot_dashboard.py` - Tkinter-based dashboard with MQTT control and LIDAR point cloud visualization

---

### `build/`
Generated directory containing compiled firmware files (created during build process).

---

### Root Files
- `CMakeLists.txt` - Top-level build configuration that includes both Pico projects
- `README.md` - This file

---

## 🚀 How to Compile

### Prerequisites
- **CMake** 3.13 or higher
- **Arm GNU Toolchain** (arm-none-eabi-gcc)
- **Pico SDK** v1.5.1 or higher
- Set environment variable: `PICO_SDK_PATH=/path/to/pico-sdk`

### Build Steps

1. **Navigate to project directory:**
   ```bash
   cd INF2004-Embedded
   ```

2. **Create build directory:**
   ```bash
   mkdir build
   cd build
   ```

3. **Configure with CMake:**
   ```bash
   cmake ..
   ```

4. **Compile firmware:**
   ```bash
   ninja
   ```
   Or on Linux/macOS:
   ```bash
   make -j4
   ```

5. **Output files:**
   - `build/firmware/robo_pico/mecanum_robot.uf2` (for Pico W)
   - `build/firmware/sensor_pico/sensor_pico.uf2` (for Pico W)

---

## 📥 How to Flash Firmware

1. **Enter bootloader mode:**
   - Hold BOOTSEL button on the Pico
   - Connect USB cable
   - Release BOOTSEL (Pico appears as USB drive `RPI-RP2`)

2. **Copy firmware:**
   - **Robo Pico (Pico W):** Copy `mecanum_robot.uf2` to the drive
   - **Sensor Pico (Pico W):** Copy `sensor_pico.uf2` to the drive

3. **Automatic reset:**
   - Pico automatically reboots after file copy completes

---

## 🖥️ How to Run

### Start the Robot
1. Power on both Pico boards
2. Robo Pico connects to WiFi and MQTT broker (configured in `config.h`)
3. Sensor Pico begins reading sensors and sending data via UART

### Run the Dashboard
```bash
cd dashboard
python robot_dashboard.py
```

**Requirements:**
- Python 3.8+
- `paho-mqtt` library: `pip install paho-mqtt`

The dashboard connects to the MQTT broker and displays:
- Real-time LIDAR point cloud
- Ultrasonic sensor readings
- Robot relative heading, pitch and roll
- Control interface (keyboard or buttons)

---

## 🧪 How to Test

### Test 1: Serial Output
- Connect Pico via USB
- Open serial monitor (115200 baud)
- Verify sensor readings are printed

### Test 2: Sensor Verification
- Check LIDAR: Rotate robot, observe changing distance values
- Check IMU: Tilt robot, verify pitch/heading changes
- Check Ultrasonic: Place obstacle, verify distance readings

### Test 3: Motor Control
- Use dashboard to send movement commands
- Verify all 4 wheels respond correctly
- Test arm and claw motors

### Test 4: MQTT Communication
- Run dashboard and verify connection to broker
- Send commands, observe robot response
- Check telemetry data updates in real-time

### Test 5: Camera Marker Mapping (4 terminals)
The ESP32-S3 camera streams to the laptop, `vision/vision_publisher.py` detects ArUco markers (DICT_6X6_250, IDs 1-36, 74 mm printed) and publishes them on `camera/marker`, and the dashboard places each marker in the 6x6 grid (60 cm cells) on the Map panel.

Before starting:
- Laptop on the `RoverII` Wi-Fi; `MQTT_BROKER_IP` in `firmware/robo_pico/include/config.h` set to the laptop's IP (`ipconfig getifaddr en0` on macOS), then robo_pico rebuilt and flashed
- Camera powered (stream at `http://192.168.50.5/stream`); close any browser tab showing it, as it serves one viewer at a time
- Python packages: `pip install "paho-mqtt<2" matplotlib numpy opencv-python` (paho-mqtt 2.x breaks the dashboard)

Open 4 terminals in the repo root:

| Terminal | Command | Leave running |
|---|---|---|
| 1. Broker | `mosquitto -c mosquitto-rover.conf -v` (Homebrew: `/opt/homebrew/opt/mosquitto/sbin/mosquitto`) | Yes |
| 2. Dashboard | `cd dashboard && python robot_dashboard.py`, then click **Connect** (`localhost`, 1883) | Yes |
| 3. Feed viewer (optional) | `mosquitto_sub -t camera/marker \| cut -c1-2000` | Yes |
| 4. Camera publisher | `python vision/vision_publisher.py --marker-mm 74 --mqtt localhost` (`q` in the video window quits) | Yes |

Then:
1. Power on the rover and keep it still for a few seconds while the IMU calibrates. Terminal 1 should show `New client connected ... as robot_pico_w`, and the dashboard's "telemetry lost" warning should clear.
2. Place the rover in the start cell (4th column, bottom row; `map_start_cell` in `robot_dashboard.py`) facing up the row of markers. The heading at the first telemetry becomes "up" on the map.
3. Drive along the row. The Map title shows `Marker ahead: ID … at … mm`, and each marker's cell turns green with its ID the first time it is seen.

Notes:
- Stop the broker with Ctrl+C, not Ctrl+Z; a paused broker keeps port 1883 and passes no messages (`Address already in use` on the next start).
- The map title reads `Waiting for fresh IMU heading` until telemetry reports a healthy IMU (`"status":0`); check with `mosquitto_sub -t robot/telemetry -C 3`.
- Markers seen with no mapped marker in view are placed from the rover's last known cell. At the end of a line (front ToF ≤ 45 cm) a turn puts the rover in the last marker's cell; tune `AT_WALL_MM` in `dashboard/vision_feed.py`.
- Without the camera, replay a recording: `python <ESP32-LogiCam-Test>/software/laptop/replay.py session.jsonl | mosquitto_pub -l -t camera/marker`.
- Unit tests: `python -m unittest discover -s tests -p "test_*.py"`.

---

## 📷 ESP32-S3 On-board Vision (updated 2026-11-06)

**For anyone picking this up, including AI sessions: read this before changing vision code.**

The ESP32-S3 can now find ArUco markers and measure range and bearing by itself, with no laptop, and send the results over UART. The firmware is in `firmware/esp32_s3_vision/`. It is an ESP-IDF project (v5.2.7, target `esp32s3`) and is **not** part of the Pico CMake build.

**Nothing on the Robo Pico reads this link yet.** The rover still gets markers from the laptop (`vision/vision_publisher.py`, see Test 5) until the receiver exists.

| Work | Status |
|---|---|
| Detection, range and bearing on the S3 | Done |
| UART sender on the S3 | Done, tested by loopback only |
| Robo Pico receiver on UART1 (GP21) | **Not started** (Vivi's side) |
| End-to-end test with a Pico | Not done |
| Moving the rover from the laptop publisher to UART | Not done |

### What changed

- Added `firmware/esp32_s3_vision/`, moved from the `ESP32-LogiCam-Test` repo so the rover's code is in one place. This folder is now the copy to edit. The old repo keeps the test images, detector comparison tools, generator scripts for `aruco_dict.h` and `camera_calib.h`, the marker sheet and an older handover (`ONBOARD_VISION_HANDOVER_2026-11-05.md`, which still says to use a resistor and that step 7 is not started).
- `main/vision_frame.c` builds the UART message. `main/app_main.c` runs the pipeline in `vision_task` and sends a message for every processed image.
- `main/wifi_secrets.h` is committed on purpose. The router is a local test network, and Jeremy chose not to hide its password. **This repo is public.**

### Wiring

| ESP32-S3 | Robo Pico | Notes |
|---|---|---|
| GPIO17 (UART1 TX) | GP21 (UART1 RX) | Direct wire, with no resistor |
| GND | GND | Required |

Both boards use 3.3 V logic. Because there is no resistor, do not press the Robo Pico's GP21 button while the wire is connected: it shorts GPIO17 to GND. Also avoid powering the S3 while the Pico is off.

### UART message

115200 baud, 8N1, one-way from the S3 to the Pico. Multi-byte fields are little-endian. The format was agreed with Vivi, so **ask Jeremy before changing it**.

| Field | Bytes | Meaning |
|---|---|---|
| Sync | 1 | `0xA5` |
| Sequence | 1 | Goes up per processed image and wraps at 255. Health reports keep the last value |
| Status | 1 | 0 = OK, 1 = NO_MARKER, 2 = STALLED, 3 = CAMERA_LOST |
| `age_ms` | 2 | `uint16_t`, capped at 65535. Keeps rising in health reports |
| Marker count N | 1 | 0 to 4 |
| Marker ID (per marker) | 1 | 1 to 36 by default. Change `VISION_ID_MIN` and `VISION_ID_MAX` in `app_main.c` to test others |
| Range (per marker) | 2 | `uint16_t`, straight-line mm from the lens |
| Bearing (per marker) | 2 | `int16_t`, hundredths of a degree, positive = right |
| Quality (per marker) | 1 | 0 = OK, 1 = pose failed (range and bearing are 0), 2 = bad shape, 3 = too small, 4 = too far |
| CRC | 1 | CRC-8: polynomial 0x07, init 0, no reflection, xorout 0. Covers sequence through the last payload byte |

A message is 7 bytes with no marker, 13 with one and 31 with four. `vision_crc8()` and `vision_frame_build()` in `vision_frame.c` are the reference for the receiver.

### Still to agree with Vivi

- **Health limits:** STALLED after 0.5 s without a processed image, and CAMERA_LOST after 5 s without a camera frame, repeated every 0.2 s. These are the S3's current values (`VISION_STALL_US`, `VISION_LOST_US`, `VISION_HEALTH_US`), not agreed numbers.
- **`age_ms` start point:** it starts when the frame reaches the S3, because the S3 cannot see when the picture was taken. It reads low by the camera and USB delay, which has not been measured.

Vivi has not been told either point yet.

### Build, flash and test

```bash
cd firmware/esp32_s3_vision
idf.py set-target esp32s3   # first build only
idf.py build
idf.py -p <port> flash      # hold BOOT, tap EN/RST, release BOOT if it cannot connect
```

Use the S3's UART USB-C port for flashing, with the webcam in the other port. Tapping EN/RST by itself can leave the board in download mode.

Host tests, which need only a C compiler and, for the second one, OpenCV:

```bash
python3 firmware/esp32_s3_vision/tools/test_vision_frame.py
python3 firmware/esp32_s3_vision/tools/test_mjpeg_dht.py
```

**Loopback test without a Pico:** `VISION_LOOPBACK_TEST` in `app_main.c` is **currently on**. With a wire from GPIO17 to GPIO18, a second UART reads the bytes back and logs them. Save the serial log and check it with `tools/check_uart_log.py <log>`. Comment the define out for the final rover build. Leaving it on does not change what the Pico receives; it only adds log lines, and `nothing on GPIO18` warnings when the wire is absent.

### Measured on the board (2026-11-06)

- **Speed:** 5.3 fps processed with the camera at 15 fps. Decode takes 41 ms, detection 134 ms, and `age_ms` is about 180 to 250 ms.
- **Loopback:** 296 of 297 frames read back in a 60 s run, the last still being sent when the capture ended. There were no mismatches, bad frames or sequence gaps.
- **Camera unplugged:** STALLED appeared with `age_ms` at 776, CAMERA_LOST at 5196, and the sequence held still. The camera recovered without a reset when plugged back in.
- **Marker removed:** NO_MARKER with N = 0, and the sequence kept rising.

### Known limits

- **Not tested on the board:** a real Pico receiver, several real markers in one frame, and the filter that drops IDs 37 to 40.
- **Wi-Fi blocks vision:** the S3 waits forever for the `RoverII` router at startup, so with the router off the Pico gets no messages at all, not even health reports.
- **`/stream` slows vision:** the stream handler holds the frame lock while sending, so processing slows with a viewer connected. Running the detector with the stream open has not been measured.
- **Range accuracy:** range and bearing were compared with OpenCV on the laptop, not with a tape measure. With the marker placed straight ahead, the bearing read about +1.8° at 60 cm and 120 cm, which may be camera twist but has not been confirmed. The lens offset from the rover's centre has not been measured. Do not change the camera calibration without asking Jeremy.

---

## 🔧 Configuration

**WiFi/MQTT Settings** (`firmware/robo_pico/include/config.h`):
```c
#define WIFI_SSID "your-network"
#define WIFI_PASSWORD "your-password"
#define MQTT_BROKER_IP "192.168.1.100"
```

**Motor Pin Assignments** (`firmware/robo_pico/include/config.h`):
```c
#define MOTOR_FL_IN1 2   // Front-left motor
#define MOTOR_FL_IN2 3
// ... (see file for complete pin list)
```

**Sensor Pin Assignments** (`firmware/sensor_pico/include/sensor_config.h`):
```c
#define LIDAR_RX_PIN 9       // RPLIDAR UART
#define IMU_SDA_PIN 6        // IMU I2C1 SDA
#define IMU_SCL_PIN 7        // IMU I2C1 SCL
#define ULTRASONIC_TRIG_PIN 10  // Ultrasonic trigger
```

---

**Last Updated:** December 2025
