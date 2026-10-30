# INF2004 - Embedded Mecanum Robot Project

An embedded systems project using Pico W and Robo Pico to control a mecanum-wheeled robot with LIDAR, IMU, and ultrasonic sensors. The system features autonomous navigation and remote control via MQTT.

See [MPU6500 integration](MPU6500.md) for current IMU wiring, calibration and firmware compatibility.

---

## Merge summary (30 Oct 2026)

This branch merges the Jia Hui (Subsystem 1 safety/fault handling), Vivi (vision, marker map, MPU6500) and Buddy 2 (Subsystem 4 mapping and path planning) work into one repo. Details: [MERGE_NOTES.md](MERGE_NOTES.md), [MAPPING_INTEGRATION.md](MAPPING_INTEGRATION.md).

**Changes**
- **Dashboard** (`dashboard/robot_dashboard.py`): Vivi's live marker grid, IMU roll and strafe-odometry fix, plus Jia Hui's link-lost / SAFE STOP / REFUSED / RECONNECTED handling, STOP latency, log files and clean disconnect. New **Mapping & path** tab (DFS graph, A* routes, probable links, ToF obstacle layer). Works with paho-mqtt 1.x and 2.x. Default broker is `localhost`.
- **Fake rover** (`dashboard/fake_rover.py`): Jia Hui's safety-aware fake rover, now with `--maze [--geometry]` which simulates a 6x6 marker maze, camera markers and box obstacles.
- **Mapping** (`rover_mapping/`): mapping statechart, marker graph with probable links, A* planner, route runner with blockage hook, 5 cm occupancy (obstacle) grid. 80 tests, no hardware needed.
- **Firmware**: `sensor_pico` is Vivi's MPU6500 version (sends roll; required by the `robo_pico` packet). `robo_pico` is Jia Hui's safety/link-loss firmware plus Vivi's telemetry fields (roll, accel, gyro, IMU status), telemetry every 100 ms, pitch baseline 0.
- **Build**: root and firmware `CMakeLists.txt` now import the Pico SDK once through the repo's `pico_sdk_import.cmake` (fixes a cpu-flag build failure and the VS Code extension path error).

**Run it (no hardware)**
```
mosquitto -c mosquitto-rover.conf -v
cd dashboard && python fake_rover.py --maze --geometry      # keep --speed 1
MAPPING_STANDOFF_CM=0 MAPPING_CAMERA_HEIGHT_CM=0 python robot_dashboard.py
```
Connect (`localhost`), **Reset Safety**, DFS target markers `36`, open path `500`, tick **LAPTOP DFS**. Real rover: see "Test 5" below.

**Known issues / not yet verified**
- Firmware merge builds in CMake but is untested on the rover; confirm 100 ms telemetry is stable.
- `ARRIVAL_BEARING_DEG` is Vivi's loosened 30 deg (restore to 5 before real runs).
- Next-marker arrival, DFS-phase blockage handling and the DFS blocked-return `from_marker` bug are not done.
- Stand-off, ToF forward offset and camera tilt are unmeasured defaults.
- Wi-Fi SSID in `config.h` must be plain ASCII (no emoji) and 2.4 GHz.

---

## 📁 Project Structure

```
INF2004-Embedded/
├── firmware/
│   ├── robo_pico/          # Robot controller (Pico W)
│   ├── sensor_pico/        # Sensor processor (Pico)
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
