# INF2004 - Embedded Mecanum Robot Project

An embedded systems project using Pico W and Robo Pico to control a mecanum-wheeled robot with LIDAR, IMU, and ultrasonic sensors. The system features autonomous navigation and remote control via MQTT.

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
- `accelero.c` - LSM303DLHC IMU driver (accelerometer + magnetometer) via I2C
- `ultrasonic.c` - HC-SR04 ultrasonic distance sensor interface (2 sensors)
- `sd_card.c` - SD card file operations for data logging
- `sd_config.c` - SD card hardware configuration
- `uart_protocol.c` - Sends binary sensor packets to robo_pico via UART

**include/**
- `rplidar.h` - RPLIDAR function declarations
- `accelero.h` - IMU function declarations
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
- Robot heading and pitch
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
#define IMU_SDA_PIN 4        // IMU I2C
#define ULTRASONIC_TRIG_PIN 10  // Ultrasonic trigger
```

---

**Last Updated:** December 2025

