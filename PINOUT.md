# GPIO Pinout Reference

Complete pin assignments for both Pico boards in the mecanum robot system.

---

## Robo Pico (Pico W) - Motor Controller

### WiFi Configuration
- **SSID:** Brandon
- **Password:** cocomelon
- **MQTT Broker:** 172.20.10.2:1883
- **Client ID:** robot_pico_w

### Motor Control Pins

#### Front L298N Motor Driver
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| ENA (Enable A) | GP4 | Front-left motor PWM speed control |
| ENB (Enable B) | GP5 | Front-right motor PWM speed control |
| IN1 | GP3 | Front-left motor direction control |
| IN2 | GP2 | Front-left motor direction control |
| IN3 | GP16 | Front-right motor direction control |
| IN4 | GP17 | Front-right motor direction control |

#### Back L298N Motor Driver
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| ENA (Enable A) | GP27 | Back-left motor PWM speed control |
| ENB (Enable B) | GP26 | Back-right motor PWM speed control |
| IN1 | GP28 | Back-left motor direction control |
| IN2 | GP7 | Back-left motor direction control |
| IN3 | GP0 | Back-right motor direction control |
| IN4 | GP1 | Back-right motor direction control |

#### Arm Motor Pins
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| M1A (Arm Down) | GP11 | Arm downward movement |
| M1B (Arm Up) | GP10 | Arm upward movement |

#### Claw Motor Pins
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| M1A (Close) | GP8 | Claw close/grip |
| M1B (Open) | GP9 | Claw open/release |

### UART Communication (Sensor Data)
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| UART RX | GP13 | Receives sensor packets from Sensor Pico |
| UART TX | (unused) | No data sent to Sensor Pico |

**UART Settings:**
- Baud Rate: 115200
- Data Bits: 8
- Stop Bits: 1
- Parity: None

### Motor Configuration
- **PWM Frequency:** 10,000 Hz
- **Default Speed:** 90%
- **PWM Clock Divider:** 1.0
- **Timeout:** 800 ms

---

## Sensor Pico (Pico W) - Sensor Processor

### UART Communication (to Robo Pico)
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| UART TX | GP16 | Sends sensor packets to Robo Pico GP13 |
| UART RX | GP17 | (unused) |

**UART Settings:**
- UART ID: uart0
- Baud Rate: 115200
- Packet Format: Binary with 0xAA 0x55 sync bytes

### RPLIDAR A1 (2D LIDAR Scanner)
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| UART TX | GP8 | Sends commands to RPLIDAR (White wire) |
| UART RX | GP9 | Receives scan data from RPLIDAR (Green wire) |
| Motor PWM | (external) | RPLIDAR motor control (connected to 5V) |

**RPLIDAR Settings:**
- UART ID: uart1
- Baud Rate: 115200
- Scan Rate: 5-10 Hz
- Range: 0.15m - 12m
- Angular Resolution: 1°

### GY-6500 / MPU6500 (Accelerometer + Gyroscope)
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| SDA (Data) | GP6 | IMU I2C1 data |
| SCL (Clock) | GP7 | IMU I2C1 clock |
| VCC | 3.3V | Power supply |
| GND | GND | Ground |
| AD0 | GND | Selects 0x68 (0x69 when tied to 3V3) |
| CS / NCS, if exposed | 3.3V | Selects I2C mode |

The IMU uses I2C1 on GP6/GP7; ToF remains on I2C0 at GP4/GP5.
The old magnetometer driver is removed. The IMU configures
+/-4 g and +/-2000 deg/s. Core 1 samples/fuses at a target of 100 Hz. Keep the
rover level and stationary at startup for gravity-aligned yaw. Mount +X forward,
+Y left, +Z up for rover-aligned pitch/roll. Heading is clockwise from startup and drifts;
there is no magnetometer or implemented ArUco correction. See [MPU6500.md](MPU6500.md).

### HC-SR04 Ultrasonic Sensors (3 active channels)

Front, back and right are enabled. The legacy left channel on GP6/GP7 is
disabled because those pins now belong to the IMU. It reports 500/unavailable
and status bit 2. Reassign unused pins before enabling it again.

#### Front Ultrasonic Sensor
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| TRIG | GP26 | Trigger pulse output |
| ECHO | GP27 | Echo pulse input |

#### Back Ultrasonic Sensor
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| TRIG | GP0 | Trigger pulse output |
| ECHO | GP1 | Echo pulse input |

**Ultrasonic Settings:**
- Echo Timeout: 30,000 µs (30 ms, ~5m max range)
- Sample Delay: 60 ms (prevents echo overlap)
- Range: 2 cm - 4 m
- Accuracy: ±3 mm

### SD Card (SPI - FatFS Filesystem)
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| MISO | GP12 | SPI data input (card → Pico) |
| MOSI | GP11 | SPI data output (Pico → card) |
| SCK | GP10 | SPI clock |
| CS | GP13 | Chip select (active low) |

**SD Card Settings:**
- SPI Port: spi1
- Clock Speed: 10 MHz
- Filesystem: FatFS R0.15
- File Format: CSV for LIDAR data logging

### Status LED
| Function | GPIO Pin | Description |
|----------|----------|-------------|
| Onboard LED | GP25 | Indicates system status |

### System Configuration
- **Sensor Update Rate:** 20 Hz (50 ms interval)
- **Packet Sync Bytes:** 0xAA 0x55
- **Max LIDAR Points per Packet:** 100

---

## Pin Usage Summary

### Robo Pico (Pico W)
| GPIO | Function | Direction |
|------|----------|-----------|
| GP0 | Back Motor IN3 | Output |
| GP1 | Back Motor IN4 | Output |
| GP2 | Front Motor IN2 | Output |
| GP3 | Front Motor IN1 | Output |
| GP4 | Front Motor ENA (PWM) | Output |
| GP5 | Front Motor ENB (PWM) | Output |
| GP7 | Back Motor IN2 | Output |
| GP8 | Claw M1A (Close) | Output |
| GP9 | Claw M1B (Open) | Output |
| GP10 | Arm M1B (Up) | Output |
| GP11 | Arm M1A (Down) | Output |
| GP13 | UART RX (Sensor Data) | Input |
| GP16 | Front Motor IN3 | Output |
| GP17 | Front Motor IN4 | Output |
| GP26 | Back Motor ENB (PWM) | Output |
| GP27 | Back Motor ENA (PWM) | Output |
| GP28 | Back Motor IN1 | Output |
| **WiFi** | CYW43 (onboard) | WiFi/MQTT |

### Sensor Pico (Pico W)
| GPIO | Function | Direction |
|------|----------|-----------|
| GP0 | Ultrasonic Back TRIG | Output |
| GP1 | Ultrasonic Back ECHO | Input |
| GP2 | Ultrasonic Right TRIG | Output |
| GP3 | Ultrasonic Right ECHO | Input |
| GP4 | ToF SDA (I2C0 Data) | I/O |
| GP5 | ToF SCL (I2C0 Clock) | Output |
| GP6 | IMU SDA (I2C1 Data) | I/O |
| GP7 | IMU SCL (I2C1 Clock) | Output |
| GP8 | RPLIDAR UART TX | Output |
| GP9 | RPLIDAR UART RX | Input |
| GP10 | SD Card SCK (SPI Clock) | Output |
| GP11 | SD Card MOSI (SPI Out) | Output |
| GP12 | SD Card MISO (SPI In) | Input |
| GP13 | SD Card CS (Chip Select) | Output |
| GP16 | UART TX (to Robo Pico) | Output |
| GP17 | UART RX (unused) | Input |
| GP25 | Onboard LED | Output |
| GP26 | Ultrasonic Front TRIG | Output |
| GP27 | Ultrasonic Front ECHO | Input |

---

## Wiring Connections Between Boards

### UART Connection (Sensor Pico → Robo Pico)
```
Sensor Pico GP16 (TX) ────→ Robo Pico GP13 (RX)
Sensor Pico GND      ────→ Robo Pico GND (Common Ground)
```

---

## Power Requirements

### Robo Pico
- **Pico W Power:** 5V via USB or VSYS
- **Motor Drivers:** External 12V battery for L298N (separate power rail)
- **Motor Current:** ~2A per motor (total ~12A for 6 motors)

### Sensor Pico
- **Pico W Power:** 5V via USB or VSYS
- **RPLIDAR:** 5V @ 500mA (external power recommended)
- **SD Card:** 3.3V @ 100mA (from Pico)
- **IMU:** 3.3V @ 5mA (from Pico)
- **Ultrasonics:** 5V @ 15mA each (~60mA total)

---

## Configuration Files Reference

### Robo Pico
- Pin definitions: `firmware/robo_pico/include/config.h`
- Motor control: `firmware/robo_pico/src/motor_driver.c`
- WiFi/MQTT: `firmware/robo_pico/src/wifi_mqtt.c`

### Sensor Pico
- Pin definitions: `firmware/sensor_pico/include/sensor_config.h`
- RPLIDAR driver: `firmware/sensor_pico/src/rplidar.c`
- IMU driver: `firmware/sensor_pico/src/mpu6500.c`
- Tilt/heading filter: `firmware/sensor_pico/src/imu_filter.c`
- Ultrasonic driver: `firmware/sensor_pico/src/ultrasonic.c`
- SD card driver: `firmware/sensor_pico/src/sd_card.c`

---

**Last Updated:** December 2025  
**Hardware Version:** v1.0
