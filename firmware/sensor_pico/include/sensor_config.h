#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

// ====================== UART CONFIG ======================
// Communication with Robo Pico
#define UART_ID         uart0
#define UART_BAUD_RATE  115200
#define UART_TX_PIN     16   // GP16 - connects to Robo Pico RX (GP1)
#define UART_RX_PIN     17   // GP1 - connects to Robo Pico TX (GP0)

// ====================== RPLIDAR CONFIG ======================
// RPLIDAR A1 UART (uses UART1, separate from Robo Pico)
#define RPLIDAR_UART_ID     uart1       // UART1 for RPLIDAR
#define RPLIDAR_TX_PIN      8           // GP8 → RPLIDAR RX (White wire)
#define RPLIDAR_RX_PIN      9           // GP9 ← RPLIDAR TX (Green wire)
#define RPLIDAR_BAUD_RATE   115200      // Standard RPLIDAR baud rate

// ====================== ACCELEROMETER CONFIG (LSM303DLHC) ======================
#define I2C_PORT        i2c0
#define SDA_PIN         4   // GP4
#define SCL_PIN         5   // GP5

// LSM303DLHC I2C Addresses
#define LSM303_ACCEL_ADDR   0x19  // Accelerometer
#define LSM303_MAG_ADDR     0x1E  // Magnetometer

// ====================== VL53L1X TOF CONFIG ======================
// Wire each XSHUT pin to the matching sensor before flashing this firmware.
// GP6/GP7/GP8 from the standalone example are already used by other sensors.
#define TOF_LEFT_XSHUT_PIN   18
#define TOF_FRONT_XSHUT_PIN  19
#define TOF_RIGHT_XSHUT_PIN  20

// Provisional obstacle thresholds. Validate against measured stopping distance.
// A blocked exit is a navigation event; only front urgency requests a proximity stop.
#define OBSTACLE_URGENT_MM      150u
#define OBSTACLE_BLOCKED_MM     400u
#define OBSTACLE_HYSTERESIS_MM   50u
#define OBSTACLE_STALE_MS       300u

// ====================== ULTRASONIC SENSOR CONFIG ======================
// Front Sensor
#define ULTRA_FRONT_TRIG_PIN    26
#define ULTRA_FRONT_ECHO_PIN    27

// Back Sensor
#define ULTRA_BACK_TRIG_PIN     0
#define ULTRA_BACK_ECHO_PIN     1

// Left Sensor
#define ULTRA_LEFT_TRIG_PIN     6
#define ULTRA_LEFT_ECHO_PIN     7

// Right Sensor
#define ULTRA_RIGHT_TRIG_PIN    2
#define ULTRA_RIGHT_ECHO_PIN    3

// Ultrasonic timing
#define ULTRASONIC_TIMEOUT_US   30000  // Echo timeout (30000µs = 30ms, ~5m max range @ 343m/s sound speed)
#define ULTRASONIC_SAMPLE_DELAY 60     // Delay between consecutive readings (60ms prevents echo overlap)

// ====================== SYSTEM CONFIG ======================
#define SENSOR_UPDATE_RATE_HZ   20     // Sensor packet transmission rate (20Hz = every 50ms)
#define LED_PIN                 25     // Onboard LED for status indication (GP25)

// Packet sync bytes for frame alignment
#define PACKET_SYNC_BYTE1       0xAA
#define PACKET_SYNC_BYTE2       0x55

#endif // SENSOR_CONFIG_H
