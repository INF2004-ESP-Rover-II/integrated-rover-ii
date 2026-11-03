#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

// ====================== UART CONFIG ======================
// Communication with Robo Pico
#define UART_ID         uart0
#define UART_BAUD_RATE  115200
#define UART_TX_PIN     16   // GP16 - connects to Robo Pico RX (GP1)
#define UART_RX_PIN     17   // GP1 - connects to Robo Pico TX (GP0)

// ====================== MPU6500 IMU (I2C1) ======================
#define IMU_I2C_PORT        i2c1
#define IMU_SDA_PIN         6   // GP6
#define IMU_SCL_PIN         7   // GP7
#define MPU6500_ADDR        0x68  // AD0 -> GND; use 0x69 for AD0 -> 3V3
#define MPU6500_YAW_SIGN   -1.0f // Clockwise heading about the startup gravity direction

// ====================== VL53L1X TOF CONFIG ======================
#define TOF_I2C_PORT        i2c0
#define TOF_SDA_PIN         4   // GP4
#define TOF_SCL_PIN         5   // GP5
// Wire each XSHUT pin to the matching sensor before flashing this firmware.
// GP6/GP7 belong to the IMU; GP8 is reserved for the legacy LIDAR UART.
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

// The old left sensor's GP6/GP7 are now the IMU bus. Do not initialize or
// trigger ultrasonic GPIO there. To restore it, wire unused pins and define
// ULTRA_LEFT_TRIG_PIN / ULTRA_LEFT_ECHO_PIN before enabling this option.
#define ULTRA_LEFT_ENABLED      0
#if ULTRA_LEFT_ENABLED
#if !defined(ULTRA_LEFT_TRIG_PIN) || !defined(ULTRA_LEFT_ECHO_PIN)
#error "Reassign the left ultrasonic pins before enabling it"
#elif ULTRA_LEFT_TRIG_PIN == IMU_SDA_PIN || ULTRA_LEFT_TRIG_PIN == IMU_SCL_PIN || \
      ULTRA_LEFT_ECHO_PIN == IMU_SDA_PIN || ULTRA_LEFT_ECHO_PIN == IMU_SCL_PIN
#error "Left ultrasonic pins overlap the MPU6500 I2C bus"
#endif
#endif

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
