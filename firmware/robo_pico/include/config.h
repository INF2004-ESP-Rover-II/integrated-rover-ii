#ifndef CONFIG_H
#define CONFIG_H

// ====================== WIFI CONFIG ======================
#define WIFI_SSID      "Brandon" //"Glenn"
#define WIFI_PASSWORD  "cocomelon" //"glennlim"

// ====================== MQTT CONFIG ======================
#define MQTT_BROKER_IP    "172.20.10.2"  // Flyingpigs Wifi IP is 192.168.50.200, Brandon Hotspot is 172.20.10.2
#define MQTT_BROKER_PORT  1883
#define MQTT_CLIENT_ID    "robot_pico_w"

// MQTT Topics
#define MQTT_TOPIC_COMMAND    "robot/command"
#define MQTT_TOPIC_STATUS     "robot/status"
#define MQTT_TOPIC_TELEMETRY  "robot/telemetry"

// ====================== MOTOR CONFIG =====================
// Front L298N pins
#define F_ENA_PIN   4
#define F_ENB_PIN   5
#define F_IN1_PIN   3
#define F_IN2_PIN   2
#define F_IN3_PIN   16
#define F_IN4_PIN   17

// Back L298N pins
#define B_ENA_PIN   27
#define B_ENB_PIN   26
#define B_IN1_PIN   28
#define B_IN2_PIN   7
#define B_IN3_PIN   0
#define B_IN4_PIN   1

// Arm motor pins (per wiring: M2A=PIN11 is Arm Down, M2B=PIN10 is Arm Up)
#define ARM_M1A_PIN  11  // Arm Down
#define ARM_M1B_PIN  10  // Arm Up

// Claw motor pins (per wiring: M1A=PIN8 is Claw Close, M1B=PIN9 is Claw Open)
#define CLAW_M1A_PIN   8  // Claw Close
#define CLAW_M1B_PIN   9  // Claw Open

// Motor tuning parameters
#define DEFAULT_SPEED      90
#define PWM_FREQ_HZ        10000
#define PWM_CLKDIV         1.0f
#define TIMEOUT_MS         800

#define ARM_BRAKE_ON_STOP  1

// =================== ARM CONTROLLER CONFIG ===============
// PID Controller Gains (tune based on arm mechanical properties)
#define ARM_PID_KP              2.0f    // Proportional gain (angular error → speed)
#define ARM_PID_KI              0.1f    // Integral gain (accumulated error correction)
#define ARM_PID_KD              0.5f    // Derivative gain (rate of change damping)
#define ARM_PID_INTEGRAL_LIMIT  50.0f   // Anti-windup limit for integral term

// Angle Limits
#define ARM_ANGLE_MIN          -90.0f   // Minimum safe angle (-90.0°, downward)
#define ARM_ANGLE_MAX           90.0f   // Maximum safe angle (90.0°, upward)
#define ARM_ANGLE_TOLERANCE     2.0f    // Position reached tolerance (±2.0°)

// Speed Control
#define ARM_MIN_SPEED           20      // Minimum PWM to overcome friction (20% duty cycle)
#define ARM_SPEED_SCALE         2.0f    // PID output → motor speed multiplier (2.0x)

// Timing
#define ARM_IMU_TIMEOUT_MS      500     // Failsafe: Stop arm if no IMU data for 500ms
#define ARM_CONTROL_INTERVAL_MS 50      // PID update rate (50ms = 20Hz control loop)

#endif // CONFIG_H