#ifndef CONFIG_H
#define CONFIG_H

// ====================== WIFI CONFIG ======================
#define WIFI_SSID         "hello"
#define WIFI_PASSWORD     "2rpxp6hswfyx837"


// ====================== MQTT CONFIG ======================
#define MQTT_BROKER_IP    "10.146.68.60"
#define MQTT_BROKER_PORT  1883
#define MQTT_CLIENT_ID    "robot_pico_w"

// MQTT Topics
#define MQTT_TOPIC_COMMAND    "robot/command"
#define MQTT_TOPIC_STATUS     "robot/status"
#define MQTT_TOPIC_TELEMETRY  "robot/telemetry"

// ====================== MOTOR CONFIG =====================
// Motor pins are in motor_pins.h; motor tuning is in motor_config.h.
// Dashboard command timeout (ms): a held move stops if no command arrives.
#define TIMEOUT_MS         800

#endif // CONFIG_H
