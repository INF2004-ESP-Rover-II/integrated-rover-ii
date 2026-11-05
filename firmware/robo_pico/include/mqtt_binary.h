/*
 * Binary MQTT payloads for the Robo Pico (docs/MQTT_BINARY.md).
 * Must stay in step with dashboard/mqtt_binary.py.
 */
#ifndef MQTT_BINARY_H
#define MQTT_BINARY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MQTT_TELEMETRY_VERSION 1u
#define MQTT_TELEMETRY_BYTES   23u   /* 179 bits used */

/*
 * Values exactly as the firmware already holds them. mqtt_pack_telemetry()
 * converts, clamps and packs; nothing here has to be pre-scaled.
 * Fill this from ONE consistent snapshot (see the note in mqtt_binary.c).
 */
typedef struct {
    uint32_t timestamp_ms;    /* Sensor Pico uptime from the UART packet */
    float    heading_deg;     /* 0..360, wrapped if outside */
    float    pitch_deg;       /* clamped to -180..180 */
    float    roll_deg;        /* clamped to -180..180 */
    bool     imu_ok;          /* true when the IMU status is 0 */
    uint16_t us_front_cm;     /* 0..500 (400 = max range, 500 = no echo) */
    uint16_t us_back_cm;
    uint32_t tof_bits;        /* Vivi's packed field, copied as is (bits 0..29) */
    uint16_t obstacle_flags;  /* Vivi's field, bits 0..9 used */
    uint32_t age_ms;          /* saturates at 1023 */
    uint32_t heading_epoch;   /* motor_heading_epoch(); sent mod 256 (change counter) */
    float    heading_offset_deg; /* motor_heading_offset(); 0.001 deg, +/-524.287 */
    uint16_t forward_threshold_mm; /* FRONT_STOP_DISTANCE_MM, clamped to 1..1000 */
} mqtt_telemetry_t;

/* Packs into out[MQTT_TELEMETRY_BYTES]; returns the number of bytes to publish. */
size_t mqtt_pack_telemetry(const mqtt_telemetry_t *t, uint8_t out[MQTT_TELEMETRY_BYTES]);

typedef enum {
    MQTT_CMD_INVALID = 0,
    MQTT_CMD_STOP = 1, MQTT_CMD_ESTOP, MQTT_CMD_RESET_SAFETY,
    MQTT_CMD_FORWARD, MQTT_CMD_BACKWARD, MQTT_CMD_LEFT, MQTT_CMD_RIGHT,
    MQTT_CMD_ROTATE_LEFT, MQTT_CMD_ROTATE_RIGHT,
    MQTT_CMD_FRONT_LEFT, MQTT_CMD_FRONT_RIGHT, MQTT_CMD_BACK_LEFT, MQTT_CMD_BACK_RIGHT,
    MQTT_CMD_CLAW_OPEN, MQTT_CMD_CLAW_CLOSE, MQTT_CMD_ARM_UP, MQTT_CMD_ARM_DOWN,
    MQTT_CMD_ZERO_HEADING, MQTT_CMD_TURN_TO, MQTT_CMD_AUTO_STOP, MQTT_CMD_AUTO_START,
    MQTT_CMD_COUNT
} mqtt_command_code_t;

typedef struct {
    mqtt_command_code_t code;
    uint16_t turn_heading_deg;   /* only for MQTT_CMD_TURN_TO: 0..359 */
} mqtt_command_t;

/* Binary commands are 1 or 2 bytes; every text command is at least 4 ("STOP"). */
static inline bool mqtt_is_binary_command(size_t len) { return len >= 1u && len <= 2u; }

/* Returns false (and leaves *out as MQTT_CMD_INVALID) for anything malformed. */
bool mqtt_decode_command(const uint8_t *payload, size_t len, mqtt_command_t *out);

/*
 * Writes the old text form ("FORWARD", "TURN_TO:90") so the existing
 * process_command() can be reused unchanged. Returns false if it does not fit.
 */
bool mqtt_command_text(const mqtt_command_t *cmd, char *buf, size_t size);

#endif /* MQTT_BINARY_H */
