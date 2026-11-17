/*
 * Binary MQTT payloads for the Robo Pico (docs/MQTT_BINARY.md).
 *
 * Thread safety: mqtt_pack_telemetry() only reads its argument. Fill the
 * mqtt_telemetry_t from one consistent copy of the shared sensor data (copy the
 * UART snapshot under the same lock / critical section that the UART callback
 * uses), then pack the private copy. Never pack directly from data an ISR or
 * another task may be half-way through updating.
 */
#include "mqtt_binary.h"

#include <stdio.h>
#include <string.h>

/* Field widths in bits, in wire order (least-significant bit first). */
enum {
    W_VERSION = 4, W_TIMESTAMP = 32, W_ANGLE = 12, W_IMU_FAULT = 1,
    W_ULTRASONIC = 9, W_TOF = 30, W_FLAGS = 10, W_AGE = 10,
    W_EPOCH = 8, W_OFFSET = 20, W_THRESHOLD = 10,
};
#define OFFSET_LIMIT_MDEG ((1L << 19) - 1L)   /* +/-524.287 degrees */

static void put_bits(uint8_t *buf, uint32_t *pos, uint32_t value, uint32_t width)
{
    for (uint32_t i = 0; i < width; i++, (*pos)++) {
        if ((value >> i) & 1u) {
            buf[*pos >> 3] |= (uint8_t)(1u << (*pos & 7u));
        }
    }
}

static int32_t round_d10(float degrees)
{
    float tenths = degrees * 10.0f;
    return (tenths >= 0.0f) ? (int32_t)(tenths + 0.5f) : -(int32_t)(-tenths + 0.5f);
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint32_t offset_mdeg(float degrees)
{
    if (!(degrees == degrees)) {
        degrees = 0.0f;
    }
    if (degrees > 524.0f) degrees = 524.287f;    /* clamp before converting */
    if (degrees < -524.0f) degrees = -524.287f;
    float milli = degrees * 1000.0f;
    int32_t v = (milli >= 0.0f) ? (int32_t)(milli + 0.5f) : -(int32_t)(-milli + 0.5f);
    if (v > OFFSET_LIMIT_MDEG) v = (int32_t)OFFSET_LIMIT_MDEG;
    if (v < -OFFSET_LIMIT_MDEG) v = -(int32_t)OFFSET_LIMIT_MDEG;
    return (uint32_t)v & ((1u << W_OFFSET) - 1u);  /* 20-bit two's complement */
}

static uint32_t heading_d10(float heading)
{
    if (!(heading == heading)) {   /* NaN */
        return 0u;
    }
    /* Wrap into [0, 360) without fmodf so no libm is needed. */
    while (heading >= 360.0f) heading -= 360.0f;
    while (heading < 0.0f) heading += 360.0f;
    return (uint32_t)(round_d10(heading) % 3600);
}

static uint32_t angle_d10(float degrees)
{
    if (!(degrees == degrees)) {
        degrees = 0.0f;
    }
    if (degrees > 180.0f) degrees = 180.0f;     /* also avoids int overflow */
    if (degrees < -180.0f) degrees = -180.0f;
    int32_t v = clamp_i32(round_d10(degrees), -1800, 1800);
    return (uint32_t)v & ((1u << W_ANGLE) - 1u);   /* 12-bit two's complement */
}

size_t mqtt_pack_telemetry(const mqtt_telemetry_t *t, uint8_t out[MQTT_TELEMETRY_BYTES])
{
    uint32_t pos = 0;
    memset(out, 0, MQTT_TELEMETRY_BYTES);

    put_bits(out, &pos, MQTT_TELEMETRY_VERSION, W_VERSION);
    put_bits(out, &pos, t->timestamp_ms, W_TIMESTAMP);
    put_bits(out, &pos, heading_d10(t->heading_deg), W_ANGLE);
    put_bits(out, &pos, angle_d10(t->pitch_deg), W_ANGLE);
    put_bits(out, &pos, angle_d10(t->roll_deg), W_ANGLE);
    put_bits(out, &pos, t->imu_ok ? 0u : 1u, W_IMU_FAULT);
    put_bits(out, &pos, t->us_front_cm > 511u ? 511u : t->us_front_cm, W_ULTRASONIC);
    put_bits(out, &pos, t->us_back_cm > 511u ? 511u : t->us_back_cm, W_ULTRASONIC);
    put_bits(out, &pos, t->tof_bits & 0x3FFFFFFFu, W_TOF);
    put_bits(out, &pos, t->obstacle_flags & 0x3FFu, W_FLAGS);
    put_bits(out, &pos, t->age_ms > 1023u ? 1023u : t->age_ms, W_AGE);
    put_bits(out, &pos, t->heading_epoch & 0xFFu, W_EPOCH);
    put_bits(out, &pos, offset_mdeg(t->heading_offset_deg), W_OFFSET);
    put_bits(out, &pos, t->forward_threshold_mm < 1u ? 1u :
                        (t->forward_threshold_mm > 1000u ? 1000u : t->forward_threshold_mm),
             W_THRESHOLD);

    return MQTT_TELEMETRY_BYTES;
}

bool mqtt_decode_command(const uint8_t *payload, size_t len, mqtt_command_t *out)
{
    out->code = MQTT_CMD_INVALID;
    out->turn_heading_deg = 0;
    if (payload == NULL || !mqtt_is_binary_command(len)) {
        return false;
    }
    uint16_t word = payload[0];
    if (len == 2u) {
        word |= (uint16_t)((uint16_t)payload[1] << 8);
    }
    uint8_t code = (uint8_t)(word & 0x1Fu);
    if (code == MQTT_CMD_INVALID || code >= MQTT_CMD_COUNT) {
        return false;
    }
    if (code == MQTT_CMD_TURN_TO) {
        uint16_t heading = (uint16_t)((word >> 5) & 0x1FFu);
        if (len != 2u || heading > 359u || (word >> 14) != 0u) {
            return false;
        }
        out->turn_heading_deg = heading;
    } else if (len != 1u || (word >> 5) != 0u) {
        return false;
    }
    out->code = (mqtt_command_code_t)code;
    return true;
}

static const char *const COMMAND_NAMES[MQTT_CMD_COUNT] = {
    [MQTT_CMD_STOP] = "STOP", [MQTT_CMD_ESTOP] = "ESTOP", [MQTT_CMD_RESET_SAFETY] = "RESET_SAFETY",
    [MQTT_CMD_FORWARD] = "FORWARD", [MQTT_CMD_BACKWARD] = "BACKWARD",
    [MQTT_CMD_LEFT] = "LEFT", [MQTT_CMD_RIGHT] = "RIGHT",
    [MQTT_CMD_ROTATE_LEFT] = "ROTATE_LEFT", [MQTT_CMD_ROTATE_RIGHT] = "ROTATE_RIGHT",
    [MQTT_CMD_FRONT_LEFT] = "FRONT_LEFT", [MQTT_CMD_FRONT_RIGHT] = "FRONT_RIGHT",
    [MQTT_CMD_BACK_LEFT] = "BACK_LEFT", [MQTT_CMD_BACK_RIGHT] = "BACK_RIGHT",
    [MQTT_CMD_CLAW_OPEN] = "CLAW_OPEN", [MQTT_CMD_CLAW_CLOSE] = "CLAW_CLOSE",
    [MQTT_CMD_ARM_UP] = "ARM_UP", [MQTT_CMD_ARM_DOWN] = "ARM_DOWN",
    [MQTT_CMD_ZERO_HEADING] = "ZERO_HEADING", [MQTT_CMD_TURN_TO] = "TURN_TO",
    [MQTT_CMD_AUTO_STOP] = "AUTO_STOP", [MQTT_CMD_AUTO_START] = "AUTO_START",
};

bool mqtt_command_text(const mqtt_command_t *cmd, char *buf, size_t size)
{
    if (cmd->code <= MQTT_CMD_INVALID || cmd->code >= MQTT_CMD_COUNT || size == 0u) {
        return false;
    }
    int n = (cmd->code == MQTT_CMD_TURN_TO)
        ? snprintf(buf, size, "TURN_TO:%u", (unsigned)cmd->turn_heading_deg)
        : snprintf(buf, size, "%s", COMMAND_NAMES[cmd->code]);
    return n > 0 && (size_t)n < size;
}
