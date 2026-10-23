#ifndef TOF_H
#define TOF_H

#include <stdint.h>

#define TOF_SENSOR_COUNT 3

// A responding sensor with no detected target. Not a measured distance; mapping
// must skip it. Reserved on UART too, so the packet layout remains unchanged.
#define TOF_NO_TARGET_MM UINT16_MAX

typedef enum {
    TOF_LEFT = 0,
    TOF_FRONT = 1,
    TOF_RIGHT = 2
} tof_sensor_t;

typedef struct {
    uint16_t distance_mm[TOF_SENSOR_COUNT];
    uint32_t timestamp_ms[TOF_SENSOR_COUNT];
    uint8_t fresh_mask;  // Bit i: a new result was read for sensor i.
    uint8_t valid_mask;  // Bit i: usable distance OR fresh no-target sentinel.
} tof_readings_t;

// Configure the three VL53L1X sensors in LEFT, FRONT, RIGHT order.
// Pass three distinct XSHUT GPIO numbers that do not conflict with rover wiring.
// Returns a bit mask of sensors that initialized successfully.
uint8_t tof_init(const uint8_t xshut_pins[TOF_SENSOR_COUNT]);

uint8_t tof_online_mask(void);

// Poll once without waiting for a measurement. Output is cleared on every call,
// so an old distance can never be mistaken for a fresh reading.
void tof_poll(tof_readings_t *readings);

#endif // TOF_H
