#ifndef TOF_H
#define TOF_H

#include <stdint.h>

#define TOF_MAX_DISTANCE_MM 1000u
#define TOF_DISTANCE_MASK   0x3FFu  // Ten set bits: 1023
#define TOF_SENSOR_COUNT 3

// Wire encoding:
// 0       = unavailable
// 1–1000  = successful measured distance in mm
// 1023    = successful measurement beyond 1 m; no target inside the reported range
//          (open for DFS, not a numerical distance of 1023 mm)
#define TOF_NO_TARGET_MM    1023u

typedef enum {
    TOF_LEFT = 0,
    TOF_FRONT = 1,
    TOF_RIGHT = 2
} tof_sensor_t;

typedef struct {
    // old 16-bit unsigned int in mm
    // uint16_t distance_mm[TOF_SENSOR_COUNT];

    /*
    the ToF distance is limited by us to maximum of 1000mm (1 metre)
    for tribunal 1 change requirement to minimise the number of bits used, this is 10 bits (1024)
    Change made:
        Store all three ToF readings in a single 32-bit field
        | spare (2b) | Right ToF (10b) | Front ToF (10b) | Left ToF (10b) |
    */
    uint32_t distance_bits; // Three 10-bit fields; top two bits unused.
    uint32_t timestamp_ms[TOF_SENSOR_COUNT];
    // Within each mask: bit 0 = left, 1 = front, 2 = right.
    uint8_t fresh_mask : 3; // A new result was read during this poll.
    uint8_t valid_mask : 3; // Successful range or confirmed beyond-range result.
} tof_readings_t;

// Configure the three VL53L1X sensors in LEFT, FRONT, RIGHT order.
// Pass three distinct XSHUT GPIO numbers that do not conflict with rover wiring.
// Returns a bit mask of sensors that initialized successfully.
uint8_t tof_init(const uint8_t xshut_pins[TOF_SENSOR_COUNT]);

uint8_t tof_online_mask(void);
// Interpret status before packing. Successful ranges >1 m become 1023;
// every warning/error status (including 2/4) is unavailable/unknown (0).
uint16_t tof_result_code(uint8_t range_status, uint16_t distance_mm);
uint16_t tof_distance_get(uint32_t packed, tof_sensor_t sensor);
// Pack an accepted result/cached value. Raw ranges require tof_result_code first.
void tof_distance_set(uint32_t *packed, tof_sensor_t sensor, uint16_t distance_mm);

// Poll once without waiting for a measurement. Output is cleared on every call,
// so an old distance can never be mistaken for a fresh reading.
void tof_poll(tof_readings_t *readings);

#endif // TOF_H
