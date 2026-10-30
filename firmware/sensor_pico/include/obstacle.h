#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>
#include "tof.h"

// Masks use the tof_sensor_t indices: bit 0 = left, 1 = front, 2 = right.
#define OBSTACLE_ALL_MASK ((uint8_t)((1u << TOF_SENSOR_COUNT) - 1u))

typedef struct {
    // uint16_t distance_mm[TOF_SENSOR_COUNT]; // 0 unavailable; TOF_NO_TARGET_MM no target.
    uint32_t distance_bits;
    uint32_t timestamp_ms[TOF_SENSOR_COUNT];
    uint8_t valid_mask : 3;       // Usable distance/no-target; age checked every update.
    uint8_t blocked_mask : 3;     // Exit blocked at the far threshold.
    uint8_t urgent_mask : 3;      // Very close obstacles; only the front bit stops.
    uint8_t fault_mask : 3;      // Offline or no accepted result for >300 ms.
    uint8_t stop_required : 1;      // Usable front urgent obstacle only.
} obstacle_state_t;

// Starts unavailable, with a first-result grace period measured from now_ms.
// The first update reports failed initialization immediately via online_mask.
void obstacle_init(obstacle_state_t *state, uint32_t now_ms);

// Call on every loop to expire old ranges, even when tof_poll() has no new results.
// readings timestamps and now_ms must use the same Sensor Pico clock.
// Fault bits clear only on a fresh accepted distance/no-target result while online.
void obstacle_update(obstacle_state_t *state, const tof_readings_t *readings,
                     uint8_t online_mask, uint32_t now_ms);

#endif
