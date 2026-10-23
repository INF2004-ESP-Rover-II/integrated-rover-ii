#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>
#include "tof.h"

// Masks use the tof_sensor_t indices: bit 0 = left, 1 = front, 2 = right.
#define OBSTACLE_ALL_MASK ((uint8_t)((1u << TOF_SENSOR_COUNT) - 1u))

typedef struct {
    uint16_t distance_mm[TOF_SENSOR_COUNT]; // 0 unavailable; TOF_NO_TARGET_MM no target.
    uint32_t timestamp_ms[TOF_SENSOR_COUNT];
    uint8_t valid_mask;       // Usable distance/no-target; age checked every update.
    uint8_t blocked_mask;     // Exit blocked at the far threshold.
    uint8_t urgent_mask;      // Very close obstacles; only the front bit stops.
    uint8_t fault_mask;       // Reserved for UART compatibility; always zero.
    bool stop_required;      // Usable front urgent obstacle only.
} obstacle_state_t;

// Starts with all directions unavailable and no obstacle stop requested.
void obstacle_init(obstacle_state_t *state);

// Call on every loop to expire old ranges, even when tof_poll() has no new results.
// readings timestamps and now_ms must use the same Sensor Pico clock.
void obstacle_update(obstacle_state_t *state, const tof_readings_t *readings,
                     uint8_t online_mask, uint32_t now_ms);

#endif
