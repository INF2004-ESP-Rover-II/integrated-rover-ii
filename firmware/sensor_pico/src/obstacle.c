#include "obstacle.h"
#include "sensor_config.h"
#include <string.h>

#if OBSTACLE_URGENT_MM == 0 || OBSTACLE_URGENT_MM >= OBSTACLE_BLOCKED_MM
#error "Require 0 < OBSTACLE_URGENT_MM < OBSTACLE_BLOCKED_MM"
#endif
#if OBSTACLE_BLOCKED_MM + OBSTACLE_HYSTERESIS_MM > 65535
#error "Obstacle thresholds must fit in a uint16_t distance"
#endif

void obstacle_init(obstacle_state_t *state) {
    memset(state, 0, sizeof(*state));
}

void obstacle_update(obstacle_state_t *state, const tof_readings_t *readings,
                     uint8_t online_mask, uint32_t now_ms) {
    // Retained as zero for compatibility with the existing UART packet layout.
    state->fault_mask = 0;

    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        const uint8_t bit = (uint8_t)(1u << i);

        // No fresh result means retain the cached result, not overwrite it with 0.
        if (readings->fresh_mask & bit) {
            if ((readings->valid_mask & bit) && readings->distance_mm[i] > 0) {
                state->distance_mm[i] = readings->distance_mm[i];
                state->timestamp_ms[i] = readings->timestamp_ms[i];
                state->valid_mask |= bit;
            } else {
                state->valid_mask &= (uint8_t)~bit;
            }
        }

        if ((online_mask & bit) == 0) {
            state->valid_mask &= (uint8_t)~bit;
        }

        if ((state->valid_mask & bit) == 0 ||
            (uint32_t)(now_ms - state->timestamp_ms[i]) > OBSTACLE_STALE_MS) {
            // Unavailable readings are omitted; they do not request a stop.
            // Clear stale proximity flags so only current usable ranges count.
            state->valid_mask &= (uint8_t)~bit;
            state->distance_mm[i] = 0;
            state->blocked_mask &= (uint8_t)~bit;
            state->urgent_mask &= (uint8_t)~bit;
            continue;
        }

        const uint16_t distance = state->distance_mm[i];

        // A fresh no-target result is healthy but has no numerical distance.
        // Clear old proximity flags rather than retaining an earlier close hit.
        if (distance == TOF_NO_TARGET_MM) {
            state->blocked_mask &= (uint8_t)~bit;
            state->urgent_mask &= (uint8_t)~bit;
            continue;
        }

        if (distance <= OBSTACLE_BLOCKED_MM) {
            state->blocked_mask |= bit;
        } else if (distance >= OBSTACLE_BLOCKED_MM + OBSTACLE_HYSTERESIS_MM) {
            state->blocked_mask &= (uint8_t)~bit;
        }

        if (distance <= OBSTACLE_URGENT_MM) {
            state->urgent_mask |= bit;
        } else if (distance >= OBSTACLE_URGENT_MM + OBSTACLE_HYSTERESIS_MM) {
            state->urgent_mask &= (uint8_t)~bit;
        }
    }

    // Keep proximity flags for all directions, but only front proximity stops.
    // Missing/invalid readings do not participate in obstacle stopping.
    state->stop_required = (state->urgent_mask & (1u << TOF_FRONT)) != 0;
}
    
