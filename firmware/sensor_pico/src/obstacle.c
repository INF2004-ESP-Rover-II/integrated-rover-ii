#include "obstacle.h"
#include "sensor_config.h"
#include <string.h>

#if OBSTACLE_URGENT_MM == 0 || OBSTACLE_URGENT_MM >= OBSTACLE_BLOCKED_MM
#error "Require 0 < OBSTACLE_URGENT_MM < OBSTACLE_BLOCKED_MM"
#endif
#if OBSTACLE_BLOCKED_MM + OBSTACLE_HYSTERESIS_MM > TOF_MAX_DISTANCE_MM
#error "Obstacle thresholds exceed the supported ToF range"
#endif

void obstacle_init(obstacle_state_t *state, uint32_t now_ms)
{
    memset(state, 0, sizeof(*state));
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i) {
        // Reuse the per-sensor clocks for startup grace; no extra RAM is needed.
        state->timestamp_ms[i] = now_ms;
    }
}

void obstacle_update(obstacle_state_t *state, const tof_readings_t *readings,
                     uint8_t online_mask, uint32_t now_ms)
{
    for (int i = 0; i < TOF_SENSOR_COUNT; ++i)
    {
        const uint8_t bit = (uint8_t)(1u << i);
        const tof_sensor_t sensor = (tof_sensor_t)i;
        const uint16_t incoming = tof_distance_get(readings->distance_bits, sensor);
        const bool online = (online_mask & bit) != 0;
        const bool accepted = (incoming > 0 && incoming <= TOF_MAX_DISTANCE_MM) ||
                              incoming == TOF_NO_TARGET_MM;

        // No fresh result means retain the cached result, not overwrite it with 0.
        if (readings->fresh_mask & bit)
        {
            if (online && (readings->valid_mask & bit) && accepted)
            {
                // state->distance_mm[i] = readings->distance_mm[i];
                tof_distance_set(&state->distance_bits, sensor, incoming);
                state->timestamp_ms[i] = readings->timestamp_ms[i];
                state->valid_mask |= bit;
                state->fault_mask &= (uint8_t)~bit;
            }
            else
            {
                state->valid_mask &= (uint8_t)~bit;
            }
        }

        if (!online || (uint32_t)(now_ms - state->timestamp_ms[i]) > OBSTACLE_STALE_MS)
        {
            state->valid_mask &= (uint8_t)~bit;
            state->fault_mask |= bit;
        }

        if ((state->valid_mask & bit) == 0)
        {
            // Unavailable readings are omitted; they do not request a stop.
            // Clear stale proximity flags so only current usable ranges count.
            // state->distance_mm[i] = 0;
            tof_distance_set(&state->distance_bits, sensor, 0);
            state->valid_mask &= (uint8_t)~bit;
            state->blocked_mask &= (uint8_t)~bit;
            state->urgent_mask &= (uint8_t)~bit;
            continue;
        }

        // const uint16_t distance = state->distance_mm[i];
        const uint16_t distance = tof_distance_get(state->distance_bits, sensor);

        // A fresh no-target result is healthy but has no numerical distance.
        // Clear old proximity flags rather than retaining an earlier close hit.
        if (distance == TOF_NO_TARGET_MM)
        {
            state->blocked_mask &= (uint8_t)~bit;
            state->urgent_mask &= (uint8_t)~bit;
            continue;
        }

        if (distance <= OBSTACLE_BLOCKED_MM)
        {
            state->blocked_mask |= bit;
        }
        else if (distance >= OBSTACLE_BLOCKED_MM + OBSTACLE_HYSTERESIS_MM)
        {
            state->blocked_mask &= (uint8_t)~bit;
        }

        if (distance <= OBSTACLE_URGENT_MM)
        {
            state->urgent_mask |= bit;
        }
        else if (distance >= OBSTACLE_URGENT_MM + OBSTACLE_HYSTERESIS_MM)
        {
            state->urgent_mask &= (uint8_t)~bit;
        }
    }

    // Keep proximity flags for all directions, but only front proximity stops.
    // Missing/invalid readings do not participate in obstacle stopping.
    state->stop_required = (state->urgent_mask & (1u << TOF_FRONT)) != 0;
}
