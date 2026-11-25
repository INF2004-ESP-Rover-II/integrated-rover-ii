/**
 * @file obstacle.c
 * @brief Obstacle firmware module.
 */
#include "obstacle.h"
#include "sensor_config.h"
#include <string.h>

#if OBSTACLE_URGENT_MM == 0 || OBSTACLE_URGENT_MM >= OBSTACLE_BLOCKED_MM
#error "Require 0 < OBSTACLE_URGENT_MM < OBSTACLE_BLOCKED_MM"
#endif
#if OBSTACLE_BLOCKED_MM + OBSTACLE_HYSTERESIS_MM > TOF_MAX_DISTANCE_MM
#error "Obstacle thresholds exceed the supported ToF range"
#endif

void
obstacle_init (obstacle_state_t * p_state, uint32_t now_ms)
{
    memset(p_state, 0, sizeof(*p_state));
    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
    {
        // Reuse the per-sensor clocks for startup grace; no extra RAM is
        // needed.
        p_state->timestamp_ms[index] = now_ms;
    }
}

void
obstacle_update (obstacle_state_t * p_state, const tof_readings_t * p_readings,
                 uint8_t online_mask, uint32_t now_ms)
{
    for (int index = 0; index < TOF_SENSOR_COUNT; ++index)
    {
        const uint8_t      bit    = (uint8_t) (1u << (uint32_t) (index));
        const tof_sensor_t sensor = (tof_sensor_t) index;

        const uint16_t incoming =
            tof_distance_get(p_readings->distance_bits, sensor);

        const bool b_online =
            ((uint32_t) (online_mask) & (uint32_t) (bit)) != 0;

        const bool b_accepted =
            (incoming > 0 && incoming <= TOF_MAX_DISTANCE_MM) ||
            incoming == TOF_NO_TARGET_MM;

        // A completed result proves the device is responding, even when its
        // range status rejects the distance (e.g. weak return with no target).
        // Range validity is separate from sensor/communication health.
        // No fresh result means retain the cached range until its timeout.
        if ((uint32_t) (p_readings->fresh_mask) & (uint32_t) (bit))
        {
            if (b_online)
            {
                p_state->timestamp_ms[index] = p_readings->timestamp_ms[index];
                p_state->fault_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            }
            if (b_online &&
                ((uint32_t) (p_readings->valid_mask) & (uint32_t) (bit)) &&
                b_accepted)
            {
                // state->distance_mm[i] = readings->distance_mm[i];
                tof_distance_set(&p_state->distance_bits, sensor, incoming);
                p_state->valid_mask |= (uint32_t) (bit);
            }
            else
            {
                p_state->valid_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            }
        }

        if (!b_online || (uint32_t) (now_ms - p_state->timestamp_ms[index]) >
                             OBSTACLE_STALE_MS)
        {
            p_state->valid_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            p_state->fault_mask |= (uint32_t) (bit);
        }

        if (((uint32_t) (p_state->valid_mask) & (uint32_t) (bit)) == 0)
        {
            // Unavailable readings are omitted; they do not request a stop.
            // Clear stale proximity flags so only current usable ranges count.
            // state->distance_mm[i] = 0;
            tof_distance_set(&p_state->distance_bits, sensor, 0);
            p_state->valid_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            p_state->blocked_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            p_state->urgent_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            continue;
        }

        // const uint16_t distance = state->distance_mm[i];

        const uint16_t distance =
            tof_distance_get(p_state->distance_bits, sensor);

        // A successful beyond-1-m result is healthy but has no numerical
        // distance. Clear old proximity flags rather than retaining an earlier
        // close hit.
        if (distance == TOF_NO_TARGET_MM)
        {
            p_state->blocked_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            p_state->urgent_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
            continue;
        }

        if (distance <= OBSTACLE_BLOCKED_MM)
        {
            p_state->blocked_mask |= (uint32_t) (bit);
        }
        else if (distance >= OBSTACLE_BLOCKED_MM + OBSTACLE_HYSTERESIS_MM)
        {
            p_state->blocked_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
        }

        if (distance <= OBSTACLE_URGENT_MM)
        {
            p_state->urgent_mask |= (uint32_t) (bit);
        }
        else if (distance >= OBSTACLE_URGENT_MM + OBSTACLE_HYSTERESIS_MM)
        {
            p_state->urgent_mask &= (uint32_t) ((uint8_t) ~(uint32_t) (bit));
        }
    }

    // Keep proximity flags for all directions, but only front proximity stops.
    // Missing/invalid readings do not participate in obstacle stopping.

    p_state->stop_required = front_obstacle_detected(
        tof_distance_get(p_state->distance_bits, TOF_FRONT),
        (p_state->fault_mask & (1u << (uint32_t) (TOF_FRONT))) != 0);
}

/* End of file. */
