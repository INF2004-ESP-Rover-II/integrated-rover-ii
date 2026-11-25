/**
 * @file camera_map.h
 * @brief Camera map firmware module.
 */
#ifndef ROVER_CAMERA_MAP_H
#define ROVER_CAMERA_MAP_H
#include "camera_uart.h"
#include "rover_map.h"

#define CAMERA_MAP_CONFIRM_FRAMES 3u
#define CAMERA_MAP_SETTLE_MS 300u
#define CAMERA_MAP_SENSOR_MAX_AGE_MS 600u
#define CAMERA_MAP_IMAGE_MAX_AGE_MS 1500u

typedef struct
{
    bool                pose_valid;
    bool                generation_valid;
    uint8_t             anchor_id;
    uint8_t             confirmations;
    uint8_t             count;
    uint8_t             seq;
    float               heading_deg;
    uint32_t            pose_ms;
    uint32_t            generation;
    map_observed_node_t candidates[VISION_MAX_MARKERS];
} camera_map_t;


void camera_map_init (camera_map_t * p_mapper);

// Raw heading and north use the same fixed IMU frame as the onboard leg
// executor. Projection is display-only; never changes current_node, visited or
// exits.

void camera_map_tick (camera_map_t * p_mapper, rover_map_t * p_map,
                      const camera_observation_t * p_observation,
                      bool b_stopped, bool b_heading_valid, float heading_deg,
                      float north_deg, uint32_t now);

#endif

/* End of file. */
