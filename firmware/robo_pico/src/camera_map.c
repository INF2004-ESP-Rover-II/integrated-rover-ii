/**
 * @file camera_map.c
 * @brief Camera map firmware module.
 */
#include "camera_map.h"
#include <limits.h>
#include <math.h>
#include <string.h>

static const int8_t g_delta_x[4] = { 0, 1, 0, -1 };
static const int8_t g_delta_y[4] = { 1, 0, -1, 0 };
static float
wrap (float first)
{
    first = fmodf(first, 360.0f);
    return first < 0 ? first + 360.0f : first;
}
static float
angle_error (float first, float second)
{
    float error_value = fabsf(wrap(first) - wrap(second));
    return error_value > 180 ? 360 - error_value : error_value;
}
void
camera_map_init (camera_map_t * p_projection)
{
    memset(p_projection, 0, sizeof(*p_projection));
}

static bool
position (const rover_map_t * p_map, uint8_t identifier,
          int16_t * p_x_coordinate, int16_t * p_y_coordinate)
{
    const map_node_t * p_known = rover_map_find_id(p_map, identifier);
    if (p_known)
    {
        *p_x_coordinate = p_known->x;
        *p_y_coordinate = p_known->y;
        return true;
    }
    for (size_t index = 0; index < p_map->observed_count; ++index)
    {
        if (p_map->observed[index].marker_id == identifier)
        {
            *p_x_coordinate = p_map->observed[index].x;
            *p_y_coordinate = p_map->observed[index].y;
            return true;
        }
    }
    return false;
}

typedef struct
{
    const map_node_t *  p_anchor;
    camera_marker_t     sorted[4];
    uint8_t             visible_count;
    uint32_t            slots[4];
    map_observed_node_t candidates[4];
    uint8_t             count;
    _Bool               b_same;
} camera_map_tick_context_t;

/** @brief Execute the prepare stage of camera_map_tick. */
static bool
camera_map_tick_prepare (camera_map_t * p_projection, rover_map_t * p_map,
                         const camera_observation_t * p_observation,
                         bool b_stopped, bool b_heading_valid, float heading,
                         float north, uint32_t now,
                         camera_map_tick_context_t * p_context)
{


    p_context->p_anchor = rover_map_find_id(p_map, p_map->current_node);
    if (!p_context->p_anchor || !p_context->p_anchor->visited || !b_stopped ||
        !b_heading_valid || !isfinite(heading) || !isfinite(north) ||
        heading < 0 || heading >= 360)
    {
        camera_map_init(p_projection);
        return true;
    }
    if (!p_projection->pose_valid ||
        p_projection->anchor_id != p_context->p_anchor->marker_id ||
        angle_error(heading, p_projection->heading_deg) > 3.0f)
    {
        camera_map_init(p_projection);
        p_projection->pose_valid  = true;
        p_projection->anchor_id   = p_context->p_anchor->marker_id;
        p_projection->heading_deg = heading;
        p_projection->pose_ms     = now;
        return true;
    }
    if (!p_observation || p_observation->frame.status != VISION_STATUS_OK ||
        p_observation->frame.count > VISION_MAX_MARKERS ||
        (uint32_t) (now - p_observation->received_ms) > 300u ||
        (uint32_t) (now - p_observation->received_ms) +
                p_observation->frame.age_ms >
            CAMERA_MAP_IMAGE_MAX_AGE_MS)
    {
        return true; // A cached image aging out is not a contradictory new
                     // frame.
    }
    if (p_projection->generation_valid &&
        p_projection->generation == p_observation->generation)
    {
        return true;
    }
    if (p_projection->generation_valid &&
        p_projection->seq == p_observation->frame.seq)
    {
        p_projection->generation = p_observation->generation;
        return true;
    }
    if (p_projection->generation_valid &&
        (uint8_t) (p_observation->frame.seq - p_projection->seq) >= 128u)
    {
        p_projection->confirmations = 0;
    }
    return false;
}

/** @brief Execute the inputs stage of camera_map_tick. */
static bool
camera_map_tick_inputs (camera_map_t *               p_projection,
                        const camera_observation_t * p_observation,
                        camera_map_tick_context_t *  p_context)
{

    p_projection->generation_valid = true;
    p_projection->generation       = p_observation->generation;
    p_projection->seq              = p_observation->frame.seq;
    // Reject images captured before the node/heading settled, including delayed
    // packets from before a turn or arrival. Only distinct images count.
    if ((int32_t) (p_observation->received_ms - p_observation->frame.age_ms -
                   p_projection->pose_ms) < (int32_t) CAMERA_MAP_SETTLE_MS)
    {
        return true;
    }


    p_context->visible_count = 0;
    for (uint8_t index = 0; index < p_observation->frame.count; ++index)
    {
        const camera_marker_t * p_marker = &p_observation->frame.markers[index];
        if (!p_marker->id || p_marker->id == p_context->p_anchor->marker_id ||
            p_marker->quality || !p_marker->range_mm ||
            p_marker->range_mm == UINT16_MAX ||
            p_marker->bearing_cdeg < -4500 || p_marker->bearing_cdeg > 4500)
        {
            continue;
        }
        bool b_duplicate = false;
        for (uint8_t inner_index = 0; inner_index < p_context->visible_count;
             ++inner_index)
        {
            if (p_context->sorted[inner_index].id == p_marker->id)
            {
                b_duplicate = true;
            }
        }
        if (!b_duplicate)
        {
            p_context->sorted[p_context->visible_count++] = *p_marker;
        }
    }
    return false;
}

/** @brief Execute the motion stage of camera_map_tick. */
static bool
camera_map_tick_motion (camera_map_tick_context_t * p_context)
{

    for (uint8_t index = 1; index < p_context->visible_count; ++index)
    {
        camera_marker_t value       = p_context->sorted[index];
        uint8_t         inner_index = index;
        while (inner_index &&
               p_context->sorted[inner_index - 1u].range_mm > value.range_mm)
        {

            p_context->sorted[inner_index] =
                p_context->sorted[inner_index - 1u];

            --inner_index;
        }
        p_context->sorted[inner_index] = value;
    }

    memset(p_context->slots, 0, sizeof(p_context->slots));


    p_context->count = 0;
    return false;
}

/** @brief Execute the camera stage of camera_map_tick. */
static bool
camera_map_tick_camera (camera_map_t * p_projection, rover_map_t * p_map,
                        float heading, float north,
                        camera_map_tick_context_t * p_context)
{

    for (uint8_t index = 0; index < p_context->visible_count; ++index)
    {

        float world =
            wrap(heading - north +
                 (float) p_context->sorted[index].bearing_cdeg / 100.0f);

        unsigned dir = (unsigned) ((world + 45.0f) / 90.0f) % 4u;
        // A diagonal sighting cannot reliably identify a cardinal grid cell.
        if (angle_error(world, 90.0f * (float) dir) > 15.0f)
        {
            continue;
        }
        int16_t x_coordinate = 0;
        int16_t y_coordinate = 0;
        if (position(p_map, p_context->sorted[index].id, &x_coordinate,
                     &y_coordinate))
        {

            int32_t along = (int32_t) (x_coordinate - p_context->p_anchor->x) *
                                g_delta_x[dir] +
                            (int32_t) (y_coordinate - p_context->p_anchor->y) *
                                g_delta_y[dir];


            int32_t across = (int32_t) (x_coordinate - p_context->p_anchor->x) *
                                 g_delta_y[dir] -
                             (int32_t) (y_coordinate - p_context->p_anchor->y) *
                                 g_delta_x[dir];

            if (!across && along > 0 &&
                (uint32_t) along > p_context->slots[dir])
            {
                p_context->slots[dir] = (uint32_t) along;
            }
            continue;
        }
        ++p_context->slots[dir];

        int32_t target_x = (int32_t) p_context->p_anchor->x +
                           (int32_t) p_context->slots[dir] * g_delta_x[dir];


        int32_t target_y = (int32_t) p_context->p_anchor->y +
                           (int32_t) p_context->slots[dir] * g_delta_y[dir];

        if (target_x < INT16_MIN || target_x > INT16_MAX ||
            target_y < INT16_MIN || target_y > INT16_MAX)
        {
            continue;
        }

        p_context->candidates[p_context->count++] =
            (map_observed_node_t) { .marker_id = p_context->sorted[index].id,
                                    .x         = (int16_t) target_x,
                                    .y         = (int16_t) target_y };
    }

    p_context->b_same = p_context->count == p_projection->count;
    return false;
}

/** @brief Execute the tracking stage of camera_map_tick. */
static bool
camera_map_tick_tracking (camera_map_t * p_projection, rover_map_t * p_map,
                          camera_map_tick_context_t * p_context)
{

    for (uint8_t index = 0; p_context->b_same && index < p_context->count;
         ++index)
    {

        p_context->b_same =
            p_context->candidates[index].marker_id ==
                p_projection->candidates[index].marker_id &&
            p_context->candidates[index].x ==
                p_projection->candidates[index].x &&
            p_context->candidates[index].y == p_projection->candidates[index].y;
    }
    if (!p_context->b_same)
    {
        memcpy(p_projection->candidates, p_context->candidates,
               (size_t) p_context->count * sizeof(p_context->candidates[0]));
        p_projection->count         = p_context->count;
        p_projection->confirmations = 1;
    }
    else if (p_projection->confirmations < CAMERA_MAP_CONFIRM_FRAMES)
    {
        ++p_projection->confirmations;
    }
    if (p_projection->confirmations >= CAMERA_MAP_CONFIRM_FRAMES)
    {
        for (uint8_t index = 0; index < p_context->count; ++index)
        {
            rover_map_observe(p_map, p_context->candidates[index].marker_id,
                              p_context->candidates[index].x,
                              p_context->candidates[index].y);
        }
    }
    return false;
}

void
camera_map_tick (camera_map_t * p_projection, rover_map_t * p_map,
                 const camera_observation_t * p_observation, bool b_stopped,
                 bool b_heading_valid, float heading, float north, uint32_t now)
{
    camera_map_tick_context_t context = { 0 };
    if (camera_map_tick_prepare(p_projection, p_map, p_observation, b_stopped,
                                b_heading_valid, heading, north, now, &context))
    {
        return;
    }
    if (camera_map_tick_inputs(p_projection, p_observation, &context))
    {
        return;
    }
    if (camera_map_tick_motion(&context))
    {
        return;
    }
    if (camera_map_tick_camera(p_projection, p_map, heading, north, &context))
    {
        return;
    }
    if (camera_map_tick_tracking(p_projection, p_map, &context))
    {
        return;
    }
}

/* End of file. */
