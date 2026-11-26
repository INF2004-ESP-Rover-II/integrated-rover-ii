/**
 * @file target_selector.c
 * @brief Target selector firmware module.
 */
#include "target_selector.h"
#include <inttypes.h>

#include <stdio.h>

void
target_selector_init (target_selector_t * p_selection)
{

    *p_selection = (target_selector_t) {
        .state = TARGET_NONE, .detail = "NEED_STOPPED_CONFIRMED_NODE"};
}

void
target_selector_set_route (target_selector_t * p_selection,
                           target_route_t      route)
{
    target_selector_init(p_selection);
    p_selection->route = route;
}

static bool
context_ok (const mission_t * p_mission, const rover_map_t * p_map, bool b_safe)
{
    const map_node_t * p_node = rover_map_find_id(p_map, p_map->current_node);
    return b_safe && !p_mission->test_mode &&
           (p_mission->state == MISSION_IDLE ||
            p_mission->state == MISSION_READY ||
            (p_mission->dfs_active && p_mission->state == MISSION_EXPLORING)) &&
           p_node && p_node->visited;
}

static bool
image_fresh (const camera_observation_t * p_observation, uint32_t now)
{
    uint32_t local_age = now - p_observation->received_ms;
    return p_observation->frame.status == VISION_STATUS_OK &&
           p_observation->frame.count <= VISION_MAX_MARKERS &&
           local_age <= TARGET_UART_MAX_AGE_MS &&
           local_age + p_observation->frame.age_ms <= TARGET_CAMERA_MAX_AGE_MS;
}

static bool
marker_usable (const camera_marker_t * p_marker, int max_bearing)
{
    return p_marker->id && p_marker->quality == 0 && p_marker->range_mm &&
           p_marker->range_mm != UINT16_MAX &&
           p_marker->bearing_cdeg >= -max_bearing &&
           p_marker->bearing_cdeg <= max_bearing;
}

static leg_direction_t
adjacent_direction (const map_node_t * p_from, const map_node_t * p_node)
{
    int32_t x_coordinate = (int32_t) p_node->x - p_from->x;
    int32_t y_coordinate = (int32_t) p_node->y - p_from->y;
    if (x_coordinate == 0 && y_coordinate == 1)
    {
        return LEG_NORTH;
    }
    if (x_coordinate == 1 && y_coordinate == 0)
    {
        return LEG_EAST;
    }
    if (x_coordinate == 0 && y_coordinate == -1)
    {
        return LEG_SOUTH;
    }
    if (x_coordinate == -1 && y_coordinate == 0)
    {
        return LEG_WEST;
    }
    return LEG_DIRECTION_UNKNOWN;
}

static bool
target_compatible (const rover_map_t * p_map, const target_route_t * p_route,
                   uint8_t identifier)
{
    if (identifier == p_map->current_node)
    {
        return false;
    }
    const map_node_t * p_target = rover_map_find_id(p_map, identifier);
    if (p_route->enabled)
    {
        if (p_route->expected_id && identifier != p_route->expected_id)
        {
            return false;
        }

        const map_node_t * p_occupant =
            rover_map_find_cell(p_map, p_route->x, p_route->y);

        if ((p_occupant && p_occupant->marker_id != identifier) ||
            (p_target &&
             (p_target->x != p_route->x || p_target->y != p_route->y)))
        {
            return false;
        }
    }
    if (!p_target)
    {
        return true; // Provisional identity, never an inferred map cell.
    }
    const map_node_t * p_from = rover_map_find_id(p_map, p_map->current_node);
    leg_direction_t    data   = adjacent_direction(p_from, p_target);
    return data != LEG_DIRECTION_UNKNOWN &&
           p_from->exits[data] != MAP_EXIT_BLOCKED &&
           p_target->exits[((unsigned) data + 2u) % 4u] != MAP_EXIT_BLOCKED;
}

static bool
reference_compatible (const rover_map_t * p_map, uint8_t departure,
                      uint8_t target_id, uint8_t ref_id)
{
    const map_node_t * p_target = rover_map_find_id(p_map, target_id);
    const map_node_t * p_ref    = rover_map_find_id(p_map, ref_id);
    if (!p_target || !p_ref)
    {
        return true; // Optical candidate only; not validated layout geometry.
    }
    const map_node_t * p_from = rover_map_find_id(p_map, departure);
    if (!p_from)
    {
        return false;
    }
    int32_t delta_x    = (int32_t) p_target->x - p_from->x;
    int32_t delta_y    = (int32_t) p_target->y - p_from->y;
    int32_t relative_x = (int32_t) p_ref->x - p_target->x;
    int32_t relative_y = (int32_t) p_ref->y - p_target->y;
    return relative_x * delta_y == relative_y * delta_x &&
           relative_x * delta_x + relative_y * delta_y > 0;
}

static const camera_marker_t *
nearest_target (const camera_frame_t * p_frame, const rover_map_t * p_map,
                const target_route_t * p_route, int max_bearing)
{
    const camera_marker_t * p_best = NULL;
    bool                    b_tied = false;
    for (uint8_t index = 0; index < p_frame->count; ++index)
    {
        const camera_marker_t * p_marker = &p_frame->markers[index];
        if (!marker_usable(p_marker, max_bearing) ||
            !target_compatible(p_map, p_route, p_marker->id))
        {
            continue;
        }
        if (!p_best || p_marker->range_mm < p_best->range_mm)
        {
            p_best = p_marker;
            b_tied = false;
        }
        else if (p_marker->range_mm == p_best->range_mm &&
                 p_marker->id != p_best->id)
        {
            b_tied = true;
        }
    }
    return b_tied ? NULL : p_best;
}

bool
target_selector_has_acquisition_candidate (const target_selector_t * p_selector,
                                           const rover_map_t *       p_map,
                                           const camera_frame_t *    p_frame)
{
    return p_selector && p_map &&
           rover_map_find_id(p_map, p_map->current_node) && p_frame &&
           p_frame->status == VISION_STATUS_OK &&
           p_frame->count <= VISION_MAX_MARKERS &&
           nearest_target(p_frame, p_map, &p_selector->route,
                          TARGET_ACQUISITION_MAX_BEARING_CDEG) != NULL;
}

static const camera_marker_t *
find_tracking_marker (const camera_frame_t * p_frame, uint8_t identifier,
                      int max_bearing)
{
    for (uint8_t index = 0; index < p_frame->count; ++index)
    {
        const camera_marker_t * p_marker = &p_frame->markers[index];
        if (p_marker->id == identifier && p_marker->quality == 0 &&
            p_marker->range_mm && p_marker->range_mm != UINT16_MAX &&
            p_marker->bearing_cdeg >= -max_bearing &&
            p_marker->bearing_cdeg <= max_bearing)
        {
            return p_marker;
        }
    }
    return NULL;
}

static const camera_marker_t *
nearest_reference (const camera_frame_t * p_frame, const rover_map_t * p_map,
                   const travel_leg_t * p_leg, uint8_t target_id,
                   const camera_marker_t * p_target, bool b_late)
{
    uint8_t departure = p_leg->departure_node;
    // Establish optical ordering from the SAME fresh image, or use already
    // known map coordinates. Never compare a new image to an old lens range.
    if (!p_target && !rover_map_find_id(p_map, target_id) && !b_late)
    {
        return NULL;
    }
    const camera_marker_t * p_best = NULL;
    bool                    b_tied = false;
    for (uint8_t index = 0; index < p_frame->count; ++index)
    {
        const camera_marker_t * p_marker = &p_frame->markers[index];

        int delta = p_target
                        ? (int) p_marker->bearing_cdeg - p_target->bearing_cdeg
                        : 0;

        if (!marker_usable(p_marker, TARGET_MAX_BEARING_CDEG) ||
            p_marker->id == target_id || p_marker->id == departure ||
            (!p_target && !b_late && !rover_map_find_id(p_map, p_marker->id)) ||
            (p_target &&
             (uint32_t) p_marker->range_mm <
                 (uint32_t) p_target->range_mm + TARGET_REFERENCE_MIN_GAP_MM) ||
            delta < -TARGET_REFERENCE_MAX_DELTA_CDEG ||
            delta > TARGET_REFERENCE_MAX_DELTA_CDEG ||
            !reference_compatible(p_map, departure, target_id, p_marker->id))
        {
            continue;
        }
        const map_node_t * p_known = rover_map_find_id(p_map, p_marker->id);
        if (b_late && !p_target && p_known && p_leg->target_cell_valid &&
            p_leg->direction != LEG_DIRECTION_UNKNOWN)
        {
            static const int8_t delta_x[] = {0, 1, 0, -1};
            static const int8_t delta_y[] = {1, 0, -1, 0};
            int32_t relative_x = (int32_t) p_known->x - p_leg->target_x;
            int32_t relative_y = (int32_t) p_known->y - p_leg->target_y;
            if (relative_x * delta_y[p_leg->direction] !=
                    relative_y * delta_x[p_leg->direction] ||
                relative_x * delta_x[p_leg->direction] +
                        relative_y * delta_y[p_leg->direction] <=
                    0)
            {
                continue;
            }
        }
        if (!p_best || p_marker->range_mm < p_best->range_mm)
        {
            p_best = p_marker;
            b_tied = false;
        }
        else if (p_marker->range_mm == p_best->range_mm &&
                 p_marker->id != p_best->id)
        {
            b_tied = true;
        }
    }
    return b_tied ? NULL : p_best;
}

static void
reset_confirmations (target_selector_t * p_selection)
{
    if (p_selection->state != TARGET_LOCKED)
    {
        p_selection->marker = p_selection->confirmations = 0;
    }
    if (p_selection->state != TARGET_LOCKED)
    {
        p_selection->reference_marker = 0;
    }
    if (!p_selection->reference_marker)
    {

        p_selection->reference_candidate =
            p_selection->reference_confirmations = 0;
    }
    p_selection->available = p_selection->reference_available = false;
}

static void
arm (target_selector_t * p_selection, mission_t * p_mission,
     const rover_map_t * p_map, const camera_observation_t * p_observation,
     uint32_t now)
{
    uint32_t       leg_id      = p_mission->leg.leg_id + 1u;
    target_route_t route       = p_selection->route;
    bool           b_acquiring = p_selection->b_acquiring;

    *p_selection = (target_selector_t) {
        .state                  = TARGET_WAITING,
        .detail                 = "AUTOMATIC_WAITING_FOR_IMAGES",
        .route                  = route,
        .b_acquiring            = b_acquiring,
        .leg_id                 = leg_id,
        .started_ms             = now,
        .map_revision           = p_map->revision,
        .last_generation        = p_observation ? p_observation->generation : 0,
        .last_seq               = p_observation ? p_observation->frame.seq : 0,
        .have_seq               = p_observation != NULL,
        .last_image_received_ms = now};

    leg_direction_t direction =
        route.enabled ? route.direction : LEG_DIRECTION_UNKNOWN;

    p_mission->leg = (travel_leg_t) {.leg_id            = leg_id,
                                     .departure_node    = p_map->current_node,
                                     .direction         = direction,
                                     .phase             = LEG_SELECTING,
                                     .target_cell_valid = route.enabled,
                                     .target_x          = route.x,
                                     .target_y          = route.y};

    p_mission->revision++;
    p_mission->changed_ms = now;
}

static void
note_change (mission_t * p_mission, uint32_t now)
{
    p_mission->revision++;
    p_mission->changed_ms = now;
}

typedef struct
{
    _Bool                   b_approaching;
    uint8_t                 delta;
    _Bool                   b_discontinuity;
    uint32_t                image_age;
    const camera_marker_t * p_target;
} target_selector_tick_context_t;

/** @brief Execute the prepare stage of target_selector_tick. */
static bool
target_selector_tick_prepare (target_selector_t * p_selection,
                              mission_t * p_mission, const rover_map_t * p_map,
                              bool b_safe, uint32_t now,
                              target_selector_tick_context_t * p_context)
{

    if (!p_mission->test_mode && !p_mission->dfs_active &&
        p_mission->leg.phase == LEG_COMPLETE)
    {
        p_selection->available = p_selection->reference_available = false;
        p_selection->detail = "TARGET_ARRIVAL_CONFIRMED";
        return true;
    }

    p_context->b_approaching =
        !p_mission->test_mode && p_mission->state == MISSION_EXPLORING &&
        p_mission->leg.configured &&
        (p_mission->leg.phase == LEG_APPROACHING ||
         p_mission->leg.phase == LEG_HOLDING ||
         p_mission->leg.phase == LEG_BRAKING ||
         p_mission->leg.phase == LEG_VERIFYING ||
         p_mission->leg.phase == LEG_BACKING_UP ||
         p_mission->leg.phase == LEG_REACQUIRING ||
         p_mission->leg.phase == LEG_SEARCHING_REFERENCE ||
         p_mission->leg.phase == LEG_OBSTACLE_VERIFYING ||
         p_mission->leg.phase == LEG_ALIGNING ||
         p_mission->leg.phase == LEG_WAITING_CAMERA ||
         p_mission->leg.phase == LEG_WAITING_INPUT ||
         p_mission->leg.phase == LEG_CORRECTING) &&
        p_mission->leg.leg_id == p_selection->leg_id &&
        p_selection->state == TARGET_LOCKED;

    if (!p_context->b_approaching && !context_ok(p_mission, p_map, b_safe))
    {
        if (p_selection->state == TARGET_WAITING ||
            p_selection->state == TARGET_LOCKED)
        {
            if (p_mission->leg.leg_id == p_selection->leg_id)
            {
                mission_clear_leg(p_mission, now);
            }
            p_selection->state = TARGET_CANCELLED;
        }
        reset_confirmations(p_selection);
        p_selection->detail = "NEED_STOPPED_CONFIRMED_NODE";
        return true;
    }
    return false;
}

/** @brief Execute the inputs stage of target_selector_tick. */
static bool
target_selector_tick_inputs (target_selector_t * p_selection,
                             mission_t * p_mission, const rover_map_t * p_map,
                             const camera_observation_t *     p_observation,
                             uint32_t                         now,
                             target_selector_tick_context_t * p_context)
{

    if (!p_context->b_approaching &&
        (p_selection->state == TARGET_WAITING ||
         p_selection->state == TARGET_LOCKED) &&
        (p_selection->map_revision != p_map->revision ||
         p_mission->leg.departure_node != p_map->current_node))
    {
        mission_clear_leg(p_mission, now);
        p_selection->state = TARGET_CANCELLED;
    }
    if (p_mission->leg.phase == LEG_NONE)
    {
        if (p_mission->leg.leg_id == UINT32_MAX)
        {
            p_selection->state = TARGET_CANCELLED;
            reset_confirmations(p_selection);
            p_selection->detail = "LEG_ID_EXHAUSTED";
            return true;
        }
        arm(p_selection, p_mission, p_map, p_observation, now);
        return true;
    }
    if (p_mission->leg.leg_id != p_selection->leg_id ||
        (p_mission->leg.phase != LEG_SELECTING &&
         p_mission->leg.phase != LEG_CONFIGURED && !p_context->b_approaching))
    {
        return true;
    }
    return false;
}

/** @brief Execute the motion stage of target_selector_tick. */
static bool
target_selector_tick_motion (target_selector_t *              p_selection,
                             const camera_observation_t *     p_observation,
                             uint32_t                         now,
                             target_selector_tick_context_t * p_context)
{

    if (!p_observation || !image_fresh(p_observation, now))
    {
        // A cached packet aging out between successive frames clears
        // visibility, not evidence from distinct good images. Explicit
        // bad/missing detections, sequence changes and a 1500 ms source gap
        // still reset confirmations.
        p_selection->available = p_selection->reference_available = false;
        if (!p_observation || p_observation->frame.status != VISION_STATUS_OK ||
            now - p_selection->last_image_received_ms >
                TARGET_CAMERA_MAX_AGE_MS ||
            (uint32_t) (now - p_observation->received_ms) +
                    p_observation->frame.age_ms >
                TARGET_CAMERA_MAX_AGE_MS)
        {
            reset_confirmations(p_selection);
        }
        p_selection->detail = "CAMERA_UNAVAILABLE_OR_STALE";
        return true;
    }
    if (now - p_selection->last_image_received_ms > TARGET_CAMERA_MAX_AGE_MS)
    {
        reset_confirmations(p_selection);
    }
    if (p_observation->generation == p_selection->last_generation)
    {
        return true;
    }
    p_selection->last_generation = p_observation->generation;

    p_context->delta =
        (uint8_t) (p_observation->frame.seq - p_selection->last_seq);

    p_context->b_discontinuity =
        p_selection->have_seq &&
        (p_context->delta >= 128u ||
         now - p_selection->last_image_received_ms > TARGET_CAMERA_MAX_AGE_MS);

    if (p_selection->have_seq && !p_context->delta)
    {
        return true;
    }
    p_selection->last_seq               = p_observation->frame.seq;
    p_selection->have_seq               = true;
    p_selection->last_image_received_ms = now;

    p_context->image_age = (uint32_t) p_observation->frame.age_ms +
                           (now - p_observation->received_ms);

    return false;
}

/** @brief Execute the camera stage of target_selector_tick. */
static bool
target_selector_tick_camera (target_selector_t * p_selection,
                             mission_t * p_mission, const rover_map_t * p_map,
                             const camera_observation_t *     p_observation,
                             uint32_t                         now,
                             target_selector_tick_context_t * p_context)
{

    if (p_context->b_discontinuity ||
        now - p_selection->started_ms < p_context->image_age)
    {
        reset_confirmations(p_selection);
        p_selection->detail = "WAITING_FOR_NEW_IMAGES";
        return true;
    }
    p_selection->sample_generation  = p_observation->generation;
    p_selection->sample_received_ms = p_observation->received_ms;
    p_selection->sample_age_ms      = p_observation->frame.age_ms;

    int tracking_bearing = p_mission->leg.phase == LEG_ALIGNING ||
                                   p_mission->leg.phase == LEG_REACQUIRING ||
                                   (p_selection->b_acquiring &&
                                    p_mission->leg.phase == LEG_CONFIGURED)
                               ? TARGET_RECOVERY_MAX_BEARING_CDEG
                               : TARGET_TRACKING_MAX_BEARING_CDEG;
    p_context->p_target =
        p_selection->state == TARGET_LOCKED
            ? find_tracking_marker(&p_observation->frame,
                                   p_mission->leg.next_target_node,
                                   tracking_bearing)
            : nearest_target(&p_observation->frame, p_map, &p_selection->route,
                             p_selection->b_acquiring
                                 ? TARGET_ACQUISITION_MAX_BEARING_CDEG
                                 : TARGET_MAX_BEARING_CDEG);

    return false;
}

/** @brief Execute the tracking stage of target_selector_tick. */
static bool
target_selector_tick_tracking (target_selector_t * p_selection,
                               mission_t * p_mission, const rover_map_t * p_map,
                               uint32_t                         now,
                               target_selector_tick_context_t * p_context)
{

    if (p_selection->state != TARGET_LOCKED)
    {
        if (!p_context->p_target)
        {
            reset_confirmations(p_selection);
            p_selection->detail = "NO_UNAMBIGUOUS_ELIGIBLE_TARGET";
            return true;
        }
        if (p_selection->marker != p_context->p_target->id)
        {

            p_selection->reference_candidate =
                p_selection->reference_confirmations = 0;
        }

        p_selection->confirmations =
            p_selection->marker == p_context->p_target->id
                ? (uint8_t) (p_selection->confirmations + 1u)
                : 1u;

        p_selection->marker = p_context->p_target->id;
        p_selection->detail = "AUTOMATIC_CONFIRMING_TARGET";
        if (p_selection->confirmations >= TARGET_CONFIRM_FRAMES)
        {
            p_mission->leg.next_target_node = p_context->p_target->id;
            p_mission->leg.configured       = true;
            p_mission->leg.phase            = LEG_CONFIGURED;

            const map_node_t * p_known =
                rover_map_find_id(p_map, p_context->p_target->id);

            if (p_known)
            {
                p_mission->leg.target_x          = p_known->x;
                p_mission->leg.target_y          = p_known->y;
                p_mission->leg.target_cell_valid = true;

                p_mission->leg.direction = adjacent_direction(
                    rover_map_find_id(p_map, p_map->current_node), p_known);
            }
            p_selection->state = TARGET_LOCKED;
            note_change(p_mission, now);
        }
    }
    p_selection->available = p_context->p_target != NULL;
    if (p_context->p_target)
    {
        p_selection->range_mm     = p_context->p_target->range_mm;
        p_selection->bearing_cdeg = p_context->p_target->bearing_cdeg;
    }
    return false;
}

/** @brief Execute the reference stage of target_selector_tick. */
static bool
target_selector_tick_reference (target_selector_t *              p_selection,
                                mission_t *                      p_mission,
                                const rover_map_t *              p_map,
                                const camera_observation_t *     p_observation,
                                uint32_t                         now,
                                target_selector_tick_context_t * p_context)
{

    if (p_mission->leg.arrival_reference_node)
    {
        // Reference lock is independent of visibility and never follows a new
        // nearest marker.

        const camera_marker_t * p_ref = find_tracking_marker(
            &p_observation->frame, p_mission->leg.arrival_reference_node,
            TARGET_RECOVERY_MAX_BEARING_CDEG);

        p_selection->reference_available = p_ref != NULL;
        if (p_ref)
        {
            p_selection->reference_range_mm     = p_ref->range_mm;
            p_selection->reference_bearing_cdeg = p_ref->bearing_cdeg;
        }
    }
    else
    {
        // Do not establish a new reference association while rotating the
        // camera.

        const camera_marker_t * p_ref =
            p_mission->leg.phase == LEG_ALIGNING
                ? NULL
                : nearest_reference(&p_observation->frame, p_map,
                                    &p_mission->leg, p_selection->marker,
                                    p_context->p_target,
                                    p_selection->allow_late_reference);

        p_selection->reference_available = false;
        if (!p_ref)
        {

            p_selection->reference_candidate =
                p_selection->reference_confirmations = 0;
        }
        else
        {

            p_selection->reference_confirmations =
                p_selection->reference_candidate == p_ref->id
                    ? (uint8_t) (p_selection->reference_confirmations + 1u)
                    : 1u;

            p_selection->reference_candidate    = p_ref->id;
            p_selection->reference_range_mm     = p_ref->range_mm;
            p_selection->reference_bearing_cdeg = p_ref->bearing_cdeg;
            if (p_selection->state == TARGET_LOCKED &&
                p_selection->reference_confirmations >= TARGET_CONFIRM_FRAMES)
            {
                p_mission->leg.arrival_reference_node = p_ref->id;
                p_selection->reference_marker         = p_ref->id;
                p_selection->reference_available      = true;
                note_change(p_mission, now);
            }
        }
    }
    return false;
}

/** @brief Execute the arrival stage of target_selector_tick. */
static bool
target_selector_tick_arrival (target_selector_t * p_selection,
                              mission_t *         p_mission)
{

    if (p_selection->state == TARGET_LOCKED)
    {

        p_selection->detail = !p_selection->available
                                  ? "TARGET_OBSERVATION_UNAVAILABLE"
                              : !p_mission->leg.arrival_reference_node
                                  ? "TARGET_LOCKED_WAITING_FOR_REFERENCE"
                              : !p_selection->reference_available
                                  ? "REFERENCE_OBSERVATION_UNAVAILABLE"
                                  : "TARGET_AND_REFERENCE_LOCKED";
    }
    return false;
}

void
target_selector_tick (target_selector_t * p_selection, mission_t * p_mission,
                      const rover_map_t *          p_map,
                      const camera_observation_t * p_observation, bool b_safe,
                      uint32_t now)
{
    target_selector_tick_context_t context = {0};
    if (target_selector_tick_prepare(p_selection, p_mission, p_map, b_safe, now,
                                     &context))
    {
        return;
    }
    if (target_selector_tick_inputs(p_selection, p_mission, p_map,
                                    p_observation, now, &context))
    {
        return;
    }
    if (target_selector_tick_motion(p_selection, p_observation, now, &context))
    {
        return;
    }
    if (target_selector_tick_camera(p_selection, p_mission, p_map,
                                    p_observation, now, &context))
    {
        return;
    }
    if (target_selector_tick_tracking(p_selection, p_mission, p_map, now,
                                      &context))
    {
        return;
    }
    if (target_selector_tick_reference(p_selection, p_mission, p_map,
                                       p_observation, now, &context))
    {
        return;
    }
    if (target_selector_tick_arrival(p_selection, p_mission))
    {
        return;
    }
}

size_t
target_selector_format (const target_selector_t * p_selection, char * p_out,
                        size_t capacity)
{
    if (!p_out || !capacity)
    {
        return 0;
    }
    static const char * states[] = {"NONE", "WAITING", "LOCKED", "CANCELLED"};
    char                marker[8];
    char                ref[8];
    if (p_selection->marker)
    {
        snprintf(marker, sizeof(marker), "%u", (unsigned) p_selection->marker);
    }
    else
    {
        snprintf(marker, sizeof(marker), "null");
    }

    uint8_t reference = p_selection->reference_marker
                            ? p_selection->reference_marker
                            : p_selection->reference_candidate;

    if (reference)
    {
        snprintf(ref, sizeof(ref), "%u", (unsigned) reference);
    }
    else
    {
        snprintf(ref, sizeof(ref), "null");
    }

    int length = snprintf(
        p_out, capacity,
        "{\"state\":\"%s\",\"request\":\"\",\"leg_id\":%" PRIu32
        ",\"role\":\"AUTOMATIC\",\"direction\":null,"
        "\"marker\":%s,\"confirmations\":%u,\"available\":%s,\"range_mm\":%u,"
        "\"bearing_cdeg\":%d,"
        "\"reference_marker\":%s,\"reference_confirmations\":%u,\"reference_"
        "available\":%s,"
        "\"reference_range_mm\":%u,\"reference_bearing_cdeg\":%d,\"detail\":\"%"
        "s\"}",
        states[p_selection->state], (uint32_t) p_selection->leg_id, marker,
        (unsigned) p_selection->confirmations,
        p_selection->available ? "true" : "false",
        (unsigned) p_selection->range_mm, (int) p_selection->bearing_cdeg, ref,
        (unsigned) p_selection->reference_confirmations,
        p_selection->reference_available ? "true" : "false",
        (unsigned) p_selection->reference_range_mm,
        (int) p_selection->reference_bearing_cdeg, p_selection->detail);

    if (length < 0 || (size_t) length >= capacity)
    {
        p_out[0] = '\0';
        return 0;
    }
    return (size_t) length;
}

/* End of file. */
