/**
 * @file onboard_drive.c
 * @brief Onboard drive firmware module.
 */
#include "onboard_drive.h"
#include "../../common/obstacle_threshold.h"
#include "motor_config.h"
#include "motor_driver.h"
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

_Static_assert(ONBOARD_DRIVE_ALIGNMENT_STEP_CDEG <=
                   MC_ROT_SLOW_ZONE_TENTHS * 10,
               "Alignment steps must use the existing slow rotation zone");

void
onboard_drive_init (onboard_drive_t * p_drive)
{

    *p_drive = (onboard_drive_t) {.result       = "NOT_STARTED",
                                  .input_reason = "NONE",
                                  .reference_arrival_mm =
                                      ONBOARD_DRIVE_REFERENCE_ARRIVAL_MM};
}

static const char *
motion_problem (const onboard_drive_inputs_t * p_inputs, uint32_t now)
{
    if (p_inputs->safety_latched)
    {
        return "SAFETY_LOCKED";
    }
    if (!p_inputs->link_ok)
    {
        return "LINK_LOST";
    }
    if (!p_inputs->sensor_valid ||
        (uint32_t) (now - p_inputs->sensor_received_ms) >
            ONBOARD_DRIVE_SENSOR_MAX_AGE_MS)
    {
        return "SENSOR_STALE";
    }
    if (!p_inputs->imu_ok || !isfinite(p_inputs->heading_deg) ||
        p_inputs->heading_deg < 0 || p_inputs->heading_deg >= 360)
    {
        return "HEADING_UNAVAILABLE";
    }
    // The Sensor Pico fault bit tracks failed initialization/result timeouts.
    // A fresh zero with fault clear is a responding sensor with no usable
    // distance (e.g. no return). It is not an obstacle or a proven clear exit.
    // Permit this bounded visual approach; never turn it into a map/arrival
    // fact.
    if (p_inputs->front_fault)
    {
        return "FRONT_SENSOR_FAULT";
    }
    if (p_inputs->front_urgent ||
        front_obstacle_detected(p_inputs->front_mm, false))
    {
        return "FRONT_BLOCKED";
    }
    return NULL;
}

static const char *
input_problem (const target_selector_t *      p_selection,
               const onboard_drive_inputs_t * p_inputs, uint32_t now)
{
    const char * p_problem = motion_problem(p_inputs, now);
    if (p_problem)
    {
        return p_problem;
    }
    if (!p_inputs->camera_present)
    {
        return "CAMERA_UART_MISSING";
    }
    if (p_inputs->camera_status != VISION_STATUS_OK)
    {
        return p_inputs->camera_status == VISION_STATUS_NO_MARKER
                   ? "CAMERA_NO_MARKERS"
               : p_inputs->camera_status == VISION_STATUS_STALLED
                   ? "CAMERA_PROCESSING_STALLED"
                   : "CAMERA_LOST";
    }
    if (p_inputs->camera_packet_age_ms > TARGET_UART_MAX_AGE_MS)
    {
        return "CAMERA_PACKET_STALE";
    }
    if (p_inputs->camera_image_age_ms > ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
    {
        return "CAMERA_IMAGE_STALE";
    }
    if (!p_inputs->camera_fresh)
    {
        return "CAMERA_DATA_UNUSABLE";
    }
    if (!p_selection->sample_generation)
    {
        return "CAMERA_IMAGE_UNAVAILABLE";
    }
    if ((uint32_t) (now - p_selection->sample_received_ms) >
        TARGET_UART_MAX_AGE_MS)
    {
        return "CAMERA_ACCEPTED_IMAGE_STALE";
    }
    if ((uint32_t) (now - p_selection->sample_received_ms) +
            p_selection->sample_age_ms >
        ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
    {
        return "CAMERA_IMAGE_STALE";
    }
    return NULL;
}

static void
note_inputs (onboard_drive_t * p_drive, const target_selector_t * p_selection,
             const onboard_drive_inputs_t * p_inputs, uint32_t now)
{
    p_drive->camera_present       = p_inputs->camera_present;
    p_drive->camera_status        = p_inputs->camera_status;
    p_drive->camera_packet_age_ms = p_inputs->camera_packet_age_ms;
    p_drive->camera_image_age_ms  = p_inputs->camera_image_age_ms;

    p_drive->accepted_image_age_ms =
        p_selection->sample_generation
            ? (uint32_t) (now - p_selection->sample_received_ms) +
                  p_selection->sample_age_ms
            : UINT32_MAX;
}

static bool
recoverable_camera_problem (const char * p_problem)
{
    return strncmp(p_problem, "CAMERA_", 7) == 0 ||
           strcmp(p_problem, "TRACKING_MARKERS_MISSING") == 0 ||
           strcmp(p_problem, "REFERENCE_BASELINE_UNAVAILABLE") == 0;
}

static bool
centred (int bearing)
{
    return bearing >= -ONBOARD_DRIVE_MAX_BEARING_CDEG &&
           bearing <= ONBOARD_DRIVE_MAX_BEARING_CDEG;
}

static const char *
track (onboard_drive_t * p_drive, const mission_t * p_mission,
       const target_selector_t * p_selection, bool b_allow_misaligned)
{
    if (p_selection->leg_id != p_drive->leg_id ||
        p_selection->marker != p_mission->leg.next_target_node)
    {
        return "TRACKING_ID_CHANGED";
    }
    p_drive->estimate_valid = false;
    if (!p_selection->available && !p_selection->reference_available)
    {
        return "TRACKING_MARKERS_MISSING";
    }
    if (p_selection->reference_available &&
        p_mission->leg.arrival_reference_node &&
        p_selection->reference_marker != p_mission->leg.arrival_reference_node)
    {
        return "TRACKING_ID_CHANGED";
    }
    if (!b_allow_misaligned && p_selection->available &&
        !centred(p_selection->bearing_cdeg))
    {
        return "TARGET_NOT_ALIGNED";
    }

    bool b_ref =
        p_selection->reference_available &&
        p_mission->leg.arrival_reference_node &&
        p_selection->reference_marker == p_mission->leg.arrival_reference_node;

    if (!p_selection->available &&
        (!b_ref ||
         (!p_drive->reference_model_valid &&
          !(p_drive->search_active && p_drive->last_target_range_mm))))
    {
        return "REFERENCE_BASELINE_UNAVAILABLE";
    }
    if (!p_selection->available &&
        (p_selection->reference_bearing_cdeg < -TARGET_MAX_BEARING_CDEG ||
         p_selection->reference_bearing_cdeg > TARGET_MAX_BEARING_CDEG))
    {
        return "REFERENCE_NOT_ALIGNED";
    }
    if (p_drive->reference_model_valid &&
        p_mission->leg.arrival_reference_node != p_drive->reference_id)
    {
        return "TRACKING_ID_CHANGED";
    }
    if (p_selection->sample_generation != p_drive->last_sample_generation)
    {
        if (p_selection->available)
        {
            p_drive->last_target_range_mm = p_selection->range_mm;
        }
        // A simultaneous pair validates ordering. A reference confirmed later
        // during slow search uses the user's known 600 mm marker spacing.
        if (p_selection->available && b_ref)
        {

            int bearing_delta = (int) p_selection->reference_bearing_cdeg -
                                p_selection->bearing_cdeg;

            if ((uint32_t) p_selection->reference_range_mm <
                    (uint32_t) p_selection->range_mm +
                        TARGET_REFERENCE_MIN_GAP_MM ||
                bearing_delta < -TARGET_REFERENCE_MAX_DELTA_CDEG ||
                bearing_delta > TARGET_REFERENCE_MAX_DELTA_CDEG)
            {
                return "REFERENCE_GEOMETRY_INCONSISTENT";
            }
            if (!p_drive->reference_model_valid)
            {
                p_drive->reference_origin_mm = p_selection->reference_range_mm;
            }
            p_drive->reference_model_valid = true;
            p_drive->reference_id     = p_mission->leg.arrival_reference_node;
            p_drive->target_anchor_mm = p_selection->range_mm;
            p_drive->reference_anchor_mm = p_selection->reference_range_mm;
        }
        if (b_ref && !p_drive->reference_model_valid &&
            p_drive->search_active && p_drive->last_target_range_mm)
        {
            p_drive->reference_model_valid = true;
            p_drive->reference_id = p_mission->leg.arrival_reference_node;

            p_drive->reference_origin_mm = p_drive->reference_anchor_mm =
                p_selection->reference_range_mm;

            p_drive->target_anchor_mm = p_drive->last_target_range_mm;
        }
        p_drive->last_sample_generation = p_selection->sample_generation;
    }
    p_drive->using_reference = b_ref && p_drive->reference_model_valid;

    p_drive->remaining_mm = p_drive->using_reference
                                ? (int32_t) p_selection->reference_range_mm -
                                      p_drive->reference_arrival_mm
                                : (int32_t) p_selection->range_mm;

    if (b_ref && p_drive->reference_model_valid)
    {

        p_drive->reference_progress_mm =
            p_drive->reference_origin_mm -
            (int32_t) p_selection->reference_range_mm;
    }
    p_drive->estimate_valid = true;
    return NULL;
}

static const char *
plan_cell (const onboard_drive_t * p_drive, const rover_map_t * p_map,
           uint8_t target_id, float heading, float * p_north,
           leg_direction_t * p_direction, int16_t * p_x_coordinate,
           int16_t * p_y_coordinate)
{
    if (!isfinite(heading) || heading < 0.0f || heading >= 360.0f)
    {
        return "HEADING_UNAVAILABLE";
    }
    if (!p_drive->frame_valid && p_map->count != 1u)
    {
        return "GRID_HEADING_UNAVAILABLE";
    }
    *p_north = p_drive->frame_valid ? p_drive->north_heading_deg : heading;
    float    delta    = fmodf(heading - *p_north + 360.0f, 360.0f);
    unsigned cardinal = (unsigned) ((delta + 45.0f) / 90.0f) % 4u;
    float    error    = fabsf(delta - cardinal * 90.0f);
    if (error > 180.0f)
    {
        error = 360.0f - error;
    }
    if (error > 15.0f)
    {
        return "GRID_DIRECTION_NOT_ALIGNED";
    }
    static const int8_t delta_x[] = {0, 1, 0, -1};
    static const int8_t delta_y[] = {1, 0, -1, 0};
    const map_node_t *  p_from = rover_map_find_id(p_map, p_map->current_node);
    int32_t             target_x = (int32_t) p_from->x + delta_x[cardinal];
    int32_t             target_y = (int32_t) p_from->y + delta_y[cardinal];
    if (target_x < INT16_MIN || target_x > INT16_MAX || target_y < INT16_MIN ||
        target_y > INT16_MAX)
    {
        return "TARGET_CELL_OVERFLOW";
    }
    const map_node_t * p_known = rover_map_find_id(p_map, target_id);

    const map_node_t * p_occupant =
        rover_map_find_cell(p_map, (int16_t) target_x, (int16_t) target_y);

    if ((p_known && (p_known->x != target_x || p_known->y != target_y)) ||
        (p_occupant && p_occupant->marker_id != target_id))
    {
        return "TARGET_MAP_CONFLICT";
    }
    if (p_from->exits[cardinal] == MAP_EXIT_BLOCKED ||
        (p_known && p_known->exits[(cardinal + 2u) % 4u] == MAP_EXIT_BLOCKED))
    {
        return "FRONT_BLOCKED";
    }
    if (!p_known && p_map->count >= ROVER_MAP_CAPACITY)
    {
        return "MAP_FULL";
    }
    *p_direction    = (leg_direction_t) cardinal;
    *p_x_coordinate = (int16_t) target_x;
    *p_y_coordinate = (int16_t) target_y;
    return NULL;
}

typedef struct
{
    const map_node_t * p_current;
    const char *       p_problem;
    float              north;
    leg_direction_t    direction;
    int16_t            x_coordinate;
    int16_t            y_coordinate;
    onboard_drive_t    prepared;
    motor_error_t      result;
    const char *       return_value;
} onboard_drive_start_context_t;

static void begin_alignment (onboard_drive_t * p_drive, mission_t * p_mission,
                             target_selector_t * p_selection, uint32_t now);

/** @brief Execute the prepare stage of onboard_drive_start. */
static bool
onboard_drive_start_prepare (onboard_drive_t * p_drive, mission_t * p_mission,
                             rover_map_t *                   p_map,
                             const target_selector_t *       p_selection,
                             const onboard_drive_inputs_t *  p_inputs,
                             uint32_t                        now,
                             onboard_drive_start_context_t * p_context)
{

    // Check this on RoboPico, never rely on the dashboard's cached map.
    if (!p_map->count)
    {
        p_context->return_value = "START_NODE_NOT_SET";
        return true;
    }
    if (p_drive->active && p_mission->state == MISSION_EXPLORING &&
        p_drive->leg_id == p_mission->leg.leg_id)
    {
        p_context->return_value = "ALREADY_RUNNING";
        return true;
    }

    p_context->p_current = rover_map_find_id(p_map, p_map->current_node);
    if (!p_context->p_current || !p_context->p_current->visited)
    {
        p_context->return_value = "CURRENT_NODE_UNKNOWN";
        return true;
    }
    if (p_inputs->safety_latched)
    {
        p_context->return_value = "SAFETY_LOCKED";
        return true;
    }
    if (p_mission->test_mode ||
        (p_mission->state != MISSION_IDLE &&
         p_mission->state != MISSION_READY &&
         !(p_mission->dfs_active && p_mission->state == MISSION_EXPLORING)))
    {
        p_context->return_value = "MISSION_NOT_IDLE";
        return true;
    }
    if (!p_inputs->stopped)
    {
        p_context->return_value = "MOTOR_BUSY";
        return true;
    }
    if (p_selection->state != TARGET_LOCKED || !p_mission->leg.configured ||
        p_mission->leg.phase != LEG_CONFIGURED ||
        !p_mission->leg.next_target_node ||
        p_selection->leg_id != p_mission->leg.leg_id ||
        p_selection->marker != p_mission->leg.next_target_node ||
        p_mission->leg.departure_node != p_map->current_node)
    {
        p_context->return_value = "NO_LOCKED_TARGET";
        return true;
    }

    p_context->p_problem = input_problem(p_selection, p_inputs, now);
    return false;
}

/** @brief Execute the inputs stage of onboard_drive_start. */
static bool
onboard_drive_start_inputs (onboard_drive_t * p_drive, mission_t * p_mission,
                            rover_map_t *                   p_map,
                            const target_selector_t *       p_selection,
                            const onboard_drive_inputs_t *  p_inputs,
                            uint32_t                        now,
                            onboard_drive_start_context_t * p_context)
{

    if (p_context->p_problem)
    {
        p_context->return_value = p_context->p_problem;
        return true;
    }
    if (!p_selection->available)
    {
        p_context->return_value = "TARGET_NOT_VISIBLE_AT_START";
        return true;
    }
    int max_bearing = p_selection->b_acquiring
                          ? TARGET_ACQUISITION_MAX_BEARING_CDEG
                          : TARGET_TRACKING_MAX_BEARING_CDEG;
    if (p_selection->bearing_cdeg < -max_bearing ||
        p_selection->bearing_cdeg > max_bearing)
    {
        p_context->return_value = "TARGET_NOT_ALIGNED";
        return true;
    }
    if (p_selection->range_mm <= ONBOARD_DRIVE_NEAR_RANGE_MM)
    {
        p_context->return_value = "TARGET_TOO_CLOSE";
        return true;
    }

    p_context->p_problem = plan_cell(
        p_drive, p_map, p_mission->leg.next_target_node, p_inputs->heading_deg,
        &p_context->north, &p_context->direction, &p_context->x_coordinate,
        &p_context->y_coordinate);

    if (p_context->p_problem)
    {
        p_context->return_value = p_context->p_problem;
        return true;
    }

    p_context->prepared = (onboard_drive_t) {
        .active               = true,
        .b_departure_pending  = !centred(p_selection->bearing_cdeg),
        .leg_id               = p_mission->leg.leg_id,
        .started_ms           = now,
        .last_refresh_ms      = now,
        .result               = "APPROACHING",
        .frame_valid          = true,
        .north_heading_deg    = p_context->north,
        .last_tick_ms         = now,
        .input_reason         = "NONE",
        .reference_arrival_mm = ONBOARD_DRIVE_REFERENCE_ARRIVAL_MM};

    note_inputs(&p_context->prepared, p_selection, p_inputs, now);
    // Preserve the measured reference baseline while stationary alignment
    // obtains the centred images required before the first forward command.
    p_context->p_problem = track(&p_context->prepared, p_mission, p_selection,
                                 p_context->prepared.b_departure_pending);
    return false;
}

/** @brief Execute the motion stage of onboard_drive_start. */
static bool
onboard_drive_start_motion (onboard_drive_t * p_drive, mission_t * p_mission,
                            rover_map_t *       p_map,
                            target_selector_t * p_selection, uint32_t now,
                            onboard_drive_start_context_t * p_context)
{

    if (p_context->p_problem)
    {
        p_context->return_value = p_context->p_problem;
        return true;
    }

    if (!p_context->prepared.b_departure_pending)
    {
        p_context->result = motor_auto_drive_leased(MC_AUTO_APPROACH_PCT,
                                                    ONBOARD_DRIVE_LEASE_MS);
        if (p_context->result != MOTOR_OK)
        {

            p_context->return_value =
                p_context->result == MOTOR_ERR_NO_HEADING
                    ? "HEADING_UNAVAILABLE"
                : p_context->result == MOTOR_ERR_FRONT_BLOCKED
                    ? "FRONT_BLOCKED"
                    : "MOTOR_REFUSED";

            return true;
        }
    }
    if (!mission_begin_drive(p_mission, now))
    {
        motor_emergency_stop();
        p_context->return_value = "MISSION_NOT_IDLE";
        return true;
    }
    *p_drive                 = p_context->prepared;
    p_mission->leg.direction = p_context->direction;
    p_mission->leg.target_x  = p_context->x_coordinate;
    p_mission->leg.target_y  = p_context->y_coordinate;

    p_mission->leg.target_cell_valid = true;
    // Planned adjacency, not yet inserted/visited.
    if (p_drive->b_departure_pending)
    {
        begin_alignment(p_drive, p_mission, p_selection, now);
    }
    else
    {
        rover_map_depart(p_map); // Only after forward motor acceptance.
    }
    p_drive->map_revision   = p_map->revision;
    p_context->return_value = "STARTED";
    return true;
}

const char *
onboard_drive_start (onboard_drive_t * p_drive, mission_t * p_mission,
                     rover_map_t * p_map, target_selector_t * p_selection,
                     const onboard_drive_inputs_t * p_inputs, uint32_t now)
{
    onboard_drive_start_context_t context = {0};
    if (onboard_drive_start_prepare(p_drive, p_mission, p_map, p_selection,
                                    p_inputs, now, &context))
    {
        return context.return_value;
    }
    if (onboard_drive_start_inputs(p_drive, p_mission, p_map, p_selection,
                                   p_inputs, now, &context))
    {
        return context.return_value;
    }
    if (onboard_drive_start_motion(p_drive, p_mission, p_map, p_selection, now,
                                   &context))
    {
        return context.return_value;
    }
    return context.return_value;
}

void
onboard_drive_stop (onboard_drive_t * p_drive, mission_t * p_mission,
                    mission_reason_t reason, const char * p_result,
                    uint32_t now)
{
    motor_emergency_stop();
    mission_pause(p_mission, reason, now);
    p_drive->active                = false;
    p_drive->reference_recovery    = false;
    p_drive->search_active         = false;
    p_drive->front_arrival_pending = false;
    p_drive->aligning = p_drive->alignment_rotating = false;
    p_drive->camera_waiting                         = false;
    p_drive->input_waiting = p_drive->correcting = false;
    p_drive->estimate_valid                      = false;
    p_drive->result                              = p_result;
}

static bool
reference_absent (const onboard_drive_inputs_t * p_inputs)
{
    return p_inputs->reference_absent && p_inputs->camera_present &&
           (p_inputs->camera_status == VISION_STATUS_OK ||
            p_inputs->camera_status == VISION_STATUS_NO_MARKER) &&
           p_inputs->camera_packet_age_ms <= TARGET_UART_MAX_AGE_MS &&
           p_inputs->camera_image_age_ms <= ONBOARD_DRIVE_IMAGE_MAX_AGE_MS;
}

/** @brief Fresh images, including explicit no-marker frames, permit recovery.
 */
static bool
live_camera_view (const onboard_drive_inputs_t * p_inputs)
{
    return p_inputs->camera_present &&
           (p_inputs->camera_status == VISION_STATUS_NO_MARKER ||
            (p_inputs->camera_status == VISION_STATUS_OK &&
             p_inputs->camera_fresh)) &&
           p_inputs->camera_packet_age_ms <= TARGET_UART_MAX_AGE_MS &&
           p_inputs->camera_image_age_ms <= ONBOARD_DRIVE_IMAGE_MAX_AGE_MS;
}

static bool
tracking_markers_missing (const target_selector_t *      p_selection,
                          const onboard_drive_inputs_t * p_inputs)
{
    return !p_selection->available && !p_selection->reference_available &&
           live_camera_view(p_inputs);
}

static bool
waitable_input (const char * p_problem)
{
    return p_problem && (!strcmp(p_problem, "LINK_LOST") ||
                         !strcmp(p_problem, "SENSOR_STALE") ||
                         !strcmp(p_problem, "HEADING_UNAVAILABLE") ||
                         !strcmp(p_problem, "DRIVE_LEASE_EXPIRED") ||
                         !strcmp(p_problem, "BACKUP_MOTOR_STOPPED") ||
                         !strcmp(p_problem, "MOTOR_STOPPED"));
}

static bool
begin_input_wait (onboard_drive_t * p_drive, mission_t * p_mission,
                  const char * p_problem, uint32_t now)
{
    if (!p_drive->input_waiting)
    {
        motor_emergency_stop();

        leg_phase_t from = p_drive->camera_waiting
                               ? p_drive->camera_resume_phase
                               : p_mission->leg.phase;

        if (from == LEG_BACKING_UP)
        {
            from                       = LEG_REACQUIRING;
            p_drive->backup_stopped_ms = now;
        }
        if (from == LEG_CORRECTING)
        {
            from               = LEG_BRAKING;
            p_drive->braked_ms = now;
        }
        p_drive->input_resume_phase    = from;
        p_drive->input_wait_started_ms = now;
        p_drive->input_waiting         = true;
        p_drive->camera_waiting        = false;
        p_drive->correcting            = false;
        p_drive->alignment_rotating    = false;
        p_drive->alignment_frames      = 0;
        p_drive->verification_frames   = 0;
        p_drive->alignment_stopped_ms  = now;
        mission_recovery_phase(p_mission, LEG_WAITING_INPUT, now);
    }
    p_drive->result         = "WAITING_FOR_INPUT";
    p_drive->input_reason   = p_problem;
    p_drive->estimate_valid = false;
    return true;
}

static bool
recovery_failure (onboard_drive_t * p_drive, mission_t * p_mission,
                  const char * p_problem, uint32_t now)
{
    if (waitable_input(p_problem))
    {
        return begin_input_wait(p_drive, p_mission, p_problem, now);
    }
    p_drive->input_reason = p_problem;
    onboard_drive_stop(p_drive, p_mission, MISSION_REASON_INPUT_LOST, p_problem,
                       now);
    return true;
}

/** @brief Retain the leg while waiting for landmarks, then reverse-search. */
static bool
begin_marker_recovery (onboard_drive_t * p_drive, mission_t * p_mission,
                       const target_selector_t * p_selection, uint32_t now)
{
    motor_emergency_stop();
    if (p_mission->leg.phase != LEG_HOLDING &&
        !mission_reference_phase(p_mission, LEG_HOLDING, now))
    {
        return recovery_failure(p_drive, p_mission,
                                "REFERENCE_RECOVERY_STATE_INVALID", now);
    }
    p_drive->reference_recovery   = true;
    p_drive->reference_wait_ms    = now;
    p_drive->reference_missing    = true;
    p_drive->reference_missing_ms = now;
    p_drive->hold_started_ms      = now;
    p_drive->recovery_generation  = p_selection->sample_generation;
    p_drive->recovery_frames      = 0;
    p_drive->verification_frames  = 0;
    p_drive->estimate_valid       = false;
    p_drive->input_reason         = "TRACKING_MARKERS_MISSING";
    p_drive->result               = "WAITING_FOR_TRACKING_MARKERS";
    return true;
}

static bool
recovery_position (const rover_map_t * p_map, uint8_t identifier,
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

static bool
recovery_marker_usable (const camera_marker_t * p_marker)
{
    return p_marker->id && !p_marker->quality && p_marker->range_mm &&
           p_marker->range_mm != UINT16_MAX &&
           p_marker->bearing_cdeg >= -TARGET_RECOVERY_MAX_BEARING_CDEG &&
           p_marker->bearing_cdeg <= TARGET_RECOVERY_MAX_BEARING_CDEG;
}

static bool
recovery_landmark (const rover_map_t * p_map, const mission_t * p_mission,
                   const target_selector_t *      p_selection,
                   const onboard_drive_inputs_t * p_inputs, uint32_t now,
                   camera_marker_t * p_out)
{
    if (!p_inputs->camera_present ||
        p_inputs->camera_status != VISION_STATUS_OK ||
        p_inputs->camera_packet_age_ms > TARGET_UART_MAX_AGE_MS ||
        p_inputs->camera_image_age_ms > ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
    {
        return false;
    }
    bool b_found = false;
    if (p_inputs->recovery_marker_count <= VISION_MAX_MARKERS)
    {
        for (uint8_t index = 0; index < p_inputs->recovery_marker_count;
             ++index)
        {

            const camera_marker_t * p_marker =
                &p_inputs->recovery_markers[index];

            int16_t x_coordinate = 0;
            int16_t y_coordinate = 0;
            if (!recovery_marker_usable(p_marker))
            {
                continue;
            }
            if (p_marker->id == p_mission->leg.arrival_reference_node)
            {
                *p_out = *p_marker;
                return true;
            }
            if ((p_marker->id == p_mission->leg.next_target_node ||
                 recovery_position(p_map, p_marker->id, &x_coordinate,
                                   &y_coordinate)) &&
                (!b_found || p_marker->range_mm < p_out->range_mm))
            {
                *p_out  = *p_marker;
                b_found = true;
            }
        }
    }
    // The selector also supplies the original reference (used by native callers
    // without a full frame). Never infer a different landmark from this
    // fallback.
    if (p_selection->reference_available &&
        p_selection->reference_marker ==
            p_mission->leg.arrival_reference_node &&
        !input_problem(p_selection, p_inputs, now))
    {

        *p_out = (camera_marker_t) {.id       = p_selection->reference_marker,
                                    .range_mm = p_selection->reference_range_mm,
                                    .bearing_cdeg =
                                        p_selection->reference_bearing_cdeg};

        return true;
    }
    return b_found;
}

static void
note_recovery_landmark (onboard_drive_t * p_drive, const rover_map_t * p_map,
                        const camera_marker_t * p_marker)
{
    int16_t x_coordinate = 0;
    int16_t y_coordinate = 0;
    if (p_drive->recovery_marker_id != p_marker->id)
    {
        p_drive->mapped_recovery_frames   = 0;
        p_drive->mapped_recovery_have_seq = false;
    }
    p_drive->recovery_marker_id = p_marker->id;

    p_drive->recovery_marker_mapped =
        recovery_position(p_map, p_marker->id, &x_coordinate, &y_coordinate);
}

static bool
begin_reverse_search (onboard_drive_t * p_drive, mission_t * p_mission,
                      uint32_t now)
{
    if (motor_auto_drive_leased(-ONBOARD_DRIVE_BACKUP_PCT,
                                ONBOARD_DRIVE_LEASE_MS) != MOTOR_OK)
    {
        return recovery_failure(p_drive, p_mission, "BACKUP_MOTOR_REFUSED",
                                now);
    }
    p_drive->backup_used       = true;
    p_drive->backup_started_ms = p_drive->last_refresh_ms = now;
    p_drive->recovery_frames = p_drive->mapped_recovery_frames = 0;
    p_drive->estimate_valid                                    = false;
    p_drive->recovery_marker_id                                = 0;
    p_drive->recovery_marker_mapped                            = false;
    p_drive->mapped_recovery_have_seq                          = false;
    mission_reference_phase(p_mission, LEG_BACKING_UP, now);
    p_drive->result = "REVERSING_FOR_MAPPED_MARKER";
    return true;
}

// A mapped sighting is a landmark, never an arrival. Use it as a replacement
// reference only when its cell lies ahead of the locked destination on this
// leg.
typedef struct
{
    uint8_t delta;
    _Bool   return_value;
} adopt_mapped_reference_context_t;

/** @brief Execute the prepare stage of adopt_mapped_reference. */
static bool
adopt_mapped_reference_prepare (onboard_drive_t *                  p_drive,
                                const onboard_drive_inputs_t *     p_inputs,
                                adopt_mapped_reference_context_t * p_context)
{

    if (p_inputs->camera_generation == p_drive->mapped_recovery_generation)
    {
        p_context->return_value = false;
        return true;
    }
    p_drive->mapped_recovery_generation = p_inputs->camera_generation;

    p_context->delta =
        (uint8_t) (p_inputs->camera_seq - p_drive->mapped_recovery_seq);

    if (p_drive->mapped_recovery_have_seq &&
        (!p_context->delta || p_context->delta >= 128u))
    {
        if (p_context->delta >= 128u)
        {
            p_drive->mapped_recovery_frames = 0;
        }
        p_drive->mapped_recovery_seq = p_inputs->camera_seq;
        p_context->return_value      = false;
        return true;
    }
    p_drive->mapped_recovery_have_seq = true;
    p_drive->mapped_recovery_seq      = p_inputs->camera_seq;
    if (p_drive->mapped_recovery_frames < TARGET_CONFIRM_FRAMES)
    {
        p_drive->mapped_recovery_frames++;
    }
    if (p_drive->mapped_recovery_frames < TARGET_CONFIRM_FRAMES)
    {
        p_drive->result         = "CONFIRMING_MAPPED_MARKER";
        p_context->return_value = false;
        return true;
    }
    return false;
}

/** @brief Execute the inputs stage of adopt_mapped_reference. */
static bool
adopt_mapped_reference_inputs (onboard_drive_t * p_drive, mission_t * p_mission,
                               target_selector_t *                p_selection,
                               const rover_map_t *                p_map,
                               const camera_marker_t *            p_marker,
                               adopt_mapped_reference_context_t * p_context)
{

    if (p_marker->id == p_mission->leg.next_target_node &&
        p_marker->range_mm > ONBOARD_DRIVE_NEAR_RANGE_MM)
    {
        // Restore the same destination, discarding the old range model. A
        // misaligned sighting re-enters alignment before any forward motion.
        p_drive->reference_model_valid = false;
        p_drive->using_reference       = false;
        p_drive->last_target_range_mm  = p_marker->range_mm;
        p_selection->available         = true;
        p_selection->range_mm          = p_marker->range_mm;
        p_selection->bearing_cdeg      = p_marker->bearing_cdeg;
        p_drive->result                = "MAPPED_TARGET_REACQUIRED";
    }
    else
    {
        int16_t x_coordinate = 0;
        int16_t y_coordinate = 0;
        if (!p_mission->leg.target_cell_valid ||
            (unsigned) p_mission->leg.direction >= 4u ||
            !recovery_position(p_map, p_marker->id, &x_coordinate,
                               &y_coordinate))
        {
            p_context->return_value = false;
            return true;
        }
        static const int8_t delta_x[] = {0, 1, 0, -1};
        static const int8_t delta_y[] = {1, 0, -1, 0};
        int32_t relative_x = (int32_t) x_coordinate - p_mission->leg.target_x;
        int32_t relative_y = (int32_t) y_coordinate - p_mission->leg.target_y;

        int32_t along = relative_x * delta_x[p_mission->leg.direction] +
                        relative_y * delta_y[p_mission->leg.direction];

        int32_t across = relative_x * delta_y[p_mission->leg.direction] -
                         relative_y * delta_x[p_mission->leg.direction];

        int bearing_delta =
            (int) p_marker->bearing_cdeg - p_selection->bearing_cdeg;

        if (across || along <= 0 ||
            along > UINT16_MAX / ONBOARD_DRIVE_REFERENCE_ARRIVAL_MM ||
            p_marker->bearing_cdeg < -TARGET_RECOVERY_MAX_BEARING_CDEG ||
            p_marker->bearing_cdeg > TARGET_RECOVERY_MAX_BEARING_CDEG ||
            (p_selection->available &&
             ((uint32_t) p_marker->range_mm < (uint32_t) p_selection->range_mm +
                                                  TARGET_REFERENCE_MIN_GAP_MM ||
              bearing_delta < -TARGET_REFERENCE_MAX_DELTA_CDEG ||
              bearing_delta > TARGET_REFERENCE_MAX_DELTA_CDEG)))
        {
            p_drive->result         = "MAPPED_MARKER_NEEDS_REFERENCE";
            p_drive->input_reason   = "MAPPED_MARKER_NOT_FORWARD_REFERENCE";
            p_context->return_value = false;
            return true;
        }

        p_drive->reference_arrival_mm =
            (uint16_t) (along * ONBOARD_DRIVE_REFERENCE_ARRIVAL_MM);

        p_mission->leg.arrival_reference_node = p_marker->id;
        p_drive->reference_id                 = p_marker->id;
        p_drive->reference_model_valid        = true;

        p_drive->reference_origin_mm = p_drive->reference_anchor_mm =
            p_marker->range_mm;

        p_drive->reference_progress_mm       = 0;
        p_selection->reference_marker        = p_marker->id;
        p_selection->reference_available     = true;
        p_selection->reference_candidate     = p_marker->id;
        p_selection->reference_confirmations = TARGET_CONFIRM_FRAMES;
        p_selection->reference_range_mm      = p_marker->range_mm;
        p_selection->reference_bearing_cdeg  = p_marker->bearing_cdeg;
        p_drive->result                      = "MAPPED_REFERENCE_REACQUIRED";
    }
    return false;
}

/** @brief Execute the motion stage of adopt_mapped_reference. */
static bool
adopt_mapped_reference_motion (onboard_drive_t * p_drive, mission_t * p_mission,
                               target_selector_t *                p_selection,
                               const onboard_drive_inputs_t *     p_inputs,
                               uint32_t                           now,
                               adopt_mapped_reference_context_t * p_context)
{

    p_selection->sample_generation  = p_inputs->camera_generation;
    p_selection->sample_received_ms = now - p_inputs->camera_packet_age_ms;

    p_selection->sample_age_ms =
        (uint16_t) (p_inputs->camera_image_age_ms >
                            p_inputs->camera_packet_age_ms
                        ? p_inputs->camera_image_age_ms -
                              p_inputs->camera_packet_age_ms
                        : 0);

    p_drive->reference_recovery     = false;
    p_drive->reference_missing      = false;
    p_drive->verification_frames    = 0;
    p_drive->last_sample_generation = 0;
    mission_reference_phase(p_mission, LEG_APPROACHING, now);
    p_context->return_value = true;
    return true;
}

static bool
adopt_mapped_reference (onboard_drive_t * p_drive, mission_t * p_mission,
                        target_selector_t *            p_selection,
                        const rover_map_t *            p_map,
                        const onboard_drive_inputs_t * p_inputs,
                        const camera_marker_t * p_marker, uint32_t now)
{
    adopt_mapped_reference_context_t context = {0};
    if (adopt_mapped_reference_prepare(p_drive, p_inputs, &context))
    {
        return context.return_value;
    }
    if (adopt_mapped_reference_inputs(p_drive, p_mission, p_selection, p_map,
                                      p_marker, &context))
    {
        return context.return_value;
    }
    if (adopt_mapped_reference_motion(p_drive, p_mission, p_selection, p_inputs,
                                      now, &context))
    {
        return context.return_value;
    }
    return context.return_value;
}

static void
finish_backup (onboard_drive_t * p_drive, mission_t * p_mission,
               const target_selector_t * p_selection, uint32_t now)
{
    motor_emergency_stop();
    mission_reference_phase(p_mission, LEG_REACQUIRING, now);
    p_drive->backup_stopped_ms   = now;
    p_drive->recovery_frames     = 0;
    p_drive->verification_frames = 0;
    p_drive->recovery_generation = p_selection->sample_generation;
    p_drive->estimate_valid      = false;
    p_drive->result              = "SETTLING_AFTER_BACKUP";
}

// Returns true while recovery owns this tick (including a terminal failure).
// Fresh images without usable tracking markers permit reverse search; stale
// camera data always stops motion and waits for observations to recover.
typedef struct
{
    const char * p_problem;
    _Bool        b_after_backup;
    uint32_t     settled;
    _Bool        return_value;
} recover_reference_context_t;

/** @brief Execute the prepare stage of recover_reference. */
static bool
recover_reference_prepare (target_selector_t *            p_selection,
                           const onboard_drive_inputs_t * p_inputs,
                           uint32_t                       now,
                           recover_reference_context_t *  p_context)
{

    p_context->p_problem = input_problem(p_selection, p_inputs, now);
    return false;
}

/** @brief Execute the inputs stage of recover_reference. */
static bool
recover_reference_inputs (onboard_drive_t * p_drive, mission_t * p_mission,
                          target_selector_t *            p_selection,
                          const rover_map_t *            p_map,
                          const onboard_drive_inputs_t * p_inputs, uint32_t now,
                          recover_reference_context_t * p_context)
{

    if (p_mission->leg.phase == LEG_BACKING_UP)
    {
        if (!p_inputs->camera_present ||
            (p_inputs->camera_status != VISION_STATUS_OK &&
             p_inputs->camera_status != VISION_STATUS_NO_MARKER))
        {

            p_context->return_value = recovery_failure(
                p_drive, p_mission,
                !p_inputs->camera_present ? "CAMERA_UART_MISSING"
                : p_inputs->camera_status == VISION_STATUS_STALLED
                    ? "CAMERA_PROCESSING_STALLED"
                    : "CAMERA_LOST",
                now);

            return true;
        }
        camera_marker_t mk = {0};

        bool b_seen_now = recovery_landmark(p_map, p_mission, p_selection,
                                            p_inputs, now, &mk);

        uint32_t captured =
            mk.id == p_mission->leg.arrival_reference_node &&
                    !p_inputs->recovery_marker_count
                ? p_selection->sample_received_ms - p_selection->sample_age_ms
                : now - p_inputs->camera_image_age_ms;

        b_seen_now = b_seen_now &&
                     (int32_t) (captured - p_drive->backup_started_ms) >= 0;

        if (b_seen_now)
        {
            note_recovery_landmark(p_drive, p_map, &mk);
            finish_backup(p_drive, p_mission, p_selection, now);
            p_context->return_value = true;
            return true;
        }
        if (motor_auto_status() != MOTOR_AUTO_RUNNING)
        {

            p_context->return_value = recovery_failure(
                p_drive, p_mission, "BACKUP_MOTOR_STOPPED", now);

            return true;
        }
        if ((uint32_t) (now - p_drive->last_refresh_ms) >=
            ONBOARD_DRIVE_REFRESH_MS)
        {
            if (!motor_auto_renew_lease())
            {

                p_context->return_value = recovery_failure(
                    p_drive, p_mission, "DRIVE_LEASE_EXPIRED", now);

                return true;
            }
            p_drive->last_refresh_ms = now;
        }
        p_drive->result         = "REVERSING_FOR_MAPPED_MARKER";
        p_context->return_value = true;
        return true;
    }
    return false;
}

/** @brief Execute the motion stage of recover_reference. */
static bool
recover_reference_motion (onboard_drive_t * p_drive, mission_t * p_mission,
                          const onboard_drive_inputs_t * p_inputs, uint32_t now,
                          recover_reference_context_t * p_context)
{

    p_context->b_after_backup = p_mission->leg.phase == LEG_REACQUIRING;
    if (reference_absent(p_inputs) && !p_drive->reference_missing)
    {
        p_drive->reference_missing    = true;
        p_drive->reference_missing_ms = now;
    }

    p_context->settled =
        p_context->b_after_backup
            ? p_drive->backup_stopped_ms + ONBOARD_DRIVE_SETTLE_MS
            : p_drive->reference_wait_ms;

    if (p_context->b_after_backup &&
        (uint32_t) (now - p_drive->backup_stopped_ms) < ONBOARD_DRIVE_SETTLE_MS)
    {
        p_context->return_value = true;
        return true;
    }
    return false;
}

/** @brief Execute the camera stage of recover_reference. */
static bool
recover_reference_camera (onboard_drive_t * p_drive, mission_t * p_mission,
                          target_selector_t * p_selection, uint32_t now,
                          bool *                        p_resuming,
                          recover_reference_context_t * p_context)
{

    // No geometry or confirmation is taken from a pre-recovery/moving image.
    if (!p_context->p_problem && p_selection->reference_available &&
        (int32_t) (p_selection->sample_received_ms -
                   p_selection->sample_age_ms - p_context->settled) >= 0)
    {
        target_selector_t reference_only = *p_selection;
        if (reference_only.available && !centred(reference_only.bearing_cdeg))
        {
            reference_only.available = false;
        }
        p_context->p_problem =
            track(p_drive, p_mission, &reference_only, false);
        if (p_context->p_problem &&
            strcmp(p_context->p_problem, "REFERENCE_NOT_ALIGNED") == 0)
        {
            p_drive->reference_recovery = false;
            begin_alignment(p_drive, p_mission, p_selection, now);
            p_context->return_value = true;
            return true;
        }
        if (p_context->p_problem &&
            !recoverable_camera_problem(p_context->p_problem))
        {

            p_context->return_value =
                recovery_failure(p_drive, p_mission, p_context->p_problem, now);

            return true;
        }
        if (!p_context->p_problem &&
            p_selection->sample_generation != p_drive->recovery_generation)
        {
            p_drive->reference_missing   = false;
            p_drive->recovery_marker_id  = p_selection->reference_marker;
            p_drive->recovery_generation = p_selection->sample_generation;
            if (++p_drive->recovery_frames >= ONBOARD_DRIVE_RECOVERY_FRAMES)
            {
                // Restart ordinary braking/verification from the new stationary
                // position. No pre-backup verification count can survive
                // motion.
                p_drive->reference_recovery  = false;
                p_drive->verification_frames = 0;
                mission_reference_phase(p_mission, LEG_APPROACHING, now);
                p_drive->last_refresh_ms = now;
                *p_resuming              = true;
                p_context->return_value  = false;
                return true;
            }
            p_drive->result         = "CONFIRMING_REFERENCE_RECOVERY";
            p_context->return_value = true;
            return true;
        }
    }
    return false;
}

/** @brief Execute the tracking stage of recover_reference. */
static bool
recover_reference_tracking (onboard_drive_t * p_drive, mission_t * p_mission,
                            target_selector_t *            p_selection,
                            const rover_map_t *            p_map,
                            const onboard_drive_inputs_t * p_inputs,
                            uint32_t now, bool * p_resuming,
                            recover_reference_context_t * p_context)
{

    uint32_t landmark_settled =
        p_context->b_after_backup
            ? p_context->settled
            : p_drive->reference_wait_ms + ONBOARD_DRIVE_SETTLE_MS;
    if (p_inputs->camera_present &&
        (p_inputs->camera_status == VISION_STATUS_OK ||
         p_inputs->camera_status == VISION_STATUS_NO_MARKER) &&
        p_inputs->camera_packet_age_ms <= TARGET_UART_MAX_AGE_MS &&
        p_inputs->camera_image_age_ms <= ONBOARD_DRIVE_IMAGE_MAX_AGE_MS &&
        (int32_t) (now - p_inputs->camera_image_age_ms - landmark_settled) >= 0)
    {
        camera_marker_t mk;
        if (recovery_landmark(p_map, p_mission, p_selection, p_inputs, now,
                              &mk))
        {
            note_recovery_landmark(p_drive, p_map, &mk);
            bool b_adopt = mk.id != p_mission->leg.arrival_reference_node ||
                           !p_drive->reference_model_valid;
            if (b_adopt &&
                adopt_mapped_reference(p_drive, p_mission, p_selection, p_map,
                                       p_inputs, &mk, now))
            {
                *p_resuming             = true;
                p_context->return_value = false;
                return true;
            }
            if (b_adopt)
            {
                p_context->return_value = true;
                return true;
            }
        }
        else if (p_context->b_after_backup)
        {

            p_context->return_value =
                begin_reverse_search(p_drive, p_mission, now);

            return true;
        }
    }
    return false;
}

/** @brief Execute the reference stage of recover_reference. */
static bool
recover_reference_reference (onboard_drive_t * p_drive, mission_t * p_mission,
                             const target_selector_t *      p_selection,
                             const onboard_drive_inputs_t * p_inputs,
                             uint32_t                       now,
                             recover_reference_context_t *  p_context)
{

    if (!p_context->b_after_backup && p_drive->reference_missing &&
        (uint32_t) (now - p_drive->reference_missing_ms) >=
            ONBOARD_DRIVE_REFERENCE_MISSING_MS)
    {
        if (reference_absent(p_inputs) ||
            tracking_markers_missing(p_selection, p_inputs))
        {
            // Keep renewing a bounded lease until a mapped/locked marker is
            // seen. The existing overall journey deadline remains
            // authoritative.

            p_context->return_value =
                begin_reverse_search(p_drive, p_mission, now);

            return true;
        }
        // The two-second missing threshold is not a camera deadline. A packet
        // can age past the image/UART limit at exactly this boundary, between
        // successive healthy images. Stay stopped until fresh absence arrives
        // or the existing overall journey deadline expires; never reverse
        // using stale evidence. Camera health gaps remain stopped in the
        // common camera-wait path.
        if (p_context->p_problem)
        {
            if (!p_inputs->camera_present ||
                (p_inputs->camera_status != VISION_STATUS_OK &&
                 p_inputs->camera_status != VISION_STATUS_NO_MARKER))
            {

                p_drive->input_reason   = "CAMERA_LOST";
                p_drive->result         = "WAITING_FOR_FRESH_REFERENCE_FRAME";
                p_context->return_value = true;
                return true;
            }

            p_drive->input_reason =
                p_inputs->camera_packet_age_ms > TARGET_UART_MAX_AGE_MS
                    ? "CAMERA_PACKET_STALE"
                : p_inputs->camera_image_age_ms > ONBOARD_DRIVE_IMAGE_MAX_AGE_MS
                    ? "CAMERA_IMAGE_STALE"
                    : p_context->p_problem;

            p_drive->estimate_valid = false;
            p_drive->result         = "WAITING_FOR_FRESH_REFERENCE_FRAME";
            p_context->return_value = true;
            return true;
        }
    }
    if (reference_absent(p_inputs))
    {
        p_drive->recovery_frames = 0;
        p_drive->estimate_valid  = false;
    }

    p_drive->input_reason = p_context->b_after_backup
                                ? "REFERENCE_REACQUISITION"
                                : "REFERENCE_NOT_VISIBLE";

    return false;
}

/** @brief Execute the arrival stage of recover_reference. */
static bool
recover_reference_arrival (onboard_drive_t *             p_drive,
                           recover_reference_context_t * p_context)
{

    p_drive->result = p_context->b_after_backup ? "REACQUIRING_REFERENCE"
                                                : "WAITING_FOR_REFERENCE";

    p_context->return_value = true;
    return true;
}

static bool
recover_reference (onboard_drive_t * p_drive, mission_t * p_mission,
                   target_selector_t * p_selection, const rover_map_t * p_map,
                   const onboard_drive_inputs_t * p_inputs, uint32_t now,
                   bool * p_resuming)
{
    recover_reference_context_t context = {0};
    if (recover_reference_prepare(p_selection, p_inputs, now, &context))
    {
        return context.return_value;
    }
    if (recover_reference_inputs(p_drive, p_mission, p_selection, p_map,
                                 p_inputs, now, &context))
    {
        return context.return_value;
    }
    if (recover_reference_motion(p_drive, p_mission, p_inputs, now, &context))
    {
        return context.return_value;
    }
    if (recover_reference_camera(p_drive, p_mission, p_selection, now,
                                 p_resuming, &context))
    {
        return context.return_value;
    }
    if (recover_reference_tracking(p_drive, p_mission, p_selection, p_map,
                                   p_inputs, now, p_resuming, &context))
    {
        return context.return_value;
    }
    if (recover_reference_reference(p_drive, p_mission, p_selection, p_inputs,
                                    now, &context))
    {
        return context.return_value;
    }
    if (recover_reference_arrival(p_drive, &context))
    {
        return context.return_value;
    }
    return context.return_value;
}

bool
onboard_drive_note_front_stop (onboard_drive_t * p_drive,
                               const mission_t * p_mission)
{
    leg_phase_t phase = p_mission->leg.phase;
    if (phase == LEG_WAITING_CAMERA && p_drive->camera_waiting)
    {
        phase = p_drive->camera_resume_phase;
    }
    if (phase == LEG_WAITING_INPUT && p_drive->input_waiting)
    {
        phase = p_drive->input_resume_phase;
    }
    bool b_arrival_phase =
        phase == LEG_APPROACHING || phase == LEG_SEARCHING_REFERENCE ||
        phase == LEG_HOLDING || phase == LEG_BRAKING ||
        phase == LEG_VERIFYING || phase == LEG_OBSTACLE_VERIFYING;
    if (!p_drive->active || p_mission->test_mode ||
        p_mission->state != MISSION_EXPLORING || !p_mission->leg.configured ||
        !p_mission->leg.target_cell_valid || !p_mission->leg.next_target_node ||
        !p_drive->last_target_range_mm || p_drive->b_departure_pending ||
        p_drive->aligning || !b_arrival_phase ||
        p_drive->leg_id != p_mission->leg.leg_id ||
        p_mission->leg.phase == LEG_BACKING_UP ||
        p_mission->leg.phase == LEG_REACQUIRING)
    {
        return false;
    }
    p_drive->front_arrival_pending = true;
    return true;
}

static bool
commit_arrival (onboard_drive_t * p_drive, mission_t * p_mission,
                rover_map_t * p_map, bool b_wall, uint32_t now)
{
    if (!p_mission->leg.target_cell_valid ||
        rover_map_add(p_map, p_mission->leg.next_target_node,
                      p_mission->leg.target_x, p_mission->leg.target_y,
                      false) != MAP_OK ||
        rover_map_arrive(p_map, p_mission->leg.next_target_node) != MAP_OK)
    {
        onboard_drive_stop(p_drive, p_mission, MISSION_REASON_INPUT_LOST,
                           "ARRIVAL_MAP_COMMIT_FAILED", now);
        return false;
    }
    mission_complete_target(p_mission, now);
    rover_map_set_exit(p_map, p_mission->leg.departure_node,
                       (map_direction_t) p_mission->leg.direction,
                       MAP_EXIT_TRAVERSED);
    rover_map_set_exit(
        p_map, p_mission->leg.next_target_node,
        (map_direction_t) (((unsigned) p_mission->leg.direction + 2u) % 4u),
        MAP_EXIT_TRAVERSED);
    if (b_wall)
    {
        rover_map_set_exit(p_map, p_mission->leg.next_target_node,
                           (map_direction_t) p_mission->leg.direction,
                           MAP_EXIT_BLOCKED);
    }
    p_drive->active        = false;
    p_drive->search_active = false;
    p_drive->result        = "ARRIVED";
    return true;
}

static void
verify_obstacle (onboard_drive_t * p_drive, mission_t * p_mission,
                 rover_map_t * p_map, const target_selector_t * p_selection,
                 const onboard_drive_inputs_t * p_inputs, uint32_t now)
{
    if ((uint32_t) (now - p_drive->braked_ms) < ONBOARD_DRIVE_SETTLE_MS ||
        (int32_t) (p_inputs->sensor_received_ms - p_drive->braked_ms -
                   ONBOARD_DRIVE_SETTLE_MS) < 0 ||
        p_inputs->sensor_received_ms == p_drive->last_obstacle_ms)
    {
        return;
    }
    p_drive->last_obstacle_ms = p_inputs->sensor_received_ms;
    if (!front_obstacle_detected(p_inputs->front_mm, p_inputs->front_fault))
    {
        p_drive->obstacle_frames = p_drive->verification_frames = 0;
        p_drive->result = "WAITING_FOR_OBSTACLE_CONFIRMATION";
        if (!input_problem(p_selection, p_inputs, now) &&
            p_selection->reference_available && p_drive->reference_model_valid)
        {
            p_drive->using_obstacle = false;
            p_drive->braked_ms      = now;
            mission_recovery_phase(p_mission, LEG_BRAKING, now);
            p_drive->result = "RECHECKING_VISUAL_ARRIVAL";
        }
        return;
    }
    p_drive->verification_frames = ++p_drive->obstacle_frames;
    if (p_drive->obstacle_frames >= ONBOARD_DRIVE_OBSTACLE_CONFIRM_FRAMES)
    {
        commit_arrival(p_drive, p_mission, p_map, true, now);
    }
}

static const char *
search_camera_problem (const onboard_drive_inputs_t * p_inputs)
{
    if (!p_inputs->camera_present)
    {
        return "CAMERA_UART_MISSING";
    }
    if (p_inputs->camera_status != VISION_STATUS_OK &&
        p_inputs->camera_status != VISION_STATUS_NO_MARKER)
    {
        return p_inputs->camera_status == VISION_STATUS_STALLED
                   ? "CAMERA_PROCESSING_STALLED"
                   : "CAMERA_LOST";
    }
    if (p_inputs->camera_packet_age_ms > TARGET_UART_MAX_AGE_MS)
    {
        return "CAMERA_PACKET_STALE";
    }
    if (p_inputs->camera_image_age_ms > ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
    {
        return "CAMERA_IMAGE_STALE";
    }
    return NULL;
}

static bool
start_search (onboard_drive_t * p_drive, mission_t * p_mission,
              target_selector_t * p_selection, uint32_t now)
{
    // Entry and camera/alignment resumes use the same front obstacle guard.
    // The caller checks motion_problem and handles obstacle arrival before
    // search. No-return (0) permits bounded search, never a clear-exit or
    // arrival fact. A separate clearance gate must not cancel the journey.
    if (motor_auto_drive_leased(ONBOARD_DRIVE_SEARCH_PCT,
                                ONBOARD_DRIVE_LEASE_MS) != MOTOR_OK)
    {
        recovery_failure(p_drive, p_mission, "SEARCH_MOTOR_REFUSED", now);
        return false;
    }
    p_drive->search_active = p_drive->search_used = true;
    p_drive->last_refresh_ms                      = now;
    p_drive->estimate_valid                       = false;
    p_drive->recovery_frames                      = 0;
    p_drive->input_reason                         = "NONE";
    p_selection->allow_late_reference             = true;
    mission_search_phase(p_mission, LEG_SEARCHING_REFERENCE, now);
    p_drive->result = "SEARCHING_FOR_REFERENCE";
    return true;
}

// Fresh NO_MARKER images keep the slow search alive. Transport/processing gaps
// brake and require post-hold images; they never authorise unobserved travel.
typedef struct
{
    const char * p_problem;
    _Bool        return_value;
} search_tick_context_t;

/** @brief Execute the prepare stage of search_tick. */
static bool
search_tick_prepare (onboard_drive_t * p_drive, mission_t * p_mission,
                     const onboard_drive_inputs_t * p_inputs, uint32_t now,
                     search_tick_context_t * p_context)
{

    if (p_drive->search_driving_ms >= ONBOARD_DRIVE_SEARCH_MAX_MS)
    {

        p_context->return_value =
            recovery_failure(p_drive, p_mission, "REFERENCE_SEARCH_LIMIT", now);

        return true;
    }

    p_context->p_problem = search_camera_problem(p_inputs);
    if (p_context->p_problem)
    {
        if (p_mission->leg.phase == LEG_SEARCHING_REFERENCE)
        {
            motor_emergency_stop();
            mission_search_phase(p_mission, LEG_HOLDING, now);
            p_drive->search_hold_ms    = now;
            p_drive->recovery_frames   = 0;
            p_drive->search_generation = p_inputs->camera_generation;
        }
        p_drive->estimate_valid = false;
        p_drive->input_reason   = p_context->p_problem;
        p_drive->result         = "HOLDING_FOR_SEARCH_CAMERA";
        p_context->return_value = true;
        return true;
    }
    return false;
}

/** @brief Execute the inputs stage of search_tick. */
static bool
search_tick_inputs (onboard_drive_t * p_drive, mission_t * p_mission,
                    target_selector_t *            p_selection,
                    const onboard_drive_inputs_t * p_inputs, uint32_t now,
                    bool * p_reference_ready, search_tick_context_t * p_context)
{

    if (p_mission->leg.phase == LEG_HOLDING)
    {
        if ((int32_t) (now - p_inputs->camera_image_age_ms -
                       p_drive->search_hold_ms) < 0 ||
            p_inputs->camera_generation == p_drive->search_generation)
        {
            p_context->return_value = true;
            return true;
        }
        p_drive->search_generation = p_inputs->camera_generation;
        if (++p_drive->recovery_frames < ONBOARD_DRIVE_RECOVERY_FRAMES)
        {
            p_context->return_value = true;
            return true;
        }
        if (motor_auto_drive_leased(ONBOARD_DRIVE_SEARCH_PCT,
                                    ONBOARD_DRIVE_LEASE_MS) != MOTOR_OK)
        {

            p_context->return_value = recovery_failure(
                p_drive, p_mission, "SEARCH_MOTOR_REFUSED", now);

            return true;
        }
        p_drive->last_refresh_ms = now;
        mission_search_phase(p_mission, LEG_SEARCHING_REFERENCE, now);
    }
    if (p_selection->reference_available &&
        !input_problem(p_selection, p_inputs, now))
    {
        p_context->p_problem = track(p_drive, p_mission, p_selection, false);
        if (p_context->p_problem)
        {

            p_context->return_value =
                recovery_failure(p_drive, p_mission, p_context->p_problem, now);

            return true;
        }
        p_drive->search_active            = false;
        p_selection->allow_late_reference = false;
        mission_search_phase(p_mission, LEG_APPROACHING, now);
        *p_reference_ready      = true;
        p_drive->input_reason   = "NONE";
        p_drive->result         = "LATE_REFERENCE_LOCKED";
        p_context->return_value = false;
        return true;
    }
    return false;
}

/** @brief Execute the motion stage of search_tick. */
static bool
search_tick_motion (onboard_drive_t * p_drive, mission_t * p_mission,
                    target_selector_t *            p_selection,
                    const onboard_drive_inputs_t * p_inputs, uint32_t now,
                    search_tick_context_t * p_context)
{

    if (p_selection->available && !input_problem(p_selection, p_inputs, now))
    {
        p_drive->last_target_range_mm = p_selection->range_mm;
    }
    p_drive->estimate_valid = false;

    p_drive->input_reason =
        p_inputs->front_mm ? "NONE" : "FRONT_DISTANCE_UNKNOWN";

    p_drive->result = p_inputs->front_mm ? "SEARCHING_FOR_REFERENCE"
                                         : "SEARCHING_WITH_NO_FRONT_RETURN";

    if (motor_auto_status() != MOTOR_AUTO_RUNNING)
    {

        p_context->return_value =
            recovery_failure(p_drive, p_mission, "MOTOR_STOPPED", now);

        return true;
    }
    if ((uint32_t) (now - p_drive->last_refresh_ms) >= ONBOARD_DRIVE_REFRESH_MS)
    {
        if (!motor_auto_renew_lease())
        {

            p_context->return_value = recovery_failure(
                p_drive, p_mission, "DRIVE_LEASE_EXPIRED", now);

            return true;
        }
        p_drive->last_refresh_ms = now;
    }
    p_context->return_value = true;
    return true;
}

static bool
search_tick (onboard_drive_t * p_drive, mission_t * p_mission,
             target_selector_t *            p_selection,
             const onboard_drive_inputs_t * p_inputs, uint32_t now,
             bool * p_reference_ready)
{
    search_tick_context_t context = {0};
    if (search_tick_prepare(p_drive, p_mission, p_inputs, now, &context))
    {
        return context.return_value;
    }
    if (search_tick_inputs(p_drive, p_mission, p_selection, p_inputs, now,
                           p_reference_ready, &context))
    {
        return context.return_value;
    }
    if (search_tick_motion(p_drive, p_mission, p_selection, p_inputs, now,
                           &context))
    {
        return context.return_value;
    }
    return context.return_value;
}

static void
alignment_pose (onboard_drive_t *         p_drive,
                const target_selector_t * p_selection, bool b_reference,
                uint32_t now)
{
    p_drive->b_alignment_reference = b_reference;
    p_drive->alignment_rotating    = false;
    p_drive->alignment_stopped_ms  = now;
    p_drive->alignment_generation  = p_selection->sample_generation;
    p_drive->alignment_wait_ms     = 0;
    p_drive->alignment_frames      = 0;
    p_drive->verification_frames   = 0;
    p_drive->estimate_valid        = false;
    p_drive->result                = p_drive->input_reason =
        b_reference ? "REFERENCE_NOT_ALIGNED" : "TARGET_NOT_ALIGNED";
}

static void
begin_alignment (onboard_drive_t * p_drive, mission_t * p_mission,
                 target_selector_t * p_selection, uint32_t now)
{
    motor_emergency_stop();
    if (!mission_alignment_phase(p_mission, true, now))
    {
        recovery_failure(p_drive, p_mission, "TARGET_ALIGNMENT_STATE_INVALID",
                         now);
        return;
    }
    p_drive->aligning           = true;
    p_drive->reference_recovery = false;
    alignment_pose(p_drive, p_selection,
                   p_selection->reference_available &&
                       (!p_selection->available ||
                        (centred(p_selection->bearing_cdeg) &&
                         !centred(p_selection->reference_bearing_cdeg))),
                   now);
    p_selection->allow_late_reference = false;
}

/** @brief Recover a missing alignment target using the same locked journey. */
static bool
begin_alignment_reverse (onboard_drive_t * p_drive, mission_t * p_mission,
                         target_selector_t * p_selection, uint32_t now)
{
    motor_emergency_stop();
    if (!mission_alignment_phase(p_mission, false, now) ||
        !mission_reference_phase(p_mission, LEG_HOLDING, now))
    {
        return recovery_failure(p_drive, p_mission,
                                "TARGET_ALIGNMENT_STATE_INVALID", now);
    }
    p_drive->aligning             = false;
    p_drive->alignment_rotating   = false;
    p_drive->alignment_wait_ms    = 0;
    p_drive->alignment_frames     = 0;
    p_drive->search_active        = false;
    p_drive->reference_recovery   = true;
    p_drive->reference_missing    = true;
    p_drive->reference_wait_ms    = now;
    p_drive->reference_missing_ms = now;
    p_drive->hold_started_ms      = now;
    p_drive->recovery_generation  = p_selection->sample_generation;
    p_drive->verification_frames  = 0;
    p_drive->input_reason         = p_drive->b_alignment_reference
                                        ? "REFERENCE_ALIGNMENT_MARKER_LOST"
                                        : "TARGET_ALIGNMENT_TARGET_LOST";
    return begin_reverse_search(p_drive, p_mission, now);
}

// Rotate using a stationary image, then verify using distinct post-stop images.
// Repeat corrections until centred; never steer from an old bearing captured
// during a previous turn. Camera, motion and journey guards remain independent.
typedef struct
{
    const char * p_problem;
    uint32_t     image_ms;
    int          correction;
    float        goal;
    int          motor_heading;
    int          bearing_cdeg;
    bool         b_marker_available;
    _Bool        return_value;
} alignment_tick_context_t;

/** @brief Execute the prepare stage of alignment_tick. */
static bool
alignment_tick_prepare (onboard_drive_t * p_drive, mission_t * p_mission,
                        target_selector_t *            p_selection,
                        const onboard_drive_inputs_t * p_inputs, uint32_t now,
                        alignment_tick_context_t * p_context)
{

    if (p_selection->leg_id != p_drive->leg_id ||
        p_selection->marker != p_mission->leg.next_target_node ||
        (p_selection->reference_available &&
         p_selection->reference_marker !=
             p_mission->leg.arrival_reference_node))
    {

        p_context->return_value =
            recovery_failure(p_drive, p_mission, "TRACKING_ID_CHANGED", now);

        return true;
    }

    p_context->p_problem = input_problem(p_selection, p_inputs, now);
    bool b_reference     = p_drive->b_alignment_reference;
    if (!p_context->p_problem &&
        ((b_reference && !p_selection->reference_available &&
          p_selection->available) ||
         (!b_reference && !p_selection->available &&
          p_selection->reference_available)))
    {
        // A different locked landmark is now visible. Stop and obtain new
        // stationary images before steering from that marker's bearing.
        motor_emergency_stop();
        b_reference = !b_reference;
        alignment_pose(p_drive, p_selection, b_reference, now);
    }
    p_context->b_marker_available =
        b_reference ? p_selection->reference_available : p_selection->available;
    p_context->bearing_cdeg = b_reference ? p_selection->reference_bearing_cdeg
                                          : p_selection->bearing_cdeg;
    if (!p_context->p_problem && !p_context->b_marker_available)
    {
        p_context->p_problem = b_reference ? "REFERENCE_ALIGNMENT_MARKER_LOST"
                                           : "TARGET_ALIGNMENT_TARGET_LOST";
    }
    return false;
}

/** @brief Execute the inputs stage of alignment_tick. */
static bool
alignment_tick_inputs (onboard_drive_t * p_drive, mission_t * p_mission,
                       target_selector_t * p_selection, uint32_t now,
                       alignment_tick_context_t * p_context)
{

    if (p_context->p_problem)
    {

        bool b_age_gap =
            strcmp(p_context->p_problem, "CAMERA_PACKET_STALE") == 0 ||
            strcmp(p_context->p_problem, "CAMERA_ACCEPTED_IMAGE_STALE") == 0 ||
            strcmp(p_context->p_problem, "CAMERA_IMAGE_STALE") == 0 ||
            strcmp(p_context->p_problem, "CAMERA_DATA_UNUSABLE") == 0;

        // Like arrival verification, a stopped camera's cached frame aging out
        // is not a new contradictory observation. Preserve distinct good frames
        // while waiting for recovery; never count cached images twice.
        if (!b_age_gap || p_drive->alignment_rotating)
        {
            p_drive->alignment_frames = 0;
        }
        if (p_drive->alignment_rotating)
        {
            motor_emergency_stop();
            p_drive->alignment_rotating   = false;
            p_drive->alignment_stopped_ms = now;
            p_drive->alignment_generation = p_selection->sample_generation;
        }
        if (!p_drive->alignment_wait_ms)
        {
            p_drive->alignment_wait_ms = now;
        }
        p_drive->input_reason = p_context->p_problem;
        p_drive->result       = "WAITING_FOR_ALIGNMENT_CAMERA";
        if ((uint32_t) (now - p_drive->alignment_wait_ms) >=
            ONBOARD_DRIVE_CAMERA_RECOVERY_MS)
        {
            // Live observations without the target require a position search,
            // not cancellation. Stale/unhealthy camera data never starts it.
            if (!p_context->b_marker_available &&
                (strcmp(p_context->p_problem, "TARGET_ALIGNMENT_TARGET_LOST") ==
                     0 ||
                 strcmp(p_context->p_problem,
                        "REFERENCE_ALIGNMENT_MARKER_LOST") == 0 ||
                 strcmp(p_context->p_problem, "CAMERA_NO_MARKERS") == 0))
            {
                p_context->return_value = begin_alignment_reverse(
                    p_drive, p_mission, p_selection, now);
                return true;
            }
            p_context->return_value = recovery_failure(
                p_drive, p_mission,
                p_selection->available ? "TARGET_ALIGNMENT_CAMERA_TIMEOUT"
                                       : "TARGET_ALIGNMENT_TARGET_LOST",
                now);

            return true;
        }
        p_context->return_value = true;
        return true;
    }
    p_drive->alignment_wait_ms = 0;

    p_drive->input_reason = centred(p_context->bearing_cdeg) ? "NONE"
                            : p_drive->b_alignment_reference
                                ? "REFERENCE_NOT_ALIGNED"
                                : "TARGET_NOT_ALIGNED";

    return false;
}

/** @brief Execute the motion stage of alignment_tick. */
static bool
alignment_tick_motion (onboard_drive_t * p_drive, mission_t * p_mission,
                       target_selector_t * p_selection, uint32_t now,
                       alignment_tick_context_t * p_context)
{

    if (p_drive->alignment_rotating)
    {
        motor_auto_status_t state = motor_auto_status();
        if (state == MOTOR_AUTO_HEADING_REACHED)
        {
            motor_emergency_stop();
            p_drive->alignment_rotating   = false;
            p_drive->alignment_stopped_ms = now;
            p_drive->alignment_generation = p_selection->sample_generation;
            p_drive->alignment_frames     = 0;
            p_drive->result               = "WAITING_FOR_ALIGNMENT_IMAGE";
            p_context->return_value       = true;
            return true;
        }
        if (state != MOTOR_AUTO_RUNNING)
        {

            p_context->return_value = recovery_failure(
                p_drive, p_mission,
                state == MOTOR_AUTO_STOPPED ? "MOTOR_STOPPED"
                                            : "TARGET_ALIGNMENT_TURN_FAILED",
                now);

            return true;
        }
        if ((uint32_t) (now - p_drive->last_refresh_ms) >=
            ONBOARD_DRIVE_REFRESH_MS)
        {
            if (!motor_auto_renew_lease())
            {

                p_context->return_value = recovery_failure(
                    p_drive, p_mission, "DRIVE_LEASE_EXPIRED", now);

                return true;
            }
            p_drive->last_refresh_ms = now;
        }
        p_drive->result         = p_drive->b_alignment_reference
                                      ? "ROTATING_TO_REFERENCE"
                                      : "ROTATING_TO_TARGET";
        p_context->return_value = true;
        return true;
    }
    p_drive->result = "WAITING_FOR_ALIGNMENT_IMAGE";

    p_context->image_ms =
        p_selection->sample_received_ms - p_selection->sample_age_ms;

    return false;
}

/** @brief Execute the camera stage of alignment_tick. */
static bool
alignment_tick_camera (onboard_drive_t * p_drive, mission_t * p_mission,
                       target_selector_t *            p_selection,
                       const onboard_drive_inputs_t * p_inputs, uint32_t now,
                       bool * p_resuming, alignment_tick_context_t * p_context)
{

    if ((uint32_t) (now - p_drive->alignment_stopped_ms) <
            ONBOARD_DRIVE_SETTLE_MS ||
        (int32_t) (p_context->image_ms - (p_drive->alignment_stopped_ms +
                                          ONBOARD_DRIVE_SETTLE_MS)) < 0 ||
        p_selection->sample_generation == p_drive->alignment_generation)
    {
        p_context->return_value = true;
        return true;
    }
    p_drive->alignment_generation = p_selection->sample_generation;
    if (centred(p_context->bearing_cdeg))
    {
        p_drive->result = p_drive->b_alignment_reference
                              ? "VERIFYING_REFERENCE_ALIGNMENT"
                              : "VERIFYING_TARGET_ALIGNMENT";
        if (++p_drive->alignment_frames <
            ONBOARD_DRIVE_ALIGNMENT_CONFIRM_FRAMES)
        {
            p_context->return_value = true;
            return true;
        }
        if (!mission_alignment_phase(p_mission, false, now))
        {

            p_context->return_value = recovery_failure(
                p_drive, p_mission, "TARGET_ALIGNMENT_STATE_INVALID", now);

            return true;
        }
        p_drive->aligning                 = false;
        p_selection->allow_late_reference = p_drive->search_active;
        p_drive->input_reason             = "NONE";
        p_drive->result = p_drive->b_alignment_reference ? "REFERENCE_ALIGNED"
                                                         : "TARGET_ALIGNED";
        *p_resuming     = true;
        p_context->return_value = false;
        return true; // Ordinary camera/reference/arrival checks run before
                     // forward motion.
    }
    p_drive->alignment_frames = 0;
    if (!isfinite(p_inputs->heading_deg) ||
        !isfinite(p_inputs->heading_offset_deg) || p_inputs->heading_deg < 0 ||
        p_inputs->heading_deg >= 360 || p_inputs->heading_offset_deg < 0 ||
        p_inputs->heading_offset_deg >= 360)
    {

        p_context->return_value =
            recovery_failure(p_drive, p_mission, "HEADING_UNAVAILABLE", now);

        return true;
    }

    p_context->correction = p_context->bearing_cdeg;
    if (p_context->correction > ONBOARD_DRIVE_ALIGNMENT_STEP_CDEG)
    {
        p_context->correction = ONBOARD_DRIVE_ALIGNMENT_STEP_CDEG;
    }
    return false;
}

/** @brief Execute the tracking stage of alignment_tick. */
static bool
alignment_tick_tracking (onboard_drive_t * p_drive, mission_t * p_mission,
                         const onboard_drive_inputs_t * p_inputs, uint32_t now,
                         alignment_tick_context_t * p_context)
{

    if (p_context->correction < -ONBOARD_DRIVE_ALIGNMENT_STEP_CDEG)
    {
        p_context->correction = -ONBOARD_DRIVE_ALIGNMENT_STEP_CDEG;
    }
    // Positive camera bearing is right; motor headings increase clockwise.

    p_context->goal = p_inputs->heading_deg - p_inputs->heading_offset_deg +
                      (float) p_context->correction / 100.0f;

    p_context->motor_heading =
        (int) lroundf(fmodf(p_context->goal + 360.0f, 360.0f)) % 360;

    if (motor_auto_rotate_leased(p_context->motor_heading,
                                 ONBOARD_DRIVE_LEASE_MS) != MOTOR_OK)
    {

        p_context->return_value = recovery_failure(
            p_drive, p_mission, "TARGET_ALIGNMENT_MOTOR_REFUSED", now);

        return true;
    }
    p_drive->alignment_rotating = true;
    p_drive->last_refresh_ms    = now;
    p_drive->result = p_drive->b_alignment_reference ? "ROTATING_TO_REFERENCE"
                                                     : "ROTATING_TO_TARGET";
    p_context->return_value = true;
    return true;
}

static bool
alignment_tick (onboard_drive_t * p_drive, mission_t * p_mission,
                target_selector_t *            p_selection,
                const onboard_drive_inputs_t * p_inputs, uint32_t now,
                bool * p_resuming)
{
    alignment_tick_context_t context = {0};
    if (alignment_tick_prepare(p_drive, p_mission, p_selection, p_inputs, now,
                               &context))
    {
        return context.return_value;
    }
    if (alignment_tick_inputs(p_drive, p_mission, p_selection, now, &context))
    {
        return context.return_value;
    }
    if (alignment_tick_motion(p_drive, p_mission, p_selection, now, &context))
    {
        return context.return_value;
    }
    if (alignment_tick_camera(p_drive, p_mission, p_selection, p_inputs, now,
                              p_resuming, &context))
    {
        return context.return_value;
    }
    if (alignment_tick_tracking(p_drive, p_mission, p_inputs, now, &context))
    {
        return context.return_value;
    }
    return context.return_value;
}

static const char *
camera_wait_problem (const target_selector_t *      p_selection,
                     const onboard_drive_inputs_t * p_inputs, uint32_t now)
{
    // Health reports are recoverable input outages, just like stale frames.
    // A live UART health packet is not a fresh camera observation.
    if (!p_inputs->camera_present)
    {
        return "CAMERA_UART_MISSING";
    }
    if (p_inputs->camera_status == VISION_STATUS_STALLED)
    {
        return "CAMERA_PROCESSING_STALLED";
    }
    if (p_inputs->camera_status == VISION_STATUS_CAMERA_LOST)
    {
        return "CAMERA_LOST";
    }
    if (p_inputs->camera_status != VISION_STATUS_OK &&
        p_inputs->camera_status != VISION_STATUS_NO_MARKER)
    {
        return "CAMERA_DATA_UNUSABLE";
    }
    if (p_inputs->camera_packet_age_ms > TARGET_UART_MAX_AGE_MS)
    {
        return "CAMERA_PACKET_STALE";
    }
    if (p_inputs->camera_image_age_ms > ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
    {
        return "CAMERA_IMAGE_STALE";
    }
    if (p_inputs->camera_status == VISION_STATUS_OK && !p_inputs->camera_fresh)
    {
        return "CAMERA_DATA_UNUSABLE";
    }
    if (p_inputs->camera_status == VISION_STATUS_OK &&
        p_selection->sample_generation)
    {
        if ((uint32_t) (now - p_selection->sample_received_ms) >
            TARGET_UART_MAX_AGE_MS)
        {
            return "CAMERA_ACCEPTED_IMAGE_STALE";
        }
        if ((uint32_t) (now - p_selection->sample_received_ms) +
                p_selection->sample_age_ms >
            ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
        {
            return "CAMERA_IMAGE_STALE";
        }
    }
    return NULL;
}

static uint32_t
camera_generation (const target_selector_t *      p_selection,
                   const onboard_drive_inputs_t * p_inputs)
{
    return p_inputs->camera_status == VISION_STATUS_NO_MARKER
               ? p_inputs->camera_generation
               : p_selection->sample_generation;
}

// Preserve the leg while stopped. Camera-wait time is excluded from the journey
// allowance and must not expire phase-local recovery/verification windows.
typedef struct
{
    uint32_t image_ms;
    uint32_t generation;
    uint32_t waited;
    _Bool    return_value;
} wait_for_camera_context_t;

/** @brief Execute the prepare stage of wait_for_camera. */
static bool
wait_for_camera_prepare (onboard_drive_t * p_drive, mission_t * p_mission,
                         target_selector_t *            p_selection,
                         const onboard_drive_inputs_t * p_inputs, uint32_t now,
                         wait_for_camera_context_t * p_context)
{

    if (!p_drive->camera_waiting)
    {
        if (p_mission->leg.phase == LEG_CORRECTING)
        {
            motor_emergency_stop();
            p_drive->correcting = false;
            mission_recovery_phase(p_mission, LEG_BRAKING, now);
            p_drive->braked_ms           = now;
            p_drive->verification_frames = 0;
        }
        p_drive->camera_wait_pose_ms = now;
        if (p_mission->leg.phase == LEG_BRAKING ||
            p_mission->leg.phase == LEG_VERIFYING)
        {
            p_drive->camera_wait_pose_ms = p_drive->braked_ms;
        }
        else if (p_mission->leg.phase == LEG_HOLDING)
        {

            p_drive->camera_wait_pose_ms =
                p_drive->reference_recovery ? p_drive->reference_wait_ms
                : p_drive->search_active    ? p_drive->search_hold_ms
                                            : p_drive->hold_started_ms;
        }
        else if (p_mission->leg.phase == LEG_REACQUIRING)
        {
            p_drive->camera_wait_pose_ms = p_drive->backup_stopped_ms;
        }
        else if (p_mission->leg.phase == LEG_ALIGNING &&
                 !p_drive->alignment_rotating)
        {
            p_drive->camera_wait_pose_ms = p_drive->alignment_stopped_ms;
        }
        if (p_mission->leg.phase == LEG_BACKING_UP)
        {
            finish_backup(p_drive, p_mission, p_selection, now);
        }
        else if (p_mission->leg.phase == LEG_APPROACHING ||
                 p_mission->leg.phase == LEG_SEARCHING_REFERENCE ||
                 p_drive->alignment_rotating)
        {
            motor_emergency_stop();
        }
        if (p_drive->alignment_rotating)
        {
            p_drive->alignment_rotating   = false;
            p_drive->alignment_frames     = 0;
            p_drive->alignment_stopped_ms = now;
            p_drive->alignment_generation = p_selection->sample_generation;
        }
        p_drive->camera_resume_phase = p_mission->leg.phase;
        if (!mission_camera_wait_phase(p_mission, LEG_WAITING_CAMERA, now))
        {

            p_context->return_value = recovery_failure(
                p_drive, p_mission, "CAMERA_WAIT_STATE_INVALID", now);

            return true;
        }
        p_drive->camera_waiting         = true;
        p_drive->camera_wait_started_ms = now;
        p_drive->camera_wait_frames     = 0;

        p_drive->camera_wait_generation =
            camera_generation(p_selection, p_inputs);
    }
    return false;
}

/** @brief Execute the inputs stage of wait_for_camera. */
static bool
wait_for_camera_inputs (onboard_drive_t * p_drive, mission_t * p_mission,
                        target_selector_t *            p_selection,
                        const onboard_drive_inputs_t * p_inputs,
                        const char * p_age_problem, uint32_t now,
                        wait_for_camera_context_t * p_context)
{

    p_drive->estimate_valid = false;
    if (p_age_problem)
    {
        p_drive->input_reason   = p_age_problem;
        p_drive->result         = "WAITING_FOR_FRESH_CAMERA";
        p_context->return_value = true;
        return true;
    }
    if (!p_inputs->camera_present ||
        (p_inputs->camera_status != VISION_STATUS_OK &&
         p_inputs->camera_status != VISION_STATUS_NO_MARKER))
    {

        p_context->return_value =
            recovery_failure(p_drive, p_mission,
                             !p_inputs->camera_present ? "CAMERA_UART_MISSING"
                             : p_inputs->camera_status == VISION_STATUS_STALLED
                                 ? "CAMERA_PROCESSING_STALLED"
                                 : "CAMERA_LOST",
                             now);

        return true;
    }
    if (p_inputs->camera_status == VISION_STATUS_OK && !p_inputs->camera_fresh)
    {

        p_context->return_value =
            recovery_failure(p_drive, p_mission, "CAMERA_DATA_UNUSABLE", now);

        return true;
    }
    if (p_inputs->camera_status == VISION_STATUS_OK && p_selection->available &&
        p_selection->leg_id == p_drive->leg_id &&
        p_selection->marker == p_mission->leg.next_target_node)
    {
        p_drive->last_target_range_mm = p_selection->range_mm;
    }

    p_context->image_ms =
        p_inputs->camera_status == VISION_STATUS_NO_MARKER
            ? now - p_inputs->camera_image_age_ms
            : p_selection->sample_received_ms - p_selection->sample_age_ms;

    p_drive->input_reason = "NONE";
    p_drive->result       = "WAITING_FOR_POST_STOP_CAMERA";
    if ((int32_t) (p_context->image_ms - (p_drive->camera_wait_pose_ms +
                                          ONBOARD_DRIVE_SETTLE_MS)) < 0)
    {
        p_context->return_value = true;
        return true;
    }

    p_context->generation = camera_generation(p_selection, p_inputs);
    return false;
}

/** @brief Execute the motion stage of wait_for_camera. */
static bool
wait_for_camera_motion (onboard_drive_t * p_drive, uint32_t now,
                        wait_for_camera_context_t * p_context)
{

    if (p_context->generation == p_drive->camera_wait_generation)
    {
        p_context->return_value = true;
        return true;
    }
    p_drive->camera_wait_generation = p_context->generation;
    p_drive->result                 = "CONFIRMING_FRESH_CAMERA";
    if (++p_drive->camera_wait_frames < ONBOARD_DRIVE_RECOVERY_FRAMES)
    {
        p_context->return_value = true;
        return true;
    }

    p_context->waited = now - p_drive->camera_wait_started_ms;
    // Freeze phase-local deadlines, not started_ms or accumulated driving time.
    if (p_drive->camera_resume_phase == LEG_HOLDING)
    {
        p_drive->hold_started_ms += p_context->waited;
    }
    if (p_drive->camera_resume_phase == LEG_BRAKING ||
        p_drive->camera_resume_phase == LEG_VERIFYING)
    {
        p_drive->braked_ms += p_context->waited;
    }
    if (p_drive->search_active)
    {
        p_drive->search_hold_ms += p_context->waited;
    }
    if (p_drive->aligning)
    {
        p_drive->alignment_wait_ms = 0;
    }
    if (p_drive->reference_recovery)
    {
        p_drive->reference_wait_ms += p_context->waited;
        if (p_drive->reference_missing)
        {
            p_drive->reference_missing_ms += p_context->waited;
        }
        if (p_drive->backup_used)
        {
            p_drive->backup_stopped_ms += p_context->waited;
        }
    }
    return false;
}

/** @brief Execute the camera stage of wait_for_camera. */
static bool
wait_for_camera_camera (onboard_drive_t * p_drive, mission_t * p_mission,
                        uint32_t now, bool * p_resuming,
                        wait_for_camera_context_t * p_context)
{

    if (!mission_camera_wait_phase(p_mission, p_drive->camera_resume_phase,
                                   now))
    {

        p_context->return_value = recovery_failure(
            p_drive, p_mission, "CAMERA_WAIT_STATE_INVALID", now);

        return true;
    }
    p_drive->camera_waiting = false;
    p_drive->result         = "RESUMED_AFTER_CAMERA_WAIT";
    *p_resuming             = true;
    p_context->return_value = false;
    return true;
}

static bool
wait_for_camera (onboard_drive_t * p_drive, mission_t * p_mission,
                 target_selector_t *            p_selection,
                 const onboard_drive_inputs_t * p_inputs,
                 const char * p_age_problem, uint32_t now, bool * p_resuming)
{
    wait_for_camera_context_t context = {0};
    if (wait_for_camera_prepare(p_drive, p_mission, p_selection, p_inputs, now,
                                &context))
    {
        return context.return_value;
    }
    if (wait_for_camera_inputs(p_drive, p_mission, p_selection, p_inputs,
                               p_age_problem, now, &context))
    {
        return context.return_value;
    }
    if (wait_for_camera_motion(p_drive, now, &context))
    {
        return context.return_value;
    }
    if (wait_for_camera_camera(p_drive, p_mission, now, p_resuming, &context))
    {
        return context.return_value;
    }
    return context.return_value;
}

static void
begin_correction (onboard_drive_t * p_drive, mission_t * p_mission,
                  const target_selector_t * p_selection, uint32_t now)
{
    motor_emergency_stop();
    p_drive->verification_frames    = 0;
    p_drive->last_verify_generation = p_selection->sample_generation;
    mission_recovery_phase(p_mission, LEG_CORRECTING, now);

    int speed = p_drive->remaining_mm > 0 ? ONBOARD_DRIVE_CORRECTION_PCT
                                          : -ONBOARD_DRIVE_CORRECTION_PCT;

    if (motor_auto_drive_leased(speed, ONBOARD_DRIVE_LEASE_MS) != MOTOR_OK)
    {
        begin_input_wait(p_drive, p_mission, "MOTOR_STOPPED", now);
        return;
    }
    p_drive->correcting            = true;
    p_drive->correction_started_ms = p_drive->last_refresh_ms = now;

    p_drive->result =
        speed > 0 ? "CORRECTING_SHORT_ARRIVAL" : "CORRECTING_OVERSHOOT";
}

typedef struct
{
    uint32_t     elapsed;
    const char * p_problem;
    uint32_t     wall_limit;
    _Bool        b_front_stop;
    _Bool        b_camera_resuming;
    const char * p_age_problem;
    _Bool        b_alignment_resuming;
    _Bool        b_search_reference_ready;
    _Bool        b_reference_resuming;
    _Bool        b_resuming;
} onboard_drive_tick_context_t;

/** @brief Execute the prepare stage of onboard_drive_tick. */
static bool
onboard_drive_tick_prepare (onboard_drive_t * p_drive, mission_t * p_mission,
                            target_selector_t *            p_selection,
                            const onboard_drive_inputs_t * p_inputs,
                            uint32_t                       now,
                            onboard_drive_tick_context_t * p_context)
{

    if (!p_drive->active)
    {
        return true;
    }

    p_context->elapsed    = now - p_drive->last_tick_ms;
    p_drive->last_tick_ms = now;
    if (p_drive->camera_waiting)
    {
        p_drive->camera_waited_ms += p_context->elapsed;
    }
    if (!p_drive->b_departure_pending &&
        p_mission->leg.phase == LEG_APPROACHING && !p_drive->search_used)
    {
        p_drive->driving_ms += p_context->elapsed;
    }
    if (!p_drive->b_departure_pending &&
        (p_mission->leg.phase == LEG_SEARCHING_REFERENCE ||
         (p_mission->leg.phase == LEG_APPROACHING && p_drive->search_used)))
    {
        p_drive->search_driving_ms += p_context->elapsed;
    }
    note_inputs(p_drive, p_selection, p_inputs, now);
    if (p_mission->state != MISSION_EXPLORING || p_mission->test_mode ||
        !p_mission->leg.configured || p_mission->leg.leg_id != p_drive->leg_id)
    {
        motor_emergency_stop();
        if (p_mission->state == MISSION_EXPLORING)
        {
            mission_pause(p_mission, MISSION_REASON_MOTOR_STOPPED, now);
        }
        p_drive->active         = false;
        p_drive->estimate_valid = false;

        p_drive->result =
            p_mission->reason == MISSION_REASON_LINK_LOST    ? "LINK_LOST"
            : p_mission->reason == MISSION_REASON_FRONT_STOP ? "FRONT_BLOCKED"
            : p_mission->reason == MISSION_REASON_ESTOP      ? "SAFETY_LOCKED"
                                                             : "CANCELLED";

        return true;
    }

    p_context->p_problem = motion_problem(p_inputs, now);
    if (waitable_input(p_context->p_problem))
    {
        begin_input_wait(p_drive, p_mission, p_context->p_problem, now);
        return true;
    }
    return false;
}

/** @brief Execute the inputs stage of onboard_drive_tick. */
static bool
onboard_drive_tick_inputs (onboard_drive_t * p_drive, mission_t * p_mission,
                           target_selector_t *            p_selection,
                           const onboard_drive_inputs_t * p_inputs,
                           uint32_t                       now,
                           onboard_drive_tick_context_t * p_context)
{

    if (p_drive->input_waiting &&
        (!p_context->p_problem ||
         strcmp(p_context->p_problem, "FRONT_BLOCKED") == 0))
    {
        if ((uint32_t) (now - p_drive->input_wait_started_ms) <
                ONBOARD_DRIVE_SETTLE_MS ||
            (int32_t) (p_inputs->sensor_received_ms -
                       p_drive->input_wait_started_ms -
                       ONBOARD_DRIVE_SETTLE_MS) < 0)
        {
            return true;
        }
        uint32_t waited = now - p_drive->input_wait_started_ms;
        p_drive->started_ms +=
            waited; // Input outages never exhaust the journey allowance.
        p_drive->hold_started_ms += waited;
        p_drive->search_hold_ms += waited;
        p_drive->reference_wait_ms += waited;
        p_drive->reference_missing_ms += waited;
        p_drive->backup_stopped_ms += waited;
        p_drive->braked_ms += waited;
        p_drive->alignment_stopped_ms = now;
        p_drive->input_waiting        = false;
        mission_recovery_phase(p_mission, p_drive->input_resume_phase, now);
        if (p_mission->leg.phase == LEG_OBSTACLE_VERIFYING)
        {
            p_drive->braked_ms           = now;
            p_drive->verification_frames = p_drive->obstacle_frames = 0;
            p_drive->last_obstacle_ms = p_inputs->sensor_received_ms;
            return false; // Resume stationary ToF verification, not camera
                          // recovery.
        }
        if (p_drive->front_arrival_pending ||
            (p_context->p_problem &&
             strcmp(p_context->p_problem, "FRONT_BLOCKED") == 0))
        {
            return false; // Fresh inputs returned with a front stop to verify.
        }
        bool b_ignored = false;
        wait_for_camera(p_drive, p_mission, p_selection, p_inputs,
                        "INPUT_RECOVERY", now, &b_ignored);
        return true; // Two fresh, post-stop images must precede a new motion
                     // lease.
    }

    p_context->wall_limit =
        ONBOARD_DRIVE_MAX_WALL_MS +
        ((p_drive->reference_recovery || p_drive->backup_used)
             ? ONBOARD_DRIVE_RECOVERY_WALL_MS
             : 0u);

    if (p_drive->search_used)
    {
        p_context->wall_limit = ONBOARD_DRIVE_SEARCH_WALL_MS;
    }
    if (!p_drive->b_departure_pending &&
        (!p_context->p_problem ||
         recoverable_camera_problem(p_context->p_problem)) &&
        (uint32_t) (now - p_drive->started_ms - p_drive->camera_waited_ms) >=
            p_context->wall_limit)
    {
        p_context->p_problem = "LEG_TIME_LIMIT";
    }

    p_context->b_front_stop =
        p_context->p_problem &&
        strcmp(p_context->p_problem, "FRONT_BLOCKED") == 0;

    return false;
}

/** @brief Execute the motion stage of onboard_drive_tick. */
static bool
onboard_drive_tick_motion (onboard_drive_t * p_drive, mission_t * p_mission,
                           rover_map_t * p_map, target_selector_t * p_selection,
                           const onboard_drive_inputs_t * p_inputs,
                           uint32_t                       now,
                           onboard_drive_tick_context_t * p_context)
{

    if ((!p_context->p_problem || p_context->b_front_stop) &&
        p_map->revision != p_drive->map_revision)
    {
        p_context->p_problem = "MAP_CHANGED_DURING_LEG";
    }

    p_context->b_camera_resuming = false;

    p_context->p_age_problem = camera_wait_problem(p_selection, p_inputs, now);
    if (!p_context->p_problem &&
        p_mission->leg.phase != LEG_OBSTACLE_VERIFYING &&
        !p_drive->front_arrival_pending &&
        (p_context->p_age_problem || p_drive->camera_waiting))
    {
        if (wait_for_camera(p_drive, p_mission, p_selection, p_inputs,
                            p_context->p_age_problem, now,
                            &p_context->b_camera_resuming))
        {
            return true;
        }
    }
    return false;
}

/** @brief Execute the camera stage of onboard_drive_tick. */
static bool
onboard_drive_tick_camera (onboard_drive_t * p_drive, mission_t * p_mission,
                           target_selector_t *            p_selection,
                           const onboard_drive_inputs_t * p_inputs,
                           uint32_t                       now,
                           onboard_drive_tick_context_t * p_context)
{

    if (!p_context->p_problem && p_drive->correcting &&
        p_mission->leg.phase == LEG_CORRECTING)
    {

        const char * p_camera_problem =
            input_problem(p_selection, p_inputs, now);

        if (p_camera_problem || !p_selection->reference_available ||
            (uint32_t) (now - p_drive->correction_started_ms) >=
                ONBOARD_DRIVE_CORRECTION_MS)
        {
            motor_emergency_stop();
            p_drive->correcting = false;
            mission_recovery_phase(p_mission, LEG_BRAKING, now);
            p_drive->braked_ms              = now;
            p_drive->verification_frames    = 0;
            p_drive->last_verify_generation = p_selection->sample_generation;
            p_drive->result                 = "SETTLING_AFTER_CORRECTION";
            return true;
        }
        if (motor_auto_status() != MOTOR_AUTO_RUNNING)
        {
            begin_input_wait(p_drive, p_mission, "MOTOR_STOPPED", now);
            return true;
        }
        if ((uint32_t) (now - p_drive->last_refresh_ms) >=
            ONBOARD_DRIVE_REFRESH_MS)
        {
            if (!motor_auto_renew_lease())
            {
                begin_input_wait(p_drive, p_mission, "DRIVE_LEASE_EXPIRED",
                                 now);
                return true;
            }
            p_drive->last_refresh_ms = now;
        }
        return true;
    }

    p_context->b_alignment_resuming = false;
    return false;
}

/** @brief Execute the tracking stage of onboard_drive_tick. */
static bool
onboard_drive_tick_tracking (onboard_drive_t * p_drive, mission_t * p_mission,
                             target_selector_t *            p_selection,
                             const onboard_drive_inputs_t * p_inputs,
                             uint32_t                       now,
                             onboard_drive_tick_context_t * p_context)
{

    if (p_drive->aligning)
    {
        if (p_context->p_problem)
        {
            onboard_drive_stop(
                p_drive, p_mission,
                p_inputs->safety_latched  ? MISSION_REASON_ESTOP
                : !p_inputs->link_ok      ? MISSION_REASON_LINK_LOST
                : p_context->b_front_stop ? MISSION_REASON_FRONT_STOP
                : strcmp(p_context->p_problem, "LEG_TIME_LIMIT") == 0
                    ? MISSION_REASON_DRIVE_TIMEOUT
                    : MISSION_REASON_INPUT_LOST,
                p_context->p_problem, now);
            return true;
        }
        if (alignment_tick(p_drive, p_mission, p_selection, p_inputs, now,
                           &p_context->b_alignment_resuming))
        {
            return true;
        }
    }

    p_context->b_front_stop =
        p_context->p_problem &&
        strcmp(p_context->p_problem, "FRONT_BLOCKED") == 0;

    if ((!p_context->p_problem || p_context->b_front_stop) &&
        (p_drive->front_arrival_pending || p_inputs->front_urgent ||
         front_obstacle_detected(p_inputs->front_mm, p_inputs->front_fault)) &&
        onboard_drive_note_front_stop(p_drive, p_mission))
    {
        p_drive->front_arrival_pending = false;
        if (p_mission->leg.phase != LEG_OBSTACLE_VERIFYING)
        {
            motor_emergency_stop();
            if (!mission_search_phase(p_mission, LEG_OBSTACLE_VERIFYING, now))
            {
                recovery_failure(p_drive, p_mission,
                                 "OBSTACLE_VERIFY_STATE_INVALID", now);
                return true;
            }
            // A real front stop supersedes the camera wait. Do not let its
            // saved approach/search phase restart motion during verification.
            p_drive->camera_waiting      = false;
            p_drive->camera_wait_frames  = 0;
            p_drive->camera_resume_phase = LEG_OBSTACLE_VERIFYING;
            p_drive->correcting          = false;
            p_drive->using_obstacle      = true;
            p_drive->using_reference     = false;
            p_drive->search_active       = false;
            p_drive->reference_recovery  = false;
            p_drive->braked_ms           = now;
            p_drive->verification_frames = p_drive->obstacle_frames = 0;
            p_drive->last_obstacle_ms         = p_inputs->sensor_received_ms;
            p_selection->allow_late_reference = false;
        }
        p_context->p_problem = NULL;
    }
    return false;
}

/** @brief Execute the reference stage of onboard_drive_tick. */
static bool
onboard_drive_tick_reference (onboard_drive_t * p_drive, mission_t * p_mission,
                              rover_map_t *                  p_map,
                              target_selector_t *            p_selection,
                              const onboard_drive_inputs_t * p_inputs,
                              uint32_t                       now,
                              onboard_drive_tick_context_t * p_context)
{

    if ((!p_context->p_problem || p_context->b_front_stop) &&
        p_mission->leg.phase == LEG_OBSTACLE_VERIFYING)
    {
        p_drive->remaining_mm   = 0;
        p_drive->estimate_valid = true;
        p_drive->input_reason   = "NONE";
        p_drive->result         = "VERIFYING_OBSTACLE_ARRIVAL";
        verify_obstacle(p_drive, p_mission, p_map, p_selection, p_inputs, now);
        return true;
    }

    p_context->b_search_reference_ready = false;
    if (!p_context->p_problem && !p_drive->reference_recovery &&
        !input_problem(p_selection, p_inputs, now) &&
        p_selection->leg_id == p_drive->leg_id &&
        p_selection->marker == p_mission->leg.next_target_node &&
        ((p_selection->available && !centred(p_selection->bearing_cdeg)) ||
         (p_selection->reference_available &&
          (!p_selection->available || centred(p_selection->bearing_cdeg)) &&
          !centred(p_selection->reference_bearing_cdeg))) &&
        (p_mission->leg.phase == LEG_APPROACHING ||
         p_mission->leg.phase == LEG_HOLDING ||
         p_mission->leg.phase == LEG_SEARCHING_REFERENCE ||
         p_mission->leg.phase == LEG_BRAKING ||
         p_mission->leg.phase == LEG_VERIFYING ||
         p_mission->leg.phase == LEG_REACQUIRING))
    {
        begin_alignment(p_drive, p_mission, p_selection, now);
        return true;
    }
    if (!p_context->p_problem &&
        (p_context->b_alignment_resuming ||
         (p_context->b_camera_resuming &&
          p_mission->leg.phase == LEG_SEARCHING_REFERENCE)) &&
        p_drive->search_active && !p_selection->reference_available)
    {
        if (!start_search(p_drive, p_mission, p_selection, now))
        {
            return true;
        }
    }
    return false;
}

/** @brief Execute the arrival stage of onboard_drive_tick. */
static bool
onboard_drive_tick_arrival (onboard_drive_t * p_drive, mission_t * p_mission,
                            rover_map_t *                  p_map,
                            target_selector_t *            p_selection,
                            const onboard_drive_inputs_t * p_inputs,
                            uint32_t                       now,
                            onboard_drive_tick_context_t * p_context)
{

    if (!p_context->p_problem && !p_drive->search_active &&
        !p_drive->reference_model_valid && !p_selection->available &&
        p_drive->last_target_range_mm &&
        (p_mission->leg.phase == LEG_APPROACHING ||
         p_mission->leg.phase == LEG_HOLDING) &&
        !search_camera_problem(p_inputs))
    {
        if (!start_search(p_drive, p_mission, p_selection, now))
        {
            return true;
        }
    }
    if (!p_context->p_problem && p_drive->search_active &&
        search_tick(p_drive, p_mission, p_selection, p_inputs, now,
                    &p_context->b_search_reference_ready))
    {
        return true;
    }

    p_context->b_reference_resuming = false;
    if (!p_context->p_problem && !p_drive->reference_recovery &&
        p_drive->reference_model_valid &&
        p_mission->leg.arrival_reference_node == p_drive->reference_id &&
        (reference_absent(p_inputs) ||
         tracking_markers_missing(p_selection, p_inputs)))
    {
        begin_marker_recovery(p_drive, p_mission, p_selection, now);
        if (!p_drive->active)
        {
            return true;
        }
    }
    if (!p_context->p_problem && p_drive->reference_recovery &&
        recover_reference(p_drive, p_mission, p_selection, p_map, p_inputs, now,
                          &p_context->b_reference_resuming))
    {
        return true;
    }
    return false;
}

/** @brief Execute the recovery stage of onboard_drive_tick. */
static bool
onboard_drive_tick_recovery (onboard_drive_t * p_drive, mission_t * p_mission,
                             target_selector_t *            p_selection,
                             const onboard_drive_inputs_t * p_inputs,
                             uint32_t                       now,
                             onboard_drive_tick_context_t * p_context)
{

    if (!p_context->p_problem)
    {
        p_context->p_problem = input_problem(p_selection, p_inputs, now);
    }
    if (!p_context->p_problem)
    {
        p_context->p_problem = track(p_drive, p_mission, p_selection, false);
    }
    if (p_context->p_problem &&
        (!strcmp(p_context->p_problem, "TARGET_NOT_ALIGNED") ||
         !strcmp(p_context->p_problem, "REFERENCE_NOT_ALIGNED")))
    {
        begin_alignment(p_drive, p_mission, p_selection, now);
        return true;
    }
    if (p_context->p_problem && live_camera_view(p_inputs) &&
        (!strcmp(p_context->p_problem, "TRACKING_MARKERS_MISSING") ||
         !strcmp(p_context->p_problem, "REFERENCE_BASELINE_UNAVAILABLE")))
    {
        return begin_marker_recovery(p_drive, p_mission, p_selection, now);
    }
    return false;
}

/** @brief Execute the finish stage of onboard_drive_tick. */
static bool
onboard_drive_tick_finish (onboard_drive_t * p_drive, mission_t * p_mission,
                           target_selector_t *            p_selection,
                           const onboard_drive_inputs_t * p_inputs,
                           uint32_t                       now,
                           onboard_drive_tick_context_t * p_context)
{

    if (p_context->p_problem)
    {
        p_drive->input_reason   = p_context->p_problem;
        p_drive->estimate_valid = false;
        if (recoverable_camera_problem(p_context->p_problem))
        {
            if (p_mission->leg.phase == LEG_BRAKING ||
                p_mission->leg.phase == LEG_VERIFYING)
            {
                p_drive->result = "WAITING_FOR_CAMERA_VERIFICATION";
                return true; // Keep stopped; fresh observations may still
                             // verify or correct arrival.
            }
            if (live_camera_view(p_inputs) &&
                (!strcmp(p_context->p_problem, "TRACKING_MARKERS_MISSING") ||
                 !strcmp(p_context->p_problem,
                         "REFERENCE_BASELINE_UNAVAILABLE") ||
                 !strcmp(p_context->p_problem, "CAMERA_NO_MARKERS")))
            {
                return begin_marker_recovery(p_drive, p_mission, p_selection,
                                             now);
            }
            bool b_ignored = false;
            wait_for_camera(p_drive, p_mission, p_selection, p_inputs,
                            p_context->p_problem, now, &b_ignored);
            return true;
        }
        onboard_drive_stop(
            p_drive, p_mission,
            p_inputs->safety_latched ? MISSION_REASON_ESTOP
            : !p_inputs->link_ok     ? MISSION_REASON_LINK_LOST
            : p_inputs->front_urgent ||
                    front_obstacle_detected(p_inputs->front_mm,
                                            p_inputs->front_fault)
                ? MISSION_REASON_FRONT_STOP
            : strcmp(p_context->p_problem, "LEG_TIME_LIMIT") == 0
                ? MISSION_REASON_DRIVE_TIMEOUT
                : MISSION_REASON_INPUT_LOST,
            p_context->p_problem, now);
        return true;
    }
    return false;
}

/** @brief Execute the output stage of onboard_drive_tick. */
static bool
onboard_drive_tick_output (onboard_drive_t * p_drive, mission_t * p_mission,
                           target_selector_t * p_selection, uint32_t now,
                           onboard_drive_tick_context_t * p_context)
{

    p_context->b_resuming = (p_context->b_camera_resuming &&
                             p_mission->leg.phase == LEG_APPROACHING) ||
                            p_context->b_alignment_resuming ||
                            p_context->b_reference_resuming ||
                            p_context->b_search_reference_ready;

    if (p_mission->leg.phase == LEG_HOLDING)
    {
        uint32_t image_ms =
            p_selection->sample_received_ms - p_selection->sample_age_ms;

        if ((int32_t) (image_ms - p_drive->hold_started_ms) < 0)
        {
            p_drive->result = "WAITING_FOR_POST_HOLD_IMAGE";
            return true;
        }
        if (p_selection->sample_generation == p_drive->recovery_generation)
        {
            return true;
        }
        p_drive->recovery_generation = p_selection->sample_generation;
        if (++p_drive->recovery_frames < ONBOARD_DRIVE_RECOVERY_FRAMES)
        {
            p_drive->result = "CONFIRMING_CAMERA_RECOVERY";
            return true;
        }
        mission_camera_hold(p_mission, false, now);
        p_context->b_resuming = true;
    }
    p_drive->input_reason = "NONE";
    return false;
}

/** @brief Execute the dispatch stage of onboard_drive_tick. */
static bool
onboard_drive_tick_dispatch (onboard_drive_t * p_drive, mission_t * p_mission,
                             rover_map_t *       p_map,
                             target_selector_t * p_selection, uint32_t now)
{

    if (p_mission->leg.phase == LEG_BRAKING ||
        p_mission->leg.phase == LEG_VERIFYING)
    {
        if ((uint32_t) (now - p_drive->braked_ms) < ONBOARD_DRIVE_SETTLE_MS)
        {
            return true;
        }
        if (p_mission->leg.phase == LEG_BRAKING)
        {
            mission_arrival_phase(p_mission, LEG_VERIFYING, now);
        }
        p_drive->result = "VERIFYING_ARRIVAL";
        // Do not confirm using cached/pre-brake images from a delayed pipeline.

        uint32_t image_ms =
            p_selection->sample_received_ms - p_selection->sample_age_ms;

        if ((int32_t) (image_ms -
                       (p_drive->braked_ms + ONBOARD_DRIVE_SETTLE_MS)) < 0 ||
            p_selection->sample_generation == p_drive->last_verify_generation)
        {
            return true;
        }
        p_drive->last_verify_generation = p_selection->sample_generation;
        if (!p_drive->using_reference || !p_drive->reference_model_valid)
        {
            p_drive->verification_frames = 0;
            p_drive->result              = "WAITING_FOR_ARRIVAL_REFERENCE";
            return true;
        }
        if (p_drive->remaining_mm < -ONBOARD_DRIVE_VERIFY_ERROR_MM ||
            p_drive->remaining_mm > ONBOARD_DRIVE_VERIFY_ERROR_MM)
        {
            begin_correction(p_drive, p_mission, p_selection, now);
            return true;
        }
        if (++p_drive->verification_frames < ONBOARD_DRIVE_VERIFY_FRAMES)
        {
            return true;
        }
        commit_arrival(p_drive, p_mission, p_map, false, now);
        return true;
    }
    return false;
}

/** @brief Execute the settle stage of onboard_drive_tick. */
static bool
onboard_drive_tick_settle (onboard_drive_t * p_drive, mission_t * p_mission,
                           target_selector_t * p_selection, uint32_t now)
{

    if (!p_drive->b_departure_pending && p_drive->using_reference &&
        p_drive->remaining_mm <= ONBOARD_DRIVE_ARRIVAL_STOP_MM)
    {
        motor_emergency_stop();
        mission_arrival_phase(p_mission, LEG_BRAKING, now);
        p_drive->braked_ms = now;
        p_drive->result    = "BRAKING_FOR_ARRIVAL";
        return true;
    }
    if (!p_drive->using_reference &&
        p_selection->range_mm <= ONBOARD_DRIVE_NEAR_RANGE_MM)
    {
        onboard_drive_stop(p_drive, p_mission, MISSION_REASON_APPROACH_LIMIT,
                           "NEAR_TARGET_UNCONFIRMED", now);
        return true;
    }
    if (p_drive->search_used &&
        p_drive->search_driving_ms >= ONBOARD_DRIVE_SEARCH_MAX_MS)
    {
        recovery_failure(p_drive, p_mission, "REFERENCE_SEARCH_LIMIT", now);
        return true;
    }
    if (!p_drive->search_used && p_drive->driving_ms >= ONBOARD_DRIVE_MAX_MS)
    {
        onboard_drive_stop(p_drive, p_mission, MISSION_REASON_DRIVE_TIMEOUT,
                           "DRIVE_TIME_LIMIT", now);
        return true;
    }
    return false;
}

/** @brief Execute the resume stage of onboard_drive_tick. */
static bool
onboard_drive_tick_resume (onboard_drive_t * p_drive, mission_t * p_mission,
                           uint32_t                       now,
                           onboard_drive_tick_context_t * p_context)
{

    if (p_context->b_resuming)
    {

        motor_error_t result = motor_auto_drive_leased(
            p_drive->search_used ? ONBOARD_DRIVE_SEARCH_PCT
                                 : MC_AUTO_APPROACH_PCT,
            ONBOARD_DRIVE_LEASE_MS);

        if (result != MOTOR_OK)
        {
            onboard_drive_stop(p_drive, p_mission, MISSION_REASON_MOTOR_STOPPED,
                               "MOTOR_REFUSED", now);
            return true;
        }
        p_drive->last_refresh_ms = now;

        p_drive->result =
            p_context->b_alignment_resuming
                ? (p_drive->b_alignment_reference ? "REFERENCE_ALIGNED"
                                                  : "TARGET_ALIGNED")
            : p_context->b_camera_resuming        ? "RESUMED_AFTER_CAMERA_WAIT"
            : p_context->b_search_reference_ready ? "LATE_REFERENCE_LOCKED"
            : p_context->b_reference_resuming
                ? "RESUMED_AFTER_REFERENCE_RECOVERY"
                : "RESUMED_AFTER_CAMERA_GAP";

        return true;
    }
    if (motor_auto_status() != MOTOR_AUTO_RUNNING)
    {
        begin_input_wait(p_drive, p_mission, "MOTOR_STOPPED", now);
        return true;
    }
    if ((uint32_t) (now - p_drive->last_refresh_ms) >= ONBOARD_DRIVE_REFRESH_MS)
    {
        if (!motor_auto_renew_lease())
        {
            begin_input_wait(p_drive, p_mission, "DRIVE_LEASE_EXPIRED", now);
            return true;
        }
        p_drive->last_refresh_ms = now;
    }
    return false;
}

static void
onboard_drive_tick_stages (onboard_drive_t * p_drive, mission_t * p_mission,
                           rover_map_t * p_map, target_selector_t * p_selection,
                           const onboard_drive_inputs_t * p_inputs,
                           uint32_t                       now)
{
    onboard_drive_tick_context_t context = {0};
    if (onboard_drive_tick_prepare(p_drive, p_mission, p_selection, p_inputs,
                                   now, &context))
    {
        return;
    }
    if (onboard_drive_tick_inputs(p_drive, p_mission, p_selection, p_inputs,
                                  now, &context))
    {
        return;
    }
    if (onboard_drive_tick_motion(p_drive, p_mission, p_map, p_selection,
                                  p_inputs, now, &context))
    {
        return;
    }
    if (onboard_drive_tick_camera(p_drive, p_mission, p_selection, p_inputs,
                                  now, &context))
    {
        return;
    }
    if (onboard_drive_tick_tracking(p_drive, p_mission, p_selection, p_inputs,
                                    now, &context))
    {
        return;
    }
    if (onboard_drive_tick_reference(p_drive, p_mission, p_map, p_selection,
                                     p_inputs, now, &context))
    {
        return;
    }
    if (onboard_drive_tick_arrival(p_drive, p_mission, p_map, p_selection,
                                   p_inputs, now, &context))
    {
        return;
    }
    if (onboard_drive_tick_recovery(p_drive, p_mission, p_selection, p_inputs,
                                    now, &context))
    {
        return;
    }
    if (onboard_drive_tick_finish(p_drive, p_mission, p_selection, p_inputs,
                                  now, &context))
    {
        return;
    }
    if (onboard_drive_tick_output(p_drive, p_mission, p_selection, now,
                                  &context))
    {
        return;
    }
    if (onboard_drive_tick_dispatch(p_drive, p_mission, p_map, p_selection,
                                    now))
    {
        return;
    }
    if (onboard_drive_tick_settle(p_drive, p_mission, p_selection, now))
    {
        return;
    }
    if (onboard_drive_tick_resume(p_drive, p_mission, now, &context))
    {
        return;
    }
}

void
onboard_drive_tick (onboard_drive_t * p_drive, mission_t * p_mission,
                    rover_map_t * p_map, target_selector_t * p_selection,
                    const onboard_drive_inputs_t * p_inputs, uint32_t now)
{
    onboard_drive_tick_stages(p_drive, p_mission, p_map, p_selection, p_inputs,
                              now);
    if (p_drive->active && p_drive->b_departure_pending &&
        motor_auto_status() == MOTOR_AUTO_RUNNING &&
        (p_mission->leg.phase == LEG_APPROACHING ||
         p_mission->leg.phase == LEG_BACKING_UP ||
         p_mission->leg.phase == LEG_SEARCHING_REFERENCE))
    {
        // Turning in place retains the confirmed node. Start journey timing
        // and invalidate that pose only when translation has been accepted.
        rover_map_depart(p_map);
        p_drive->map_revision        = p_map->revision;
        p_drive->b_departure_pending = false;
        p_drive->started_ms = p_drive->last_tick_ms = now;
        p_drive->camera_waited_ms                   = 0;
    }
}

size_t
onboard_drive_format (const onboard_drive_t * p_drive,
                      const mission_t * p_mission, char * p_out,
                      size_t capacity)
{
    if (!p_out || !capacity)
    {
        return 0;
    }
    char remaining[20];
    if (p_drive->estimate_valid)
    {
        snprintf(remaining, sizeof(remaining), "%" PRId32 "",
                 (int32_t) p_drive->remaining_mm);
    }
    else
    {
        snprintf(remaining, sizeof(remaining), "null");
    }

    int length = snprintf(
        p_out, capacity,
        "{\"leg_id\":%" PRIu32
        ",\"active\":%s,\"phase\":\"%s\",\"source\":\"%s\",\"remaining_mm\":%s,"
        "\"reference_progress_mm\":%" PRId32
        ",\"reference_model_valid\":%s,\"verification_frames\":%u,\"result\":"
        "\"%s\","
        "\"input_reason\":\"%s\",\"camera_present\":%s,\"camera_status\":%u,"
        "\"camera_packet_age_ms\":%" PRIu32 ","
        "\"camera_image_age_ms\":%" PRIu32 ",\"accepted_image_age_ms\":%" PRIu32
        ",\"recovery_frames\":%u,\"driving_ms\":%" PRIu32 ","
        "\"search_driving_ms\":%" PRIu32
        ",\"last_target_range_mm\":%u,\"reference_arrival_mm\":%u,"
        "\"verification_goal\":%u,"
        "\"recovery_marker_id\":%u,\"recovery_marker_mapped\":%s}",
        (uint32_t) p_drive->leg_id, p_drive->active ? "true" : "false",
        travel_leg_phase_name(p_mission->leg.phase),
        !p_drive->estimate_valid   ? "NONE"
        : p_drive->using_obstacle  ? "OBSTACLE"
        : p_drive->using_reference ? "REFERENCE"
                                   : "TARGET",
        remaining, (int32_t) p_drive->reference_progress_mm,
        p_drive->reference_model_valid ? "true" : "false",
        (unsigned) p_drive->verification_frames, p_drive->result,
        p_drive->input_reason ? p_drive->input_reason : "NONE",
        p_drive->camera_present ? "true" : "false",
        (unsigned) p_drive->camera_status,
        (uint32_t) p_drive->camera_packet_age_ms,
        (uint32_t) p_drive->camera_image_age_ms,
        (uint32_t) p_drive->accepted_image_age_ms,
        (unsigned) p_drive->recovery_frames, (uint32_t) p_drive->driving_ms,
        (uint32_t) p_drive->search_driving_ms,
        (unsigned) p_drive->last_target_range_mm,
        (unsigned) p_drive->reference_arrival_mm,
        p_drive->using_obstacle
            ? (unsigned) ONBOARD_DRIVE_OBSTACLE_CONFIRM_FRAMES
            : (unsigned) ONBOARD_DRIVE_VERIFY_FRAMES,
        (unsigned) p_drive->recovery_marker_id,
        p_drive->recovery_marker_mapped ? "true" : "false");

    if (length < 0 || (size_t) length >= capacity)
    {
        p_out[0] = '\0';
        return 0;
    }
    return (size_t) length;
}

/* End of file. */
