/**
 * @file onboard_dfs.c
 * @brief Onboard dfs firmware module.
 */
#include "onboard_dfs.h"
#include "../../common/obstacle_threshold.h"
#include "motor_driver.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Fixed dimensions and iteration limits used by this module. */
#define GRID_DIRECTION_COUNT 4
#define TOF_SCAN_COUNT 3

static const int8_t g_delta_x[4] = {0, 1, 0, -1};
static const int8_t g_delta_y[4] = {1, 0, -1, 0};
static bool
seen (const onboard_dfs_t * p_dfs, uint8_t identifier)
{
    return (p_dfs->seen[identifier / 8u] & (1u << (identifier % 8u))) != 0;
}
static void
mark_seen (onboard_dfs_t * p_dfs, uint8_t identifier)
{
    p_dfs->seen[identifier / 8u] |=
        (uint32_t) ((uint8_t) (1u << (identifier % 8u)));
}
static bool
cell (const map_node_t * p_from, unsigned dir, int16_t * p_x_coordinate,
      int16_t * p_y_coordinate)
{
    int32_t target_x = (int32_t) p_from->x + g_delta_x[dir];
    int32_t target_y = (int32_t) p_from->y + g_delta_y[dir];
    if (target_x < INT16_MIN || target_x > INT16_MAX || target_y < INT16_MIN ||
        target_y > INT16_MAX)
    {
        return false;
    }
    *p_x_coordinate = (int16_t) target_x;
    *p_y_coordinate = (int16_t) target_y;
    return true;
}
static const map_node_t *
neighbour (const rover_map_t * p_map, const map_node_t * p_node, unsigned dir)
{
    int16_t x_coordinate = 0;
    int16_t y_coordinate = 0;
    return cell(p_node, dir, &x_coordinate, &y_coordinate)
               ? rover_map_find_cell(p_map, x_coordinate, y_coordinate)
               : NULL;
}
static bool
whole_map_resolved (const onboard_dfs_t * p_dfs, const rover_map_t * p_map)
{
    // Sighted unvisited markers remain unresolved, even though their estimated
    // display positions must not be used as verified planner connectivity.
    if (p_map->observed_count)
    {
        return false;
    }
    for (size_t index = 0; index < p_map->count; ++index)
    {
        const map_node_t * p_node = &p_map->nodes[index];
        if (!p_node->visited || !seen(p_dfs, p_node->marker_id))
        {
            return false;
        }
        for (unsigned dir = 0; dir < GRID_DIRECTION_COUNT; ++dir)
        {
            if (p_node->exits[dir] == MAP_EXIT_UNKNOWN)
            {
                return false;
            }
            if (p_node->exits[dir] == MAP_EXIT_OPEN ||
                p_node->exits[dir] == MAP_EXIT_TRAVERSED)
            {
                const map_node_t * p_next = neighbour(p_map, p_node, dir);
                if (!p_next || !p_next->visited ||
                    !seen(p_dfs, p_next->marker_id) ||
                    p_next->exits[(dir + 2u) % 4u] == MAP_EXIT_BLOCKED)
                {
                    return false;
                }
            }
        }
    }
    return p_map->count != 0;
}

dfs_choice_t
onboard_dfs_choose (const onboard_dfs_t * p_dfs, const rover_map_t * p_map,
                    map_direction_t facing, bool b_camera_hint)
{
    const map_node_t * p_node = rover_map_find_id(p_map, p_map->current_node);
    if (!p_dfs->depth || !p_node ||
        p_dfs->stack[p_dfs->depth - 1u] != p_node->marker_id)
    {
        return (dfs_choice_t) {.kind = DFS_CHOOSE_ERROR};
    }

    unsigned order[4] = {(unsigned) facing, ((unsigned) facing + 1u) % 4u,
                         ((unsigned) facing + 3u) % 4u,
                         ((unsigned) facing + 2u) % 4u};

    for (unsigned index = 0; index < GRID_DIRECTION_COUNT; ++index)
    {
        unsigned           dir    = order[index];
        const map_node_t * p_next = neighbour(p_map, p_node, dir);

        bool b_open = p_node->exits[dir] == MAP_EXIT_OPEN ||
                      p_node->exits[dir] == MAP_EXIT_TRAVERSED;

        bool b_provisional = index == 0 && b_camera_hint &&
                             p_node->exits[dir] != MAP_EXIT_BLOCKED;

        if ((b_open || b_provisional) &&
            (!p_next || !seen(p_dfs, p_next->marker_id)))
        {
            return (dfs_choice_t) {.kind      = DFS_CHOOSE_EXPLORE,
                                   .direction = (map_direction_t) dir,
                                   .expected_id =
                                       p_next ? p_next->marker_id : 0};
        }
    }
    bool b_unknown = false;
    for (unsigned index = 0; index < GRID_DIRECTION_COUNT; ++index)
    {
        unsigned dir = order[index];
        if (p_node->exits[dir] != MAP_EXIT_UNKNOWN)
        {
            continue;
        }
        b_unknown = true;
        // Side ToF samples already resolve side walls/openings. Facing a
        // direction is not required merely to re-confirm that known result.
        if (!(p_dfs->scanned[p_dfs->depth - 1u] & (1u << dir)))
        {
            return (dfs_choice_t) {.kind      = DFS_CHOOSE_SCAN,
                                   .direction = (map_direction_t) dir};
        }
    }
    if (b_unknown)
    {
        return (dfs_choice_t) {.kind = DFS_CHOOSE_UNRESOLVED};
    }
    if (p_dfs->depth > 1u)
    {

        const map_node_t * p_parent =
            rover_map_find_id(p_map, p_dfs->stack[p_dfs->depth - 2u]);

        for (unsigned dir = 0; p_parent && dir < GRID_DIRECTION_COUNT; ++dir)
        {
            const map_node_t * p_next = neighbour(p_map, p_node, dir);
            if (p_next && p_next->marker_id == p_parent->marker_id &&
                p_node->exits[dir] != MAP_EXIT_BLOCKED)
            {
                return (dfs_choice_t) {.kind        = DFS_CHOOSE_BACKTRACK,
                                       .direction   = (map_direction_t) dir,
                                       .expected_id = p_parent->marker_id};
            }
        }
        return (dfs_choice_t) {.kind = DFS_CHOOSE_ERROR};
    }
    return (dfs_choice_t) {.kind = whole_map_resolved(p_dfs, p_map)
                                       ? DFS_CHOOSE_COMPLETE
                                       : DFS_CHOOSE_UNRESOLVED};
}

bool
onboard_dfs_note_arrival (onboard_dfs_t * p_dfs, const rover_map_t * p_map)
{
    if (!p_dfs->depth || !p_map->current_node ||
        (p_dfs->expected_id && p_dfs->expected_id != p_map->current_node))
    {
        return false;
    }
    if (p_dfs->from_node && p_dfs->stack[p_dfs->depth - 1u] != p_dfs->from_node)
    {
        return false;
    }
    if (p_dfs->backtracking)
    {
        if (p_dfs->depth < 2u ||
            p_dfs->stack[p_dfs->depth - 2u] != p_map->current_node)
        {
            return false;
        }
        p_dfs->depth--;
    }
    else
    {
        if (seen(p_dfs, p_map->current_node) ||
            p_dfs->depth >= ROVER_MAP_CAPACITY)
        {
            return false;
        }
        p_dfs->stack[p_dfs->depth]   = p_map->current_node;
        p_dfs->scanned[p_dfs->depth] = 0;
        p_dfs->depth++;
        mark_seen(p_dfs, p_map->current_node);
    }
    return true;
}

static unsigned
confirmed_nodes (const rover_map_t * p_map)
{
    unsigned count = 0;
    for (size_t index = 0; index < p_map->count; ++index)
    {
        count += p_map->nodes[index].visited ? 1u : 0u;
    }
    return count;
}

/** @brief Read a bounded unsigned decimal without overflow or signs. */
static bool
setting_number (const char ** p_text, unsigned maximum, unsigned * p_value)
{
    const char * p_cursor = *p_text;
    unsigned     value    = 0;
    if (*p_cursor < '0' || *p_cursor > '9')
    {
        return false;
    }
    do
    {
        unsigned digit = (unsigned) (*p_cursor - '0');
        if (digit > maximum || value > (maximum - digit) / 10u)
        {
            return false;
        }
        value = value * 10u + digit;
        ++p_cursor;
    } while (*p_cursor >= '0' && *p_cursor <= '9');
    *p_value = value;
    *p_text  = p_cursor;
    return true;
}

const char *
onboard_dfs_configure (onboard_dfs_t * p_dfs, const char * p_settings)
{
    if (!*p_settings)
    {
        return NULL;
    }
    unsigned opening = 0;
    unsigned limit   = p_dfs->node_limit;
    if (*p_settings++ != ':' || !setting_number(&p_settings, 1000u, &opening) ||
        opening <= FRONT_STOP_DISTANCE_MM ||
        (*p_settings && *p_settings != ':'))
    {
        return "INVALID_OPENING_THRESHOLD";
    }
    if (*p_settings == ':')
    {
        ++p_settings;
        if (!setting_number(&p_settings, ROVER_MAP_CAPACITY, &limit) ||
            *p_settings)
        {
            return "INVALID_NODE_LIMIT";
        }
    }
    if (p_dfs->active &&
        (opening != p_dfs->open_distance_mm || limit != p_dfs->node_limit))
    {
        return "STOP_DFS_BEFORE_CONFIG_CHANGE";
    }
    p_dfs->open_distance_mm = (uint16_t) opening;
    p_dfs->node_limit       = (uint8_t) limit;
    return NULL;
}

void
onboard_dfs_init (onboard_dfs_t * p_dfs)
{

    *p_dfs = (onboard_dfs_t) {.stage            = DFS_IDLE,
                              .detail           = "NOT_STARTED",
                              .open_distance_mm = DFS_OPEN_DISTANCE_MM};
}
static void
phase (mission_t * p_mission, leg_phase_t value, uint32_t now)
{
    p_mission->leg.phase = value;
    p_mission->revision++;
    p_mission->changed_ms = now;
}
void
onboard_dfs_stop (onboard_dfs_t * p_dfs, mission_t * p_mission,
                  const char * p_detail, uint32_t now)
{
    motor_emergency_stop();
    p_dfs->active        = false;
    p_dfs->input_waiting = false;
    p_dfs->stage         = DFS_STOPPED;
    p_dfs->detail        = p_detail;
    mission_pause(p_mission, MISSION_REASON_INPUT_LOST, now);
}
static float
wrap (float heading)
{
    heading = fmodf(heading, 360.0f);
    return heading < 0 ? heading + 360.0f : heading;
}
static map_direction_t
facing (const onboard_drive_t * p_drive, float raw)
{
    return (
        map_direction_t) ((unsigned) ((wrap(raw - p_drive->north_heading_deg) +
                                       45.0f) /
                                      90.0f) %
                          4u);
}
static float
error (float first, float second)
{
    float error_value = fabsf(wrap(first) - wrap(second));
    return error_value > 180 ? 360 - error_value : error_value;
}
static void
begin_node (onboard_dfs_t * p_dfs, mission_t * p_mission,
            onboard_drive_t * p_drive, target_selector_t * p_selection,
            uint32_t now)
{
    bool  b_frame = p_drive->frame_valid;
    float north   = p_drive->north_heading_deg;
    onboard_drive_init(p_drive);
    p_drive->frame_valid       = b_frame;
    p_drive->north_heading_deg = north;
    p_drive->result            = "SCANNING_NODE";
    mission_clear_leg(p_mission, now);
    target_selector_init(p_selection);
    phase(p_mission, LEG_SCANNING, now);
    p_dfs->stage             = DFS_AT_NODE;
    p_dfs->b_target_scanning = p_dfs->b_target_scan_rotating = false;
    p_dfs->b_target_scan_backing = p_dfs->b_target_scan_backup_done = false;
    p_dfs->target_scan_backup_ms                                    = 0;
    p_dfs->detail                    = "READING_NODE_EXITS";
    p_dfs->b_target_scan_marker_hold = false;
    p_dfs->b_target_camera_waiting   = false;
    memset(p_dfs->scan_visible_markers, 0, sizeof(p_dfs->scan_visible_markers));
}
const char *
onboard_dfs_start (onboard_dfs_t * p_dfs, mission_t * p_mission,
                   const rover_map_t * p_map, onboard_drive_t * p_drive,
                   target_selector_t *          p_selection,
                   const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{
    if (!p_map->count)
    {
        return "START_NODE_NOT_SET";
    }
    if (p_dfs->active && p_mission->dfs_active)
    {
        return "ALREADY_RUNNING";
    }
    const map_node_t * p_node = rover_map_find_id(p_map, p_map->current_node);
    if (!p_node || !p_node->visited)
    {
        return "CURRENT_NODE_UNKNOWN";
    }
    if (p_inputs->drive.safety_latched)
    {
        return "SAFETY_LOCKED";
    }
    if (!p_inputs->drive.link_ok)
    {
        return "LINK_LOST";
    }
    if (!p_inputs->drive.sensor_valid ||
        (uint32_t) (now - p_inputs->drive.sensor_received_ms) >
            ONBOARD_DRIVE_SENSOR_MAX_AGE_MS)
    {
        return "SENSOR_STALE";
    }
    if (!p_inputs->drive.imu_ok || !isfinite(p_inputs->drive.heading_deg) ||
        p_inputs->drive.heading_deg < 0 || p_inputs->drive.heading_deg >= 360)
    {
        return "HEADING_UNAVAILABLE";
    }
    if (!p_inputs->drive.stopped)
    {
        return "MOTOR_BUSY";
    }
    if (!p_drive->frame_valid && p_map->count != 1u)
    {
        return "GRID_HEADING_UNAVAILABLE";
    }
    if (!mission_start_dfs(p_mission, now))
    {
        return "MISSION_NOT_IDLE";
    }
    if (!p_drive->frame_valid)
    {
        p_drive->north_heading_deg = p_inputs->drive.heading_deg;
        p_drive->frame_valid       = true;
    }

    uint16_t opening = p_dfs->open_distance_mm ? p_dfs->open_distance_mm
                                               : DFS_OPEN_DISTANCE_MM;
    uint8_t  limit   = p_dfs->node_limit;

    *p_dfs = (onboard_dfs_t) {.active           = true,
                              .stage            = DFS_AT_NODE,
                              .depth            = 1,
                              .detail           = "READING_NODE_EXITS",
                              .open_distance_mm = opening,
                              .node_limit       = limit};

    p_dfs->stack[0] = p_map->current_node;
    mark_seen(p_dfs, p_map->current_node);
    begin_node(p_dfs, p_mission, p_drive, p_selection, now);
    return "STARTED";
}

bool
onboard_dfs_selecting (const onboard_dfs_t * p_dfs, uint32_t now)
{
    return p_dfs->active && !p_dfs->input_waiting &&
           !p_dfs->b_target_scan_rotating && !p_dfs->b_target_scan_backing &&
           p_dfs->stage == DFS_WAIT_TARGET &&
           (int32_t) (now - p_dfs->ready_ms) >= 0;
}
static void
sampling (onboard_dfs_t * p_dfs, mission_t * p_mission, uint32_t now)
{
    p_dfs->stage            = DFS_SAMPLING;
    p_dfs->stage_started_ms = now;
    p_dfs->ready_ms         = now + DFS_SETTLE_MS;
    p_dfs->last_sensor_ms   = 0;
    memset(p_dfs->sample_code, 0, sizeof(p_dfs->sample_code));
    memset(p_dfs->sample_count, 0, sizeof(p_dfs->sample_count));
    phase(p_mission, LEG_SCANNING, now);
    p_dfs->detail = "SAMPLING_TOF_EXITS";
}
static bool
aim (onboard_dfs_t * p_dfs, mission_t * p_mission,
     const onboard_drive_t * p_drive, const onboard_dfs_inputs_t * p_inputs,
     dfs_stage_t after, uint32_t now)
{
    p_dfs->after_turn = after;

    float raw_target =
        wrap(p_drive->north_heading_deg + 90.0f * (float) p_dfs->direction);

    if (error(raw_target, p_inputs->drive.heading_deg) <= 3.0f)
    {
        if (after == DFS_SAMPLING)
        {
            sampling(p_dfs, p_mission, now);
        }
        else
        {
            p_dfs->stage            = DFS_WAIT_TARGET;
            p_dfs->stage_started_ms = now;
            p_dfs->ready_ms         = now + DFS_SETTLE_MS;
        }
        return true;
    }

    int target =
        (int) (wrap(raw_target - p_inputs->heading_offset_deg) + 0.5f) % 360;

    if (motor_auto_rotate_leased(target, ONBOARD_DRIVE_LEASE_MS) != MOTOR_OK)
    {
        return false;
    }
    phase(p_mission, LEG_TURNING, now);
    if (p_dfs->stage != DFS_TURNING)
    {
        p_dfs->stage_started_ms = now;
    }
    p_dfs->stage         = DFS_TURNING;
    p_dfs->last_renew_ms = p_dfs->turn_attempt_started_ms = now;
    p_dfs->b_turn_retry_waiting                           = false;

    p_dfs->detail =
        after == DFS_SAMPLING ? "TURNING_TO_SCAN" : "TURNING_TO_NEXT_EXIT";

    return true;
}

/** @brief Bound a turn across retries; stopped input waits are excluded. */
static bool
turn_timed_out (onboard_dfs_t * p_dfs, mission_t * p_mission, uint32_t now)
{
    if ((uint32_t) (now - p_dfs->stage_started_ms) < DFS_TURN_TIMEOUT_MS)
    {
        return false;
    }
    onboard_dfs_stop(p_dfs, p_mission, "DFS_TURN_TIMEOUT", now);
    return true;
}

/** @brief Stop, settle and retry the same heading without resetting its limit.
 */
static bool
retry_turn (onboard_dfs_t * p_dfs, mission_t * p_mission,
            const onboard_drive_t *      p_drive,
            const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{
    if (p_dfs->b_turn_retry_waiting)
    {
        if ((int32_t) (now - p_dfs->ready_ms) < 0 || !p_inputs->drive.stopped ||
            (int32_t) (p_inputs->drive.sensor_received_ms - p_dfs->ready_ms) <
                0)
        {
            return true;
        }
        if (!aim(p_dfs, p_mission, p_drive, p_inputs, p_dfs->after_turn, now))
        {
            onboard_dfs_stop(p_dfs, p_mission, "DFS_TURN_REFUSED", now);
        }
        else if (p_dfs->stage == DFS_TURNING)
        {
            p_dfs->detail = "DFS_TURN_RETRYING";
        }
        return true;
    }
    if ((uint32_t) (now - p_dfs->turn_attempt_started_ms) < DFS_TURN_RETRY_MS)
    {
        return false;
    }
    motor_emergency_stop();
    p_dfs->b_turn_retry_waiting = true;
    p_dfs->ready_ms             = now + DFS_SETTLE_MS;
    phase(p_mission, LEG_HOLDING, now);
    p_dfs->detail = "DFS_TURN_RETRY_WAIT";
    return true;
}
static bool
camera_hint (const camera_observation_t * p_observation,
             const rover_map_t * p_map, map_direction_t dir, uint32_t settled,
             uint32_t now)
{
    if (!p_observation || p_observation->frame.status != VISION_STATUS_OK ||
        p_observation->frame.count > VISION_MAX_MARKERS ||
        (uint32_t) (now - p_observation->received_ms) >
            TARGET_UART_MAX_AGE_MS ||
        (uint32_t) (now - p_observation->received_ms) +
                p_observation->frame.age_ms >
            ONBOARD_DRIVE_IMAGE_MAX_AGE_MS ||
        (int32_t) (p_observation->received_ms - p_observation->frame.age_ms -
                   settled) < 0)
    {
        return false;
    }
    const map_node_t * p_from = rover_map_find_id(p_map, p_map->current_node);
    int16_t            x_coordinate = 0;
    int16_t            y_coordinate = 0;
    if (!p_from || !cell(p_from, (unsigned) dir, &x_coordinate, &y_coordinate))
    {
        return false;
    }

    const map_node_t * p_occupant =
        rover_map_find_cell(p_map, x_coordinate, y_coordinate);

    for (uint8_t index = 0; index < p_observation->frame.count; ++index)
    {
        const camera_marker_t * p_marker = &p_observation->frame.markers[index];
        if (!p_marker->id || p_marker->id == p_from->marker_id ||
            p_marker->quality ||
            p_marker->range_mm <= ONBOARD_DRIVE_NEAR_RANGE_MM ||
            p_marker->range_mm == UINT16_MAX ||
            p_marker->bearing_cdeg < -ONBOARD_DRIVE_MAX_BEARING_CDEG ||
            p_marker->bearing_cdeg > ONBOARD_DRIVE_MAX_BEARING_CDEG ||
            (p_occupant && p_occupant->marker_id != p_marker->id))
        {
            continue;
        }
        const map_node_t * p_known = rover_map_find_id(p_map, p_marker->id);
        if (!p_known ||
            (p_known->x == x_coordinate && p_known->y == y_coordinate))
        {
            return true;
        }
    }
    return false;
}

static void
wait_inputs (onboard_dfs_t * p_dfs, mission_t * p_mission,
             const char * p_reason, uint32_t now)
{
    if (!p_dfs->input_waiting)
    {
        motor_emergency_stop();
        p_dfs->input_waiting         = true;
        p_dfs->input_resume_phase    = p_mission->leg.phase;
        p_dfs->input_wait_started_ms = now;
        phase(p_mission, LEG_WAITING_INPUT, now);
    }
    p_dfs->detail = p_reason;
}

/** @brief Discard moving-image candidates and require new settled images. */
static void
target_scan_pose (onboard_dfs_t * p_dfs, mission_t * p_mission,
                  target_selector_t * p_selection, const char * p_detail,
                  uint32_t now)
{
    motor_emergency_stop();
    target_route_t route = p_selection->route;
    mission_clear_leg(p_mission, now);
    target_selector_set_route(p_selection, route);
    p_dfs->b_target_scan_rotating    = false;
    p_dfs->b_target_scan_backing     = false;
    p_dfs->b_target_scan_marker_hold = false;
    p_dfs->b_target_camera_waiting   = false;
    p_dfs->b_turn_retry_waiting      = false;
    p_dfs->stage_started_ms          = now;
    p_dfs->ready_ms                  = now + DFS_SETTLE_MS;
    p_dfs->detail                    = p_detail;
}

/** @brief Correct grid heading before retrying the same destination. */
static void
recover_grid_heading (onboard_dfs_t * p_dfs, mission_t * p_mission,
                      const onboard_drive_t *      p_drive,
                      target_selector_t *          p_selection,
                      const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{
    target_route_t route = p_selection->route;
    route.expected_id    = p_mission->leg.next_target_node;
    target_scan_pose(p_dfs, p_mission, p_selection, "RECOVERING_GRID_HEADING",
                     now);
    target_selector_set_route(p_selection, route);
    p_dfs->b_target_scanning = false;
    p_dfs->target_scan_step  = 0;
    // Keep the original exit/stack and reverse budget. A failed command waits
    // for fresh inputs, then the normal turn owner retries this same heading.
    p_dfs->stage = DFS_TURNING;
    phase(p_mission, LEG_TURNING, now);
    if (!aim(p_dfs, p_mission, p_drive, p_inputs, DFS_WAIT_TARGET, now))
    {
        wait_inputs(p_dfs, p_mission, "GRID_HEADING_MOTOR_WAIT", now);
    }
    else
    {
        p_dfs->detail = "RECOVERING_GRID_HEADING";
    }
}

/** @brief Stop once for each newly sighted valid marker, even off-axis. */
static bool
scan_marker_sighting (onboard_dfs_t * p_dfs, const camera_frame_t * p_frame)
{
    uint8_t visible[32] = {0};
    bool    b_new       = false;
    if (p_frame->status == VISION_STATUS_OK)
    {
        for (uint8_t index = 0; index < p_frame->count; ++index)
        {
            const camera_marker_t * p_marker = &p_frame->markers[index];
            if (!p_marker->id || p_marker->quality || !p_marker->range_mm ||
                p_marker->range_mm == UINT16_MAX)
            {
                continue;
            }
            uint8_t bit = (uint8_t) (1u << (p_marker->id % 8u));
            visible[p_marker->id / 8u] |= bit;
            b_new |=
                (p_dfs->scan_visible_markers[p_marker->id / 8u] & bit) == 0;
        }
    }
    memcpy(p_dfs->scan_visible_markers, visible, sizeof(visible));
    return b_new;
}

/** @brief Search only around the selected exit; never change the DFS route. */
static void
start_target_scan (onboard_dfs_t * p_dfs, mission_t * p_mission,
                   const onboard_drive_t *      p_drive,
                   target_selector_t *          p_selection,
                   const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{
    // Small poses stay inside the grid planner's 15-degree heading tolerance.
    static const int8_t offsets[] = {-10, 0, 10, 0};
    if (p_dfs->target_scan_step == 0)
    {
        p_dfs->b_target_scan_backup_done = false;
    }
    p_dfs->b_target_scanning = true;
    p_dfs->target_scan_goal_deg =
        wrap(p_drive->north_heading_deg + 90.0f * (float) p_dfs->direction +
             offsets[p_dfs->target_scan_step]);
    p_dfs->target_scan_step = (uint8_t) ((p_dfs->target_scan_step + 1u) % 4u);
    target_scan_pose(p_dfs, p_mission, p_selection, "TARGET_SCAN_SETTLING",
                     now);
    if (error(p_dfs->target_scan_goal_deg, p_inputs->drive.heading_deg) <= 3.0f)
    {
        return;
    }
    int target = (int) (wrap(p_dfs->target_scan_goal_deg -
                             p_inputs->heading_offset_deg) +
                        0.5f) %
                 360;
    if (motor_auto_rotate_leased(target, ONBOARD_DRIVE_LEASE_MS) != MOTOR_OK)
    {
        wait_inputs(p_dfs, p_mission, "TARGET_SCAN_MOTOR_WAIT", now);
        return;
    }
    p_dfs->b_target_scan_rotating  = true;
    p_dfs->turn_attempt_started_ms = p_dfs->last_renew_ms = now;
    phase(p_mission, LEG_TURNING, now);
    p_dfs->detail = "TARGET_SCAN_TURNING";
}

/** @brief A short leased reverse after a complete unsuccessful scan. */
static void
target_scan_backup (onboard_dfs_t * p_dfs, mission_t * p_mission,
                    target_selector_t * p_selection, uint32_t now)
{
    uint16_t duration = (uint16_t) (DFS_TARGET_SCAN_BACKUP_TOTAL_MS -
                                    p_dfs->target_scan_backup_ms);
    if (duration > DFS_TARGET_SCAN_BACKUP_MS)
    {
        duration = DFS_TARGET_SCAN_BACKUP_MS;
    }
    target_scan_pose(p_dfs, p_mission, p_selection,
                     "TARGET_SCAN_BACKUP_SETTLING", now);
    p_dfs->b_target_scan_backup_done = true;
    if (motor_auto_drive_leased(-DFS_TARGET_SCAN_BACKUP_PCT, duration) !=
        MOTOR_OK)
    {
        wait_inputs(p_dfs, p_mission, "TARGET_SCAN_BACKUP_MOTOR_WAIT", now);
        return;
    }
    // The motor's independent lease bounds motion even if the main loop stalls.
    p_dfs->target_scan_backup_ms += duration;
    p_dfs->b_target_scan_backing = true;
    p_dfs->ready_ms              = now + duration;
    phase(p_mission, LEG_BACKING_UP, now);
    p_dfs->detail = "TARGET_SCAN_BACKING_UP";
}

/** @brief Bound each scan turn, then retry searching without pausing DFS. */
static void
target_scan_turn (onboard_dfs_t * p_dfs, mission_t * p_mission,
                  target_selector_t *          p_selection,
                  const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{
    motor_auto_status_t status = motor_auto_status();
    if ((status == MOTOR_AUTO_HEADING_REACHED && p_inputs->drive.stopped) ||
        (uint32_t) (now - p_dfs->stage_started_ms) >=
            DFS_TARGET_SCAN_TURN_TIMEOUT_MS)
    {
        target_scan_pose(p_dfs, p_mission, p_selection,
                         status == MOTOR_AUTO_HEADING_REACHED
                             ? "TARGET_SCAN_SETTLING"
                             : "TARGET_SCAN_TURN_LIMIT",
                         now);
        return;
    }
    if (status == MOTOR_AUTO_HEADING_REACHED)
    {
        return; // Wait for a stopped input snapshot before accepting images.
    }
    if (p_dfs->b_turn_retry_waiting)
    {
        if ((int32_t) (now - p_dfs->ready_ms) < 0 || !p_inputs->drive.stopped ||
            (int32_t) (p_inputs->drive.sensor_received_ms - p_dfs->ready_ms) <
                0)
        {
            return;
        }
        int target = (int) (wrap(p_dfs->target_scan_goal_deg -
                                 p_inputs->heading_offset_deg) +
                            0.5f) %
                     360;
        if (motor_auto_rotate_leased(target, ONBOARD_DRIVE_LEASE_MS) !=
            MOTOR_OK)
        {
            wait_inputs(p_dfs, p_mission, "TARGET_SCAN_MOTOR_WAIT", now);
            return;
        }
        p_dfs->b_turn_retry_waiting    = false;
        p_dfs->turn_attempt_started_ms = p_dfs->last_renew_ms = now;
        phase(p_mission, LEG_TURNING, now);
        p_dfs->detail = "TARGET_SCAN_TURNING";
        return;
    }
    if (status != MOTOR_AUTO_RUNNING)
    {
        target_scan_pose(p_dfs, p_mission, p_selection,
                         "TARGET_SCAN_MOTOR_WAIT", now);
    }
    else if ((uint32_t) (now - p_dfs->turn_attempt_started_ms) >=
             DFS_TURN_RETRY_MS)
    {
        motor_emergency_stop();
        p_dfs->b_turn_retry_waiting = true;
        p_dfs->ready_ms             = now + DFS_SETTLE_MS;
        phase(p_mission, LEG_HOLDING, now);
        p_dfs->detail = "TARGET_SCAN_RETRY_WAIT";
    }
    else if ((uint32_t) (now - p_dfs->last_renew_ms) >=
             ONBOARD_DRIVE_REFRESH_MS)
    {
        if (!motor_auto_renew_lease())
        {
            wait_inputs(p_dfs, p_mission, "TARGET_SCAN_LEASE_WAIT", now);
        }
        p_dfs->last_renew_ms = now;
    }
}

typedef struct
{
    const char *       p_missing;
    const map_node_t * p_node;
    dfs_choice_t       choice;
    int16_t            x_coordinate;
    int16_t            y_coordinate;
} onboard_dfs_tick_context_t;

/** @brief Execute the prepare stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_prepare (onboard_dfs_t * p_dfs, mission_t * p_mission,
                          onboard_drive_t *            p_drive,
                          const onboard_dfs_inputs_t * p_inputs, uint32_t now,
                          onboard_dfs_tick_context_t * p_context)
{

    if (!p_dfs->active)
    {
        return true;
    }
    if (!p_mission->dfs_active || p_mission->state != MISSION_EXPLORING ||
        p_mission->test_mode)
    {
        motor_emergency_stop();
        p_dfs->active        = false;
        p_dfs->input_waiting = false;
        p_dfs->stage         = DFS_STOPPED;
        p_dfs->detail        = "CANCELLED";
        return true;
    }
    if (p_drive->active)
    {
        return true; // The leg executor owns all motion/input checks until
                     // arrival or cancellation.
    }
    if (p_inputs->drive.safety_latched)
    {
        onboard_dfs_stop(p_dfs, p_mission, "SAFETY_LOCKED", now);
        return true;
    }

    p_context->p_missing =
        !p_inputs->drive.link_ok ? "LINK_LOST"
        : !p_inputs->drive.sensor_valid ||
                (uint32_t) (now - p_inputs->drive.sensor_received_ms) >
                    ONBOARD_DRIVE_SENSOR_MAX_AGE_MS
            ? "SENSOR_STALE"
        : !p_inputs->drive.imu_ok || !isfinite(p_inputs->drive.heading_deg) ||
                p_inputs->drive.heading_deg < 0 ||
                p_inputs->drive.heading_deg >= 360
            ? "HEADING_UNAVAILABLE"
            : NULL;

    if (p_context->p_missing)
    {
        wait_inputs(p_dfs, p_mission, p_context->p_missing, now);
        return true;
    }
    return false;
}

/** @brief Execute the inputs stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_inputs (onboard_dfs_t * p_dfs, mission_t * p_mission,
                         rover_map_t * p_map, onboard_drive_t * p_drive,
                         target_selector_t *          p_selection,
                         const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{

    if (p_dfs->input_waiting)
    {
        if ((uint32_t) (now - p_dfs->input_wait_started_ms) < DFS_SETTLE_MS ||
            (int32_t) (p_inputs->drive.sensor_received_ms -
                       p_dfs->input_wait_started_ms - DFS_SETTLE_MS) < 0)
        {
            return true;
        }
        uint32_t waited = now - p_dfs->input_wait_started_ms;
        p_dfs->stage_started_ms += waited;
        p_dfs->ready_ms += waited;
        if (p_dfs->b_target_camera_waiting)
        {
            // This interval is already excluded from the stage clock here.
            p_dfs->target_camera_wait_started_ms += waited;
        }
        p_dfs->input_waiting = false;
        phase(p_mission, p_dfs->input_resume_phase, now);
        p_dfs->detail = "INPUTS_RESTORED";
        if (p_dfs->b_target_scan_rotating || p_dfs->b_target_scan_backing)
        {
            target_scan_pose(p_dfs, p_mission, p_selection,
                             "TARGET_SCAN_SETTLING", now);
            return true;
        }
        if (p_dfs->stage == DFS_TURNING)
        {
            if (turn_timed_out(p_dfs, p_mission, now))
            {
                return true;
            }
            if (!aim(p_dfs, p_mission, p_drive, p_inputs, p_dfs->after_turn,
                     now))
            {
                wait_inputs(p_dfs, p_mission, "DFS_TURN_MOTOR_WAIT", now);
            }
            return true;
        }
        if (p_dfs->stage == DFS_SAMPLING)
        {
            sampling(p_dfs, p_mission, now);
            return true;
        }
    }
    if (p_dfs->stage == DFS_TRAVELLING)
    {
        if (p_mission->leg.phase != LEG_COMPLETE ||
            !onboard_dfs_note_arrival(p_dfs, p_map))
        {
            onboard_dfs_stop(p_dfs, p_mission, "DFS_ARRIVAL_MISMATCH", now);
            return true;
        }
        begin_node(p_dfs, p_mission, p_drive, p_selection, now);
    }
    return false;
}

/** @brief Execute the motion stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_motion (onboard_dfs_t * p_dfs, mission_t * p_mission,
                         rover_map_t * p_map, onboard_drive_t * p_drive,
                         const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{

    if (!p_map->current_node ||
        p_dfs->stack[p_dfs->depth - 1u] != p_map->current_node)
    {
        onboard_dfs_stop(p_dfs, p_mission, "DFS_POSITION_UNKNOWN", now);
        return true;
    }
    if (p_dfs->stage == DFS_AT_NODE)
    {
        p_dfs->direction = facing(p_drive, p_inputs->drive.heading_deg);
        p_dfs->exit_retry_direction = p_dfs->direction;
        if (!aim(p_dfs, p_mission, p_drive, p_inputs, DFS_SAMPLING, now))
        {
            onboard_dfs_stop(p_dfs, p_mission, "DFS_TURN_REFUSED", now);
        }
        return true;
    }
    return false;
}

/** @brief Execute the camera stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_camera (onboard_dfs_t * p_dfs, mission_t * p_mission,
                         const onboard_drive_t *      p_drive,
                         const onboard_dfs_inputs_t * p_inputs, uint32_t now)
{

    if (p_dfs->stage == DFS_TURNING)
    {
        motor_auto_status_t status = motor_auto_status();
        // Heading feedback completes the turn. A retry shares the 60-second
        // limit; stopped input outages retain the turn and its used allowance.
        if (status == MOTOR_AUTO_HEADING_REACHED)
        {
            if (!p_inputs->drive.stopped)
            {
                return true; // Motor ISR may finish after this input snapshot
                             // was taken.
            }
            motor_emergency_stop();
            if (p_dfs->after_turn == DFS_SAMPLING)
            {
                sampling(p_dfs, p_mission, now);
            }
            else
            {
                mission_clear_leg(p_mission, now);
                p_dfs->stage            = DFS_WAIT_TARGET;
                p_dfs->stage_started_ms = now;
                p_dfs->ready_ms         = now + DFS_SETTLE_MS;
            }
        }
        else if (turn_timed_out(p_dfs, p_mission, now))
        {
            return true;
        }
        else if ((p_dfs->b_turn_retry_waiting ||
                  status == MOTOR_AUTO_RUNNING) &&
                 retry_turn(p_dfs, p_mission, p_drive, p_inputs, now))
        {
            return true;
        }
        else if (status != MOTOR_AUTO_RUNNING)
        {
            if (status == MOTOR_AUTO_STOPPED)
            {
                wait_inputs(p_dfs, p_mission, "DFS_TURN_MOTOR_WAIT", now);
            }
            else
            {
                onboard_dfs_stop(p_dfs, p_mission,
                                 status == MOTOR_AUTO_REJECTED
                                     ? "DFS_TURN_REJECTED"
                                     : "DFS_TURN_FAILED",
                                 now);
            }
        }
        else if ((uint32_t) (now - p_dfs->last_renew_ms) >=
                 ONBOARD_DRIVE_REFRESH_MS)
        {
            if (!motor_auto_renew_lease())
            {
                wait_inputs(p_dfs, p_mission, "DFS_TURN_LEASE_EXPIRED", now);
            }
            p_dfs->last_renew_ms = now;
        }
        return true;
    }
    return false;
}

/** @brief Execute the tracking stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_tracking (onboard_dfs_t * p_dfs, mission_t * p_mission,
                           rover_map_t * p_map, onboard_drive_t * p_drive,
                           target_selector_t *          p_selection,
                           const onboard_dfs_inputs_t * p_inputs,
                           const camera_observation_t * p_camera, uint32_t now)
{

    if (p_dfs->stage == DFS_WAIT_TARGET)
    {

        bool b_healthy_camera =
            p_camera && (p_camera->frame.status == VISION_STATUS_OK ||
                         p_camera->frame.status == VISION_STATUS_NO_MARKER);

        if (!b_healthy_camera || p_camera->frame.count > VISION_MAX_MARKERS ||
            (uint32_t) (now - p_camera->received_ms) > TARGET_UART_MAX_AGE_MS ||
            (uint32_t) (now - p_camera->received_ms) + p_camera->frame.age_ms >
                ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
        {
            if (p_dfs->b_target_scan_rotating || p_dfs->b_target_scan_backing)
            {
                target_scan_pose(p_dfs, p_mission, p_selection,
                                 "WAITING_FOR_FRESH_CAMERA", now);
            }
            // Freeze the stationary recheck/dwell instead of restarting it.
            // Motion interrupted above starts a new settled pose as usual.
            if (!p_dfs->b_target_camera_waiting)
            {
                p_dfs->b_target_camera_waiting       = true;
                p_dfs->target_camera_wait_started_ms = now;
            }
            p_dfs->detail = "WAITING_FOR_FRESH_CAMERA";
            return true;
        }
        if (p_dfs->b_target_camera_waiting)
        {
            p_dfs->stage_started_ms +=
                now - p_dfs->target_camera_wait_started_ms;
            p_dfs->b_target_camera_waiting = false;
        }
        bool b_new_marker = scan_marker_sighting(p_dfs, &p_camera->frame);
        bool b_candidate  = target_selector_has_acquisition_candidate(
            p_selection, p_map, &p_camera->frame);
        if (!p_dfs->b_target_scan_marker_hold &&
            p_selection->state != TARGET_LOCKED &&
            (b_new_marker || b_candidate))
        {
            // Moving images are stop hints, never destination confirmations.
            target_scan_pose(p_dfs, p_mission, p_selection,
                             "MARKER_SEEN_STOPPING_SCAN", now);
            p_dfs->b_target_scan_marker_hold = true;
            p_selection->b_acquiring         = true;
            return true;
        }
        if (p_dfs->b_target_scan_backing)
        {
            if ((int32_t) (now - p_dfs->ready_ms) >= 0 ||
                motor_auto_status() != MOTOR_AUTO_RUNNING)
            {
                target_scan_pose(p_dfs, p_mission, p_selection,
                                 "TARGET_SCAN_BACKUP_SETTLING", now);
            }
            return true;
        }
        if (p_dfs->b_target_scan_rotating)
        {
            target_scan_turn(p_dfs, p_mission, p_selection, p_inputs, now);
            return true;
        }
        if ((int32_t) (now - p_dfs->ready_ms) < 0 || !p_inputs->drive.stopped ||
            (int32_t) (p_inputs->drive.sensor_received_ms - p_dfs->ready_ms) <
                0)
        {
            return true;
        }
        if (p_dfs->b_target_scan_marker_hold)
        {
            p_dfs->detail = b_candidate ? "CONFIRMING_VISIBLE_MARKER"
                                        : "CHECKING_VISIBLE_MARKER";
            if (b_candidate)
            {
                p_dfs->stage_started_ms = now;
            }
            else if ((uint32_t) (now - p_dfs->stage_started_ms) >=
                         DFS_TARGET_SCAN_DWELL_MS + DFS_SETTLE_MS &&
                     p_selection->state != TARGET_LOCKED)
            {
                p_dfs->b_target_scan_marker_hold = false;
                p_selection->b_acquiring         = false;
                start_target_scan(p_dfs, p_mission, p_drive, p_selection,
                                  p_inputs, now);
                p_dfs->detail = "MARKER_UNUSABLE_RESUMING_SCAN";
                return true;
            }
        }
        if (!p_dfs->b_target_scan_marker_hold &&
            (strcmp(p_dfs->detail, "WAITING_FOR_FRESH_CAMERA") == 0 ||
             p_dfs->b_target_scanning))
        {

            p_dfs->detail = p_dfs->b_target_scanning
                                ? "SEARCHING_FOR_NEXT_TARGET"
                            : p_dfs->backtracking ? "SELECTING_PARENT_TARGET"
                                                  : "SELECTING_NEXT_TARGET";
            if (p_dfs->b_target_scanning &&
                p_dfs->target_scan_backup_ms >= DFS_TARGET_SCAN_BACKUP_TOTAL_MS)
            {
                p_dfs->detail = "SEARCHING_TARGET_REVERSE_LIMIT";
            }
        }
        if (p_selection->state != TARGET_LOCKED ||
            p_mission->leg.phase != LEG_CONFIGURED)
        {
            if (p_dfs->b_target_scan_marker_hold)
            {
                return true;
            }
            uint32_t dwell = p_dfs->b_target_scanning
                                 ? DFS_TARGET_SCAN_DWELL_MS
                                 : DFS_TARGET_SCAN_DELAY_MS;
            if ((uint32_t) (now - p_dfs->stage_started_ms) >=
                dwell + DFS_SETTLE_MS)
            {
                if (p_dfs->b_target_scanning && p_dfs->target_scan_step == 0 &&
                    !p_dfs->b_target_scan_backup_done &&
                    p_dfs->target_scan_backup_ms <
                        DFS_TARGET_SCAN_BACKUP_TOTAL_MS &&
                    error(wrap(p_drive->north_heading_deg +
                               90.0f * (float) p_dfs->direction),
                          p_inputs->drive.heading_deg) <= 3.0f)
                {
                    target_scan_backup(p_dfs, p_mission, p_selection, now);
                    return true;
                }
                start_target_scan(p_dfs, p_mission, p_drive, p_selection,
                                  p_inputs, now);
            }
            return true;
        }

        const char * p_result = onboard_drive_start(
            p_drive, p_mission, p_map, p_selection, &p_inputs->drive, now);

        if (strcmp(p_result, "STARTED") == 0)
        {
            p_dfs->b_target_scanning         = false;
            p_dfs->b_target_scan_marker_hold = false;
            p_dfs->stage                     = DFS_TRAVELLING;

            p_dfs->detail =
                p_dfs->backtracking ? "BACKTRACKING" : "EXPLORING_EXIT";
        }
        else if (strcmp(p_result, "GRID_DIRECTION_NOT_ALIGNED") == 0)
        {
            recover_grid_heading(p_dfs, p_mission, p_drive, p_selection,
                                 p_inputs, now);
        }
        else if (strncmp(p_result, "CAMERA_", 7) == 0 ||
                 strcmp(p_result, "TARGET_NOT_VISIBLE_AT_START") == 0)
        {
            return true;
        }
        else
        {
            onboard_dfs_stop(p_dfs, p_mission, p_result, now);
        }
        return true;
    }
    return false;
}

/** @brief Execute the reference stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_reference (onboard_dfs_t * p_dfs, rover_map_t * p_map,
                            const onboard_dfs_inputs_t * p_inputs, uint32_t now,
                            onboard_dfs_tick_context_t * p_context)
{

    if (p_dfs->stage != DFS_SAMPLING || (int32_t) (now - p_dfs->ready_ms) < 0)
    {
        return true;
    }

    p_context->p_node = rover_map_find_id(p_map, p_map->current_node);
    if (p_inputs->drive.sensor_received_ms != p_dfs->last_sensor_ms &&
        (int32_t) (p_inputs->drive.sensor_received_ms - p_dfs->ready_ms) >= 0)
    {
        p_dfs->last_sensor_ms = p_inputs->drive.sensor_received_ms;
        for (unsigned index = 0; index < TOF_SCAN_COUNT; ++index)
        {

            unsigned dir = ((unsigned) p_dfs->direction + (index == 0   ? 3u
                                                           : index == 1 ? 0u
                                                                        : 1u)) %
                           4u;

            uint16_t result = p_inputs->tof_mm[index];

            uint8_t code =
                (p_inputs->fault_mask & (1u << index)) || !result
                    ? MAP_EXIT_UNKNOWN
                : result == 1023u || result >= p_dfs->open_distance_mm
                    ? MAP_EXIT_OPEN
                    : MAP_EXIT_BLOCKED;

            if (code == MAP_EXIT_UNKNOWN)
            {
                p_dfs->sample_count[dir] = 0;
                continue;
            }

            p_dfs->sample_count[dir] =
                p_dfs->sample_code[dir] == code
                    ? (uint8_t) (p_dfs->sample_count[dir] + 1u)
                    : 1u;

            p_dfs->sample_code[dir] = code;
            if (p_dfs->sample_count[dir] >= 2u &&
                p_context->p_node->exits[dir] != MAP_EXIT_TRAVERSED)
            {
                rover_map_set_exit(p_map, p_context->p_node->marker_id,
                                   (map_direction_t) dir, (map_exit_t) code);
            }
        }
    }
    return false;
}

/** @brief Execute the arrival stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_arrival (onboard_dfs_t * p_dfs, mission_t * p_mission,
                          rover_map_t * p_map, onboard_drive_t * p_drive,
                          const onboard_dfs_inputs_t * p_inputs,
                          const camera_observation_t * p_camera, uint32_t now,
                          onboard_dfs_tick_context_t * p_context)
{

    // Wait a bounded interval even for unknown returns, then inspect another
    // heading.
    if ((uint32_t) (now - p_dfs->ready_ms) < DFS_SAMPLE_MS)
    {
        return true;
    }
    p_dfs->scanned[p_dfs->depth - 1u] |=
        (uint32_t) ((uint8_t) (1u << (unsigned) p_dfs->direction));

    if (p_dfs->node_limit && confirmed_nodes(p_map) >= p_dfs->node_limit)
    {
        // Finish a settled scan of the last node, without exploring another
        // exit. Give each unknown direction one attempt; retain unknown facts
        // rather than claiming that limited mapping exhausted the maze.
        for (unsigned dir = 0; dir < GRID_DIRECTION_COUNT; ++dir)
        {
            if (p_context->p_node->exits[dir] == MAP_EXIT_UNKNOWN &&
                !(p_dfs->scanned[p_dfs->depth - 1u] & (1u << dir)))
            {
                p_dfs->direction = (map_direction_t) dir;
                if (!aim(p_dfs, p_mission, p_drive, p_inputs, DFS_SAMPLING,
                         now))
                {
                    onboard_dfs_stop(p_dfs, p_mission, "DFS_TURN_REFUSED", now);
                }
                else
                {
                    p_dfs->detail = "SCANNING_FINAL_NODE";
                }
                return true;
            }
        }
        motor_emergency_stop();
        mission_finish_node_limit(p_mission, now);
        p_dfs->active = false;
        p_dfs->depth  = 0;
        p_dfs->stage  = DFS_FINISHED;
        p_dfs->detail = "NODE_LIMIT_REACHED";
        return true;
    }

    p_context->choice = onboard_dfs_choose(
        p_dfs, p_map, p_dfs->direction,
        camera_hint(p_camera, p_map, p_dfs->direction, p_dfs->ready_ms, now));

    if (p_context->choice.kind == DFS_CHOOSE_COMPLETE)
    {
        motor_emergency_stop();
        mission_finish_dfs(p_mission, now);
        p_dfs->active = false;
        p_dfs->depth  = 0;
        p_dfs->stage  = DFS_FINISHED;
        p_dfs->detail = "DFS_EXHAUSTED";
        return true;
    }
    if (p_context->choice.kind == DFS_CHOOSE_UNRESOLVED)
    {
        // Retry unknown directions without discarding confirmed map facts.
        // Cycle the retry heading independently of planner scan turns so a
        // different ToF sensor can eventually observe the same unknown side.
        for (unsigned dir = 0; dir < GRID_DIRECTION_COUNT; ++dir)
        {
            if (p_context->p_node->exits[dir] == MAP_EXIT_UNKNOWN)
            {
                p_dfs->scanned[p_dfs->depth - 1u] &= (uint8_t) ~(1u << dir);
            }
        }
        p_dfs->exit_retry_direction =
            (map_direction_t) (((unsigned) p_dfs->exit_retry_direction + 1u) %
                               GRID_DIRECTION_COUNT);
        p_dfs->direction = p_dfs->exit_retry_direction;
        if (!aim(p_dfs, p_mission, p_drive, p_inputs, DFS_SAMPLING, now))
        {
            onboard_dfs_stop(p_dfs, p_mission, "DFS_TURN_REFUSED", now);
        }
        else
        {
            p_dfs->detail = "RESCANNING_UNRESOLVED_EXITS";
        }
        return true;
    }
    if (p_context->choice.kind == DFS_CHOOSE_ERROR)
    {
        onboard_dfs_stop(p_dfs, p_mission, "DFS_STACK_MISMATCH", now);
        return true;
    }
    p_dfs->direction = p_context->choice.direction;
    if (p_context->choice.kind == DFS_CHOOSE_SCAN)
    {
        if (!aim(p_dfs, p_mission, p_drive, p_inputs, DFS_SAMPLING, now))
        {
            onboard_dfs_stop(p_dfs, p_mission, "DFS_TURN_REFUSED", now);
        }
        return true;
    }

    return false;
}

/** @brief Execute the recovery stage of onboard_dfs_tick. */
static bool
onboard_dfs_tick_recovery (onboard_dfs_t * p_dfs, mission_t * p_mission,
                           onboard_drive_t *            p_drive,
                           target_selector_t *          p_selection,
                           const onboard_dfs_inputs_t * p_inputs, uint32_t now,
                           onboard_dfs_tick_context_t * p_context)
{

    if (!cell(p_context->p_node, (unsigned) p_context->choice.direction,
              &p_context->x_coordinate, &p_context->y_coordinate))
    {
        onboard_dfs_stop(p_dfs, p_mission, "TARGET_CELL_OVERFLOW", now);
        return true;
    }
    p_dfs->backtracking      = p_context->choice.kind == DFS_CHOOSE_BACKTRACK;
    p_dfs->expected_id       = p_context->choice.expected_id;
    p_dfs->from_node         = p_context->p_node->marker_id;
    p_dfs->b_target_scanning = p_dfs->b_target_scan_rotating = false;
    p_dfs->target_scan_step                                  = 0;
    mission_clear_leg(p_mission, now);
    target_selector_set_route(
        p_selection,
        (target_route_t) {.enabled     = true,
                          .expected_id = p_context->choice.expected_id,
                          .x           = p_context->x_coordinate,
                          .y           = p_context->y_coordinate,
                          .direction =
                              (leg_direction_t) p_context->choice.direction});
    if (!aim(p_dfs, p_mission, p_drive, p_inputs, DFS_WAIT_TARGET, now))
    {
        onboard_dfs_stop(p_dfs, p_mission, "DFS_TURN_REFUSED", now);
    }
    else
    {

        p_dfs->detail = p_dfs->backtracking ? "SELECTING_PARENT_TARGET"
                                            : "SELECTING_NEXT_TARGET";
    }
    return false;
}

void
onboard_dfs_tick (onboard_dfs_t * p_dfs, mission_t * p_mission,
                  rover_map_t * p_map, onboard_drive_t * p_drive,
                  target_selector_t *          p_selection,
                  const onboard_dfs_inputs_t * p_inputs,
                  const camera_observation_t * p_camera, uint32_t now)
{
    onboard_dfs_tick_context_t context = {0};
    if (onboard_dfs_tick_prepare(p_dfs, p_mission, p_drive, p_inputs, now,
                                 &context))
    {
        return;
    }
    if (onboard_dfs_tick_inputs(p_dfs, p_mission, p_map, p_drive, p_selection,
                                p_inputs, now))
    {
        return;
    }
    if (onboard_dfs_tick_motion(p_dfs, p_mission, p_map, p_drive, p_inputs,
                                now))
    {
        return;
    }
    if (onboard_dfs_tick_camera(p_dfs, p_mission, p_drive, p_inputs, now))
    {
        return;
    }
    if (onboard_dfs_tick_tracking(p_dfs, p_mission, p_map, p_drive, p_selection,
                                  p_inputs, p_camera, now))
    {
        return;
    }
    if (onboard_dfs_tick_reference(p_dfs, p_map, p_inputs, now, &context))
    {
        return;
    }
    if (onboard_dfs_tick_arrival(p_dfs, p_mission, p_map, p_drive, p_inputs,
                                 p_camera, now, &context))
    {
        return;
    }
    if (onboard_dfs_tick_recovery(p_dfs, p_mission, p_drive, p_selection,
                                  p_inputs, now, &context))
    {
        return;
    }
}

size_t
onboard_dfs_format (const onboard_dfs_t * p_dfs, const rover_map_t * p_map,
                    char * p_out, size_t cap)
{
    if (!p_out || !cap)
    {
        return 0;
    }

    static const char * names[] = {"IDLE",     "AT_NODE",     "TURNING",
                                   "SCANNING", "WAIT_TARGET", "TRAVELLING",
                                   "COMPLETE", "STOPPED"};

    int count = snprintf(
        p_out, cap,
        "{\"active\":%s,\"stage\":\"%s\",\"current_node\":%u,\"depth\":%u,"
        "\"direction\":\"%s\",\"expected_node\":%u,\"detail\":\"%s\","
        "\"confirmed_nodes\":%u,\"node_limit\":%u}",
        p_dfs->active ? "true" : "false",
        p_dfs->active && p_dfs->input_waiting ? "WAITING_INPUT"
                                              : names[p_dfs->stage],
        (unsigned) p_map->current_node, (unsigned) p_dfs->depth,
        travel_leg_direction_name((leg_direction_t) p_dfs->direction),
        (unsigned) p_dfs->expected_id, p_dfs->detail, confirmed_nodes(p_map),
        (unsigned) p_dfs->node_limit);

    if (count < 0 || (size_t) count >= cap)
    {
        p_out[0] = '\0';
        return 0;
    }
    return (size_t) count;
}

/* End of file. */
