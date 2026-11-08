#include "onboard_drive.h"
#include "motor_driver.h"
#include "motor_config.h"
#include "../../common/obstacle_threshold.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void onboard_drive_init(onboard_drive_t *drive)
{
    *drive = (onboard_drive_t){.result = "NOT_STARTED", .input_reason = "NONE"};
}

static const char *input_problem(const target_selector_t *s, const onboard_drive_inputs_t *in, uint32_t now)
{
    if (in->safety_latched) return "SAFETY_LOCKED";
    if (!in->link_ok) return "LINK_LOST";
    if (!in->sensor_valid || (uint32_t)(now - in->sensor_received_ms) > ONBOARD_DRIVE_SENSOR_MAX_AGE_MS)
        return "SENSOR_STALE";
    if (!in->imu_ok) return "HEADING_UNAVAILABLE";
    // The Sensor Pico fault bit tracks failed initialization/result timeouts.
    // A fresh zero with fault clear is a responding sensor with no usable
    // distance (e.g. no return). It is not an obstacle or a proven clear exit.
    // Permit this bounded visual approach; never turn it into a map/arrival fact.
    if (in->front_fault) return "FRONT_SENSOR_FAULT";
    if (in->front_urgent || front_obstacle_detected(in->front_mm, false)) return "FRONT_BLOCKED";
    if (!in->camera_present) return "CAMERA_UART_MISSING";
    if (in->camera_status != VISION_STATUS_OK) return in->camera_status == VISION_STATUS_NO_MARKER ?
        "CAMERA_NO_MARKERS" : in->camera_status == VISION_STATUS_STALLED ? "CAMERA_PROCESSING_STALLED" : "CAMERA_LOST";
    if (in->camera_packet_age_ms > TARGET_UART_MAX_AGE_MS) return "CAMERA_PACKET_STALE";
    if (in->camera_image_age_ms > ONBOARD_DRIVE_IMAGE_MAX_AGE_MS) return "CAMERA_IMAGE_STALE";
    if (!in->camera_fresh) return "CAMERA_DATA_UNUSABLE";
    if (!s->sample_generation) return "CAMERA_IMAGE_UNAVAILABLE";
    if ((uint32_t)(now - s->sample_received_ms) > TARGET_UART_MAX_AGE_MS) return "CAMERA_ACCEPTED_IMAGE_STALE";
    if ((uint32_t)(now - s->sample_received_ms) + s->sample_age_ms > ONBOARD_DRIVE_IMAGE_MAX_AGE_MS)
        return "CAMERA_IMAGE_STALE";
    return NULL;
}

static void note_inputs(onboard_drive_t *d, const target_selector_t *s,
                        const onboard_drive_inputs_t *in, uint32_t now)
{
    d->camera_present = in->camera_present;
    d->camera_status = in->camera_status;
    d->camera_packet_age_ms = in->camera_packet_age_ms;
    d->camera_image_age_ms = in->camera_image_age_ms;
    d->accepted_image_age_ms = s->sample_generation ? (uint32_t)(now - s->sample_received_ms) + s->sample_age_ms : UINT32_MAX;
}

static bool recoverable_camera_problem(const char *problem)
{
    return strncmp(problem, "CAMERA_", 7) == 0 || strcmp(problem, "TRACKING_MARKERS_MISSING") == 0 ||
        strcmp(problem, "REFERENCE_BASELINE_UNAVAILABLE") == 0;
}

static bool centred(int bearing)
{
    return bearing >= -ONBOARD_DRIVE_MAX_BEARING_CDEG && bearing <= ONBOARD_DRIVE_MAX_BEARING_CDEG;
}

static const char *track(onboard_drive_t *d, const mission_t *m, const target_selector_t *s)
{
    if (s->leg_id != d->leg_id || s->marker != m->leg.next_target_node) return "TRACKING_ID_CHANGED";
    d->estimate_valid = false;
    if (!s->available && !s->reference_available) return "TRACKING_MARKERS_MISSING";
    if (s->reference_available && m->leg.arrival_reference_node && s->reference_marker != m->leg.arrival_reference_node)
        return "TRACKING_ID_CHANGED";
    if (s->available && !centred(s->bearing_cdeg)) return "TARGET_NOT_ALIGNED";
    bool ref = s->reference_available && m->leg.arrival_reference_node &&
        s->reference_marker == m->leg.arrival_reference_node;
    if (!s->available && (!ref || !d->reference_model_valid)) return "REFERENCE_BASELINE_UNAVAILABLE";
    if (!s->available && (s->reference_bearing_cdeg < -TARGET_MAX_BEARING_CDEG ||
        s->reference_bearing_cdeg > TARGET_MAX_BEARING_CDEG)) return "REFERENCE_NOT_ALIGNED";
    if (d->reference_model_valid && m->leg.arrival_reference_node != d->reference_id) return "TRACKING_ID_CHANGED";
    if (s->sample_generation != d->last_sample_generation) {
        if (ref && d->reference_model_valid) {
            int32_t step = d->last_reference_mm - s->reference_range_mm;
            if (step > ONBOARD_DRIVE_REFERENCE_MAX_STEP_MM || step < -ONBOARD_DRIVE_REFERENCE_BACKWARDS_MM)
                return "REFERENCE_RANGE_INCONSISTENT";
        }
        // Freeze a synchronised pair when the target leaves view. This is the
        // user's empirical range-delta model, not a calibrated ground pose.
        if (s->available && ref) {
            int bearing_delta = (int)s->reference_bearing_cdeg - s->bearing_cdeg;
            if ((uint32_t)s->reference_range_mm < (uint32_t)s->range_mm + TARGET_REFERENCE_MIN_GAP_MM ||
                bearing_delta < -TARGET_REFERENCE_MAX_DELTA_CDEG || bearing_delta > TARGET_REFERENCE_MAX_DELTA_CDEG)
                return "REFERENCE_GEOMETRY_INCONSISTENT";
            if (!d->reference_model_valid) d->reference_origin_mm = s->reference_range_mm;
            d->reference_model_valid = true;
            d->reference_id = m->leg.arrival_reference_node;
            d->target_anchor_mm = s->range_mm;
            d->reference_anchor_mm = s->reference_range_mm;
        }
        if (ref) d->last_reference_mm = s->reference_range_mm;
        d->last_sample_generation = s->sample_generation;
    }
    d->using_reference = !s->available;
    d->remaining_mm = s->available ? (int32_t)s->range_mm :
        d->target_anchor_mm - (d->reference_anchor_mm - (int32_t)s->reference_range_mm);
    if (ref && d->reference_model_valid)
        d->reference_progress_mm = d->reference_origin_mm - (int32_t)s->reference_range_mm;
    d->estimate_valid = true;
    return NULL;
}

static const char *plan_cell(const onboard_drive_t *d, const rover_map_t *map, uint8_t target_id,
    float heading, float *north, leg_direction_t *direction, int16_t *x, int16_t *y)
{
    if (!isfinite(heading) || heading < 0.0f || heading >= 360.0f) return "HEADING_UNAVAILABLE";
    if (!d->frame_valid && map->count != 1u) return "GRID_HEADING_UNAVAILABLE";
    *north = d->frame_valid ? d->north_heading_deg : heading;
    float delta = fmodf(heading - *north + 360.0f, 360.0f);
    unsigned cardinal = (unsigned)((delta + 45.0f) / 90.0f) % 4u;
    float error = fabsf(delta - cardinal * 90.0f);
    if (error > 180.0f) error = 360.0f - error;
    if (error > 15.0f) return "GRID_DIRECTION_NOT_ALIGNED";
    static const int8_t dx[] = {0,1,0,-1}, dy[] = {1,0,-1,0};
    const map_node_t *from = rover_map_find_id(map, map->current_node);
    int32_t tx = (int32_t)from->x + dx[cardinal], ty = (int32_t)from->y + dy[cardinal];
    if (tx < INT16_MIN || tx > INT16_MAX || ty < INT16_MIN || ty > INT16_MAX) return "TARGET_CELL_OVERFLOW";
    const map_node_t *known = rover_map_find_id(map, target_id);
    const map_node_t *occupant = rover_map_find_cell(map, (int16_t)tx, (int16_t)ty);
    if ((known && (known->x != tx || known->y != ty)) || (occupant && occupant->marker_id != target_id))
        return "TARGET_MAP_CONFLICT";
    if (from->exits[cardinal] == MAP_EXIT_BLOCKED ||
        (known && known->exits[(cardinal + 2u) % 4u] == MAP_EXIT_BLOCKED)) return "FRONT_BLOCKED";
    if (!known && map->count >= ROVER_MAP_CAPACITY) return "MAP_FULL";
    *direction = (leg_direction_t)cardinal;
    *x = (int16_t)tx; *y = (int16_t)ty;
    return NULL;
}

const char *onboard_drive_start(onboard_drive_t *d, mission_t *m, rover_map_t *map,
    const target_selector_t *s, const onboard_drive_inputs_t *in, uint32_t now)
{
    // Check this on RoboPico, never rely on the dashboard's cached map.
    if (!map->count) return "START_NODE_NOT_SET";
    if (d->active && m->state == MISSION_EXPLORING && d->leg_id == m->leg.leg_id) return "ALREADY_RUNNING";
    const map_node_t *current = rover_map_find_id(map, map->current_node);
    if (!current || !current->visited) return "CURRENT_NODE_UNKNOWN";
    if (in->safety_latched) return "SAFETY_LOCKED";
    if (m->test_mode || (m->state != MISSION_IDLE && m->state != MISSION_READY &&
        !(m->dfs_active && m->state == MISSION_EXPLORING))) return "MISSION_NOT_IDLE";
    if (!in->stopped) return "MOTOR_BUSY";
    if (s->state != TARGET_LOCKED || !m->leg.configured || m->leg.phase != LEG_CONFIGURED ||
        !m->leg.next_target_node || s->leg_id != m->leg.leg_id || s->marker != m->leg.next_target_node ||
        m->leg.departure_node != map->current_node) return "NO_LOCKED_TARGET";
    const char *problem = input_problem(s, in, now);
    if (problem) return problem;
    if (!s->available) return "TARGET_NOT_VISIBLE_AT_START";
    if (!centred(s->bearing_cdeg)) return "TARGET_NOT_ALIGNED";
    if (s->range_mm <= ONBOARD_DRIVE_NEAR_RANGE_MM) return "TARGET_TOO_CLOSE";
    float north;
    leg_direction_t direction;
    int16_t x, y;
    problem = plan_cell(d, map, m->leg.next_target_node, in->heading_deg, &north, &direction, &x, &y);
    if (problem) return problem;
    onboard_drive_t prepared = {.active = true, .leg_id = m->leg.leg_id,
        .started_ms = now, .last_refresh_ms = now, .result = "APPROACHING",
        .frame_valid = true, .north_heading_deg = north, .last_tick_ms = now, .input_reason = "NONE"};
    note_inputs(&prepared, s, in, now);
    problem = track(&prepared, m, s);
    if (problem) return problem;
    motor_error_t result = motor_auto_drive_leased(MC_AUTO_APPROACH_PCT, ONBOARD_DRIVE_LEASE_MS);
    if (result != MOTOR_OK) return result == MOTOR_ERR_NO_HEADING ? "HEADING_UNAVAILABLE" :
        result == MOTOR_ERR_FRONT_BLOCKED ? "FRONT_BLOCKED" : "MOTOR_REFUSED";
    if (!mission_begin_drive(m, now)) {
        motor_emergency_stop();
        return "MISSION_NOT_IDLE";
    }
    *d = prepared;
    m->leg.direction = direction;
    m->leg.target_x = x; m->leg.target_y = y;
    m->leg.target_cell_valid = true; // Planned adjacency, not yet inserted/visited.
    rover_map_depart(map); // Only after motor acceptance; no invented arrival.
    d->map_revision = map->revision;
    return "STARTED";
}

void onboard_drive_stop(onboard_drive_t *d, mission_t *m, mission_reason_t reason,
                        const char *result, uint32_t now)
{
    motor_emergency_stop();
    mission_pause(m, reason, now);
    d->active = false;
    d->estimate_valid = false;
    d->result = result;
}

void onboard_drive_tick(onboard_drive_t *d, mission_t *m, rover_map_t *map, const target_selector_t *s,
                        const onboard_drive_inputs_t *in, uint32_t now)
{
    if (!d->active) return;
    uint32_t elapsed = now - d->last_tick_ms;
    d->last_tick_ms = now;
    if (m->leg.phase == LEG_APPROACHING) d->driving_ms += elapsed;
    note_inputs(d, s, in, now);
    if (m->state != MISSION_EXPLORING || m->test_mode || !m->leg.configured || m->leg.leg_id != d->leg_id) {
        motor_emergency_stop();
        if (m->state == MISSION_EXPLORING) mission_pause(m, MISSION_REASON_MOTOR_STOPPED, now);
        d->active = false;
        d->estimate_valid = false;
        d->result = m->reason == MISSION_REASON_LINK_LOST ? "LINK_LOST" :
            m->reason == MISSION_REASON_FRONT_STOP ? "FRONT_BLOCKED" :
            m->reason == MISSION_REASON_ESTOP ? "SAFETY_LOCKED" : "CANCELLED";
        return;
    }
    const char *problem = input_problem(s, in, now);
    if ((!problem || recoverable_camera_problem(problem)) &&
        (uint32_t)(now - d->started_ms) >= ONBOARD_DRIVE_MAX_WALL_MS) problem = "LEG_TIME_LIMIT";
    if (!problem && map->revision != d->map_revision) problem = "MAP_CHANGED_DURING_LEG";
    if (!problem) problem = track(d, m, s);
    if (problem) {
        d->input_reason = problem;
        d->estimate_valid = false;
        if (recoverable_camera_problem(problem)) {
            if (m->leg.phase == LEG_BRAKING || m->leg.phase == LEG_VERIFYING) {
                // Cached-frame expiry/missing detections supply no contradictory
                // position measurement. Keep distinct valid post-brake samples
                // within the bounded verification window; wheels stay stopped.
                if ((uint32_t)(now - d->braked_ms) < ONBOARD_DRIVE_VERIFY_TIMEOUT_MS) {
                    d->result = "WAITING_FOR_CAMERA_VERIFICATION";
                    return; // Already stopped; wait for fresh confirmation, never restart motion.
                }
            } else {
                if (m->leg.phase == LEG_APPROACHING) {
                    motor_emergency_stop();
                    mission_camera_hold(m, true, now);
                    d->hold_started_ms = now;
                    d->recovery_generation = s->sample_generation;
                    d->recovery_frames = 0;
                }
                if ((uint32_t)(now - d->hold_started_ms) < ONBOARD_DRIVE_CAMERA_RECOVERY_MS) {
                    // Likewise preserve a valid recovery image while waiting
                    // for the next distinct one. Never count stale/missing input.
                    d->result = "HOLDING_FOR_CAMERA";
                    return;
                }
            }
            problem = m->leg.phase == LEG_BRAKING || m->leg.phase == LEG_VERIFYING ?
                "ARRIVAL_VERIFICATION_TIMEOUT" : "CAMERA_RECOVERY_TIMEOUT";
        }
        onboard_drive_stop(d, m, in->safety_latched ? MISSION_REASON_ESTOP :
            !in->link_ok ? MISSION_REASON_LINK_LOST : in->front_urgent || front_obstacle_detected(in->front_mm, in->front_fault) ?
            MISSION_REASON_FRONT_STOP : strcmp(problem, "LEG_TIME_LIMIT") == 0 ? MISSION_REASON_DRIVE_TIMEOUT :
            MISSION_REASON_INPUT_LOST, problem, now);
        return;
    }
    bool resuming = false;
    if (m->leg.phase == LEG_HOLDING) {
        if ((uint32_t)(now - d->hold_started_ms) >= ONBOARD_DRIVE_CAMERA_RECOVERY_MS) {
            onboard_drive_stop(d, m, MISSION_REASON_INPUT_LOST, "CAMERA_RECOVERY_TIMEOUT", now);
            return;
        }
        uint32_t image_ms = s->sample_received_ms - s->sample_age_ms;
        if ((int32_t)(image_ms - d->hold_started_ms) < 0) {
            d->result = "WAITING_FOR_POST_HOLD_IMAGE";
            return;
        }
        if (s->sample_generation == d->recovery_generation) return;
        d->recovery_generation = s->sample_generation;
        if (++d->recovery_frames < ONBOARD_DRIVE_RECOVERY_FRAMES) {
            d->result = "CONFIRMING_CAMERA_RECOVERY";
            return;
        }
        mission_camera_hold(m, false, now);
        resuming = true;
    }
    d->input_reason = "NONE";
    if (m->leg.phase == LEG_BRAKING || m->leg.phase == LEG_VERIFYING) {
        if ((uint32_t)(now - d->braked_ms) >= ONBOARD_DRIVE_VERIFY_TIMEOUT_MS) {
            onboard_drive_stop(d, m, MISSION_REASON_INPUT_LOST, "ARRIVAL_VERIFICATION_TIMEOUT", now);
            return;
        }
        if ((uint32_t)(now - d->braked_ms) < ONBOARD_DRIVE_SETTLE_MS) return;
        if (m->leg.phase == LEG_BRAKING) {
            mission_arrival_phase(m, LEG_VERIFYING, now);
        }
        d->result = "VERIFYING_ARRIVAL";
        // Do not confirm using cached/pre-brake images from a delayed pipeline.
        uint32_t image_ms = s->sample_received_ms - s->sample_age_ms;
        if ((int32_t)(image_ms - (d->braked_ms + ONBOARD_DRIVE_SETTLE_MS)) < 0 ||
            s->sample_generation == d->last_verify_generation) return;
        d->last_verify_generation = s->sample_generation;
        if (!d->using_reference || !d->reference_model_valid ||
            d->remaining_mm < -ONBOARD_DRIVE_VERIFY_ERROR_MM || d->remaining_mm > ONBOARD_DRIVE_VERIFY_ERROR_MM) {
            onboard_drive_stop(d, m, MISSION_REASON_INPUT_LOST, "ARRIVAL_ESTIMATE_NOT_CONFIRMED", now);
            return;
        }
        if (++d->verification_frames < ONBOARD_DRIVE_VERIFY_FRAMES) return;
        if (!m->leg.target_cell_valid || rover_map_add(map, m->leg.next_target_node,
                m->leg.target_x, m->leg.target_y, false) != MAP_OK ||
            rover_map_arrive(map, m->leg.next_target_node) != MAP_OK) {
            onboard_drive_stop(d, m, MISSION_REASON_INPUT_LOST, "ARRIVAL_MAP_COMMIT_FAILED", now);
            return;
        }
        mission_complete_target(m, now);
        rover_map_set_exit(map, m->leg.departure_node, (map_direction_t)m->leg.direction, MAP_EXIT_TRAVERSED);
        rover_map_set_exit(map, m->leg.next_target_node, (map_direction_t)(((unsigned)m->leg.direction + 2u) % 4u), MAP_EXIT_TRAVERSED);
        d->active = false;
        d->result = "ARRIVED";
        return;
    }
    if (d->using_reference && d->remaining_mm <= ONBOARD_DRIVE_ARRIVAL_STOP_MM) {
        if (d->remaining_mm < -ONBOARD_DRIVE_VERIFY_ERROR_MM) {
            onboard_drive_stop(d, m, MISSION_REASON_APPROACH_LIMIT, "TARGET_OVERSHOOT_UNCONFIRMED", now);
            return;
        }
        motor_emergency_stop();
        mission_arrival_phase(m, LEG_BRAKING, now);
        d->braked_ms = now;
        d->result = "BRAKING_FOR_ARRIVAL";
        return;
    }
    if (!d->using_reference && s->range_mm <= ONBOARD_DRIVE_NEAR_RANGE_MM) {
        onboard_drive_stop(d, m, MISSION_REASON_APPROACH_LIMIT, "NEAR_TARGET_UNCONFIRMED", now);
        return;
    }
    if (d->driving_ms >= ONBOARD_DRIVE_MAX_MS) {
        onboard_drive_stop(d, m, MISSION_REASON_DRIVE_TIMEOUT, "DRIVE_TIME_LIMIT", now);
        return;
    }
    if (resuming) {
        motor_error_t result = motor_auto_drive_leased(MC_AUTO_APPROACH_PCT, ONBOARD_DRIVE_LEASE_MS);
        if (result != MOTOR_OK) {
            onboard_drive_stop(d, m, MISSION_REASON_MOTOR_STOPPED, "MOTOR_REFUSED", now);
            return;
        }
        d->last_refresh_ms = now;
        d->result = "RESUMED_AFTER_CAMERA_GAP";
        return;
    }
    if (motor_auto_status() != MOTOR_AUTO_RUNNING) {
        onboard_drive_stop(d, m, MISSION_REASON_MOTOR_STOPPED, "MOTOR_STOPPED", now);
        return;
    }
    if ((uint32_t)(now - d->last_refresh_ms) >= ONBOARD_DRIVE_REFRESH_MS) {
        if (!motor_auto_renew_lease()) {
            onboard_drive_stop(d, m, MISSION_REASON_MOTOR_STOPPED, "DRIVE_LEASE_EXPIRED", now);
            return;
        }
        d->last_refresh_ms = now;
    }
}

size_t onboard_drive_format(const onboard_drive_t *d, const mission_t *m, char *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    char remaining[20];
    if (d->estimate_valid) snprintf(remaining, sizeof(remaining), "%ld", (long)d->remaining_mm);
    else snprintf(remaining, sizeof(remaining), "null");
    int length = snprintf(out, capacity,
        "{\"leg_id\":%lu,\"active\":%s,\"phase\":\"%s\",\"source\":\"%s\",\"remaining_mm\":%s,"
        "\"reference_progress_mm\":%ld,\"reference_model_valid\":%s,\"verification_frames\":%u,\"result\":\"%s\","
        "\"input_reason\":\"%s\",\"camera_present\":%s,\"camera_status\":%u,\"camera_packet_age_ms\":%lu,"
        "\"camera_image_age_ms\":%lu,\"accepted_image_age_ms\":%lu,\"recovery_frames\":%u,\"driving_ms\":%lu}",
        (unsigned long)d->leg_id, d->active ? "true" : "false", travel_leg_phase_name(m->leg.phase),
        !d->estimate_valid ? "NONE" : d->using_reference ? "REFERENCE" : "TARGET", remaining,
        (long)d->reference_progress_mm, d->reference_model_valid ? "true" : "false",
        (unsigned)d->verification_frames, d->result, d->input_reason ? d->input_reason : "NONE",
        d->camera_present ? "true" : "false", (unsigned)d->camera_status,
        (unsigned long)d->camera_packet_age_ms, (unsigned long)d->camera_image_age_ms,
        (unsigned long)d->accepted_image_age_ms, (unsigned)d->recovery_frames, (unsigned long)d->driving_ms);
    if (length < 0 || (size_t)length >= capacity) { out[0] = '\0'; return 0; }
    return (size_t)length;
}
