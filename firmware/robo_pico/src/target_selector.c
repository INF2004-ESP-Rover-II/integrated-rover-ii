#include "target_selector.h"

#include <stdio.h>

void target_selector_init(target_selector_t *s)
{
    *s = (target_selector_t){.state = TARGET_NONE, .detail = "NEED_STOPPED_CONFIRMED_NODE"};
}

void target_selector_set_route(target_selector_t *s, target_route_t route)
{
    target_selector_init(s);
    s->route = route;
}

static bool context_ok(const mission_t *m, const rover_map_t *map, bool safe)
{
    const map_node_t *node = rover_map_find_id(map, map->current_node);
    return safe && !m->test_mode && (m->state == MISSION_IDLE || m->state == MISSION_READY ||
        (m->dfs_active && m->state == MISSION_EXPLORING)) &&
        node && node->visited;
}

static bool image_fresh(const camera_observation_t *ob, uint32_t now)
{
    uint32_t local_age = now - ob->received_ms;
    return ob->frame.status == VISION_STATUS_OK && ob->frame.count <= VISION_MAX_MARKERS &&
        local_age <= TARGET_UART_MAX_AGE_MS &&
        local_age + ob->frame.age_ms <= TARGET_CAMERA_MAX_AGE_MS;
}

static bool marker_usable(const camera_marker_t *marker)
{
    return marker->id && marker->quality == 0 && marker->range_mm &&
        marker->range_mm != UINT16_MAX && marker->bearing_cdeg >= -TARGET_MAX_BEARING_CDEG &&
        marker->bearing_cdeg <= TARGET_MAX_BEARING_CDEG;
}

static leg_direction_t adjacent_direction(const map_node_t *from, const map_node_t *to)
{
    int32_t x = (int32_t)to->x - from->x, y = (int32_t)to->y - from->y;
    if (x == 0 && y == 1) return LEG_NORTH;
    if (x == 1 && y == 0) return LEG_EAST;
    if (x == 0 && y == -1) return LEG_SOUTH;
    if (x == -1 && y == 0) return LEG_WEST;
    return LEG_DIRECTION_UNKNOWN;
}

static bool target_compatible(const rover_map_t *map, const target_route_t *route, uint8_t id)
{
    if (id == map->current_node) return false;
    const map_node_t *target = rover_map_find_id(map, id);
    if (route->enabled) {
        if (route->expected_id && id != route->expected_id) return false;
        const map_node_t *occupant = rover_map_find_cell(map, route->x, route->y);
        if ((occupant && occupant->marker_id != id) ||
            (target && (target->x != route->x || target->y != route->y))) return false;
    }
    if (!target) return true; // Provisional identity, never an inferred map cell.
    const map_node_t *from = rover_map_find_id(map, map->current_node);
    leg_direction_t d = adjacent_direction(from, target);
    return d != LEG_DIRECTION_UNKNOWN && from->exits[d] != MAP_EXIT_BLOCKED &&
        target->exits[((unsigned)d + 2u) % 4u] != MAP_EXIT_BLOCKED;
}

static bool reference_compatible(const rover_map_t *map, uint8_t departure, uint8_t target_id, uint8_t ref_id)
{
    const map_node_t *target = rover_map_find_id(map, target_id);
    const map_node_t *ref = rover_map_find_id(map, ref_id);
    if (!target || !ref) return true; // Optical candidate only; not validated layout geometry.
    const map_node_t *from = rover_map_find_id(map, departure);
    if (!from) return false;
    int32_t dx = (int32_t)target->x - from->x, dy = (int32_t)target->y - from->y;
    int32_t rx = (int32_t)ref->x - target->x, ry = (int32_t)ref->y - target->y;
    return rx * dy == ry * dx && rx * dx + ry * dy > 0;
}

static const camera_marker_t *nearest_target(const camera_frame_t *frame, const rover_map_t *map, const target_route_t *route)
{
    const camera_marker_t *best = NULL;
    bool tied = false;
    for (uint8_t i = 0; i < frame->count; ++i) {
        const camera_marker_t *marker = &frame->markers[i];
        if (!marker_usable(marker) || !target_compatible(map, route, marker->id)) continue;
        if (!best || marker->range_mm < best->range_mm) { best = marker; tied = false; }
        else if (marker->range_mm == best->range_mm && marker->id != best->id) tied = true;
    }
    return tied ? NULL : best;
}

static const camera_marker_t *find_marker(const camera_frame_t *frame, uint8_t id)
{
    for (uint8_t i = 0; i < frame->count; ++i)
        if (frame->markers[i].id == id && marker_usable(&frame->markers[i])) return &frame->markers[i];
    return NULL;
}

static const camera_marker_t *nearest_reference(const camera_frame_t *frame,
    const rover_map_t *map, uint8_t departure, uint8_t target_id, const camera_marker_t *target)
{
    // Establish optical ordering from the SAME fresh image, or use already
    // known map coordinates. Never compare a new image to an old lens range.
    if (!target && !rover_map_find_id(map, target_id)) return NULL;
    const camera_marker_t *best = NULL;
    bool tied = false;
    for (uint8_t i = 0; i < frame->count; ++i) {
        const camera_marker_t *marker = &frame->markers[i];
        int delta = target ? (int)marker->bearing_cdeg - target->bearing_cdeg : 0;
        if (!marker_usable(marker) || marker->id == target_id || marker->id == departure ||
            (!target && !rover_map_find_id(map, marker->id)) ||
            (target && (uint32_t)marker->range_mm < (uint32_t)target->range_mm + TARGET_REFERENCE_MIN_GAP_MM) ||
            delta < -TARGET_REFERENCE_MAX_DELTA_CDEG || delta > TARGET_REFERENCE_MAX_DELTA_CDEG ||
            !reference_compatible(map, departure, target_id, marker->id)) continue;
        if (!best || marker->range_mm < best->range_mm) { best = marker; tied = false; }
        else if (marker->range_mm == best->range_mm && marker->id != best->id) tied = true;
    }
    return tied ? NULL : best;
}

static void reset_confirmations(target_selector_t *s)
{
    if (s->state != TARGET_LOCKED) { s->marker = s->confirmations = 0; }
    if (s->state != TARGET_LOCKED) s->reference_marker = 0;
    if (!s->reference_marker) s->reference_candidate = s->reference_confirmations = 0;
    s->available = s->reference_available = false;
}

static void arm(target_selector_t *s, mission_t *m, const rover_map_t *map,
                const camera_observation_t *ob, uint32_t now)
{
    uint32_t leg_id = m->leg.leg_id + 1u;
    target_route_t route = s->route;
    *s = (target_selector_t){.state = TARGET_WAITING, .detail = "AUTOMATIC_WAITING_FOR_IMAGES",
        .route = route,
        .leg_id = leg_id, .started_ms = now, .map_revision = map->revision,
        .last_generation = ob ? ob->generation : 0, .last_seq = ob ? ob->frame.seq : 0,
        .have_seq = ob != NULL, .last_image_received_ms = now};
    m->leg = (travel_leg_t){.leg_id = leg_id, .departure_node = map->current_node,
        .direction = route.enabled ? route.direction : LEG_DIRECTION_UNKNOWN, .phase = LEG_SELECTING,
        .target_cell_valid = route.enabled, .target_x = route.x, .target_y = route.y};
    m->revision++;
    m->changed_ms = now;
}

static void note_change(mission_t *m, uint32_t now)
{
    m->revision++;
    m->changed_ms = now;
}

void target_selector_tick(target_selector_t *s, mission_t *m, const rover_map_t *map,
    const camera_observation_t *ob, bool safe, uint32_t now)
{
    if (!m->test_mode && !m->dfs_active && m->leg.phase == LEG_COMPLETE) {
        s->available = s->reference_available = false;
        s->detail = "TARGET_ARRIVAL_CONFIRMED";
        return;
    }
    bool approaching = !m->test_mode && m->state == MISSION_EXPLORING && m->leg.configured &&
        (m->leg.phase == LEG_APPROACHING || m->leg.phase == LEG_HOLDING || m->leg.phase == LEG_BRAKING || m->leg.phase == LEG_VERIFYING) &&
        m->leg.leg_id == s->leg_id && s->state == TARGET_LOCKED;
    if (!approaching && !context_ok(m, map, safe)) {
        if (s->state == TARGET_WAITING || s->state == TARGET_LOCKED) {
            if (m->leg.leg_id == s->leg_id) mission_clear_leg(m, now);
            s->state = TARGET_CANCELLED;
        }
        reset_confirmations(s);
        s->detail = "NEED_STOPPED_CONFIRMED_NODE";
        return;
    }
    if (!approaching && (s->state == TARGET_WAITING || s->state == TARGET_LOCKED) &&
        (s->map_revision != map->revision || m->leg.departure_node != map->current_node)) {
        mission_clear_leg(m, now);
        s->state = TARGET_CANCELLED;
    }
    if (m->leg.phase == LEG_NONE) {
        if (m->leg.leg_id == UINT32_MAX) {
            s->state = TARGET_CANCELLED;
            reset_confirmations(s);
            s->detail = "LEG_ID_EXHAUSTED";
            return;
        }
        arm(s, m, map, ob, now);
        return;
    }
    if (m->leg.leg_id != s->leg_id ||
        (m->leg.phase != LEG_SELECTING && m->leg.phase != LEG_CONFIGURED && !approaching)) return;
    if (!ob || !image_fresh(ob, now)) {
        reset_confirmations(s);
        s->detail = "CAMERA_UNAVAILABLE_OR_STALE";
        return;
    }
    if (now - s->last_image_received_ms > TARGET_CAMERA_MAX_AGE_MS) reset_confirmations(s);
    if (ob->generation == s->last_generation) return;
    s->last_generation = ob->generation;
    uint8_t delta = (uint8_t)(ob->frame.seq - s->last_seq);
    bool discontinuity = s->have_seq &&
        (delta >= 128u || now - s->last_image_received_ms > TARGET_CAMERA_MAX_AGE_MS);
    if (s->have_seq && !delta) return;
    s->last_seq = ob->frame.seq;
    s->have_seq = true;
    s->last_image_received_ms = now;
    uint32_t image_age = (uint32_t)ob->frame.age_ms + (now - ob->received_ms);
    if (discontinuity || now - s->started_ms < image_age) {
        reset_confirmations(s);
        s->detail = "WAITING_FOR_NEW_IMAGES";
        return;
    }
    s->sample_generation = ob->generation;
    s->sample_received_ms = ob->received_ms;
    s->sample_age_ms = ob->frame.age_ms;

    const camera_marker_t *target = s->state == TARGET_LOCKED ?
        find_marker(&ob->frame, m->leg.next_target_node) : nearest_target(&ob->frame, map, &s->route);
    if (s->state != TARGET_LOCKED) {
        if (!target) {
            reset_confirmations(s);
            s->detail = "NO_UNAMBIGUOUS_ELIGIBLE_TARGET";
            return;
        }
        if (s->marker != target->id) { s->reference_candidate = s->reference_confirmations = 0; }
        s->confirmations = s->marker == target->id ? (uint8_t)(s->confirmations + 1u) : 1u;
        s->marker = target->id;
        s->detail = "AUTOMATIC_CONFIRMING_TARGET";
        if (s->confirmations >= TARGET_CONFIRM_FRAMES) {
            m->leg.next_target_node = target->id;
            m->leg.configured = true;
            m->leg.phase = LEG_CONFIGURED;
            const map_node_t *known = rover_map_find_id(map, target->id);
            if (known) {
                m->leg.target_x = known->x;
                m->leg.target_y = known->y;
                m->leg.target_cell_valid = true;
                m->leg.direction = adjacent_direction(rover_map_find_id(map, map->current_node), known);
            }
            s->state = TARGET_LOCKED;
            note_change(m, now);
        }
    }
    s->available = target != NULL;
    if (target) { s->range_mm = target->range_mm; s->bearing_cdeg = target->bearing_cdeg; }

    if (m->leg.arrival_reference_node) {
        // Reference lock is independent of visibility and never follows a new nearest marker.
        const camera_marker_t *ref = find_marker(&ob->frame, m->leg.arrival_reference_node);
        s->reference_available = ref != NULL;
        if (ref) { s->reference_range_mm = ref->range_mm; s->reference_bearing_cdeg = ref->bearing_cdeg; }
    } else {
        const camera_marker_t *ref = nearest_reference(&ob->frame, map, m->leg.departure_node, s->marker, target);
        s->reference_available = false;
        if (!ref) { s->reference_candidate = s->reference_confirmations = 0; }
        else {
            s->reference_confirmations = s->reference_candidate == ref->id ?
                (uint8_t)(s->reference_confirmations + 1u) : 1u;
            s->reference_candidate = ref->id;
            s->reference_range_mm = ref->range_mm;
            s->reference_bearing_cdeg = ref->bearing_cdeg;
            if (s->state == TARGET_LOCKED && s->reference_confirmations >= TARGET_CONFIRM_FRAMES) {
                m->leg.arrival_reference_node = ref->id;
                s->reference_marker = ref->id;
                s->reference_available = true;
                note_change(m, now);
            }
        }
    }
    if (s->state == TARGET_LOCKED)
        s->detail = !s->available ? "TARGET_OBSERVATION_UNAVAILABLE" :
            !m->leg.arrival_reference_node ? "TARGET_LOCKED_WAITING_FOR_REFERENCE" :
            !s->reference_available ? "REFERENCE_OBSERVATION_UNAVAILABLE" : "TARGET_AND_REFERENCE_LOCKED";
}

size_t target_selector_format(const target_selector_t *s, char *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    static const char *states[] = {"NONE", "WAITING", "LOCKED", "CANCELLED"};
    char marker[8], ref[8];
    if (s->marker) snprintf(marker, sizeof(marker), "%u", (unsigned)s->marker);
    else snprintf(marker, sizeof(marker), "null");
    uint8_t reference = s->reference_marker ? s->reference_marker : s->reference_candidate;
    if (reference) snprintf(ref, sizeof(ref), "%u", (unsigned)reference);
    else snprintf(ref, sizeof(ref), "null");
    int length = snprintf(out, capacity,
        "{\"state\":\"%s\",\"request\":\"\",\"leg_id\":%lu,\"role\":\"AUTOMATIC\",\"direction\":null,"
        "\"marker\":%s,\"confirmations\":%u,\"available\":%s,\"range_mm\":%u,\"bearing_cdeg\":%d,"
        "\"reference_marker\":%s,\"reference_confirmations\":%u,\"reference_available\":%s,"
        "\"reference_range_mm\":%u,\"reference_bearing_cdeg\":%d,\"detail\":\"%s\"}",
        states[s->state], (unsigned long)s->leg_id, marker, (unsigned)s->confirmations,
        s->available ? "true" : "false", (unsigned)s->range_mm, (int)s->bearing_cdeg,
        ref, (unsigned)s->reference_confirmations, s->reference_available ? "true" : "false",
        (unsigned)s->reference_range_mm, (int)s->reference_bearing_cdeg, s->detail);
    if (length < 0 || (size_t)length >= capacity) { out[0] = '\0'; return 0; }
    return (size_t)length;
}
