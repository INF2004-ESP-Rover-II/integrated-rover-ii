#include "travel_leg.h"

#include <stdio.h>

void travel_leg_init(travel_leg_t *leg)
{
    *leg = (travel_leg_t){.direction = LEG_DIRECTION_UNKNOWN, .phase = LEG_NONE};
}

bool travel_leg_clear(travel_leg_t *leg)
{
    bool changed = leg->departure_node || leg->next_target_node || leg->arrival_reference_node ||
        leg->target_x || leg->target_y || leg->target_cell_valid || leg->configured ||
        leg->direction != LEG_DIRECTION_UNKNOWN || leg->phase != LEG_NONE;
    uint32_t last_id = leg->leg_id;
    travel_leg_init(leg);
    leg->leg_id = last_id;
    return changed;
}

const char *travel_leg_phase_name(leg_phase_t phase)
{
    switch (phase) {
    case LEG_NONE: return "NONE";
    case LEG_SCANNING: return "SCANNING";
    case LEG_SELECTING: return "SELECTING";
    case LEG_CONFIGURED: return "CONFIGURED";
    case LEG_TURNING: return "TURNING";
    case LEG_APPROACHING: return "APPROACHING";
    case LEG_HOLDING: return "HOLDING";
    case LEG_BRAKING: return "BRAKING";
    case LEG_VERIFYING: return "VERIFYING";
    case LEG_COMPLETE: return "COMPLETE";
    case LEG_CANCELLED: return "CANCELLED";
    default: return "UNKNOWN";
    }
}

const char *travel_leg_direction_name(leg_direction_t direction)
{
    switch (direction) {
    case LEG_NORTH: return "NORTH";
    case LEG_EAST: return "EAST";
    case LEG_SOUTH: return "SOUTH";
    case LEG_WEST: return "WEST";
    default: return "UNKNOWN";
    }
}

static const char *node_json(uint8_t node, char *buffer, size_t capacity)
{
    if (!node) return "null";
    snprintf(buffer, capacity, "%u", (unsigned)node);
    return buffer;
}

size_t travel_leg_format_json(const travel_leg_t *leg, char *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    char departure[4], target[4], reference[4], cell[48], direction[16];
    if (leg->target_cell_valid)
        snprintf(cell, sizeof(cell), "{\"x\":%d,\"y\":%d}", (int)leg->target_x, (int)leg->target_y);
    else snprintf(cell, sizeof(cell), "null");
    if (leg->direction == LEG_DIRECTION_UNKNOWN) snprintf(direction, sizeof(direction), "null");
    else snprintf(direction, sizeof(direction), "\"%s\"", travel_leg_direction_name(leg->direction));
    int length = snprintf(out, capacity,
        "{\"leg_id\":%lu,\"configured\":%s,\"departure_node\":%s,\"next_target_node\":%s,"
        "\"arrival_reference_node\":%s,\"target_cell\":%s,\"direction\":%s,\"phase\":\"%s\"}",
        (unsigned long)leg->leg_id, leg->configured ? "true" : "false",
        node_json(leg->departure_node, departure, sizeof(departure)),
        node_json(leg->next_target_node, target, sizeof(target)),
        node_json(leg->arrival_reference_node, reference, sizeof(reference)), cell, direction,
        travel_leg_phase_name(leg->phase));
    if (length < 0 || (size_t)length >= capacity) { out[0] = '\0'; return 0; }
    return (size_t)length;
}

size_t travel_leg_format_status(const travel_leg_t *leg, char *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    char cell[24];
    if (leg->target_cell_valid) snprintf(cell, sizeof(cell), "%d,%d", (int)leg->target_x, (int)leg->target_y);
    else snprintf(cell, sizeof(cell), "UNKNOWN");
    int length = snprintf(out, capacity,
        "leg_id=%lu leg_configured=%u departure_node=%u next_target_node=%u arrival_reference_node=%u "
        "target_cell=%s direction=%s leg_phase=%s",
        (unsigned long)leg->leg_id, (unsigned)leg->configured, (unsigned)leg->departure_node,
        (unsigned)leg->next_target_node, (unsigned)leg->arrival_reference_node, cell,
        travel_leg_direction_name(leg->direction), travel_leg_phase_name(leg->phase));
    if (length < 0 || (size_t)length >= capacity) { out[0] = '\0'; return 0; }
    return (size_t)length;
}
