/**
 * @file travel_leg.c
 * @brief Travel leg firmware module.
 */
#include "travel_leg.h"
#include <inttypes.h>

#include <stdio.h>

void
travel_leg_init (travel_leg_t * p_leg)
{

    *p_leg =
        (travel_leg_t) {.direction = LEG_DIRECTION_UNKNOWN, .phase = LEG_NONE};
}

bool
travel_leg_clear (travel_leg_t * p_leg)
{

    bool b_changed =
        p_leg->departure_node || p_leg->next_target_node ||
        p_leg->arrival_reference_node || p_leg->target_x || p_leg->target_y ||
        p_leg->target_cell_valid || p_leg->configured ||
        p_leg->direction != LEG_DIRECTION_UNKNOWN || p_leg->phase != LEG_NONE;

    uint32_t last_id = p_leg->leg_id;
    travel_leg_init(p_leg);
    p_leg->leg_id = last_id;
    return b_changed;
}

const char *
travel_leg_phase_name (leg_phase_t phase)
{
    switch (phase)
    {
    case LEG_NONE:
        return "NONE";
    case LEG_SCANNING:
        return "SCANNING";
    case LEG_SELECTING:
        return "SELECTING";
    case LEG_CONFIGURED:
        return "CONFIGURED";
    case LEG_TURNING:
        return "TURNING";
    case LEG_APPROACHING:
        return "APPROACHING";
    case LEG_HOLDING:
        return "HOLDING";
    case LEG_BRAKING:
        return "BRAKING";
    case LEG_VERIFYING:
        return "VERIFYING";
    case LEG_BACKING_UP:
        return "BACKING_UP";
    case LEG_REACQUIRING:
        return "REACQUIRING";
    case LEG_SEARCHING_REFERENCE:
        return "SEARCHING_REFERENCE";
    case LEG_OBSTACLE_VERIFYING:
        return "OBSTACLE_VERIFYING";
    case LEG_ALIGNING:
        return "ALIGNING";
    case LEG_WAITING_CAMERA:
        return "WAITING_CAMERA";
    case LEG_WAITING_INPUT:
        return "WAITING_INPUT";
    case LEG_CORRECTING:
        return "CORRECTING";
    case LEG_COMPLETE:
        return "COMPLETE";
    case LEG_CANCELLED:
        return "CANCELLED";
    default:
        return "UNKNOWN";
    }
}

const char *
travel_leg_direction_name (leg_direction_t direction)
{
    switch (direction)
    {
    case LEG_NORTH:
        return "NORTH";
    case LEG_EAST:
        return "EAST";
    case LEG_SOUTH:
        return "SOUTH";
    case LEG_WEST:
        return "WEST";
    default:
        return "UNKNOWN";
    }
}

static const char *
node_json (uint8_t node, char * p_buffer, size_t capacity)
{
    if (!node)
    {
        return "null";
    }
    snprintf(p_buffer, capacity, "%u", (unsigned) node);
    return p_buffer;
}

size_t
travel_leg_format_json (const travel_leg_t * p_leg, char * p_out,
                        size_t capacity)
{
    if (!p_out || !capacity)
    {
        return 0;
    }
    char departure[4];
    char target[4];
    char reference[4];
    char cell[48];
    char direction[16];
    if (p_leg->target_cell_valid)
    {
        snprintf(cell, sizeof(cell), "{\"x\":%d,\"y\":%d}",
                 (int) p_leg->target_x, (int) p_leg->target_y);
    }
    else
    {
        snprintf(cell, sizeof(cell), "null");
    }
    if (p_leg->direction == LEG_DIRECTION_UNKNOWN)
    {
        snprintf(direction, sizeof(direction), "null");
    }
    else
    {
        snprintf(direction, sizeof(direction), "\"%s\"",
                 travel_leg_direction_name(p_leg->direction));
    }

    int length = snprintf(
        p_out, capacity,
        "{\"leg_id\":%" PRIu32
        ",\"configured\":%s,\"departure_node\":%s,\"next_target_node\":%s,"
        "\"arrival_reference_node\":%s,\"target_cell\":%s,\"direction\":%s,"
        "\"phase\":\"%s\"}",
        (uint32_t) p_leg->leg_id, p_leg->configured ? "true" : "false",
        node_json(p_leg->departure_node, departure, sizeof(departure)),
        node_json(p_leg->next_target_node, target, sizeof(target)),
        node_json(p_leg->arrival_reference_node, reference, sizeof(reference)),
        cell, direction, travel_leg_phase_name(p_leg->phase));

    if (length < 0 || (size_t) length >= capacity)
    {
        p_out[0] = '\0';
        return 0;
    }
    return (size_t) length;
}

size_t
travel_leg_format_status (const travel_leg_t * p_leg, char * p_out,
                          size_t capacity)
{
    if (!p_out || !capacity)
    {
        return 0;
    }
    char cell[24];
    if (p_leg->target_cell_valid)
    {
        snprintf(cell, sizeof(cell), "%d,%d", (int) p_leg->target_x,
                 (int) p_leg->target_y);
    }
    else
    {
        snprintf(cell, sizeof(cell), "UNKNOWN");
    }

    int length = snprintf(
        p_out, capacity,
        "leg_id=%" PRIu32 " leg_configured=%u departure_node=%u "
        "next_target_node=%u arrival_reference_node=%u "
        "target_cell=%s direction=%s leg_phase=%s",
        (uint32_t) p_leg->leg_id, (unsigned) p_leg->configured,
        (unsigned) p_leg->departure_node, (unsigned) p_leg->next_target_node,
        (unsigned) p_leg->arrival_reference_node, cell,
        travel_leg_direction_name(p_leg->direction),
        travel_leg_phase_name(p_leg->phase));

    if (length < 0 || (size_t) length >= capacity)
    {
        p_out[0] = '\0';
        return 0;
    }
    return (size_t) length;
}

/* End of file. */
