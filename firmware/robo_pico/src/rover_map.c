/**
 * @file rover_map.c
 * @brief Rover map firmware module.
 */
#include "rover_map.h"
#include <inttypes.h>

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

_Static_assert(ROVER_MAP_CAPACITY > 0 && ROVER_MAP_CAPACITY <= 255,
               "Map capacity must be 1..255");

void
rover_map_init (rover_map_t * p_map, uint64_t boot_id)
{
    memset(p_map, 0, sizeof(*p_map));
    p_map->boot_id = boot_id;
}

void
rover_map_clear (rover_map_t * p_map)
{
    uint32_t revision = p_map->revision + 1u;
    uint64_t boot_id  = p_map->boot_id;
    rover_map_init(p_map, boot_id);
    p_map->revision = revision;
}

const map_node_t *
rover_map_find_id (const rover_map_t * p_map, uint8_t marker_id)
{
    for (size_t index = 0; index < p_map->count; ++index)
    {
        if (p_map->nodes[index].marker_id == marker_id)
        {
            return &p_map->nodes[index];
        }
    }
    return NULL;
}

const map_node_t *
rover_map_find_cell (const rover_map_t * p_map, int16_t x_coordinate,
                     int16_t y_coordinate)
{
    for (size_t index = 0; index < p_map->count; ++index)
    {
        if (p_map->nodes[index].x == x_coordinate &&
            p_map->nodes[index].y == y_coordinate)
        {
            return &p_map->nodes[index];
        }
    }
    return NULL;
}

map_result_t
rover_map_observe (rover_map_t * p_map, uint8_t marker_id, int16_t x_coordinate,
                   int16_t y_coordinate)
{
    if (!marker_id)
    {
        return MAP_INVALID_ID;
    }
    if (rover_map_find_id(p_map, marker_id))
    {
        return MAP_OK;
    }
    // Keep a first sighting stable when nearer markers leave the camera view.
    for (size_t index = 0; index < p_map->observed_count; ++index)
    {
        if (p_map->observed[index].marker_id == marker_id)
        {
            return MAP_OK;
        }
    }
    if (rover_map_find_cell(p_map, x_coordinate, y_coordinate))
    {
        return MAP_CELL_OCCUPIED;
    }
    for (size_t index = 0; index < p_map->observed_count; ++index)
    {
        if (p_map->observed[index].x == x_coordinate &&
            p_map->observed[index].y == y_coordinate)
        {
            return MAP_CELL_OCCUPIED;
        }
    }
    if (p_map->count + p_map->observed_count >= ROVER_MAP_CAPACITY)
    {
        return MAP_FULL;
    }

    p_map->observed[p_map->observed_count++] = (map_observed_node_t) {
        .marker_id = marker_id, .x = x_coordinate, .y = y_coordinate};

    p_map->revision++;
    return MAP_OK;
}

static void
remove_observed (rover_map_t * p_map, size_t index)
{
    memmove(&p_map->observed[index], &p_map->observed[index + 1u],
            (p_map->observed_count - index - 1u) * sizeof(p_map->observed[0]));
    p_map->observed_count--;
}

map_result_t
rover_map_add (rover_map_t * p_map, uint8_t marker_id, int16_t x_coordinate,
               int16_t y_coordinate, bool b_visited)
{
    if (!marker_id)
    {
        return MAP_INVALID_ID;
    }
    for (size_t index = 0; index < p_map->count; ++index)
    {
        map_node_t * p_node = &p_map->nodes[index];
        if (p_node->marker_id != marker_id)
        {
            continue;
        }
        if (p_node->x != x_coordinate || p_node->y != y_coordinate)
        {
            return MAP_ID_CONFLICT;
        }
        if (b_visited && !p_node->visited)
        {
            p_node->visited = true;
            p_map->revision++;
        }
        return MAP_OK;
    }
    if (rover_map_find_cell(p_map, x_coordinate, y_coordinate))
    {
        return MAP_CELL_OCCUPIED;
    }
    if (p_map->count >= ROVER_MAP_CAPACITY)
    {
        return MAP_FULL;
    }
    // Confirmed coordinates take priority over display estimates, including a
    // mistaken estimate occupying this cell. Camera sightings never set
    // visited.
    for (size_t index = 0; index < p_map->observed_count;)
    {
        if (p_map->observed[index].marker_id == marker_id ||
            (p_map->observed[index].x == x_coordinate &&
             p_map->observed[index].y == y_coordinate))
        {
            remove_observed(p_map, index);
        }
        else
        {
            ++index;
        }
    }
    if (p_map->count + p_map->observed_count >= ROVER_MAP_CAPACITY)
    {
        p_map->observed_count--;
    }

    p_map->nodes[p_map->count] = (map_node_t) {.marker_id = marker_id,
                                               .x         = x_coordinate,
                                               .y         = y_coordinate,
                                               .visited   = b_visited};

    if (!p_map->count)
    {
        p_map->min_x = p_map->max_x = x_coordinate;
        p_map->min_y = p_map->max_y = y_coordinate;
    }
    else
    {
        if (x_coordinate < p_map->min_x)
        {
            p_map->min_x = x_coordinate;
        }
        if (x_coordinate > p_map->max_x)
        {
            p_map->max_x = x_coordinate;
        }
        if (y_coordinate < p_map->min_y)
        {
            p_map->min_y = y_coordinate;
        }
        if (y_coordinate > p_map->max_y)
        {
            p_map->max_y = y_coordinate;
        }
    }
    p_map->count++;
    p_map->revision++;
    return MAP_OK;
}

map_result_t
rover_map_start (rover_map_t * p_map, uint8_t marker_id)
{
    if (!marker_id)
    {
        return MAP_INVALID_ID;
    }
    uint32_t revision = p_map->revision + 1u;
    uint64_t boot_id  = p_map->boot_id;
    rover_map_init(p_map, boot_id);
    p_map->nodes[0] = (map_node_t) {.marker_id = marker_id, .visited = true};
    p_map->count    = 1;
    p_map->current_node = marker_id;
    p_map->revision     = revision;
    return MAP_OK;
}

map_result_t
rover_map_arrive (rover_map_t * p_map, uint8_t marker_id)
{
    for (size_t index = 0; index < p_map->count; ++index)
    {
        if (p_map->nodes[index].marker_id == marker_id)
        {
            if (p_map->current_node != marker_id ||
                !p_map->nodes[index].visited)
            {
                p_map->nodes[index].visited = true;
                p_map->current_node         = marker_id;
                p_map->revision++;
            }
            return MAP_OK;
        }
    }
    return MAP_NOT_FOUND;
}

void
rover_map_depart (rover_map_t * p_map)
{
    if (p_map->current_node)
    {
        p_map->current_node = MAP_NODE_UNKNOWN;
        p_map->revision++;
    }
}

map_result_t
rover_map_set_exit (rover_map_t * p_map, uint8_t marker_id,
                    map_direction_t direction, map_exit_t exit)
{
    if ((unsigned) direction > MAP_WEST || (unsigned) exit > MAP_EXIT_BLOCKED)
    {
        return MAP_INVALID_EXIT;
    }
    for (size_t index = 0; index < p_map->count; ++index)
    {
        if (p_map->nodes[index].marker_id != marker_id)
        {
            continue;
        }
        if (p_map->nodes[index].exits[direction] != (uint8_t) exit)
        {
            p_map->nodes[index].exits[direction] = (uint8_t) exit;
            p_map->revision++;
        }
        return MAP_OK;
    }
    return MAP_NOT_FOUND;
}

uint32_t
rover_map_columns (const rover_map_t * p_map)
{
    return p_map->count ? (uint32_t) ((int32_t) p_map->max_x - p_map->min_x + 1)
                        : 0;
}

uint32_t
rover_map_rows (const rover_map_t * p_map)
{
    return p_map->count ? (uint32_t) ((int32_t) p_map->max_y - p_map->min_y + 1)
                        : 0;
}

static bool
append (char * p_out, size_t capacity, size_t * p_used, const char * p_format,
        ...)
{
    if (*p_used >= capacity)
    {
        return false;
    }
    va_list args;
    va_start(args, p_format);
    int length = vsnprintf(p_out + *p_used, capacity - *p_used, p_format, args);
    va_end(args);
    if (length < 0 || (size_t) length >= capacity - *p_used)
    {
        return false;
    }
    *p_used += (size_t) length;
    return true;
}

size_t
rover_map_format (const rover_map_t * p_map, const char * p_last_request,
                  char * p_out, size_t capacity)
{
    if (!p_out || !capacity)
    {
        return 0;
    }
    p_out[0] = '\0';
    // The request is empty or a 32-digit hex token, never arbitrary JSON text.
    size_t request_length = strlen(p_last_request);
    if (request_length && request_length != 32)
    {
        return 0;
    }
    for (size_t index = 0; index < request_length; ++index)
    {
        if (!strchr("0123456789abcdef", p_last_request[index]))
        {
            return 0;
        }
    }
    size_t used = 0;
    if (!append(
            p_out, capacity, &used,
            "{\"schema\":1,\"source\":\"robo_pico\",\"boot_id\":\"%016" PRIx64
            "\",\"revision\":%" PRIu32 ","
            "\"capacity\":%u,\"last_request\":\"%s\",\"current_node\":",
            (uint64_t) p_map->boot_id, (uint32_t) p_map->revision,
            (unsigned) ROVER_MAP_CAPACITY, p_last_request))
    {
        goto full;
    }
    if (p_map->current_node)
    {
        if (!append(p_out, capacity, &used, "%u",
                    (unsigned) p_map->current_node))
        {
            goto full;
        }
    }
    else if (!append(p_out, capacity, &used, "null"))
    {
        goto full;
    }
    if (!append(p_out, capacity, &used,
                ",\"columns\":%" PRIu32 ",\"rows\":%" PRIu32 ",\"bounds\":",
                (uint32_t) rover_map_columns(p_map),
                (uint32_t) rover_map_rows(p_map)))
    {
        goto full;
    }
    if (p_map->count)
    {
        if (!append(p_out, capacity, &used,
                    "{\"min_x\":%d,\"max_x\":%d,\"min_y\":%d,\"max_y\":%d}",
                    (int) p_map->min_x, (int) p_map->max_x, (int) p_map->min_y,
                    (int) p_map->max_y))
        {
            goto full;
        }
    }
    else if (!append(p_out, capacity, &used, "null"))
    {
        goto full;
    }
    if (!append(p_out, capacity, &used, ",\"nodes\":["))
    {
        goto full;
    }
    for (size_t index = 0; index < p_map->count; ++index)
    {
        const map_node_t * p_node = &p_map->nodes[index];
        if (!append(p_out, capacity, &used,
                    "%s{\"marker_id\":%u,\"x\":%d,\"y\":%d,\"visited\":%s,"
                    "\"exits\":[%u,%u,%u,%u]}",
                    index ? "," : "", (unsigned) p_node->marker_id,
                    (int) p_node->x, (int) p_node->y,
                    p_node->visited ? "true" : "false",
                    (unsigned) p_node->exits[0], (unsigned) p_node->exits[1],
                    (unsigned) p_node->exits[2], (unsigned) p_node->exits[3]))
        {
            goto full;
        }
    }
    if (!append(p_out, capacity, &used, "],\"observed_nodes\":["))
    {
        goto full;
    }
    for (size_t index = 0; index < p_map->observed_count; ++index)
    {
        const map_observed_node_t * p_node = &p_map->observed[index];
        if (!append(p_out, capacity, &used,
                    "%s{\"marker_id\":%u,\"x\":%d,\"y\":%d}", index ? "," : "",
                    (unsigned) p_node->marker_id, (int) p_node->x,
                    (int) p_node->y))
        {
            goto full;
        }
    }
    if (!append(p_out, capacity, &used, "]}"))
    {
        goto full;
    }
    return used;
full:
    p_out[0] = '\0';
    return 0;
}

const char *
rover_map_result_name (map_result_t result)
{
    switch (result)
    {
    case MAP_OK:
        return "OK";
    case MAP_INVALID_ID:
        return "INVALID_ID";
    case MAP_FULL:
        return "CAPACITY_FULL";
    case MAP_ID_CONFLICT:
        return "ID_COORDINATE_CONFLICT";
    case MAP_CELL_OCCUPIED:
        return "CELL_OCCUPIED";
    case MAP_NOT_FOUND:
        return "NOT_FOUND";
    case MAP_INVALID_EXIT:
        return "INVALID_EXIT";
    default:
        return "UNKNOWN";
    }
}

/* End of file. */
