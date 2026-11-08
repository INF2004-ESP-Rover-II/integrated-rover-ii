#include "rover_map.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

_Static_assert(ROVER_MAP_CAPACITY > 0 && ROVER_MAP_CAPACITY <= 255, "Map capacity must be 1..255");

void rover_map_init(rover_map_t *map, uint64_t boot_id)
{
    memset(map, 0, sizeof(*map));
    map->boot_id = boot_id;
}

const map_node_t *rover_map_find_id(const rover_map_t *map, uint8_t marker_id)
{
    for (size_t i = 0; i < map->count; ++i)
        if (map->nodes[i].marker_id == marker_id) return &map->nodes[i];
    return NULL;
}

const map_node_t *rover_map_find_cell(const rover_map_t *map, int16_t x, int16_t y)
{
    for (size_t i = 0; i < map->count; ++i)
        if (map->nodes[i].x == x && map->nodes[i].y == y) return &map->nodes[i];
    return NULL;
}

map_result_t rover_map_observe(rover_map_t *map, uint8_t marker_id, int16_t x, int16_t y)
{
    if (!marker_id) return MAP_INVALID_ID;
    if (rover_map_find_id(map, marker_id)) return MAP_OK;
    // Keep a first sighting stable when nearer markers leave the camera view.
    for (size_t i=0; i<map->observed_count; ++i)
        if (map->observed[i].marker_id == marker_id) return MAP_OK;
    if (rover_map_find_cell(map,x,y)) return MAP_CELL_OCCUPIED;
    for (size_t i=0; i<map->observed_count; ++i)
        if (map->observed[i].x==x && map->observed[i].y==y) return MAP_CELL_OCCUPIED;
    if (map->count + map->observed_count >= ROVER_MAP_CAPACITY) return MAP_FULL;
    map->observed[map->observed_count++] = (map_observed_node_t){.marker_id=marker_id,.x=x,.y=y};
    map->revision++;
    return MAP_OK;
}

static void remove_observed(rover_map_t *map, size_t i)
{
    memmove(&map->observed[i], &map->observed[i+1u],
        (map->observed_count-i-1u)*sizeof(map->observed[0]));
    map->observed_count--;
}

map_result_t rover_map_add(rover_map_t *map, uint8_t marker_id, int16_t x, int16_t y, bool visited)
{
    if (!marker_id) return MAP_INVALID_ID;
    for (size_t i = 0; i < map->count; ++i) {
        map_node_t *node = &map->nodes[i];
        if (node->marker_id != marker_id) continue;
        if (node->x != x || node->y != y) return MAP_ID_CONFLICT;
        if (visited && !node->visited) { node->visited = true; map->revision++; }
        return MAP_OK;
    }
    if (rover_map_find_cell(map, x, y)) return MAP_CELL_OCCUPIED;
    if (map->count >= ROVER_MAP_CAPACITY) return MAP_FULL;
    // Confirmed coordinates take priority over display estimates, including a
    // mistaken estimate occupying this cell. Camera sightings never set visited.
    for (size_t i=0; i<map->observed_count; ) {
        if (map->observed[i].marker_id==marker_id || (map->observed[i].x==x && map->observed[i].y==y))
            remove_observed(map,i);
        else ++i;
    }
    if (map->count + map->observed_count >= ROVER_MAP_CAPACITY)
        map->observed_count--;
    map->nodes[map->count] = (map_node_t){.marker_id = marker_id, .x = x, .y = y, .visited = visited};
    if (!map->count) {
        map->min_x = map->max_x = x;
        map->min_y = map->max_y = y;
    } else {
        if (x < map->min_x) map->min_x = x;
        if (x > map->max_x) map->max_x = x;
        if (y < map->min_y) map->min_y = y;
        if (y > map->max_y) map->max_y = y;
    }
    map->count++;
    map->revision++;
    return MAP_OK;
}

map_result_t rover_map_start(rover_map_t *map, uint8_t marker_id)
{
    if (!marker_id) return MAP_INVALID_ID;
    uint32_t revision = map->revision + 1u;
    uint64_t boot_id = map->boot_id;
    rover_map_init(map, boot_id);
    map->nodes[0] = (map_node_t){.marker_id = marker_id, .visited = true};
    map->count = 1;
    map->current_node = marker_id;
    map->revision = revision;
    return MAP_OK;
}

map_result_t rover_map_arrive(rover_map_t *map, uint8_t marker_id)
{
    for (size_t i = 0; i < map->count; ++i) {
        if (map->nodes[i].marker_id == marker_id) {
            if (map->current_node != marker_id || !map->nodes[i].visited) {
                map->nodes[i].visited = true;
                map->current_node = marker_id;
                map->revision++;
            }
            return MAP_OK;
        }
    }
    return MAP_NOT_FOUND;
}

void rover_map_depart(rover_map_t *map)
{
    if (map->current_node) { map->current_node = MAP_NODE_UNKNOWN; map->revision++; }
}

map_result_t rover_map_set_exit(rover_map_t *map, uint8_t marker_id, map_direction_t direction, map_exit_t exit)
{
    if ((unsigned)direction > MAP_WEST || (unsigned)exit > MAP_EXIT_BLOCKED) return MAP_INVALID_EXIT;
    for (size_t i = 0; i < map->count; ++i) {
        if (map->nodes[i].marker_id != marker_id) continue;
        if (map->nodes[i].exits[direction] != (uint8_t)exit) {
            map->nodes[i].exits[direction] = (uint8_t)exit;
            map->revision++;
        }
        return MAP_OK;
    }
    return MAP_NOT_FOUND;
}

uint32_t rover_map_columns(const rover_map_t *map)
{
    return map->count ? (uint32_t)((int32_t)map->max_x - map->min_x + 1) : 0;
}

uint32_t rover_map_rows(const rover_map_t *map)
{
    return map->count ? (uint32_t)((int32_t)map->max_y - map->min_y + 1) : 0;
}

static bool append(char *out, size_t capacity, size_t *used, const char *format, ...)
{
    if (*used >= capacity) return false;
    va_list args;
    va_start(args, format);
    int length = vsnprintf(out + *used, capacity - *used, format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= capacity - *used) return false;
    *used += (size_t)length;
    return true;
}

size_t rover_map_format(const rover_map_t *map, const char *last_request, char *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    out[0] = '\0';
    // The request is empty or a 32-digit hex token, never arbitrary JSON text.
    size_t request_length = strlen(last_request);
    if (request_length && request_length != 32) return 0;
    for (size_t i = 0; i < request_length; ++i)
        if (!strchr("0123456789abcdef", last_request[i])) return 0;
    size_t used = 0;
    if (!append(out, capacity, &used,
        "{\"schema\":1,\"source\":\"robo_pico\",\"boot_id\":\"%016llx\",\"revision\":%lu,"
        "\"capacity\":%u,\"last_request\":\"%s\",\"current_node\":",
        (unsigned long long)map->boot_id, (unsigned long)map->revision,
        (unsigned)ROVER_MAP_CAPACITY, last_request)) goto full;
    if (map->current_node) {
        if (!append(out, capacity, &used, "%u", (unsigned)map->current_node)) goto full;
    } else if (!append(out, capacity, &used, "null")) goto full;
    if (!append(out, capacity, &used, ",\"columns\":%lu,\"rows\":%lu,\"bounds\":",
        (unsigned long)rover_map_columns(map), (unsigned long)rover_map_rows(map))) goto full;
    if (map->count) {
        if (!append(out, capacity, &used, "{\"min_x\":%d,\"max_x\":%d,\"min_y\":%d,\"max_y\":%d}",
            (int)map->min_x, (int)map->max_x, (int)map->min_y, (int)map->max_y)) goto full;
    } else if (!append(out, capacity, &used, "null")) goto full;
    if (!append(out, capacity, &used, ",\"nodes\":[")) goto full;
    for (size_t i = 0; i < map->count; ++i) {
        const map_node_t *node = &map->nodes[i];
        if (!append(out, capacity, &used,
            "%s{\"marker_id\":%u,\"x\":%d,\"y\":%d,\"visited\":%s,\"exits\":[%u,%u,%u,%u]}",
            i ? "," : "", (unsigned)node->marker_id, (int)node->x, (int)node->y,
            node->visited ? "true" : "false", (unsigned)node->exits[0], (unsigned)node->exits[1],
            (unsigned)node->exits[2], (unsigned)node->exits[3])) goto full;
    }
    if (!append(out, capacity, &used, "],\"observed_nodes\":[")) goto full;
    for (size_t i=0; i<map->observed_count; ++i) {
        const map_observed_node_t *node=&map->observed[i];
        if (!append(out,capacity,&used,"%s{\"marker_id\":%u,\"x\":%d,\"y\":%d}",
            i?",":"",(unsigned)node->marker_id,(int)node->x,(int)node->y)) goto full;
    }
    if (!append(out, capacity, &used, "]}")) goto full;
    return used;
full:
    out[0] = '\0';
    return 0;
}

const char *rover_map_result_name(map_result_t result)
{
    switch (result) {
    case MAP_OK: return "OK";
    case MAP_INVALID_ID: return "INVALID_ID";
    case MAP_FULL: return "CAPACITY_FULL";
    case MAP_ID_CONFLICT: return "ID_COORDINATE_CONFLICT";
    case MAP_CELL_OCCUPIED: return "CELL_OCCUPIED";
    case MAP_NOT_FOUND: return "NOT_FOUND";
    case MAP_INVALID_EXIT: return "INVALID_EXIT";
    default: return "UNKNOWN";
    }
}
