#ifndef ROVER_MAP_H
#define ROVER_MAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef ROVER_MAP_CAPACITY
#define ROVER_MAP_CAPACITY 64u
#endif
#define ROVER_MAP_JSON_CAPACITY (100u * ROVER_MAP_CAPACITY + 512u)
#define MAP_NODE_UNKNOWN 0u

typedef enum { MAP_NORTH, MAP_EAST, MAP_SOUTH, MAP_WEST } map_direction_t;
typedef enum { MAP_EXIT_UNKNOWN, MAP_EXIT_OPEN, MAP_EXIT_TRAVERSED, MAP_EXIT_BLOCKED } map_exit_t;
typedef enum { MAP_OK, MAP_INVALID_ID, MAP_FULL, MAP_ID_CONFLICT, MAP_CELL_OCCUPIED,
               MAP_NOT_FOUND, MAP_INVALID_EXIT } map_result_t;

typedef struct {
    uint8_t marker_id;
    int16_t x, y;
    bool visited;
    uint8_t exits[4]; // N, E, S, W: unknown/open/traversed/blocked.
} map_node_t;

typedef struct {
    map_node_t nodes[ROVER_MAP_CAPACITY];
    size_t count;
    int16_t min_x, max_x, min_y, max_y;
    uint8_t current_node; // 0 = unknown/between nodes; otherwise an existing marker ID.
    uint64_t boot_id;
    uint32_t revision;
} rover_map_t;

void rover_map_init(rover_map_t *map, uint64_t boot_id);
map_result_t rover_map_start(rover_map_t *map, uint8_t marker_id);
// Coordinates must already be confirmed by the onboard localiser/caller.
// Camera sightings alone must never call this function with invented coordinates.
map_result_t rover_map_add(rover_map_t *map, uint8_t marker_id, int16_t x, int16_t y, bool visited);
const map_node_t *rover_map_find_id(const rover_map_t *map, uint8_t marker_id);
const map_node_t *rover_map_find_cell(const rover_map_t *map, int16_t x, int16_t y);
map_result_t rover_map_arrive(rover_map_t *map, uint8_t marker_id);
void rover_map_depart(rover_map_t *map);
map_result_t rover_map_set_exit(rover_map_t *map, uint8_t marker_id, map_direction_t direction, map_exit_t exit);
uint32_t rover_map_columns(const rover_map_t *map);
uint32_t rover_map_rows(const rover_map_t *map);
// Full authoritative snapshot; returns 0 instead of publishing truncated JSON.
size_t rover_map_format(const rover_map_t *map, const char *last_request, char *out, size_t capacity);
const char *rover_map_result_name(map_result_t result);

#endif
