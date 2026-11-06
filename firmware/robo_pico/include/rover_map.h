#ifndef ROVER_MAP_H
#define ROVER_MAP_H
#include <stdbool.h>
#include <stdint.h>

/* Capacity limits memory, not row/column dimensions. +y is north. */
#define ROVER_MAX_NODES 64u
#define ROVER_MAX_MARKER_ID UINT8_MAX
#define ROVER_DEFAULT_GOAL ((ROVER_MAX_NODES < 36u) ? ROVER_MAX_NODES : 36u)
#if ROVER_MAX_NODES < 1u || ROVER_MAX_NODES > UINT8_MAX
#error "Node capacity must fit the one-byte count/stack indices"
#endif
#define ROVER_COORD_LIMIT ((int)ROVER_MAX_NODES - 1)
enum { MAP_NORTH, MAP_EAST, MAP_SOUTH, MAP_WEST };
enum { MAP_UNCHECKED, MAP_OPEN, MAP_EXPLORED, MAP_BLOCKED };
typedef struct {
    uint8_t marker_id;
    bool visited;
    uint8_t exits[4];
    int16_t x, y;
} rover_node_t;
typedef rover_node_t rover_cell_t; /* Preserve the read-only lookup API. */
typedef struct {
    rover_node_t nodes[ROVER_MAX_NODES];
    uint8_t node_count;
    int16_t min_x, max_x, min_y, max_y; /* Occupied bounds, not array offsets. */
    uint8_t start, current, visited_count;
    uint32_t revision;
} rover_map_t;
extern const int8_t rover_dx[4], rover_dy[4];
void rover_map_init(rover_map_t *map, uint8_t start_id);
const rover_cell_t *rover_map_at(const rover_map_t *map, int x, int y);
const rover_cell_t *rover_map_find(const rover_map_t *map, uint8_t id, int *x, int *y);
/* Add a reliable association; failure leaves the map unchanged. */
bool rover_map_place(rover_map_t *map, uint8_t id, int x, int y);
unsigned rover_map_rows(const rover_map_t *map);
unsigned rover_map_columns(const rover_map_t *map);
bool rover_map_visit(rover_map_t *map, uint8_t id);
bool rover_map_exit(rover_map_t *map, uint8_t id, unsigned direction, uint8_t state);
bool rover_map_connect(rover_map_t *map, uint8_t from, uint8_t to, unsigned direction);
#endif
