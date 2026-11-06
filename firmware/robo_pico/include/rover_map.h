/* rover_map.h - marker graph for Subsystem 4 (mapping), runs on the RoboPico.
 *
 * Port of rover_mapping/mapping/graph.py. Plain C11, integers only, NO malloc, NO Pico SDK includes,
 * so the same file builds on the RP2040 and on a laptop (host tests in test/).
 *
 * Nodes are markers. A marker's grid position is inferred from the heading of each hop:
 * pos = previous + step. Start marker = (0,0), heading 0 = +y (north), 1 = +x (east), 2 = south, 3 = west.
 */
#ifndef ROVER_MAP_H
#define ROVER_MAP_H

#include <stdbool.h>
#include <stdint.h>

#define MAP_MAX_MARKERS 36u     /* 6 x 6 arena */
#define MAP_ID_LIMIT    64u     /* marker IDs are 6 bit on the wire (0..63) */
#define MAP_NONE        0xFFu   /* "no marker" index */

enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3 };

typedef enum {
    EXIT_UNCHECKED  = 0,   /* not scanned yet */
    EXIT_UNEXPLORED = 1,   /* open, not travelled */
    EXIT_EXPLORED   = 2,   /* travelled, leads to a known marker */
    EXIT_BLOCKED    = 3    /* wall / far limit / blocked while driving */
} exit_status_t;

typedef struct {
    uint8_t id;
    int8_t  x, y;
    uint8_t exits[4];      /* exit_status_t per absolute direction */
    uint8_t nbr[4];        /* index of the driven neighbour in that direction, or MAP_NONE */
    uint8_t blocked[4];    /* 1 = the edge in that direction is blocked (both ends are set) */
} map_marker_t;

typedef enum {
    MAP_REACHED_KNOWN    = 0,   /* arrived at a marker that was already on the map */
    MAP_REACHED_NEW      = 1,   /* new marker added */
    MAP_REACHED_CONFLICT = -1,  /* position clashes with the map; map unchanged, counted in .inconsistencies */
    MAP_REACHED_ERROR    = -2   /* not started, bad ID or map full */
} map_reached_t;

typedef struct {
    map_marker_t markers[MAP_MAX_MARKERS];
    uint8_t  count;
    uint8_t  index_of[MAP_ID_LIMIT];   /* marker ID -> index in markers[], MAP_NONE if unknown */
    int16_t  start_id;                 /* -1 until started */
    uint8_t  current;                  /* index of the current marker, MAP_NONE until started */
    uint8_t  inconsistencies;          /* how many position clashes were seen */
    uint8_t  last_clash_id;            /* marker ID of the most recent clash (diagnostics) */
} rover_map_t;

/* ---- construction ---- */
void map_init(rover_map_t *m);                                   /* empty map */
bool map_start(rover_map_t *m, uint8_t marker_id);               /* first marker at (0,0) */
map_reached_t map_marker_reached(rover_map_t *m, uint8_t marker_id, uint8_t heading);

/* ---- exit statuses ---- */
void map_set_exit_status(rover_map_t *m, uint8_t marker_id, uint8_t d, exit_status_t s);
void map_apply_exit_report(rover_map_t *m, uint8_t marker_id, const uint8_t exits[4]);
void map_mark_blocked(rover_map_t *m, uint8_t marker_id, uint8_t d);
void map_link(rover_map_t *m, uint8_t a_id, uint8_t d, uint8_t b_id);   /* a driven connection */

/* ---- queries (indices are positions in markers[]; IDs are what the camera reports) ---- */
uint8_t map_index(const rover_map_t *m, uint8_t marker_id);            /* MAP_NONE if unknown */
uint8_t map_count(const rover_map_t *m);
uint8_t map_neighbour_at(const rover_map_t *m, uint8_t idx, uint8_t d); /* driven neighbour, else the marker one pitch away on the grid */
int     map_is_probable(const rover_map_t *m, uint8_t idx, uint8_t d);  /* -1 not probable, 0 possible, 1 likely (ToF saw it open) */
int     map_direction_between(const rover_map_t *m, uint8_t a_id, uint8_t b_id); /* -1 if not adjacent */

typedef struct { uint8_t dir; uint8_t idx; bool probable; } map_edge_t;
/* Usable edges out of idx: driven and not blocked; with use_probable also the probable ones.
 * Sorted by neighbour marker ID (ascending). Returns how many were written (max 4). */
uint8_t map_usable_neighbours(const rover_map_t *m, uint8_t idx, bool use_probable, map_edge_t out[4]);

/* ---- helpers shared with the planner ---- */
uint8_t map_opposite(uint8_t d);
int     map_turn_steps(int a, int b);    /* 90-degree turns needed to change heading a -> b (0..2) */

#endif
