#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H
/* Subsystem 4: shortest route over the verified graph of rover_map_t (read only).
 *
 * Rules (agreed with the B1 handover):
 *  - an edge is usable only if BOTH ends are MAP_EXPLORED and the neighbour node exists one grid step away;
 *    UNCHECKED, OPEN (not driven yet) and BLOCKED are never routed through
 *  - cost = 1000 per hop + 1 per 90 degrees turned (start heading optional), so fewest hops wins, then
 *    fewest turns, then the lexicographically smallest marker-ID path (deterministic)
 *  - search state is (node slot, heading); no malloc, no marker-ID indexing (IDs go up to 255) */
#include <stdbool.h>
#include <stdint.h>
#include "rover_map.h"

#define ROUTE_HEADING_UNKNOWN (-1)

typedef enum {
    ROUTE_OK = 0,              /* ids[0..hops], headings[0..hops-1] filled */
    ROUTE_ALREADY_THERE = 1,   /* start == goal: hops = 0, ids[0] = start */
    ROUTE_UNKNOWN_TARGET = 2,  /* goal not in the map */
    ROUTE_UNKNOWN_START = 3,   /* start not in the map */
    ROUTE_NO_ROUTE = 4         /* both exist, no verified connection */
} route_status_t;

typedef struct {
    uint8_t ids[ROVER_MAX_NODES];        /* marker IDs, ids[0] is the start */
    uint8_t headings[ROVER_MAX_NODES];   /* MAP_NORTH..MAP_WEST for each hop */
    uint8_t hops;
} route_t;

route_status_t route_plan(const rover_map_t *map, uint8_t start_id, uint8_t goal_id,
                          int start_heading, route_t *out);
const char *route_status_name(route_status_t s);
#endif
