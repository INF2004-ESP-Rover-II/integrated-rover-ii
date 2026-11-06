#ifndef PLANNER_FIXTURE_H
#define PLANNER_FIXTURE_H
#include "rover_map.h"

/* Offline only: map provider for Buddy 3's planner. No DFS or hardware. */
bool planner_fixture_build(rover_map_t *map);
/* Instant simulated arrival on a reciprocal explored edge. No real movement. */
bool planner_fixture_arrive(rover_map_t *map, uint8_t target, unsigned heading);
#endif
