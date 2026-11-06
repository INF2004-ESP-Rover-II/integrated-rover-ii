/* route_planner.h - A* over the marker graph (FR-27). Port of rover_mapping/mapping/planner.py.
 * Plain C11, no malloc, no Pico SDK includes. Static working memory is about 15 KB (see route_planner.c).
 *
 * Cost = hops * W + 90-degree turns (+ PROBABLE_PENALTY per never-driven edge), so the route has the
 * fewest hops first, then the fewest turns; remaining ties go to the lexicographically smallest
 * marker-ID path, which makes the result deterministic. Heuristic = Manhattan distance * W
 * (admissible: the markers sit on a regular grid, one hop = one grid step). */
#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "rover_map.h"

#define PLAN_W                1000   /* larger than any possible number of turns */
#define PLAN_PROBABLE_PENALTY 200    /* an unproven edge costs this much extra */
#define PLAN_MAX_PATH         (MAP_MAX_MARKERS + 1u)

/* Plan start_id -> goal_id. start_heading is 0..3, or -1 when the heading is unknown.
 * Writes marker IDs (start first, goal last) into path_ids[] and returns how many; returns 0 if there is
 * no route, an unknown marker, or max_len is too small. start == goal returns 1. */
int plan_route(const rover_map_t *m, uint8_t start_id, uint8_t goal_id, int start_heading,
               bool use_probable, uint8_t *path_ids, int max_len);

/* Hop list for a path: heading[i] / target_id[i] for hop i (path[i] -> path[i+1]).
 * Returns the number of hops, or -1 if two consecutive markers are not adjacent. */
int plan_path_headings(const rover_map_t *m, const uint8_t *path_ids, int path_len,
                       uint8_t *heading, uint8_t *target_id);

#endif
