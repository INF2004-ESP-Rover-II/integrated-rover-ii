#ifndef GOTO_DRY_H
#define GOTO_DRY_H
/* Subsystem 4: plan-only test helpers. They never touch motors or the real map. */
#include <stddef.h>
#include <stdint.h>
#include "rover_map.h"

/* A 3 x 2 demo arena (markers 1..6, ring plus a middle link, all driven) as a rover_map_t. */
bool demo_map_build(rover_map_t *m);

/* Plan from map->current to target, then walk the route on a COPY of the map with a simulated executor
 * (instant arrivals) through the real goto_ctrl. Writes e.g.
 *   "GOTO_DRY 1>2>3>4 hops=3 plan_us=420 result=REACHED"   or   "GOTO_DRY none result=NO_ROUTE". */
size_t goto_dry_run(const rover_map_t *map, uint8_t target, char *buf, size_t cap, uint32_t (*clock_us)(void));
#endif
