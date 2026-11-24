/**
 * @file goto_dry.h
 * @brief Subsystem 4: plan-only test helpers. They never touch the motors or
 *        the real map.
 */

#ifndef GOTO_DRY_H
#define GOTO_DRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "rover_map.h"

/** Function that returns a microsecond clock, used to time the planning. */
typedef uint32_t (* goto_dry_clock_fn_t)(void);

/**
 * @brief Build a 3 x 2 demo arena (markers 1 to 6, a ring plus a middle
 *        link, all driven) as a rover_map_t.
 * @param p_map Map to fill. It is re-initialised.
 * @return True if the whole arena was built.
 */
bool goto_dry_build_demo_map(rover_map_t * p_map);

/**
 * @brief Plan from the map's current node to a target, then walk the route
 *        on a COPY of the map with a simulated executor (instant arrivals)
 *        through the real goto_ctrl.
 *
 * Writes, for example, "GOTO_DRY 1>2>3>4 hops=3 plan_us=420 result=REACHED"
 * or "GOTO_DRY none hops=0 plan_us=3 result=NO_ROUTE".
 *
 * @param p_map    Map to plan on (read only).
 * @param target   Marker to reach.
 * @param p_buf    Output text buffer.
 * @param cap      Size of the buffer in bytes.
 * @param clock_us Microsecond clock, or NULL to report 0.
 * @return Number of characters written (not counting the zero).
 */
size_t goto_dry_run(const rover_map_t * p_map,
                    uint8_t target,
                    char * p_buf,
                    size_t cap,
                    goto_dry_clock_fn_t clock_us);

#endif /* GOTO_DRY_H */

/* end of file goto_dry.h */
