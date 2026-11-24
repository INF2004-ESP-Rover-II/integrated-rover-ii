/**
 * @file route_planner.h
 * @brief Subsystem 4: shortest route over the verified graph of a
 *        rover_map_t. The map is only read, never written.
 *
 * Rules (agreed with the Buddy 1 handover):
 *  - An edge is usable only if BOTH ends are MAP_EXPLORED and the neighbour
 *    node exists one grid step away. MAP_UNCHECKED, MAP_OPEN (seen but not
 *    driven yet) and MAP_BLOCKED edges are never routed through.
 *  - Cost is 1000 per hop plus 1 per 90 degrees turned. Fewest hops wins,
 *    then fewest turns, then the smallest marker-ID path (deterministic).
 *  - The search state is (node slot, heading). There is no dynamic memory
 *    and no indexing by marker ID, because IDs go up to 255.
 *
 * Use by the DFS (for example to backtrack to an earlier node while
 * exploring): call route_planner_next_hop() with the current node, the node
 * to return to and the current heading. Drive the returned direction with the
 * DFS's own leg (movement) function. Call it again after each confirmed
 * arrival, because the map may have changed.
 *
 * NOTE: the planner never moves the rover and holds no state between calls
 * except a scratch table, so it is not re-entrant.
 */

#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <stdbool.h>
#include <stdint.h>
#include "rover_map.h"

/** Pass as the start heading when the rover's heading is not known. */
#define ROUTE_PLANNER_HEADING_UNKNOWN (-1)

/** Result of a planning request. */
typedef enum
{
    ROUTE_PLANNER_OK = 0,             /**< Route found. */
    ROUTE_PLANNER_ALREADY_THERE = 1,  /**< Start equals goal, no hops. */
    ROUTE_PLANNER_UNKNOWN_TARGET = 2, /**< Goal marker is not in the map. */
    ROUTE_PLANNER_UNKNOWN_START = 3,  /**< Start marker is not in the map. */
    ROUTE_PLANNER_NO_ROUTE = 4        /**< No verified connection exists. */
} route_planner_status_t;

/** A planned route. ids[0] is the start and ids[hops] is the goal. */
typedef struct
{
    uint8_t ids[ROVER_MAX_NODES];      /**< Marker IDs along the route. */
    uint8_t headings[ROVER_MAX_NODES]; /**< MAP_NORTH..MAP_WEST per hop. */
    uint8_t hops;                      /**< Number of hops (edges). */
} route_planner_route_t;

/** The first hop of a route, for callers that move one hop at a time. */
typedef struct
{
    uint8_t next_id;    /**< Marker to drive to, 0 when there is no hop. */
    uint8_t direction;  /**< MAP_NORTH..MAP_WEST to leave the current node. */
    uint8_t hops_to_go; /**< Hops left to the goal, including this one. */
} route_planner_step_t;

/**
 * @brief Plan the shortest verified route between two markers.
 * @param p_map         Rover map (read only).
 * @param start_id      Marker the rover is at.
 * @param goal_id       Marker to reach.
 * @param start_heading MAP_NORTH..MAP_WEST, or
 *                      ROUTE_PLANNER_HEADING_UNKNOWN (first turn is free).
 * @param p_route       Output route. hops is 0 unless the result is
 *                      ROUTE_PLANNER_OK.
 * @return Planning status.
 */
route_planner_status_t route_planner_plan(const rover_map_t * p_map,
                                          uint8_t start_id,
                                          uint8_t goal_id,
                                          int32_t start_heading,
                                          route_planner_route_t * p_route);

/**
 * @brief Plan a route and return only its first hop.
 * @param p_map     Rover map (read only).
 * @param from_id   Marker the rover is at (the confirmed current node).
 * @param to_id     Marker to reach, for example an earlier node to
 *                  backtrack to.
 * @param heading   Current heading, or ROUTE_PLANNER_HEADING_UNKNOWN.
 * @param p_step    Output first hop. All fields are 0 unless the result is
 *                  ROUTE_PLANNER_OK.
 * @return Planning status. ROUTE_PLANNER_ALREADY_THERE means no move is
 *         needed.
 */
route_planner_status_t route_planner_next_hop(const rover_map_t * p_map,
                                              uint8_t from_id,
                                              uint8_t to_id,
                                              int32_t heading,
                                              route_planner_step_t * p_step);

/**
 * @brief Text name of a planning status, for logs and MQTT status lines.
 * @param status Status to name.
 * @return Constant string, never NULL.
 */
const char * route_planner_status_name(route_planner_status_t status);

#endif /* ROUTE_PLANNER_H */

/* end of file route_planner.h */
