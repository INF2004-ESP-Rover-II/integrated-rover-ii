/**
 * @file route_planner.c
 * @brief Subsystem 4: shortest route over the verified rover map.
 *
 * The search computes the cost to the goal for every (node slot, heading)
 * state by repeated relaxation (Bellman-Ford style), then walks forward from
 * the start along optimal moves. All storage is fixed size: at most
 * ROVER_MAX_NODES nodes times five headings.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "route_planner.h"

/* ---- Constants ---------------------------------------------------------- */

/** Directions on the grid: north, east, south, west. */
#define ROUTE_PLANNER_DIRECTIONS (4u)

/** Headings used by the search: the four directions plus "unknown". */
#define ROUTE_PLANNER_HEADINGS (5u)

/** Heading index meaning "unknown", so the first turn costs nothing. */
#define ROUTE_PLANNER_HEADING_ANY (4u)

/** Number of search states: one per node slot and heading. */
#define ROUTE_PLANNER_STATES (ROVER_MAX_NODES * ROUTE_PLANNER_HEADINGS)

/** Cost of one hop. Large so that fewer hops always beats fewer turns. */
#define ROUTE_PLANNER_HOP_COST (1000u)

/** Cost value for a state that cannot reach the goal. */
#define ROUTE_PLANNER_INFINITE (UINT32_MAX)

/** Marks "no such node slot". */
#define ROUTE_PLANNER_NO_SLOT (UINT32_MAX)

/** A 270 degree difference is one turn the other way. */
#define ROUTE_PLANNER_THREE_QUARTERS (3u)

/** Offset to the opposite direction (north to south, east to west). */
#define ROUTE_PLANNER_OPPOSITE (2u)

/** Heading values valid for the caller: 0 to 3. */
#define ROUTE_PLANNER_LAST_DIRECTION (3)

/* ---- Static data -------------------------------------------------------- */

/** Cost to reach the goal from each state, rebuilt by every search. */
static uint32_t g_cost_to_go[ROUTE_PLANNER_STATES];

/* ---- Private prototypes ------------------------------------------------- */

static uint32_t route_planner_turn_cost(uint32_t from_heading,
                                        uint32_t to_direction);
static uint32_t route_planner_edge_slot(const rover_map_t * p_map,
                                        uint32_t slot,
                                        uint32_t direction);
static uint32_t route_planner_slot_of(const rover_map_t * p_map,
                                      uint8_t marker_id);
static bool route_planner_relax_edge(uint32_t from_slot,
                                     uint32_t direction,
                                     uint32_t to_slot);
static bool route_planner_sweep(const rover_map_t * p_map);
static void route_planner_compute_costs(const rover_map_t * p_map,
                                        uint32_t goal_slot);
static uint32_t route_planner_best_next(const rover_map_t * p_map,
                                        uint32_t slot,
                                        uint32_t heading,
                                        uint32_t * p_direction);
static route_planner_status_t route_planner_search(
    const rover_map_t * p_map,
    uint32_t start_slot,
    uint32_t goal_slot,
    int32_t start_heading,
    route_planner_route_t * p_route);

/* ---- Public functions --------------------------------------------------- */

route_planner_status_t route_planner_plan(const rover_map_t * p_map,
                                          uint8_t start_id,
                                          uint8_t goal_id,
                                          int32_t start_heading,
                                          route_planner_route_t * p_route)
{
    route_planner_status_t status = ROUTE_PLANNER_NO_ROUTE;
    uint32_t start_slot = ROUTE_PLANNER_NO_SLOT;
    uint32_t goal_slot = ROUTE_PLANNER_NO_SLOT;

    if (NULL != p_route)
    {
        p_route->hops = 0u;
    }

    if ((NULL != p_map) && (NULL != p_route))
    {
        start_slot = route_planner_slot_of(p_map, start_id);
        goal_slot = route_planner_slot_of(p_map, goal_id);

        if (ROUTE_PLANNER_NO_SLOT == goal_slot)
        {
            status = ROUTE_PLANNER_UNKNOWN_TARGET;
        }
        else if (ROUTE_PLANNER_NO_SLOT == start_slot)
        {
            status = ROUTE_PLANNER_UNKNOWN_START;
        }
        else if (start_slot == goal_slot)
        {
            p_route->ids[0] = start_id;
            status = ROUTE_PLANNER_ALREADY_THERE;
        }
        else
        {
            status = route_planner_search(p_map, start_slot, goal_slot,
                                          start_heading, p_route);
        }
    }

    return status;
}

route_planner_status_t route_planner_next_hop(const rover_map_t * p_map,
                                              uint8_t from_id,
                                              uint8_t to_id,
                                              int32_t heading,
                                              route_planner_step_t * p_step)
{
    route_planner_route_t route;
    route_planner_status_t status;

    status = route_planner_plan(p_map, from_id, to_id, heading, &route);

    if (NULL != p_step)
    {
        p_step->next_id = 0u;
        p_step->direction = 0u;
        p_step->hops_to_go = 0u;

        if (ROUTE_PLANNER_OK == status)
        {
            p_step->next_id = route.ids[1];
            p_step->direction = route.headings[0];
            p_step->hops_to_go = route.hops;
        }
    }

    return status;
}

const char * route_planner_status_name(route_planner_status_t status)
{
    const char * p_name = "?";

    switch (status)
    {
        case ROUTE_PLANNER_OK:
            p_name = "OK";
            break;

        case ROUTE_PLANNER_ALREADY_THERE:
            p_name = "ALREADY_THERE";
            break;

        case ROUTE_PLANNER_UNKNOWN_TARGET:
            p_name = "UNKNOWN_TARGET";
            break;

        case ROUTE_PLANNER_UNKNOWN_START:
            p_name = "UNKNOWN_START";
            break;

        case ROUTE_PLANNER_NO_ROUTE:
            p_name = "NO_ROUTE";
            break;

        default:
            p_name = "?";
            break;
    }

    return p_name;
}

/* ---- Private functions -------------------------------------------------- */

/**
 * Cost of turning from a heading to a direction: 0 for straight on, 1 for a
 * quarter turn either way, 2 for a U-turn. An unknown heading is free.
 */
static uint32_t route_planner_turn_cost(uint32_t from_heading,
                                        uint32_t to_direction)
{
    uint32_t cost = 0u;
    uint32_t difference = 0u;

    if (from_heading < ROUTE_PLANNER_DIRECTIONS)
    {
        difference = (to_direction + ROUTE_PLANNER_DIRECTIONS - from_heading)
                     % ROUTE_PLANNER_DIRECTIONS;

        if (ROUTE_PLANNER_THREE_QUARTERS == difference)
        {
            cost = 1u;
        }
        else
        {
            cost = difference;
        }
    }

    return cost;
}

/**
 * Slot of the neighbour reached through a verified edge, or
 * ROUTE_PLANNER_NO_SLOT. Both ends of the edge must be MAP_EXPLORED.
 */
static uint32_t route_planner_edge_slot(const rover_map_t * p_map,
                                        uint32_t slot,
                                        uint32_t direction)
{
    uint32_t result = ROUTE_PLANNER_NO_SLOT;
    uint32_t opposite = (direction + ROUTE_PLANNER_OPPOSITE)
                        % ROUTE_PLANNER_DIRECTIONS;
    const rover_node_t * p_from = &p_map->nodes[slot];
    const rover_node_t * p_to = NULL;

    /* NOTE: int matches the rover_map_at() prototype. */
    int next_x = p_from->x + rover_dx[direction];
    int next_y = p_from->y + rover_dy[direction];

    if (MAP_EXPLORED == p_from->exits[direction])
    {
        p_to = rover_map_at(p_map, next_x, next_y);
    }

    if (NULL != p_to)
    {
        if (MAP_EXPLORED == p_to->exits[opposite])
        {
            /* p_to points into p_map->nodes, so the distance is in
             * 0 to ROVER_MAX_NODES - 1 and always fits in uint32_t. */
            result = (uint32_t)(p_to - p_map->nodes);
        }
    }

    return result;
}

/** Slot of a marker ID in the map, or ROUTE_PLANNER_NO_SLOT. */
static uint32_t route_planner_slot_of(const rover_map_t * p_map,
                                      uint8_t marker_id)
{
    uint32_t result = ROUTE_PLANNER_NO_SLOT;
    uint32_t index = 0u;

    while ((ROUTE_PLANNER_NO_SLOT == result) && (index < p_map->node_count))
    {
        if (marker_id == p_map->nodes[index].marker_id)
        {
            result = index;
        }

        index++;
    }

    return result;
}

/**
 * Improve the cost of every heading at from_slot using the hop to to_slot in
 * the given direction. Returns true if any cost became smaller.
 */
static bool route_planner_relax_edge(uint32_t from_slot,
                                     uint32_t direction,
                                     uint32_t to_slot)
{
    bool changed = false;
    uint32_t heading = 0u;
    uint32_t cost = 0u;
    uint32_t onward = g_cost_to_go[(to_slot * ROUTE_PLANNER_HEADINGS)
                                   + direction];

    if (ROUTE_PLANNER_INFINITE != onward)
    {
        for (heading = 0u; heading < ROUTE_PLANNER_HEADINGS; heading++)
        {
            cost = ROUTE_PLANNER_HOP_COST
                   + route_planner_turn_cost(heading, direction) + onward;

            if (cost < g_cost_to_go[(from_slot * ROUTE_PLANNER_HEADINGS)
                                    + heading])
            {
                g_cost_to_go[(from_slot * ROUTE_PLANNER_HEADINGS) + heading]
                    = cost;
                changed = true;
            }
        }
    }

    return changed;
}

/** One pass over every node and direction. True if any cost changed. */
static bool route_planner_sweep(const rover_map_t * p_map)
{
    bool changed = false;
    uint32_t slot = 0u;
    uint32_t direction = 0u;
    uint32_t next_slot = 0u;

    for (slot = 0u; slot < p_map->node_count; slot++)
    {
        for (direction = 0u; direction < ROUTE_PLANNER_DIRECTIONS;
             direction++)
        {
            next_slot = route_planner_edge_slot(p_map, slot, direction);

            if (ROUTE_PLANNER_NO_SLOT != next_slot)
            {
                if (route_planner_relax_edge(slot, direction, next_slot))
                {
                    changed = true;
                }
            }
        }
    }

    return changed;
}

/**
 * Fill g_cost_to_go for the current map. The goal costs 0 for every
 * heading. Sweeping stops when nothing improves, which takes at most one
 * sweep per state.
 */
static void route_planner_compute_costs(const rover_map_t * p_map,
                                        uint32_t goal_slot)
{
    uint32_t state_count = p_map->node_count * ROUTE_PLANNER_HEADINGS;
    uint32_t index = 0u;
    uint32_t sweeps = 0u;
    bool changed = true;

    for (index = 0u; index < state_count; index++)
    {
        g_cost_to_go[index] = ROUTE_PLANNER_INFINITE;
    }

    for (index = 0u; index < ROUTE_PLANNER_HEADINGS; index++)
    {
        g_cost_to_go[(goal_slot * ROUTE_PLANNER_HEADINGS) + index] = 0u;
    }

    while ((changed) && (sweeps <= state_count))
    {
        changed = route_planner_sweep(p_map);
        sweeps++;
    }
}

/**
 * Among the optimal first moves from a state, pick the neighbour with the
 * smallest marker ID. Returns its slot (and direction), or
 * ROUTE_PLANNER_NO_SLOT if the cost table has no optimal move.
 */
static uint32_t route_planner_best_next(const rover_map_t * p_map,
                                        uint32_t slot,
                                        uint32_t heading,
                                        uint32_t * p_direction)
{
    uint32_t best_slot = ROUTE_PLANNER_NO_SLOT;
    uint32_t here = g_cost_to_go[(slot * ROUTE_PLANNER_HEADINGS) + heading];
    uint32_t direction = 0u;
    uint32_t next_slot = 0u;
    uint32_t onward = 0u;

    for (direction = 0u; direction < ROUTE_PLANNER_DIRECTIONS; direction++)
    {
        next_slot = route_planner_edge_slot(p_map, slot, direction);

        if (ROUTE_PLANNER_NO_SLOT != next_slot)
        {
            onward = g_cost_to_go[(next_slot * ROUTE_PLANNER_HEADINGS)
                                  + direction];

            if ((ROUTE_PLANNER_INFINITE != onward)
                && (here == (ROUTE_PLANNER_HOP_COST
                             + route_planner_turn_cost(heading, direction)
                             + onward)))
            {
                if ((ROUTE_PLANNER_NO_SLOT == best_slot)
                    || (p_map->nodes[next_slot].marker_id
                        < p_map->nodes[best_slot].marker_id))
                {
                    best_slot = next_slot;
                    *p_direction = direction;
                }
            }
        }
    }

    return best_slot;
}

/** Run the search and, if the goal is reachable, write the route. */
static route_planner_status_t route_planner_search(
    const rover_map_t * p_map,
    uint32_t start_slot,
    uint32_t goal_slot,
    int32_t start_heading,
    route_planner_route_t * p_route)
{
    route_planner_status_t status = ROUTE_PLANNER_OK;
    uint32_t heading = ROUTE_PLANNER_HEADING_ANY;
    uint32_t slot = start_slot;
    uint32_t next_slot = 0u;
    uint32_t direction = 0u;
    uint32_t hops = 0u;

    if ((start_heading >= 0) && (start_heading <= ROUTE_PLANNER_LAST_DIRECTION))
    {
        /* start_heading was just checked to be 0 to 3. */
        heading = (uint32_t)start_heading;
    }

    route_planner_compute_costs(p_map, goal_slot);

    if (ROUTE_PLANNER_INFINITE
        == g_cost_to_go[(start_slot * ROUTE_PLANNER_HEADINGS) + heading])
    {
        status = ROUTE_PLANNER_NO_ROUTE;
    }
    else
    {
        p_route->ids[0] = p_map->nodes[start_slot].marker_id;
    }

    while ((ROUTE_PLANNER_OK == status) && (slot != goal_slot))
    {
        next_slot = route_planner_best_next(p_map, slot, heading,
                                            &direction);

        /* A consistent cost table always has a next move, and a shortest
         * route never revisits a node, so hops stays below the node
         * count. Anything else is reported as no route. */
        if ((ROUTE_PLANNER_NO_SLOT == next_slot)
            || ((hops + 1u) >= ROVER_MAX_NODES))
        {
            status = ROUTE_PLANNER_NO_ROUTE;
        }
        else
        {
            /* direction is 0 to 3, so it fits in uint8_t. */
            p_route->headings[hops] = (uint8_t)direction;
            hops++;
            p_route->ids[hops] = p_map->nodes[next_slot].marker_id;
            slot = next_slot;
            heading = direction;
        }
    }

    if (ROUTE_PLANNER_OK == status)
    {
        /* hops is below ROVER_MAX_NODES (64), so it fits in uint8_t. */
        p_route->hops = (uint8_t)hops;
    }

    return status;
}

/* end of file route_planner.c */
