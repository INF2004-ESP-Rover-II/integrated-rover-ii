/**
 * @file goto_dry.c
 * @brief Subsystem 4: DEMO_MAP and GOTO_DRY plan-only test helpers.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "goto_ctrl.h"
#include "goto_dry.h"
#include "route_planner.h"

/* ---- Constants ---------------------------------------------------------- */

/** Markers placed after the start marker. */
#define GOTO_DRY_EXTRA_NODES (5u)

/** Directions on the grid: north, east, south, west. */
#define GOTO_DRY_DIRECTIONS (4u)

/** Edges driven in the demo arena. */
#define GOTO_DRY_EDGES (7u)

/** Number of main-loop ticks the dry run may take before it gives up. */
#define GOTO_DRY_MAX_TICKS (400u)

/** Simulated time between ticks. */
#define GOTO_DRY_TICK_MS (10u)

/** Marker the demo arena starts at. */
#define GOTO_DRY_START_ID (1u)

/** Room for "id>" for every node plus the end of the string. */
#define GOTO_DRY_PATH_SIZE ((ROVER_MAX_NODES * 4u) + 4u)

/* ---- Types -------------------------------------------------------------- */

typedef struct
{
    uint8_t marker_id;
    int16_t x_pos;
    int16_t y_pos;
} goto_dry_node_t;

typedef struct
{
    uint8_t from_id;
    uint8_t to_id;
    uint8_t direction;
} goto_dry_edge_t;

/** The simulated executor: instant arrival, one result at a time. */
typedef struct
{
    rover_map_t map;
    bool pending;
    goto_ctrl_hop_result_t result;
} goto_dry_sim_t;

/* ---- Static data -------------------------------------------------------- */

static goto_dry_sim_t g_sim;

static const goto_dry_node_t g_extra_nodes[GOTO_DRY_EXTRA_NODES] =
{
    {2u, 1, 0}, {3u, 2, 0}, {4u, 2, 1}, {5u, 1, 1}, {6u, 0, 1}
};

static const goto_dry_edge_t g_edges[GOTO_DRY_EDGES] =
{
    {1u, 2u, MAP_EAST}, {2u, 3u, MAP_EAST}, {6u, 5u, MAP_EAST},
    {5u, 4u, MAP_EAST}, {1u, 6u, MAP_NORTH}, {2u, 5u, MAP_NORTH},
    {3u, 4u, MAP_NORTH}
};

/* ---- Private prototypes ------------------------------------------------- */

static const rover_map_t * goto_dry_sim_map(void * p_context);
static bool goto_dry_sim_ready(void * p_context);
static bool goto_dry_sim_begin(void * p_context,
                               uint8_t target,
                               uint32_t direction,
                               uint32_t sequence,
                               uint32_t now_ms);
static bool goto_dry_sim_take(void * p_context,
                              goto_ctrl_hop_result_t * p_result);
static size_t goto_dry_format_path(const route_planner_route_t * p_route,
                                   char * p_path,
                                   size_t cap);

/* ---- Public functions --------------------------------------------------- */

bool goto_dry_build_demo_map(rover_map_t * p_map)
{
    bool all_ok = true;
    uint32_t index = 0u;
    uint8_t marker = 0u;

    rover_map_init(p_map, GOTO_DRY_START_ID);

    for (index = 0u; (all_ok) && (index < GOTO_DRY_EXTRA_NODES); index++)
    {
        all_ok = rover_map_place(p_map, g_extra_nodes[index].marker_id,
                                 g_extra_nodes[index].x_pos,
                                 g_extra_nodes[index].y_pos);
    }

    for (index = 0u; (all_ok) && (index < GOTO_DRY_EDGES); index++)
    {
        all_ok = rover_map_connect(p_map, g_edges[index].from_id,
                                   g_edges[index].to_id,
                                   g_edges[index].direction);
    }

    if (all_ok)
    {
        /* Mark markers 2 to 6 visited, then return to the start. */
        for (marker = 2u; marker <= GOTO_DRY_EXTRA_NODES + 1u; marker++)
        {
            (void)rover_map_visit(p_map, marker);
        }

        all_ok = rover_map_visit(p_map, GOTO_DRY_START_ID);
    }

    return all_ok;
}

size_t goto_dry_run(const rover_map_t * p_map,
                    uint8_t target,
                    char * p_buf,
                    size_t cap,
                    goto_dry_clock_fn_t clock_us)
{
    route_planner_route_t route;
    route_planner_status_t status;
    goto_ctrl_t ctrl;
    goto_ctrl_io_t io;
    char path[GOTO_DRY_PATH_SIZE] = "";
    uint32_t start_us = 0u;
    uint32_t plan_us = 0u;
    uint32_t hops = 0u;
    uint32_t tick = 0u;
    int written = 0;
    const char * p_result = "";

    if (NULL != clock_us)
    {
        start_us = clock_us();
    }

    status = route_planner_plan(p_map, p_map->current, target,
                                ROUTE_PLANNER_HEADING_UNKNOWN, &route);

    if (NULL != clock_us)
    {
        plan_us = clock_us() - start_us;
    }

    if ((ROUTE_PLANNER_OK == status)
        || (ROUTE_PLANNER_ALREADY_THERE == status))
    {
        (void)goto_dry_format_path(&route, path, sizeof(path));
    }

    if (ROUTE_PLANNER_OK == status)
    {
        hops = route.hops;
    }

    /* Walk a copy, so the real map is never changed. */
    (void)memset(&g_sim, 0, sizeof(g_sim));
    g_sim.map = *p_map;
    io.p_context = &g_sim;
    io.map = goto_dry_sim_map;
    io.ready = goto_dry_sim_ready;
    io.begin_hop = goto_dry_sim_begin;
    io.take_result = goto_dry_sim_take;
    goto_ctrl_init(&ctrl, &io);
    (void)goto_ctrl_start(&ctrl, target, 0u);

    for (tick = 1u; (tick < GOTO_DRY_MAX_TICKS)
                    && (GOTO_CTRL_RUNNING == ctrl.state); tick++)
    {
        goto_ctrl_tick(&ctrl, tick * GOTO_DRY_TICK_MS);
    }

    if (GOTO_CTRL_RUNNING == ctrl.state)
    {
        p_result = "STUCK";
    }
    else
    {
        p_result = goto_ctrl_result_name(ctrl.last_result);
    }

    if ('\0' == path[0])
    {
        (void)snprintf(path, sizeof(path), "none");
    }

    written = snprintf(p_buf, cap,
                       "GOTO_DRY %s hops=%" PRIu32 " plan_us=%" PRIu32
                       " result=%s",
                       path, hops, plan_us, p_result);

    if (written < 0)
    {
        written = 0;
    }

    /* written is not negative here, so the conversion keeps its value. */
    return (size_t)written;
}

/* ---- Private functions -------------------------------------------------- */

/** Simulated executor: the map it works on. */
static const rover_map_t * goto_dry_sim_map(void * p_context)
{
    const goto_dry_sim_t * p_sim = p_context;

    return &p_sim->map;
}

/** Simulated executor: ready whenever no result is waiting. */
static bool goto_dry_sim_ready(void * p_context)
{
    const goto_dry_sim_t * p_sim = p_context;

    return !p_sim->pending;
}

/** Simulated executor: accept a legal hop and arrive instantly. */
static bool goto_dry_sim_begin(void * p_context,
                               uint8_t target,
                               uint32_t direction,
                               uint32_t sequence,
                               uint32_t now_ms)
{
    goto_dry_sim_t * p_sim = p_context;
    const rover_node_t * p_from = NULL;
    const rover_node_t * p_to = NULL;
    bool accepted = false;

    (void)now_ms;

    p_from = rover_map_find(&p_sim->map, p_sim->map.current, NULL, NULL);
    p_to = rover_map_find(&p_sim->map, target, NULL, NULL);

    if ((NULL != p_from) && (NULL != p_to) && (direction < GOTO_DRY_DIRECTIONS))
    {
        if ((MAP_EXPLORED == p_from->exits[direction])
            && (p_to->x == (p_from->x + rover_dx[direction]))
            && (p_to->y == (p_from->y + rover_dy[direction])))
        {
            (void)rover_map_visit(&p_sim->map, target);
            p_sim->pending = true;
            p_sim->result.sequence = sequence;
            p_sim->result.target = target;
            p_sim->result.arrived = true;
            (void)snprintf(p_sim->result.reason,
                           sizeof(p_sim->result.reason), "ARRIVED");
            accepted = true;
        }
    }

    return accepted;
}

/** Simulated executor: hand over the waiting result once. */
static bool goto_dry_sim_take(void * p_context,
                              goto_ctrl_hop_result_t * p_result)
{
    goto_dry_sim_t * p_sim = p_context;
    bool taken = false;

    if (p_sim->pending)
    {
        *p_result = p_sim->result;
        p_sim->pending = false;
        taken = true;
    }

    return taken;
}

/** Write "id>id>id" for a route. Returns the number of characters. */
static size_t goto_dry_format_path(const route_planner_route_t * p_route,
                                   char * p_path,
                                   size_t cap)
{
    size_t length = 0u;
    uint32_t index = 0u;
    uint32_t marker_id = 0u;
    int written = 0;

    for (index = 0u; (index <= p_route->hops) && ((length + 5u) < cap);
         index++)
    {
        marker_id = p_route->ids[index];

        if (0u == index)
        {
            written = snprintf(&p_path[length], cap - length, "%" PRIu32,
                               marker_id);
        }
        else
        {
            written = snprintf(&p_path[length], cap - length, ">%" PRIu32,
                               marker_id);
        }

        if (written > 0)
        {
            /* written is positive here, so the conversion keeps its
             * value. */
            length += (size_t)written;
        }
    }

    return length;
}

/* end of file goto_dry.c */
