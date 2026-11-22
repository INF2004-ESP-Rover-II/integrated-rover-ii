#include "goto_dry.h"
#include <stdio.h>
#include <string.h>
#include "goto_ctrl.h"
#include "route_planner.h"

bool demo_map_build(rover_map_t *m)
{
    static const struct { uint8_t id; int x, y; } more[] = {{2, 1, 0}, {3, 2, 0}, {4, 2, 1}, {5, 1, 1}, {6, 0, 1}};
    rover_map_init(m, 1);
    for (unsigned i = 0; i < sizeof(more) / sizeof(more[0]); ++i)
        if (!rover_map_place(m, more[i].id, more[i].x, more[i].y)) return false;
    static const struct { uint8_t a, b; unsigned dir; } edges[] = {
        {1, 2, MAP_EAST}, {2, 3, MAP_EAST}, {6, 5, MAP_EAST}, {5, 4, MAP_EAST},
        {1, 6, MAP_NORTH}, {2, 5, MAP_NORTH}, {3, 4, MAP_NORTH}};
    for (unsigned i = 0; i < sizeof(edges) / sizeof(edges[0]); ++i)
        if (!rover_map_connect(m, edges[i].a, edges[i].b, edges[i].dir)) return false;
    for (uint8_t id = 2; id <= 6; ++id) rover_map_visit(m, id);
    return rover_map_visit(m, 1);              /* back to the start */
}

typedef struct { rover_map_t map; bool pending; goto_hop_result_t result; } sim_t;
static sim_t sim;
static const rover_map_t *s_map(void *c) { return &((sim_t *)c)->map; }
static bool s_ready(void *c) { return !((sim_t *)c)->pending; }
static bool s_begin(void *c, uint8_t t, unsigned d, uint32_t seq, uint32_t now)
{
    (void)now;
    sim_t *s = c;
    const rover_node_t *cur = rover_map_find(&s->map, s->map.current, NULL, NULL);
    const rover_node_t *to = rover_map_find(&s->map, t, NULL, NULL);
    if (!cur || !to || d > 3u || cur->exits[d] != MAP_EXPLORED ||
        to->x != cur->x + rover_dx[d] || to->y != cur->y + rover_dy[d]) return false;
    rover_map_visit(&s->map, t);
    s->pending = true;
    s->result = (goto_hop_result_t){seq, t, true, "ARRIVED"};
    return true;
}
static bool s_take(void *c, goto_hop_result_t *o)
{
    sim_t *s = c;
    if (!s->pending) return false;
    *o = s->result; s->pending = false; return true;
}

size_t goto_dry_run(const rover_map_t *map, uint8_t target, char *buf, size_t cap, uint32_t (*clock_us)(void))
{
    route_t r;
    uint32_t t0 = clock_us ? clock_us() : 0u;
    route_status_t st = route_plan(map, map->current, target, ROUTE_HEADING_UNKNOWN, &r);
    uint32_t us = clock_us ? clock_us() - t0 : 0u;
    char path[ROVER_MAX_NODES * 4u + 4u] = "";
    size_t n = 0;
    if (st == ROUTE_OK || st == ROUTE_ALREADY_THERE)
        for (unsigned i = 0; i <= r.hops && n + 5u < sizeof(path); ++i)
            n += (size_t)snprintf(path + n, sizeof(path) - n, i ? ">%u" : "%u", (unsigned)r.ids[i]);
    memset(&sim, 0, sizeof sim);
    sim.map = *map;                                    /* walk a copy: the real map is never changed */
    goto_ctrl_t g;
    goto_io_t io = {&sim, s_map, s_ready, s_begin, s_take};
    goto_init(&g, &io);
    goto_start(&g, target, 0u);
    for (uint32_t t = 1u; t < 400u && g.state == GOTO_RUNNING; ++t) goto_tick(&g, t * 10u);
    int w = snprintf(buf, cap, "GOTO_DRY %s hops=%u plan_us=%lu result=%s",
                     path[0] ? path : "none", (unsigned)(st == ROUTE_OK ? r.hops : 0u),
                     (unsigned long)us, g.state == GOTO_RUNNING ? "STUCK" : goto_result_name(g.last_result));
    return w < 0 ? 0u : (size_t)w;
}
