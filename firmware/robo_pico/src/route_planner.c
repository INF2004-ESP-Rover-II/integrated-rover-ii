#include "route_planner.h"
#include <stddef.h>

#define HEADINGS 5u               /* 0..3 = N,E,S,W, 4 = unknown (first turn is free) */
#define HOP_COST 1000u
#define INF 0x7FFFFFFF

static unsigned turn_cost(unsigned from, unsigned to)
{
    if (from > 3u) return 0u;
    unsigned d = (to + 4u - from) & 3u;
    return d == 3u ? 1u : d;      /* 0, 1, 2, or 1 for a 270 degree difference */
}

/* Neighbour slot through a verified edge, or -1. */
static int edge_slot(const rover_map_t *m, unsigned slot, unsigned dir)
{
    const rover_node_t *from = &m->nodes[slot];
    if (from->exits[dir] != MAP_EXPLORED) return -1;
    const rover_node_t *to = rover_map_at(m, from->x + rover_dx[dir], from->y + rover_dy[dir]);
    if (!to || to->exits[(dir + 2u) & 3u] != MAP_EXPLORED) return -1;
    return (int)(to - m->nodes);
}

static int slot_of(const rover_map_t *m, uint8_t id)
{
    for (unsigned i = 0; i < m->node_count; ++i)
        if (m->nodes[i].marker_id == id) return (int)i;
    return -1;
}

const char *route_status_name(route_status_t s)
{
    switch (s) {
    case ROUTE_OK: return "OK";
    case ROUTE_ALREADY_THERE: return "ALREADY_THERE";
    case ROUTE_UNKNOWN_TARGET: return "UNKNOWN_TARGET";
    case ROUTE_UNKNOWN_START: return "UNKNOWN_START";
    case ROUTE_NO_ROUTE: return "NO_ROUTE";
    }
    return "?";
}

route_status_t route_plan(const rover_map_t *m, uint8_t start_id, uint8_t goal_id,
                          int start_heading, route_t *out)
{
    static int32_t togo[ROVER_MAX_NODES * HEADINGS];   /* cost from state to the goal; static: ~1.3 KB, no stack use */
    if (out) out->hops = 0;
    if (!m || !out) return ROUTE_NO_ROUTE;
    int s = slot_of(m, start_id), g = slot_of(m, goal_id);
    if (g < 0) return ROUTE_UNKNOWN_TARGET;
    if (s < 0) return ROUTE_UNKNOWN_START;
    if (s == g) { out->ids[0] = start_id; return ROUTE_ALREADY_THERE; }

    unsigned n = m->node_count;
    for (unsigned i = 0; i < n * HEADINGS; ++i) togo[i] = INF;
    for (unsigned h = 0; h < HEADINGS; ++h) togo[(unsigned)g * HEADINGS + h] = 0;

    /* Bellman-Ford style relaxation on cost-to-go; at most 64 nodes x 5 headings, converges in <= n sweeps */
    for (unsigned sweep = 0; sweep <= n * HEADINGS; ++sweep) {
        bool changed = false;
        for (unsigned v = 0; v < n; ++v) {
            for (unsigned d = 0; d < 4u; ++d) {
                int w = edge_slot(m, v, d);
                if (w < 0) continue;
                int32_t onward = togo[(unsigned)w * HEADINGS + d];
                if (onward == INF) continue;
                for (unsigned h = 0; h < HEADINGS; ++h) {
                    int32_t c = (int32_t)(HOP_COST + turn_cost(h, d)) + onward;
                    if (c < togo[v * HEADINGS + h]) { togo[v * HEADINGS + h] = c; changed = true; }
                }
            }
        }
        if (!changed) break;
    }

    unsigned h = (start_heading >= 0 && start_heading <= 3) ? (unsigned)start_heading : 4u;
    if (togo[(unsigned)s * HEADINGS + h] == INF) return ROUTE_NO_ROUTE;

    /* Walk forward. Among the optimal first moves take the neighbour with the smallest marker ID. */
    unsigned v = (unsigned)s, hops = 0;
    out->ids[0] = start_id;
    while (v != (unsigned)g) {
        int32_t here = togo[v * HEADINGS + h];
        int best_dir = -1, best_slot = -1;
        for (unsigned d = 0; d < 4u; ++d) {
            int w = edge_slot(m, v, d);
            if (w < 0) continue;
            int32_t onward = togo[(unsigned)w * HEADINGS + d];
            if (onward == INF || (int32_t)(HOP_COST + turn_cost(h, d)) + onward != here) continue;
            if (best_slot < 0 || m->nodes[w].marker_id < m->nodes[best_slot].marker_id) { best_dir = (int)d; best_slot = w; }
        }
        if (best_slot < 0 || hops + 1u >= ROVER_MAX_NODES) return ROUTE_NO_ROUTE;   /* cannot happen on a consistent table */
        out->headings[hops] = (uint8_t)best_dir;
        out->ids[++hops] = m->nodes[best_slot].marker_id;
        v = (unsigned)best_slot; h = (unsigned)best_dir;
    }
    out->hops = (uint8_t)hops;
    return ROUTE_OK;
}
