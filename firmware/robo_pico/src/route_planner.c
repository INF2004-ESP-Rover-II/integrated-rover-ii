/* route_planner.c - see route_planner.h. Port of rover_mapping/mapping/planner.py. */
#include "route_planner.h"

#include <stdlib.h>
#include <string.h>

/* An open-list record. The path is rebuilt through `parent` only when two records tie on f. */
typedef struct {
    int32_t f, g;
    uint8_t node;     /* index in the map */
    int8_t  hd;       /* heading used to enter node, -1 = none (the start) */
    int16_t parent;   /* index of the record we came from, -1 for the start */
    uint8_t alive;    /* still on the open list */
} open_rec_t;

#define POOL_SIZE 1024   /* >= 4 pushes per closed state: 36 nodes x 5 headings x 4 = 720 */
#define STATES (MAP_MAX_MARKERS * 5u)

static open_rec_t pool[POOL_SIZE];
static int32_t best[STATES];
static uint8_t closed[STATES];

static unsigned state_of(uint8_t node, int hd) { return (unsigned)node * 5u + (unsigned)(hd + 1); }

static int manhattan(const map_marker_t *a, const map_marker_t *b)
{
    return abs(a->x - b->x) + abs(a->y - b->y);
}

/* path of marker IDs from the start to record r; returns its length */
static int build_path(const rover_map_t *m, int r, uint8_t *out)
{
    uint8_t tmp[PLAN_MAX_PATH];
    int n = 0;
    while (r >= 0 && n < (int)PLAN_MAX_PATH) {
        tmp[n++] = m->markers[pool[r].node].id;
        r = pool[r].parent;
    }
    for (int i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
    return n;
}

/* true if record a should be popped before record b: smaller f, then smaller marker-ID path */
static bool better(const rover_map_t *m, int a, int b)
{
    if (pool[a].f != pool[b].f) return pool[a].f < pool[b].f;
    uint8_t pa[PLAN_MAX_PATH], pb[PLAN_MAX_PATH];
    int la = build_path(m, a, pa), lb = build_path(m, b, pb);
    int n = la < lb ? la : lb;
    for (int i = 0; i < n; i++)
        if (pa[i] != pb[i]) return pa[i] < pb[i];
    if (la != lb) return la < lb;
    return pool[a].g < pool[b].g;
}

int plan_route(const rover_map_t *m, uint8_t start_id, uint8_t goal_id, int start_heading,
               bool use_probable, uint8_t *path_ids, int max_len)
{
    uint8_t s = map_index(m, start_id), g = map_index(m, goal_id);
    if (s == MAP_NONE || g == MAP_NONE || max_len < 1) return 0;
    if (start_id == goal_id) { path_ids[0] = start_id; return 1; }
    if (start_heading < -1 || start_heading > 3) start_heading = -1;

    const map_marker_t *goal = &m->markers[g];
    for (unsigned i = 0; i < STATES; i++) { best[i] = 0x7FFFFFFF; closed[i] = 0; }
    int used = 0;
    best[state_of(s, start_heading)] = 0;
    pool[0] = (open_rec_t){ (int32_t)manhattan(&m->markers[s], goal) * PLAN_W, 0, s, (int8_t)start_heading, -1, 1 };
    used = 1;

    for (;;) {
        int top = -1;
        for (int i = 0; i < used; i++)
            if (pool[i].alive && (top < 0 || better(m, i, top))) top = i;
        if (top < 0) return 0;                       /* open list empty: no route */
        pool[top].alive = 0;
        open_rec_t cur = pool[top];
        unsigned st = state_of(cur.node, cur.hd);
        if (closed[st]) continue;
        closed[st] = 1;
        if (cur.node == g) {
            uint8_t tmp[PLAN_MAX_PATH];
            int n = build_path(m, top, tmp);
            if (n > max_len) return 0;
            memcpy(path_ids, tmp, (size_t)n);
            return n;
        }
        map_edge_t edges[4];
        uint8_t ne = map_usable_neighbours(m, cur.node, use_probable, edges);
        for (uint8_t k = 0; k < ne; k++) {
            int d = edges[k].dir;
            int32_t extra = edges[k].probable ? PLAN_PROBABLE_PENALTY : 0;
            int32_t ng = cur.g + PLAN_W + extra + (cur.hd >= 0 ? map_turn_steps(cur.hd, d) : 0);
            unsigned ns = state_of(edges[k].idx, d);
            if (closed[ns] || ng > best[ns]) continue;
            if (used >= POOL_SIZE) return 0;         /* cannot happen for 36 markers; fail safe instead of overflowing */
            best[ns] = ng;
            pool[used] = (open_rec_t){ ng + (int32_t)manhattan(&m->markers[edges[k].idx], goal) * PLAN_W,
                                       ng, edges[k].idx, (int8_t)d, (int16_t)top, 1 };
            used++;
        }
    }
}

int plan_path_headings(const rover_map_t *m, const uint8_t *path_ids, int path_len,
                       uint8_t *heading, uint8_t *target_id)
{
    int hops = 0;
    for (int i = 0; i + 1 < path_len; i++) {
        int d = map_direction_between(m, path_ids[i], path_ids[i + 1]);
        if (d < 0) return -1;
        heading[hops] = (uint8_t)d;
        target_id[hops] = path_ids[i + 1];
        hops++;
    }
    return hops;
}
