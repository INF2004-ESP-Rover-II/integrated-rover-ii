/* Host test for src/route_planner.c. Uses B1's real rover_map.c and the B1 hardcoded fixture. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "planner_fixture.h"
#include "route_planner.h"

static int checks;
#define CHECK(c) do { ++checks; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static void expect(const rover_map_t *m, uint8_t a, uint8_t b, int h, route_status_t st, const uint8_t *ids, const uint8_t *hd, unsigned hops)
{
    route_t r;
    route_status_t got = route_plan(m, a, b, h, &r);
    CHECK(got == st);
    if (st == ROUTE_OK) {
        CHECK(r.hops == hops);
        for (unsigned i = 0; i <= hops; ++i) CHECK(r.ids[i] == ids[i]);
        for (unsigned i = 0; i < hops; ++i) CHECK(r.headings[i] == hd[i]);
    }
}

static void fixture_cases(void)
{
    rover_map_t m;
    CHECK(planner_fixture_build(&m));
    const uint8_t p1[] = {7, 3, 9, 36}, h1[] = {MAP_NORTH, MAP_EAST, MAP_EAST};
    const uint8_t p2[] = {7, 12, 20, 36}, h2[] = {MAP_EAST, MAP_EAST, MAP_NORTH};
    expect(&m, 7, 36, ROUTE_HEADING_UNKNOWN, ROUTE_OK, p1, h1, 3);   /* smallest-ID tie-break */
    expect(&m, 7, 36, MAP_NORTH, ROUTE_OK, p1, h1, 3);               /* fewest turns */
    CHECK(rover_map_exit(&m, 7, MAP_NORTH, MAP_BLOCKED));
    expect(&m, 7, 36, MAP_NORTH, ROUTE_OK, p2, h2, 3);
    CHECK(rover_map_exit(&m, 7, MAP_EAST, MAP_BLOCKED));
    expect(&m, 7, 36, ROUTE_HEADING_UNKNOWN, ROUTE_NO_ROUTE, 0, 0, 0);
    CHECK(planner_fixture_build(&m));
    expect(&m, 7, 40, ROUTE_HEADING_UNKNOWN, ROUTE_NO_ROUTE, 0, 0, 0);       /* only an OPEN edge */
    expect(&m, 7, 255, ROUTE_HEADING_UNKNOWN, ROUTE_NO_ROUTE, 0, 0, 0);      /* exists, disconnected */
    expect(&m, 7, 99, ROUTE_HEADING_UNKNOWN, ROUTE_UNKNOWN_TARGET, 0, 0, 0);
    expect(&m, 99, 7, ROUTE_HEADING_UNKNOWN, ROUTE_UNKNOWN_START, 0, 0, 0);
    const uint8_t same[] = {7};
    expect(&m, 7, 7, ROUTE_HEADING_UNKNOWN, ROUTE_ALREADY_THERE, same, 0, 0);
    /* a reverse trip over the same graph, different start heading, high ID and negative coordinates are fine */
    route_t r;
    CHECK(route_plan(&m, 36, 7, ROUTE_HEADING_UNKNOWN, &r) == ROUTE_OK && r.hops == 3 && r.ids[0] == 36 && r.ids[3] == 7);
    /* walking the fixture along the planned path with the B1 mock keeps every hop legal */
    CHECK(route_plan(&m, 7, 36, MAP_NORTH, &r) == ROUTE_OK);
    for (unsigned i = 0; i < r.hops; ++i) CHECK(planner_fixture_arrive(&m, r.ids[i + 1], r.headings[i]));
    CHECK(m.current == 36);
}

/* ---- brute force reference on random small maps ---- */
typedef struct { int hops, turns; uint8_t ids[16]; int valid; } best_t;
static unsigned ref_turn(int from, unsigned to)
{
    if (from < 0) return 0;
    unsigned d = (to + 4u - (unsigned)from) & 3u;
    return d == 3 ? 1 : d;
}
static const rover_node_t *ref_edge(const rover_map_t *m, const rover_node_t *a, unsigned d)
{
    if (a->exits[d] != MAP_EXPLORED) return NULL;
    const rover_node_t *b = rover_map_at(m, a->x + rover_dx[d], a->y + rover_dy[d]);
    return (b && b->exits[(d + 2u) & 3u] == MAP_EXPLORED) ? b : NULL;
}
static void ref_search(const rover_map_t *m, const rover_node_t *at, uint8_t goal, int heading, int hops, int turns,
                       uint8_t *path, uint8_t *seen, best_t *best)
{
    if (at->marker_id == goal) {
        int better = !best->valid || hops < best->hops || (hops == best->hops && turns < best->turns);
        if (!better && best->valid && hops == best->hops && turns == best->turns) better = memcmp(path, best->ids, (size_t)hops + 1) < 0;
        if (better) { best->valid = 1; best->hops = hops; best->turns = turns; memcpy(best->ids, path, (size_t)hops + 1); }
        return;
    }
    for (unsigned d = 0; d < 4; ++d) {
        const rover_node_t *b = ref_edge(m, at, d);
        if (!b || seen[b - m->nodes]) continue;
        seen[b - m->nodes] = 1; path[hops + 1] = b->marker_id;
        ref_search(m, b, goal, (int)d, hops + 1, turns + (int)ref_turn(heading, d), path, seen, best);
        seen[b - m->nodes] = 0;
    }
}

static void random_cases(unsigned seed, unsigned rounds)
{
    srand(seed);
    unsigned routed = 0, unrouted = 0;
    for (unsigned t = 0; t < rounds; ++t) {
        rover_map_t m;
        uint8_t ids[9]; unsigned used = 0;
        memset(&m, 0, sizeof m);
        for (unsigned cell = 0; cell < 9; ++cell) {
            if (rand() % 4 == 0) continue;                       /* leave some holes */
            uint8_t id;
            do { id = (uint8_t)(1 + rand() % 255); } while (rover_map_find(&m, id, NULL, NULL));
            CHECK(rover_map_place(&m, id, (int)(cell % 3) - 1, (int)(cell / 3) - 1));
            ids[used++] = id;
        }
        if (used < 2) continue;
        for (unsigned i = 0; i < m.node_count; ++i)
            for (unsigned d = 0; d < 2; ++d) {                   /* E and N from each node, mirrored by rover_map_exit */
                const rover_node_t *a = &m.nodes[i];
                unsigned dir = d ? MAP_NORTH : MAP_EAST;
                if (!rover_map_at(&m, a->x + rover_dx[dir], a->y + rover_dy[dir])) continue;
                unsigned r = (unsigned)rand() % 5;               /* explored 2/5, open, blocked, unchecked */
                uint8_t st = r < 2 ? MAP_EXPLORED : r == 2 ? MAP_OPEN : r == 3 ? MAP_BLOCKED : MAP_UNCHECKED;
                CHECK(rover_map_exit(&m, a->marker_id, dir, st));
            }
        if (rand() % 6 == 0) {                                   /* break reciprocity on purpose: one-sided EXPLORED must not route */
            rover_node_t *a = &m.nodes[(unsigned)rand() % m.node_count];
            a->exits[rand() % 4] = MAP_EXPLORED;
        }
        for (unsigned q = 0; q < 6; ++q) {
            uint8_t a = ids[(unsigned)rand() % used], b = ids[(unsigned)rand() % used];
            int heading = (rand() % 3 == 0) ? ROUTE_HEADING_UNKNOWN : rand() % 4;
            route_t r;
            route_status_t st = route_plan(&m, a, b, heading, &r);
            if (a == b) { CHECK(st == ROUTE_ALREADY_THERE && r.hops == 0); continue; }
            best_t best; memset(&best, 0, sizeof best);
            uint8_t path[16] = {a}, seen[ROVER_MAX_NODES] = {0};
            const rover_node_t *start = rover_map_find(&m, a, NULL, NULL);
            seen[start - m.nodes] = 1;
            ref_search(&m, start, b, heading, 0, 0, path, seen, &best);
            if (!best.valid) { CHECK(st == ROUTE_NO_ROUTE); ++unrouted; continue; }
            CHECK(st == ROUTE_OK);
            CHECK(r.hops == best.hops);
            CHECK(memcmp(r.ids, best.ids, (size_t)best.hops + 1) == 0);
            int turns = 0, h = heading;
            for (unsigned i = 0; i < r.hops; ++i) {
                const rover_node_t *from = rover_map_find(&m, r.ids[i], NULL, NULL);
                const rover_node_t *to = ref_edge(&m, from, r.headings[i]);
                CHECK(to && to->marker_id == r.ids[i + 1]);
                turns += (int)ref_turn(h, r.headings[i]); h = r.headings[i];
            }
            CHECK(turns == best.turns);
            ++routed;
        }
    }
    printf("random: %u routed, %u no-route, all equal to brute force\n", routed, unrouted);
    CHECK(routed > 200 && unrouted > 50);
}

static void big_case(void)
{
    /* 64 nodes in a single line plus a 2-wide block: worst-case size must finish and be right */
    rover_map_t m; memset(&m, 0, sizeof m);
    for (unsigned i = 0; i < 64; ++i) CHECK(rover_map_place(&m, (uint8_t)(i + 1), (int)i, 0));
    for (unsigned i = 0; i + 1 < 64; ++i) CHECK(rover_map_connect(&m, (uint8_t)(i + 1), (uint8_t)(i + 2), MAP_EAST));
    route_t r;
    CHECK(route_plan(&m, 1, 64, MAP_EAST, &r) == ROUTE_OK && r.hops == 63 && r.ids[63] == 63 + 1);
    CHECK(rover_map_exit(&m, 30, MAP_EAST, MAP_BLOCKED));
    CHECK(route_plan(&m, 1, 64, MAP_EAST, &r) == ROUTE_NO_ROUTE);
}

int main(void)
{
    fixture_cases();
    random_cases(1, 3000);
    random_cases(77, 2000);
    big_case();
    printf("test_route_planner: %d checks OK\n", checks);
    return 0;
}
