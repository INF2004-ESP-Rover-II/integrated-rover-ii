/**
 * @file test_route_planner.c
 * @brief Host test for src/route_planner.c. Uses Buddy 1's real rover_map.c
 *        and the hardcoded planner fixture. Random maps are checked against
 *        a brute-force search.
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "planner_fixture.h"
#include "route_planner.h"
#include "test_support.h"

/* ---- Constants ---------------------------------------------------------- */

/** Cells in the 3 x 3 random arena. */
#define TEST_ARENA_CELLS (9u)

/** Width of the random arena. */
#define TEST_ARENA_WIDTH (3u)

/** Longest route the brute-force search handles. */
#define TEST_REF_PATH (16u)

/** Directions on the grid. */
#define TEST_DIRECTIONS (4u)

/** Planner queries per random map. */
#define TEST_QUERIES (6u)

/** Longest straight line in the big case. */
#define TEST_BIG_NODES (64u)

/* ---- Types -------------------------------------------------------------- */

/** Best route found by the brute-force search. */
typedef struct
{
    int32_t hops;
    int32_t turns;
    uint8_t ids[TEST_REF_PATH];
    bool valid;
} test_best_t;

/** What the planner is expected to return. */
typedef struct
{
    uint8_t start_id;
    uint8_t goal_id;
    int32_t heading;
    route_planner_status_t status;
    const uint8_t * p_ids;
    const uint8_t * p_headings;
    uint32_t hops;
} test_expect_t;

/* ---- Private prototypes ------------------------------------------------- */

static void test_expect_route(const rover_map_t * p_map,
                              const test_expect_t * p_expect);
static void test_fixture_cases(void);
static void test_next_hop_cases(void);
static uint32_t test_ref_turn(int32_t from_heading, uint32_t direction);
static const rover_node_t * test_ref_edge(const rover_map_t * p_map,
                                          const rover_node_t * p_from,
                                          uint32_t direction);
static bool test_ref_better(const test_best_t * p_best,
                            int32_t hops,
                            int32_t turns,
                            const uint8_t * p_path);
static void test_ref_search(const rover_map_t * p_map,
                            const rover_node_t * p_at,
                            uint8_t goal_id,
                            int32_t heading,
                            int32_t hops,
                            int32_t turns,
                            uint8_t * p_path,
                            uint8_t * p_seen,
                            test_best_t * p_best);
static uint32_t test_random(uint32_t limit);
static uint32_t test_build_random_map(rover_map_t * p_map, uint8_t * p_ids);
static void test_random_edges(rover_map_t * p_map);
static bool test_random_query(const rover_map_t * p_map,
                              uint8_t start_id,
                              uint8_t goal_id);
static void test_random_cases(uint32_t seed, uint32_t rounds);
static void test_big_case(void);

/* ---- Static data -------------------------------------------------------- */

/** Routed and unrouted queries seen by test_random_cases. */
static uint32_t g_routed = 0u;
static uint32_t g_unrouted = 0u;

/* ---- Public functions --------------------------------------------------- */

int main(void)
{
    test_fixture_cases();
    test_next_hop_cases();
    test_random_cases(1u, 3000u);
    test_random_cases(77u, 2000u);
    test_big_case();

    return test_finish("test_route_planner");
}

/* ---- Private functions -------------------------------------------------- */

/** Plan and compare with the expected status and route. */
static void test_expect_route(const rover_map_t * p_map,
                              const test_expect_t * p_expect)
{
    route_planner_route_t route;
    route_planner_status_t got;
    uint32_t index = 0u;

    got = route_planner_plan(p_map, p_expect->start_id, p_expect->goal_id,
                             p_expect->heading, &route);
    test_check(p_expect->status == got, __LINE__);

    if (ROUTE_PLANNER_OK == p_expect->status)
    {
        test_check(p_expect->hops == route.hops, __LINE__);

        for (index = 0u; index <= p_expect->hops; index++)
        {
            test_check(p_expect->p_ids[index] == route.ids[index], __LINE__);
        }

        for (index = 0u; index < p_expect->hops; index++)
        {
            test_check(p_expect->p_headings[index] == route.headings[index],
                       __LINE__);
        }
    }
}

/** Hand-checked routes on Buddy 1's fixture map. */
static void test_fixture_cases(void)
{
    rover_map_t map;
    route_planner_route_t route;
    uint32_t index = 0u;
    const uint8_t path_a[] = {7u, 3u, 9u, 36u};
    const uint8_t head_a[] = {MAP_NORTH, MAP_EAST, MAP_EAST};
    const uint8_t path_b[] = {7u, 12u, 20u, 36u};
    const uint8_t head_b[] = {MAP_EAST, MAP_EAST, MAP_NORTH};
    const uint8_t path_same[] = {7u};
    test_expect_t expect;

    test_check(planner_fixture_build(&map), __LINE__);

    /* Smallest marker ID wins a tie. */
    expect = (test_expect_t){7u, 36u, ROUTE_PLANNER_HEADING_UNKNOWN,
                             ROUTE_PLANNER_OK, path_a, head_a, 3u};
    test_expect_route(&map, &expect);

    /* Fewest turns wins when the heading is known. */
    expect = (test_expect_t){7u, 36u, MAP_NORTH, ROUTE_PLANNER_OK,
                             path_a, head_a, 3u};
    test_expect_route(&map, &expect);

    test_check(rover_map_exit(&map, 7u, MAP_NORTH, MAP_BLOCKED), __LINE__);
    expect = (test_expect_t){7u, 36u, MAP_NORTH, ROUTE_PLANNER_OK,
                             path_b, head_b, 3u};
    test_expect_route(&map, &expect);

    test_check(rover_map_exit(&map, 7u, MAP_EAST, MAP_BLOCKED), __LINE__);
    expect = (test_expect_t){7u, 36u, ROUTE_PLANNER_HEADING_UNKNOWN,
                             ROUTE_PLANNER_NO_ROUTE, NULL, NULL, 0u};
    test_expect_route(&map, &expect);

    /* Fresh map: only an OPEN edge, a disconnected node, unknown nodes. */
    test_check(planner_fixture_build(&map), __LINE__);
    expect = (test_expect_t){7u, 40u, ROUTE_PLANNER_HEADING_UNKNOWN,
                             ROUTE_PLANNER_NO_ROUTE, NULL, NULL, 0u};
    test_expect_route(&map, &expect);
    expect.goal_id = 255u;
    test_expect_route(&map, &expect);
    expect.goal_id = 99u;
    expect.status = ROUTE_PLANNER_UNKNOWN_TARGET;
    test_expect_route(&map, &expect);
    expect = (test_expect_t){99u, 7u, ROUTE_PLANNER_HEADING_UNKNOWN,
                             ROUTE_PLANNER_UNKNOWN_START, NULL, NULL, 0u};
    test_expect_route(&map, &expect);
    expect = (test_expect_t){7u, 7u, ROUTE_PLANNER_HEADING_UNKNOWN,
                             ROUTE_PLANNER_ALREADY_THERE, path_same, NULL,
                             0u};
    test_expect_route(&map, &expect);

    /* The reverse trip works, and null arguments are refused safely. */
    test_check(ROUTE_PLANNER_OK == route_planner_plan(
                   &map, 36u, 7u, ROUTE_PLANNER_HEADING_UNKNOWN, &route),
               __LINE__);
    test_check((3u == route.hops) && (36u == route.ids[0])
               && (7u == route.ids[3]), __LINE__);
    test_check(ROUTE_PLANNER_NO_ROUTE == route_planner_plan(
                   NULL, 7u, 36u, ROUTE_PLANNER_HEADING_UNKNOWN, &route),
               __LINE__);
    test_check(ROUTE_PLANNER_NO_ROUTE == route_planner_plan(
                   &map, 7u, 36u, ROUTE_PLANNER_HEADING_UNKNOWN, NULL),
               __LINE__);

    /* Walking the fixture along the planned path keeps every hop legal. */
    test_check(ROUTE_PLANNER_OK == route_planner_plan(
                   &map, 7u, 36u, MAP_NORTH, &route), __LINE__);

    for (index = 0u; index < route.hops; index++)
    {
        test_check(planner_fixture_arrive(&map, route.ids[index + 1u],
                                          route.headings[index]), __LINE__);
    }

    test_check(36u == map.current, __LINE__);
}

/** The one-hop view used by the DFS. */
static void test_next_hop_cases(void)
{
    rover_map_t map;
    route_planner_step_t step;

    test_check(planner_fixture_build(&map), __LINE__);

    test_check(ROUTE_PLANNER_OK == route_planner_next_hop(
                   &map, 7u, 36u, MAP_NORTH, &step), __LINE__);
    test_check((3u == step.next_id) && (MAP_NORTH == step.direction)
               && (3u == step.hops_to_go), __LINE__);

    /* Already there: no move, all fields zero. */
    test_check(ROUTE_PLANNER_ALREADY_THERE == route_planner_next_hop(
                   &map, 7u, 7u, MAP_NORTH, &step), __LINE__);
    test_check((0u == step.next_id) && (0u == step.hops_to_go), __LINE__);

    /* Unknown target and no route also give a zero step. */
    test_check(ROUTE_PLANNER_UNKNOWN_TARGET == route_planner_next_hop(
                   &map, 7u, 99u, MAP_NORTH, &step), __LINE__);
    test_check(0u == step.next_id, __LINE__);
    test_check(ROUTE_PLANNER_NO_ROUTE == route_planner_next_hop(
                   &map, 7u, 40u, MAP_NORTH, &step), __LINE__);
    test_check(0u == step.next_id, __LINE__);

    /* The reverse trip leaves 36 to the west, so the direction is not 0. */
    test_check(ROUTE_PLANNER_OK == route_planner_next_hop(
                   &map, 36u, 7u, ROUTE_PLANNER_HEADING_UNKNOWN, &step),
               __LINE__);
    test_check((9u == step.next_id) && (MAP_WEST == step.direction)
               && (3u == step.hops_to_go), __LINE__);

    /* A NULL step pointer is allowed. */
    test_check(ROUTE_PLANNER_OK == route_planner_next_hop(
                   &map, 7u, 36u, MAP_NORTH, NULL), __LINE__);

    /* Status names. */
    test_check(0 == strcmp("NO_ROUTE",
                           route_planner_status_name(ROUTE_PLANNER_NO_ROUTE)),
               __LINE__);
}

/* ---- Brute-force reference ---------------------------------------------- */

/** Turn cost reference: 0 straight, 1 quarter turn, 2 U-turn. */
static uint32_t test_ref_turn(int32_t from_heading, uint32_t direction)
{
    uint32_t cost = 0u;
    uint32_t difference = 0u;

    if (from_heading >= 0)
    {
        /* from_heading was just checked to be 0 or more. */
        difference = (direction + TEST_DIRECTIONS
                      - (uint32_t)from_heading) % TEST_DIRECTIONS;

        if (3u == difference)
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

/** Neighbour through an edge that is explored from both ends, or NULL. */
static const rover_node_t * test_ref_edge(const rover_map_t * p_map,
                                          const rover_node_t * p_from,
                                          uint32_t direction)
{
    const rover_node_t * p_to = NULL;
    const rover_node_t * p_result = NULL;

    if (MAP_EXPLORED == p_from->exits[direction])
    {
        p_to = rover_map_at(p_map, p_from->x + rover_dx[direction],
                            p_from->y + rover_dy[direction]);
    }

    if (NULL != p_to)
    {
        if (MAP_EXPLORED == p_to->exits[(direction + 2u) % TEST_DIRECTIONS])
        {
            p_result = p_to;
        }
    }

    return p_result;
}

/** Is the candidate better than the best route so far? */
static bool test_ref_better(const test_best_t * p_best,
                            int32_t hops,
                            int32_t turns,
                            const uint8_t * p_path)
{
    bool better = false;

    if (!p_best->valid)
    {
        better = true;
    }
    else if (hops != p_best->hops)
    {
        better = hops < p_best->hops;
    }
    else if (turns != p_best->turns)
    {
        better = turns < p_best->turns;
    }
    else
    {
        /* hops is 0 or more here. */
        better = memcmp(p_path, p_best->ids, (size_t)hops + 1u) < 0;
    }

    return better;
}

/** Depth-first enumeration of every simple path, keeping the best. */
static void test_ref_search(const rover_map_t * p_map,
                            const rover_node_t * p_at,
                            uint8_t goal_id,
                            int32_t heading,
                            int32_t hops,
                            int32_t turns,
                            uint8_t * p_path,
                            uint8_t * p_seen,
                            test_best_t * p_best)
{
    uint32_t direction = 0u;
    const rover_node_t * p_next = NULL;
    ptrdiff_t slot = 0;

    if (goal_id == p_at->marker_id)
    {
        if (test_ref_better(p_best, hops, turns, p_path))
        {
            p_best->valid = true;
            p_best->hops = hops;
            p_best->turns = turns;
            (void)memcpy(p_best->ids, p_path, (size_t)hops + 1u);
        }
    }
    else
    {
        for (direction = 0u; direction < TEST_DIRECTIONS; direction++)
        {
            p_next = test_ref_edge(p_map, p_at, direction);

            if (NULL != p_next)
            {
                slot = p_next - p_map->nodes;

                if (0u == p_seen[slot])
                {
                    p_seen[slot] = 1u;
                    p_path[hops + 1] = p_next->marker_id;
                    test_ref_search(p_map, p_next, goal_id,
                                    (int32_t)direction, hops + 1,
                                    turns + (int32_t)test_ref_turn(
                                        heading, direction),
                                    p_path, p_seen, p_best);
                    p_seen[slot] = 0u;
                }
            }
        }
    }
}

/* ---- Random maps -------------------------------------------------------- */

/** Random number in 0 to limit - 1. */
static uint32_t test_random(uint32_t limit)
{
    /* rand() returns 0 to RAND_MAX, never negative, so the cast is safe. */
    return (uint32_t)rand() % limit;
}

/** Fill a 3 x 3 arena with random holes. Returns the node count. */
static uint32_t test_build_random_map(rover_map_t * p_map, uint8_t * p_ids)
{
    uint32_t cell = 0u;
    uint32_t used = 0u;
    uint8_t marker_id = 0u;
    int x_pos = 0;
    int y_pos = 0;

    (void)memset(p_map, 0, sizeof(*p_map));

    for (cell = 0u; cell < TEST_ARENA_CELLS; cell++)
    {
        if (0u != test_random(4u))
        {
            do
            {
                /* 1 to 255 fits in uint8_t. */
                marker_id = (uint8_t)(1u + test_random(255u));
            } while (NULL != rover_map_find(p_map, marker_id, NULL, NULL));

            /* NOTE: int matches the rover_map_place() prototype. */
            x_pos = (int)(cell % TEST_ARENA_WIDTH) - 1;
            y_pos = (int)(cell / TEST_ARENA_WIDTH) - 1;
            test_check(rover_map_place(p_map, marker_id, x_pos, y_pos),
                       __LINE__);
            p_ids[used] = marker_id;
            used++;
        }
    }

    return used;
}

/** Give each east and north edge a random state, then maybe corrupt one. */
static void test_random_edges(rover_map_t * p_map)
{
    static const uint8_t states[] =
    {
        MAP_EXPLORED, MAP_EXPLORED, MAP_OPEN, MAP_BLOCKED, MAP_UNCHECKED
    };
    uint32_t index = 0u;
    uint32_t pick = 0u;
    uint32_t direction = 0u;
    const rover_node_t * p_node = NULL;
    rover_node_t * p_corrupt = NULL;

    for (index = 0u; index < p_map->node_count; index++)
    {
        for (pick = 0u; pick < 2u; pick++)
        {
            p_node = &p_map->nodes[index];

            if (0u == pick)
            {
                direction = MAP_EAST;
            }
            else
            {
                direction = MAP_NORTH;
            }

            if (NULL != rover_map_at(p_map, p_node->x + rover_dx[direction],
                                     p_node->y + rover_dy[direction]))
            {
                test_check(rover_map_exit(p_map, p_node->marker_id,
                                          direction,
                                          states[test_random(5u)]),
                           __LINE__);
            }
        }
    }

    /* Break reciprocity on purpose: a one-sided EXPLORED must not route. */
    if (0u == test_random(6u))
    {
        p_corrupt = &p_map->nodes[test_random(p_map->node_count)];
        p_corrupt->exits[test_random(TEST_DIRECTIONS)] = MAP_EXPLORED;
    }
}

/** Compare one planner answer with the brute force. True if it routed. */
static bool test_random_query(const rover_map_t * p_map,
                              uint8_t start_id,
                              uint8_t goal_id)
{
    route_planner_route_t route;
    route_planner_status_t status;
    test_best_t best;
    uint8_t path[TEST_REF_PATH];
    uint8_t seen[ROVER_MAX_NODES];
    const rover_node_t * p_start = NULL;
    const rover_node_t * p_from = NULL;
    const rover_node_t * p_to = NULL;
    int32_t heading = 0;
    int32_t turns = 0;
    int32_t walk_heading = 0;
    uint32_t index = 0u;
    bool routed = false;

    if (0u == test_random(3u))
    {
        heading = ROUTE_PLANNER_HEADING_UNKNOWN;
    }
    else
    {
        /* 0 to 3 fits in int32_t. */
        heading = (int32_t)test_random(TEST_DIRECTIONS);
    }

    status = route_planner_plan(p_map, start_id, goal_id, heading, &route);

    if (start_id == goal_id)
    {
        test_check((ROUTE_PLANNER_ALREADY_THERE == status)
                   && (0u == route.hops), __LINE__);
    }
    else
    {
        (void)memset(&best, 0, sizeof(best));
        (void)memset(seen, 0, sizeof(seen));
        path[0] = start_id;
        p_start = rover_map_find(p_map, start_id, NULL, NULL);
        seen[p_start - p_map->nodes] = 1u;
        test_ref_search(p_map, p_start, goal_id, heading, 0, 0, path, seen,
                        &best);

        if (!best.valid)
        {
            test_check(ROUTE_PLANNER_NO_ROUTE == status, __LINE__);
        }
        else
        {
            routed = true;
            test_check(ROUTE_PLANNER_OK == status, __LINE__);
            test_check(best.hops == (int32_t)route.hops, __LINE__);
            test_check(0 == memcmp(route.ids, best.ids,
                                   (size_t)best.hops + 1u), __LINE__);
            walk_heading = heading;

            for (index = 0u; index < route.hops; index++)
            {
                p_from = rover_map_find(p_map, route.ids[index], NULL, NULL);
                p_to = test_ref_edge(p_map, p_from, route.headings[index]);
                test_check((NULL != p_to)
                           && (p_to->marker_id == route.ids[index + 1u]),
                           __LINE__);
                turns += (int32_t)test_ref_turn(walk_heading,
                                                route.headings[index]);
                walk_heading = route.headings[index];
            }

            test_check(turns == best.turns, __LINE__);
        }
    }

    return routed;
}

/** Many random maps, every answer equal to the brute force. */
static void test_random_cases(uint32_t seed, uint32_t rounds)
{
    rover_map_t map;
    uint8_t ids[TEST_ARENA_CELLS];
    uint32_t round = 0u;
    uint32_t used = 0u;
    uint32_t query = 0u;
    uint32_t routed = 0u;
    uint32_t unrouted = 0u;

    srand(seed);

    for (round = 0u; round < rounds; round++)
    {
        used = test_build_random_map(&map, ids);

        if (used >= 2u)
        {
            test_random_edges(&map);

            for (query = 0u; query < TEST_QUERIES; query++)
            {
                if (test_random_query(&map, ids[test_random(used)],
                                      ids[test_random(used)]))
                {
                    routed++;
                }
                else
                {
                    unrouted++;
                }
            }
        }
    }

    g_routed += routed;
    g_unrouted += unrouted;
    (void)printf("random: %" PRIu32 " queries routed, %" PRIu32
                 " not routed or trivial, all equal to brute force\n",
                 routed, unrouted);
    test_check((routed > 200u) && (unrouted > 50u), __LINE__);
}

/** 64 nodes in a line: worst-case size must finish and be right. */
static void test_big_case(void)
{
    rover_map_t map;
    route_planner_route_t route;
    uint32_t index = 0u;

    (void)memset(&map, 0, sizeof(map));

    for (index = 0u; index < TEST_BIG_NODES; index++)
    {
        /* NOTE: int matches the rover_map_place() prototype. The ID is
         * 1 to 64 and the coordinate 0 to 63, so both casts are safe. */
        test_check(rover_map_place(&map, (uint8_t)(index + 1u),
                                   (int)index, 0), __LINE__);
    }

    for (index = 0u; (index + 1u) < TEST_BIG_NODES; index++)
    {
        test_check(rover_map_connect(&map, (uint8_t)(index + 1u),
                                     (uint8_t)(index + 2u), MAP_EAST),
                   __LINE__);
    }

    test_check(ROUTE_PLANNER_OK == route_planner_plan(
                   &map, 1u, 64u, MAP_EAST, &route), __LINE__);
    test_check((63u == route.hops) && (64u == route.ids[63]), __LINE__);
    test_check(MAP_EAST == route.headings[0], __LINE__);
    test_check(rover_map_exit(&map, 30u, MAP_EAST, MAP_BLOCKED), __LINE__);
    test_check(ROUTE_PLANNER_NO_ROUTE == route_planner_plan(
                   &map, 1u, 64u, MAP_EAST, &route), __LINE__);
}

/* end of file test_route_planner.c */
