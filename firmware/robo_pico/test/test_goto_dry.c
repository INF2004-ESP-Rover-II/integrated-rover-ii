/**
 * @file test_goto_dry.c
 * @brief Host test for src/goto_dry.c (plan-only DEMO_MAP and GOTO_DRY).
 */

#include <stdio.h>
#include <string.h>
#include "goto_dry.h"
#include "planner_fixture.h"
#include "test_support.h"

/** Size of the result text buffer. */
#define TEST_TEXT_SIZE (160u)

/** Step of the fake clock, in microseconds. */
#define TEST_CLOCK_STEP_US (7u)

/** Fake microsecond clock: advances by a fixed step on every call. */
static uint32_t test_fake_clock(void);

/** Does the text contain the given part? */
static bool test_has(const char * p_text, const char * p_part);

/** Current time of the fake clock. */
static uint32_t g_clock_us = 0u;

int main(void)
{
    rover_map_t map;
    rover_map_t before;
    char text[TEST_TEXT_SIZE];

    test_check(goto_dry_build_demo_map(&map), __LINE__);
    test_check((6u == map.node_count) && (6u == map.visited_count)
               && (1u == map.current) && (1u == map.start), __LINE__);
    before = map;

    (void)goto_dry_run(&map, 4u, text, sizeof(text), test_fake_clock);
    test_check(0 == strcmp(text, "GOTO_DRY 1>2>3>4 hops=3 plan_us=7 "
                                 "result=REACHED"), __LINE__);

    /* The real map is untouched. */
    test_check(0 == memcmp(&map, &before, sizeof(map)), __LINE__);

    (void)goto_dry_run(&map, 1u, text, sizeof(text), test_fake_clock);
    test_check(test_has(text, "GOTO_DRY 1 hops=0")
               && test_has(text, "REACHED"), __LINE__);

    (void)goto_dry_run(&map, 99u, text, sizeof(text), test_fake_clock);
    test_check(test_has(text, "none") && test_has(text, "UNKNOWN_TARGET"),
               __LINE__);

    test_check(rover_map_exit(&map, 1u, MAP_EAST, MAP_BLOCKED), __LINE__);
    (void)goto_dry_run(&map, 4u, text, sizeof(text), test_fake_clock);
    test_check(test_has(text, "1>6>5>4") && test_has(text, "REACHED"),
               __LINE__);

    test_check(rover_map_exit(&map, 1u, MAP_NORTH, MAP_BLOCKED), __LINE__);
    (void)goto_dry_run(&map, 4u, text, sizeof(text), test_fake_clock);
    test_check(test_has(text, "none") && test_has(text, "NO_ROUTE"),
               __LINE__);

    /* Buddy 1's fixture also works. */
    test_check(planner_fixture_build(&map), __LINE__);
    (void)goto_dry_run(&map, 36u, text, sizeof(text), test_fake_clock);
    test_check(test_has(text, "7>3>9>36") && test_has(text, "REACHED"),
               __LINE__);
    (void)goto_dry_run(&map, 255u, text, sizeof(text), test_fake_clock);
    test_check(test_has(text, "NO_ROUTE"), __LINE__);

    return test_finish("test_goto_dry");
}

static uint32_t test_fake_clock(void)
{
    g_clock_us += TEST_CLOCK_STEP_US;

    return g_clock_us;
}

static bool test_has(const char * p_text, const char * p_part)
{
    return NULL != strstr(p_text, p_part);
}

/* end of file test_goto_dry.c */
