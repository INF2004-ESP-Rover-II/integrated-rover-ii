#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "goto_dry.h"
#include "planner_fixture.h"
static int checks;
#define CHECK(c) do { ++checks; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
static uint32_t fake_clock(void) { static uint32_t t; return t += 7; }
int main(void)
{
    rover_map_t m, before; char out[160];
    CHECK(demo_map_build(&m));
    CHECK(m.node_count == 6 && m.visited_count == 6 && m.current == 1 && m.start == 1);
    before = m;
    goto_dry_run(&m, 4, out, sizeof out, fake_clock);
    CHECK(strcmp(out, "GOTO_DRY 1>2>3>4 hops=3 plan_us=7 result=REACHED") == 0);
    CHECK(memcmp(&m, &before, sizeof m) == 0);                 /* real map untouched */
    goto_dry_run(&m, 1, out, sizeof out, fake_clock); CHECK(strstr(out, "GOTO_DRY 1 hops=0") && strstr(out, "REACHED"));
    goto_dry_run(&m, 99, out, sizeof out, fake_clock); CHECK(strstr(out, "none") && strstr(out, "UNKNOWN_TARGET"));
    CHECK(rover_map_exit(&m, 1, MAP_EAST, MAP_BLOCKED));
    goto_dry_run(&m, 4, out, sizeof out, fake_clock); CHECK(strstr(out, "1>6>5>4") && strstr(out, "REACHED"));
    CHECK(rover_map_exit(&m, 1, MAP_NORTH, MAP_BLOCKED));
    goto_dry_run(&m, 4, out, sizeof out, fake_clock); CHECK(strstr(out, "none") && strstr(out, "NO_ROUTE"));
    CHECK(planner_fixture_build(&m));                           /* B1's fixture also works */
    goto_dry_run(&m, 36, out, sizeof out, fake_clock); CHECK(strstr(out, "7>3>9>36") && strstr(out, "REACHED"));
    goto_dry_run(&m, 255, out, sizeof out, fake_clock); CHECK(strstr(out, "NO_ROUTE"));
    printf("test_goto_dry: %d checks OK\n", checks);
    return 0;
}
