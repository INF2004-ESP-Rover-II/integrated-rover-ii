#define _POSIX_C_SOURCE 200809L
/* test_map_cmd.c - host checks for the MAP_ text commands, the JSON snapshot and the self test. */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "map_cmd.h"

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static char reply[256];
static void cmd(const char *s) { memset(reply, 0, sizeof reply); (void)map_cmd_handle(s, reply, sizeof reply); printf("  %-28s -> %s\n", s, reply); }

static uint32_t clock_us(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint32_t)(t.tv_sec * 1000000u + t.tv_nsec / 1000u); }

int main(void)
{
    char json[4096];
    map_cmd_init();
    map_cmd_set_clock_us(clock_us);
    CHECK(!map_cmd_handle("FORWARD", reply, sizeof reply), "FORWARD must not be taken by the mapping parser");

    cmd("MAP_START:4,1");        CHECK(strstr(reply, "MAP_OK") && strstr(reply, "state=MAPPING"), "start");
    cmd("MAP_REACHED:2,1");
    cmd("MAP_REACHED:3,2");
    cmd("MAP_EXITS:3,3,3,1,1");
    cmd("MAP_REACHED:4,3");      CHECK(strstr(reply, "state=MAP_COMPLETE"), "complete at 4 markers");
    cmd("MAP_REACHED:9,0");      CHECK(strstr(reply, "MAP_REFUSED"), "not in MAPPING any more");
    cmd("MAP_CONFIRM:1");        CHECK(strstr(reply, "MAP_OK"), "confirm");
    cmd("MAP_GOTO:3");           CHECK(strstr(reply, "HOP") && strstr(reply, "target=2"), "first hop via 2");
    cmd("MAP_HOP_BLOCKED:1,1");  CHECK(strstr(reply, "target=4"), "replan via 4");
    cmd("MAP_HOP_DONE");         CHECK(strstr(reply, "target=3") && strstr(reply, "last=1"), "second hop");
    cmd("MAP_HOP_DONE");         CHECK(strstr(reply, "RESULT=REACHED"), "reached");
    cmd("MAP_GOTO:99");          CHECK(strstr(reply, "MAP_REFUSED"), "unknown target");
    cmd("MAP_START:x,y");        CHECK(strstr(reply, "MAP_REFUSED"), "garbage args refused, no crash");
    cmd("MAP_BOGUS");            CHECK(strstr(reply, "MAP_REFUSED"), "unknown command");
    cmd("MAP_STATUS");

    size_t n = map_snapshot_json(map_cmd_fsm(), json, sizeof json);
    CHECK(n > 0 && json[0] == '{' && json[n - 1] == '}', "snapshot json");
    printf("  snapshot (%zu bytes): %s\n", n, json);
    CHECK(strstr(json, "\"state\":\"MAP_COMPLETE\"") && strstr(json, "\"last_result\":\"REACHED\""), "snapshot fields");
    char tiny[40];
    CHECK(map_snapshot_json(map_cmd_fsm(), tiny, sizeof tiny) == 0, "a snapshot that does not fit is dropped, not truncated");

    cmd("MAP_SELFTEST");         CHECK(strstr(reply, "PASS"), "selftest");
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
