/* Host test for src/goto_ctrl.c with a mock executor that follows the onboard_dfs hop contract:
 * begin_hop refusal returns false with no result; a start failure returns false WITH a queued result;
 * sequence numbers must increase; one terminal result per hop, consumed once. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "planner_fixture.h"
#include "goto_ctrl.h"

static int checks;
#define CHECK(c) do { ++checks; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

typedef struct {
    rover_map_t map;
    bool ready;
    uint32_t last_seq;
    bool busy;
    unsigned ticks_left;        /* hop takes this many calls to finish */
    uint8_t target; unsigned dir; uint32_t seq;
    bool fail_next;             /* the hop fails when it finishes */
    bool fail_at_start;         /* begin_hop returns false but queues a failure */
    int refuse_count;           /* refuse this many begin_hop calls first (no result) */
    bool pending; goto_hop_result_t result;
    unsigned begins;
    unsigned delay;
    int teleport;               /* after an 'arrived' result the map says the rover is elsewhere */
} mock_t;

static const rover_map_t *m_map(void *c) { return &((mock_t *)c)->map; }
static bool m_ready(void *c) { return ((mock_t *)c)->ready && !((mock_t *)c)->busy; }
static bool m_begin(void *c, uint8_t t, unsigned d, uint32_t seq, uint32_t now)
{
    (void)now;
    mock_t *k = c; ++k->begins;
    if (k->refuse_count > 0) { --k->refuse_count; return false; }
    if (k->busy || k->pending || seq <= k->last_seq) return false;
    const rover_node_t *cur = rover_map_find(&k->map, k->map.current, NULL, NULL);
    if (!cur || cur->exits[d] != MAP_EXPLORED) return false;
    k->last_seq = seq;
    if (k->fail_at_start) {
        k->fail_at_start = false;
        k->pending = true; k->result = (goto_hop_result_t){seq, t, false, "MOTOR_TURN_REFUSED"};
        return false;
    }
    k->busy = true; k->ticks_left = k->delay; k->target = t; k->dir = d; k->seq = seq;
    return true;
}
static bool m_take(void *c, goto_hop_result_t *out)
{
    mock_t *k = c;
    if (!k->pending) return false;
    *out = k->result; k->pending = false; return true;
}
static void mock_step(mock_t *k)       /* called once per loop iteration, like onboard_dfs_tick */
{
    if (!k->busy) return;
    if (k->ticks_left && --k->ticks_left) return;
    k->busy = false;
    if (k->fail_next) {
        k->fail_next = false;
        k->pending = true; k->result = (goto_hop_result_t){k->seq, k->target, false, "OBSTACLE_DURING_SEGMENT"};
        return;
    }
    CHECK(planner_fixture_arrive(&k->map, k->target, k->dir));
    if (k->teleport) k->map.current = (uint8_t)k->teleport;
    k->pending = true; k->result = (goto_hop_result_t){k->seq, k->target, true, "ARRIVED"};
}

static goto_ctrl_t G; static mock_t K; static uint32_t NOW;
static void setup(void)
{
    memset(&K, 0, sizeof K);
    CHECK(planner_fixture_build(&K.map));
    K.ready = true; K.delay = 2;
    goto_io_t io = {&K, m_map, m_ready, m_begin, m_take};
    goto_init(&G, &io);
    NOW = 1000;
}
static void run(unsigned loops)
{
    for (unsigned i = 0; i < loops && G.state == GOTO_RUNNING; ++i) { NOW += 10; mock_step(&K); goto_tick(&G, NOW); }
}
static const char *status(void) { static char b[120]; goto_status(&G, b, sizeof b); return b; }

int main(void)
{
    char buf[120];

    /* 1. normal route 7 -> 36, three hops, ends REACHED at 36 */
    setup();
    CHECK(goto_start(&G, 36, NOW));
    CHECK(G.state == GOTO_RUNNING);
    CHECK(!goto_start(&G, 9, NOW));                 /* second GOTO while running is refused */
    run(100);
    CHECK(G.state == GOTO_IDLE && G.last_result == GOTO_RES_REACHED);
    CHECK(K.map.current == 36 && K.begins == 3 && G.hops_done == 3);
    CHECK(strstr(status(), "last=REACHED") != NULL);
    CHECK(goto_take_event(&G, buf, sizeof buf));
    CHECK(!goto_take_event(&G, buf, sizeof buf));   /* only once per change */

    /* 2. already there, unknown, no route: nothing moves */
    setup();
    CHECK(goto_start(&G, 7, NOW)); CHECK(G.last_result == GOTO_RES_REACHED && K.begins == 0);
    setup(); CHECK(goto_start(&G, 99, NOW)); CHECK(G.last_result == GOTO_RES_UNKNOWN_TARGET && K.begins == 0);
    setup(); CHECK(goto_start(&G, 255, NOW)); CHECK(G.last_result == GOTO_RES_NO_ROUTE && K.begins == 0);
    setup(); CHECK(goto_start(&G, 40, NOW)); CHECK(G.last_result == GOTO_RES_NO_ROUTE && K.begins == 0);

    /* 3. blocked edge is routed around (wall added before the request) */
    setup();
    CHECK(rover_map_exit(&K.map, 7, MAP_NORTH, MAP_BLOCKED));
    CHECK(goto_start(&G, 36, NOW)); run(100);
    CHECK(G.last_result == GOTO_RES_REACHED && K.map.current == 36);

    /* 4. a hop fails midway: route stops, no replan from a guess, reason reported */
    setup();
    CHECK(goto_start(&G, 36, NOW)); run(3);
    K.fail_next = true; run(100);
    CHECK(G.state == GOTO_IDLE && G.last_result == GOTO_RES_HOP_FAILED);
    CHECK(strstr(status(), "OBSTACLE_DURING_SEGMENT") != NULL);
    unsigned begins = K.begins; run(50); CHECK(K.begins == begins);

    /* 5. refused start with a queued failure is read, not waited on forever */
    setup(); K.fail_at_start = true;
    CHECK(goto_start(&G, 36, NOW));
    CHECK(G.state == GOTO_IDLE && G.last_result == GOTO_RES_HOP_FAILED && strstr(status(), "MOTOR_TURN_REFUSED"));

    /* 6. admission refused (stale camera) for a while, then accepted: it just waits */
    setup(); K.refuse_count = 20;
    CHECK(goto_start(&G, 36, NOW)); run(300);
    CHECK(G.last_result == GOTO_RES_REACHED);
    /* ... but a permanent refusal gives up with NOT_READY */
    setup(); K.ready = false;
    CHECK(goto_start(&G, 36, NOW)); run(1000);
    CHECK(G.state == GOTO_IDLE && G.last_result == GOTO_RES_NOT_READY && K.begins == 0);
    setup(); K.refuse_count = 1000000;
    CHECK(goto_start(&G, 36, NOW)); run(1000);
    CHECK(G.last_result == GOTO_RES_NOT_READY);

    /* 7. cancel mid-hop; the late result is dropped and never starts anything */
    setup();
    CHECK(goto_start(&G, 36, NOW)); run(1);
    goto_cancel(&G);
    CHECK(G.state == GOTO_IDLE && G.last_result == GOTO_RES_CANCELLED);
    begins = K.begins;
    for (int i = 0; i < 20; ++i) { NOW += 10; mock_step(&K); goto_tick(&G, NOW); }
    CHECK(K.begins == begins && !K.pending && G.last_result == GOTO_RES_CANCELLED);

    /* 8. stale result with the wrong sequence is ignored */
    setup();
    CHECK(goto_start(&G, 36, NOW));
    K.pending = true; K.result = (goto_hop_result_t){999, 3, false, "OLD"};
    NOW += 10; goto_tick(&G, NOW);
    CHECK(G.state == GOTO_RUNNING);                 /* stale failure did not abort the route */
    run(100); CHECK(G.last_result == GOTO_RES_REACHED);

    /* 9. the map changes while a hop is in flight (a later edge becomes blocked): the next hop re-plans */
    setup();
    CHECK(goto_start(&G, 36, NOW)); run(3);        /* hop 1 (7 -> 3) done, hop 2 (3 -> 9) driving */
    CHECK(K.map.current == 3 && G.hop_in_flight);
    CHECK(rover_map_exit(&K.map, 9, MAP_EAST, MAP_BLOCKED));   /* 9-36 closes before hop 3 starts */
    run(300);
    CHECK(G.last_result == GOTO_RES_REACHED && K.map.current == 36 && K.begins > 3);

    /* 10. a second route works after a finished one (sequence keeps increasing) */
    setup();
    CHECK(goto_start(&G, 36, NOW)); run(100); CHECK(K.map.current == 36);
    CHECK(goto_start(&G, 7, NOW)); run(100);
    CHECK(G.last_result == GOTO_RES_REACHED && K.map.current == 7);

    /* 11. executor says ARRIVED but the authoritative current node is not the hop target: stop, do not trust it */
    setup(); K.teleport = 7;
    CHECK(goto_start(&G, 36, NOW)); run(100);
    CHECK(G.last_result == GOTO_RES_HOP_FAILED && strstr(status(), "UNEXPECTED_NODE"));

    printf("test_goto_ctrl: %d checks OK\n", checks);
    return 0;
}
