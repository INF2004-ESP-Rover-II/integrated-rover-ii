/**
 * @file test_goto_ctrl.c
 * @brief Host test for src/goto_ctrl.c with a mock executor that follows the
 *        onboard_dfs hop contract: a refused begin_hop returns false with no
 *        result, a start failure returns false WITH a queued result,
 *        sequence numbers must increase, and there is one terminal result
 *        per hop, consumed once.
 */

#include <stdio.h>
#include <string.h>
#include "goto_ctrl.h"
#include "planner_fixture.h"
#include "test_support.h"

/* ---- Constants ---------------------------------------------------------- */

/** Time between loop iterations in the test. */
#define TEST_TICK_MS (10u)

/** Start time of every scenario. */
#define TEST_START_MS (1000u)

/** Refusals that never end. */
#define TEST_REFUSE_FOREVER (1000000)

/* ---- Types -------------------------------------------------------------- */

/** Mock executor. */
typedef struct
{
    rover_map_t map;
    bool ready;
    uint32_t last_seq;
    bool busy;
    uint32_t ticks_left;     /**< Calls a hop takes to finish. */
    uint8_t target;
    uint32_t direction;
    uint32_t seq;
    bool fail_next;          /**< The hop fails when it finishes. */
    bool fail_at_start;      /**< begin_hop is false but queues a failure. */
    int32_t refuse_count;    /**< Refuse this many begin_hop calls first. */
    bool pending;
    goto_ctrl_hop_result_t result;
    uint32_t begins;
    uint32_t delay;
    uint8_t teleport;        /**< After ARRIVED the map says rover is here. */
} test_mock_t;

/* ---- Private prototypes ------------------------------------------------- */

static const rover_map_t * test_mock_map(void * p_context);
static bool test_mock_ready(void * p_context);
static bool test_mock_begin(void * p_context,
                            uint8_t target,
                            uint32_t direction,
                            uint32_t sequence,
                            uint32_t now_ms);
static bool test_mock_take(void * p_context,
                           goto_ctrl_hop_result_t * p_result);
static void test_queue_result(test_mock_t * p_mock,
                              uint32_t sequence,
                              uint8_t target,
                              bool arrived,
                              const char * p_reason);
static void test_mock_step(test_mock_t * p_mock);
static void test_setup(void);
static void test_run(uint32_t loops);
static bool test_status_has(const char * p_text);
static void test_normal_route(void);
static void test_no_motion_cases(void);
static void test_hop_failures(void);
static void test_admission(void);
static void test_cancel_and_stale(void);
static void test_replan_and_repeat(void);

/* ---- Static data -------------------------------------------------------- */

static goto_ctrl_t g_ctrl;
static test_mock_t g_mock;
static uint32_t g_now_ms;

/* ---- Public functions --------------------------------------------------- */

int main(void)
{
    test_normal_route();
    test_no_motion_cases();
    test_hop_failures();
    test_admission();
    test_cancel_and_stale();
    test_replan_and_repeat();

    return test_finish("test_goto_ctrl");
}

/* ---- Mock executor ------------------------------------------------------ */

/** Mock: the map the controller plans on. */
static const rover_map_t * test_mock_map(void * p_context)
{
    const test_mock_t * p_mock = p_context;

    return &p_mock->map;
}

/** Mock: ready when configured so and not busy. */
static bool test_mock_ready(void * p_context)
{
    const test_mock_t * p_mock = p_context;

    return p_mock->ready && (!p_mock->busy);
}

/** Mock: queue a terminal result. */
static void test_queue_result(test_mock_t * p_mock,
                              uint32_t sequence,
                              uint8_t target,
                              bool arrived,
                              const char * p_reason)
{
    p_mock->pending = true;
    p_mock->result.sequence = sequence;
    p_mock->result.target = target;
    p_mock->result.arrived = arrived;
    (void)snprintf(p_mock->result.reason, sizeof(p_mock->result.reason),
                   "%s", p_reason);
}

/** Mock: begin_hop with the same refusal rules as the real executor. */
static bool test_mock_begin(void * p_context,
                            uint8_t target,
                            uint32_t direction,
                            uint32_t sequence,
                            uint32_t now_ms)
{
    test_mock_t * p_mock = p_context;
    const rover_node_t * p_current = NULL;
    bool accepted = false;

    (void)now_ms;
    p_mock->begins++;
    p_current = rover_map_find(&p_mock->map, p_mock->map.current, NULL, NULL);

    if (p_mock->refuse_count > 0)
    {
        p_mock->refuse_count--;
    }
    else if (p_mock->busy || p_mock->pending
             || (sequence <= p_mock->last_seq) || (NULL == p_current))
    {
        /* Refused, no result queued. */
    }
    else if (MAP_EXPLORED != p_current->exits[direction])
    {
        /* Refused, no result queued. */
    }
    else if (p_mock->fail_at_start)
    {
        p_mock->last_seq = sequence;
        p_mock->fail_at_start = false;
        test_queue_result(p_mock, sequence, target, false,
                          "MOTOR_TURN_REFUSED");
    }
    else
    {
        p_mock->last_seq = sequence;
        p_mock->busy = true;
        p_mock->ticks_left = p_mock->delay;
        p_mock->target = target;
        p_mock->direction = direction;
        p_mock->seq = sequence;
        accepted = true;
    }

    return accepted;
}

/** Mock: hand over the queued result once. */
static bool test_mock_take(void * p_context,
                           goto_ctrl_hop_result_t * p_result)
{
    test_mock_t * p_mock = p_context;
    bool taken = false;

    if (p_mock->pending)
    {
        *p_result = p_mock->result;
        p_mock->pending = false;
        taken = true;
    }

    return taken;
}

/** Mock: advance the hop in progress, like onboard_dfs_tick. */
static void test_mock_step(test_mock_t * p_mock)
{
    bool finished = false;

    if (p_mock->busy)
    {
        if (p_mock->ticks_left > 0u)
        {
            p_mock->ticks_left--;
        }

        finished = (0u == p_mock->ticks_left);
    }

    if (finished)
    {
        p_mock->busy = false;

        if (p_mock->fail_next)
        {
            p_mock->fail_next = false;
            test_queue_result(p_mock, p_mock->seq, p_mock->target, false,
                              "OBSTACLE_DURING_SEGMENT");
        }
        else
        {
            test_check(planner_fixture_arrive(&p_mock->map, p_mock->target,
                                              p_mock->direction), __LINE__);

            if (0u != p_mock->teleport)
            {
                p_mock->map.current = p_mock->teleport;
            }

            test_queue_result(p_mock, p_mock->seq, p_mock->target, true,
                              "ARRIVED");
        }
    }
}

/* ---- Scenario helpers --------------------------------------------------- */

/** Fresh mock, fresh controller. */
static void test_setup(void)
{
    goto_ctrl_io_t io;

    (void)memset(&g_mock, 0, sizeof(g_mock));
    test_check(planner_fixture_build(&g_mock.map), __LINE__);
    g_mock.ready = true;
    g_mock.delay = 2u;

    io.p_context = &g_mock;
    io.map = test_mock_map;
    io.ready = test_mock_ready;
    io.begin_hop = test_mock_begin;
    io.take_result = test_mock_take;
    goto_ctrl_init(&g_ctrl, &io);
    g_now_ms = TEST_START_MS;
}

/** Run the main loop until the route ends or the loops run out. */
static void test_run(uint32_t loops)
{
    uint32_t count = 0u;

    for (count = 0u; (count < loops)
                     && (GOTO_CTRL_RUNNING == g_ctrl.state); count++)
    {
        g_now_ms += TEST_TICK_MS;
        test_mock_step(&g_mock);
        goto_ctrl_tick(&g_ctrl, g_now_ms);
    }
}

/** Does the controller status text contain the given text? */
static bool test_status_has(const char * p_text)
{
    char text[120];

    (void)goto_ctrl_status(&g_ctrl, text, sizeof(text));

    return NULL != strstr(text, p_text);
}

/* ---- Scenarios ---------------------------------------------------------- */

/** 1. Normal route 7 to 36: three hops, ends REACHED at 36. */
static void test_normal_route(void)
{
    char text[120];

    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_check(GOTO_CTRL_RUNNING == g_ctrl.state, __LINE__);

    /* A second GOTO while running is refused. */
    test_check(!goto_ctrl_start(&g_ctrl, 9u, g_now_ms), __LINE__);
    test_run(100u);
    test_check((GOTO_CTRL_IDLE == g_ctrl.state)
               && (GOTO_CTRL_RES_REACHED == g_ctrl.last_result), __LINE__);
    test_check((36u == g_mock.map.current) && (3u == g_mock.begins)
               && (3u == g_ctrl.hops_done), __LINE__);
    test_check(test_status_has("last=REACHED"), __LINE__);

    /* The event is reported once per change. */
    test_check(goto_ctrl_take_event(&g_ctrl, text, sizeof(text)), __LINE__);
    test_check(!goto_ctrl_take_event(&g_ctrl, text, sizeof(text)),
               __LINE__);
}

/** 2. Already there, unknown, no route: nothing moves. */
static void test_no_motion_cases(void)
{
    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 7u, g_now_ms), __LINE__);
    test_check((GOTO_CTRL_RES_REACHED == g_ctrl.last_result)
               && (0u == g_mock.begins), __LINE__);

    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 99u, g_now_ms), __LINE__);
    test_check((GOTO_CTRL_RES_UNKNOWN_TARGET == g_ctrl.last_result)
               && (0u == g_mock.begins), __LINE__);

    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 255u, g_now_ms), __LINE__);
    test_check((GOTO_CTRL_RES_NO_ROUTE == g_ctrl.last_result)
               && (0u == g_mock.begins), __LINE__);

    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 40u, g_now_ms), __LINE__);
    test_check((GOTO_CTRL_RES_NO_ROUTE == g_ctrl.last_result)
               && (0u == g_mock.begins), __LINE__);
}

/** 3, 4, 5, 11. Blocked edge, failed hops, queued failure, bad arrival. */
static void test_hop_failures(void)
{
    uint32_t begins = 0u;

    /* 3. A blocked edge is routed around (wall added before the request). */
    test_setup();
    test_check(rover_map_exit(&g_mock.map, 7u, MAP_NORTH, MAP_BLOCKED),
               __LINE__);
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(100u);
    test_check((GOTO_CTRL_RES_REACHED == g_ctrl.last_result)
               && (36u == g_mock.map.current), __LINE__);

    /* 4. A hop fails midway: route stops, reason reported, no replan. */
    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(3u);
    g_mock.fail_next = true;
    test_run(100u);
    test_check((GOTO_CTRL_IDLE == g_ctrl.state)
               && (GOTO_CTRL_RES_HOP_FAILED == g_ctrl.last_result),
               __LINE__);
    test_check(test_status_has("OBSTACLE_DURING_SEGMENT"), __LINE__);
    begins = g_mock.begins;
    test_run(50u);
    test_check(begins == g_mock.begins, __LINE__);

    /* 5. A refused start with a queued failure is read, not waited on. */
    test_setup();
    g_mock.fail_at_start = true;
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_check((GOTO_CTRL_IDLE == g_ctrl.state)
               && (GOTO_CTRL_RES_HOP_FAILED == g_ctrl.last_result)
               && test_status_has("MOTOR_TURN_REFUSED"), __LINE__);

    /* 11. ARRIVED but the current node is not the hop target: stop. */
    test_setup();
    g_mock.teleport = 7u;
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(100u);
    test_check((GOTO_CTRL_RES_HOP_FAILED == g_ctrl.last_result)
               && test_status_has("UNEXPECTED_NODE"), __LINE__);
}

/** 6. Admission refused (stale camera) for a while, or for good. */
static void test_admission(void)
{
    test_setup();
    g_mock.refuse_count = 20;
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(300u);
    test_check(GOTO_CTRL_RES_REACHED == g_ctrl.last_result, __LINE__);

    /* A permanent refusal gives up with NOT_READY. */
    test_setup();
    g_mock.ready = false;
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(1000u);
    test_check((GOTO_CTRL_IDLE == g_ctrl.state)
               && (GOTO_CTRL_RES_NOT_READY == g_ctrl.last_result)
               && (0u == g_mock.begins), __LINE__);

    test_setup();
    g_mock.refuse_count = TEST_REFUSE_FOREVER;
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(1000u);
    test_check(GOTO_CTRL_RES_NOT_READY == g_ctrl.last_result, __LINE__);
}

/** 7, 8. Cancel mid-hop, and a stale result with the wrong sequence. */
static void test_cancel_and_stale(void)
{
    uint32_t begins = 0u;
    uint32_t count = 0u;

    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(1u);
    goto_ctrl_cancel(&g_ctrl);
    test_check((GOTO_CTRL_IDLE == g_ctrl.state)
               && (GOTO_CTRL_RES_CANCELLED == g_ctrl.last_result),
               __LINE__);
    begins = g_mock.begins;

    for (count = 0u; count < 20u; count++)
    {
        g_now_ms += TEST_TICK_MS;
        test_mock_step(&g_mock);
        goto_ctrl_tick(&g_ctrl, g_now_ms);
    }

    test_check((begins == g_mock.begins) && (!g_mock.pending)
               && (GOTO_CTRL_RES_CANCELLED == g_ctrl.last_result),
               __LINE__);

    /* A stale failure with another sequence number is ignored. */
    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_queue_result(&g_mock, 999u, 3u, false, "OLD");
    g_now_ms += TEST_TICK_MS;
    goto_ctrl_tick(&g_ctrl, g_now_ms);
    test_check(GOTO_CTRL_RUNNING == g_ctrl.state, __LINE__);
    test_run(100u);
    test_check(GOTO_CTRL_RES_REACHED == g_ctrl.last_result, __LINE__);
}

/** 9, 10. Map changes in flight, and a second route after the first. */
static void test_replan_and_repeat(void)
{
    /* The map changes while a hop is in flight: the next hop re-plans. */
    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(3u);
    test_check((3u == g_mock.map.current) && g_ctrl.hop_in_flight,
               __LINE__);

    /* The 9 to 36 edge closes before hop 3 starts. */
    test_check(rover_map_exit(&g_mock.map, 9u, MAP_EAST, MAP_BLOCKED),
               __LINE__);
    test_run(300u);
    test_check((GOTO_CTRL_RES_REACHED == g_ctrl.last_result)
               && (36u == g_mock.map.current) && (g_mock.begins > 3u),
               __LINE__);

    /* A second route works after a finished one. */
    test_setup();
    test_check(goto_ctrl_start(&g_ctrl, 36u, g_now_ms), __LINE__);
    test_run(100u);
    test_check(36u == g_mock.map.current, __LINE__);
    test_check(goto_ctrl_start(&g_ctrl, 7u, g_now_ms), __LINE__);
    test_run(100u);
    test_check((GOTO_CTRL_RES_REACHED == g_ctrl.last_result)
               && (7u == g_mock.map.current), __LINE__);
}

/* end of file test_goto_ctrl.c */
