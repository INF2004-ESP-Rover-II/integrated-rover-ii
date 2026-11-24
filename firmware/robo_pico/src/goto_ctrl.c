/**
 * @file goto_ctrl.c
 * @brief Subsystem 4: hop-by-hop "go to marker" controller.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "goto_ctrl.h"

/* ---- Private prototypes ------------------------------------------------- */

static void goto_ctrl_finish(goto_ctrl_t * p_ctrl,
                             goto_ctrl_result_t result);
static void goto_ctrl_fail_hop(goto_ctrl_t * p_ctrl, const char * p_reason);
static bool goto_ctrl_admit_timed_out(const goto_ctrl_t * p_ctrl,
                                      uint32_t now_ms);
static void goto_ctrl_finish_unrouted(goto_ctrl_t * p_ctrl,
                                      route_planner_status_t status);
static void goto_ctrl_submit_hop(goto_ctrl_t * p_ctrl,
                                 const route_planner_route_t * p_route,
                                 uint32_t now_ms);
static void goto_ctrl_advance(goto_ctrl_t * p_ctrl, uint32_t now_ms);
static void goto_ctrl_confirm_arrival(goto_ctrl_t * p_ctrl, uint32_t now_ms);
static void goto_ctrl_collect_result(goto_ctrl_t * p_ctrl, uint32_t now_ms);
static void goto_ctrl_drop_late_results(goto_ctrl_t * p_ctrl);

/* ---- Public functions --------------------------------------------------- */

const char * goto_ctrl_result_name(goto_ctrl_result_t result)
{
    const char * p_name = "?";

    switch (result)
    {
        case GOTO_CTRL_RES_NONE:
            p_name = "NONE";
            break;

        case GOTO_CTRL_RES_REACHED:
            p_name = "REACHED";
            break;

        case GOTO_CTRL_RES_NO_ROUTE:
            p_name = "NO_ROUTE";
            break;

        case GOTO_CTRL_RES_UNKNOWN_TARGET:
            p_name = "UNKNOWN_TARGET";
            break;

        case GOTO_CTRL_RES_NOT_READY:
            p_name = "NOT_READY";
            break;

        case GOTO_CTRL_RES_HOP_FAILED:
            p_name = "HOP_FAILED";
            break;

        case GOTO_CTRL_RES_CANCELLED:
            p_name = "CANCELLED";
            break;

        default:
            p_name = "?";
            break;
    }

    return p_name;
}

void goto_ctrl_init(goto_ctrl_t * p_ctrl, const goto_ctrl_io_t * p_io)
{
    (void)memset(p_ctrl, 0, sizeof(*p_ctrl));
    p_ctrl->io = *p_io;
    p_ctrl->heading = ROUTE_PLANNER_HEADING_UNKNOWN;
}

bool goto_ctrl_start(goto_ctrl_t * p_ctrl,
                     uint8_t target_id,
                     uint32_t now_ms)
{
    bool started = false;

    /* One route at a time. */
    if (GOTO_CTRL_RUNNING != p_ctrl->state)
    {
        p_ctrl->target = target_id;
        p_ctrl->state = GOTO_CTRL_RUNNING;
        p_ctrl->hop_in_flight = false;
        p_ctrl->hops_done = 0u;
        p_ctrl->hops_total = 0u;
        p_ctrl->fail_reason[0] = '\0';
        p_ctrl->heading = ROUTE_PLANNER_HEADING_UNKNOWN;
        p_ctrl->started_ms = now_ms;
        p_ctrl->wait_since_ms = now_ms;
        p_ctrl->last_result = GOTO_CTRL_RES_NONE;
        p_ctrl->changed = true;
        goto_ctrl_advance(p_ctrl, now_ms);
        started = true;
    }

    return started;
}

void goto_ctrl_cancel(goto_ctrl_t * p_ctrl)
{
    if (GOTO_CTRL_RUNNING == p_ctrl->state)
    {
        goto_ctrl_finish(p_ctrl, GOTO_CTRL_RES_CANCELLED);
    }
}

void goto_ctrl_tick(goto_ctrl_t * p_ctrl, uint32_t now_ms)
{
    if (GOTO_CTRL_RUNNING != p_ctrl->state)
    {
        goto_ctrl_drop_late_results(p_ctrl);
    }
    else
    {
        if (p_ctrl->hop_in_flight)
        {
            goto_ctrl_collect_result(p_ctrl, now_ms);
        }

        /* A failed hop has already moved the state to IDLE. */
        if ((GOTO_CTRL_RUNNING == p_ctrl->state)
            && (!p_ctrl->hop_in_flight))
        {
            goto_ctrl_advance(p_ctrl, now_ms);
        }
    }
}

size_t goto_ctrl_status(const goto_ctrl_t * p_ctrl, char * p_buf, size_t cap)
{
    int written = 0;
    uint32_t target = p_ctrl->target;
    uint32_t done = p_ctrl->hops_done;
    uint32_t total = p_ctrl->hops_total;
    uint32_t next = 0u;

    if (p_ctrl->hop_in_flight)
    {
        next = p_ctrl->hop_target;
    }

    if (GOTO_CTRL_RUNNING == p_ctrl->state)
    {
        written = snprintf(p_buf, cap,
                           "GOTO RUNNING target=%" PRIu32 " hop=%" PRIu32
                           "/%" PRIu32 " next=%" PRIu32 " last=%s",
                           target, done, total, next,
                           goto_ctrl_result_name(p_ctrl->last_result));
    }
    else if (GOTO_CTRL_RES_HOP_FAILED == p_ctrl->last_result)
    {
        written = snprintf(p_buf, cap,
                           "GOTO IDLE last=HOP_FAILED target=%" PRIu32
                           " reason=%s",
                           target, p_ctrl->fail_reason);
    }
    else
    {
        written = snprintf(p_buf, cap,
                           "GOTO IDLE last=%s target=%" PRIu32,
                           goto_ctrl_result_name(p_ctrl->last_result),
                           target);
    }

    if (written < 0)
    {
        written = 0;
    }

    /* written is not negative here, so the conversion keeps its value. */
    return (size_t)written;
}

bool goto_ctrl_take_event(goto_ctrl_t * p_ctrl, char * p_buf, size_t cap)
{
    bool event = false;

    if (p_ctrl->changed)
    {
        p_ctrl->changed = false;
        (void)goto_ctrl_status(p_ctrl, p_buf, cap);
        event = true;
    }

    return event;
}

/* ---- Private functions -------------------------------------------------- */

/** End the route with a result. */
static void goto_ctrl_finish(goto_ctrl_t * p_ctrl,
                             goto_ctrl_result_t result)
{
    p_ctrl->state = GOTO_CTRL_IDLE;
    p_ctrl->hop_in_flight = false;
    p_ctrl->last_result = result;
    p_ctrl->changed = true;
}

/** End the route because a hop failed, keeping the executor's reason. */
static void goto_ctrl_fail_hop(goto_ctrl_t * p_ctrl, const char * p_reason)
{
    const char * p_text = p_reason;

    if (NULL == p_text)
    {
        p_text = "?";
    }

    (void)snprintf(p_ctrl->fail_reason, sizeof(p_ctrl->fail_reason), "%s",
                   p_text);
    goto_ctrl_finish(p_ctrl, GOTO_CTRL_RES_HOP_FAILED);
}

/** True if the executor has refused hops for longer than the limit. */
static bool goto_ctrl_admit_timed_out(const goto_ctrl_t * p_ctrl,
                                      uint32_t now_ms)
{
    return (now_ms - p_ctrl->wait_since_ms) > GOTO_CTRL_ADMIT_TIMEOUT_MS;
}

/** End the route when the planner did not give a route. */
static void goto_ctrl_finish_unrouted(goto_ctrl_t * p_ctrl,
                                      route_planner_status_t status)
{
    switch (status)
    {
        case ROUTE_PLANNER_ALREADY_THERE:
            goto_ctrl_finish(p_ctrl, GOTO_CTRL_RES_REACHED);
            break;

        case ROUTE_PLANNER_UNKNOWN_TARGET:
            goto_ctrl_finish(p_ctrl, GOTO_CTRL_RES_UNKNOWN_TARGET);
            break;

        default:
            goto_ctrl_finish(p_ctrl, GOTO_CTRL_RES_NO_ROUTE);
            break;
    }
}

/**
 * Ask the executor for the first hop of the route. A refused start can still
 * have queued a failure result, so the result is read straight away.
 */
static void goto_ctrl_submit_hop(goto_ctrl_t * p_ctrl,
                                 const route_planner_route_t * p_route,
                                 uint32_t now_ms)
{
    goto_ctrl_hop_result_t result;
    uint32_t sequence = p_ctrl->seq + 1u;
    bool started = false;
    bool have_result = false;
    bool matches = false;

    p_ctrl->seq = sequence;
    started = p_ctrl->io.begin_hop(p_ctrl->io.p_context, p_route->ids[1],
                                   p_route->headings[0], sequence, now_ms);
    have_result = p_ctrl->io.take_result(p_ctrl->io.p_context, &result);
    matches = have_result && (result.sequence == sequence);

    if (matches && (!result.arrived))
    {
        goto_ctrl_fail_hop(p_ctrl, result.reason);
    }
    else if ((!matches) && (!started))
    {
        /* Refused before moving: try again next tick, until the limit. */
        if (goto_ctrl_admit_timed_out(p_ctrl, now_ms))
        {
            goto_ctrl_finish(p_ctrl, GOTO_CTRL_RES_NOT_READY);
        }
    }
    else
    {
        p_ctrl->hop_in_flight = true;
        p_ctrl->hop_seq = sequence;
        p_ctrl->hop_target = p_route->ids[1];
        p_ctrl->heading = p_route->headings[0];
        p_ctrl->changed = true;

        if (matches)
        {
            /* The executor finished the hop instantly. */
            p_ctrl->hop_in_flight = false;
            p_ctrl->hops_done++;
            p_ctrl->wait_since_ms = now_ms;
        }
    }
}

/** Plan from the authoritative current node and start the next hop. */
static void goto_ctrl_advance(goto_ctrl_t * p_ctrl, uint32_t now_ms)
{
    const rover_map_t * p_map = p_ctrl->io.map(p_ctrl->io.p_context);
    route_planner_route_t route;
    route_planner_status_t status;

    status = route_planner_plan(p_map, p_map->current, p_ctrl->target,
                                p_ctrl->heading, &route);

    if (ROUTE_PLANNER_OK != status)
    {
        goto_ctrl_finish_unrouted(p_ctrl, status);
    }
    else
    {
        /* Both counts are below ROVER_MAX_NODES, so the sum fits. */
        p_ctrl->hops_total = (uint8_t)(p_ctrl->hops_done + route.hops);

        if (p_ctrl->io.ready(p_ctrl->io.p_context))
        {
            goto_ctrl_submit_hop(p_ctrl, &route, now_ms);
        }
        else if (goto_ctrl_admit_timed_out(p_ctrl, now_ms))
        {
            goto_ctrl_finish(p_ctrl, GOTO_CTRL_RES_NOT_READY);
        }
        else
        {
            /* Not ready yet: try again next tick. */
        }
    }
}

/**
 * The executor said ARRIVED. Check the map agrees about where the rover is
 * before counting the hop, and stop the route if it does not.
 */
static void goto_ctrl_confirm_arrival(goto_ctrl_t * p_ctrl, uint32_t now_ms)
{
    const rover_map_t * p_map = p_ctrl->io.map(p_ctrl->io.p_context);

    if (p_map->current != p_ctrl->hop_target)
    {
        goto_ctrl_fail_hop(p_ctrl, "ARRIVED_AT_UNEXPECTED_NODE");
    }
    else
    {
        p_ctrl->hops_done++;
        p_ctrl->wait_since_ms = now_ms;
        p_ctrl->changed = true;
    }
}

/** Read the result of the hop in flight, ignoring stale or wrong ones. */
static void goto_ctrl_collect_result(goto_ctrl_t * p_ctrl, uint32_t now_ms)
{
    goto_ctrl_hop_result_t result;
    bool have_result = false;

    have_result = p_ctrl->io.take_result(p_ctrl->io.p_context, &result);

    if (have_result && (result.sequence == p_ctrl->hop_seq))
    {
        p_ctrl->hop_in_flight = false;

        if (result.arrived)
        {
            goto_ctrl_confirm_arrival(p_ctrl, now_ms);
        }
        else
        {
            goto_ctrl_fail_hop(p_ctrl, result.reason);
        }
    }
}

/** Throw away results that arrive after a route was cancelled or ended. */
static void goto_ctrl_drop_late_results(goto_ctrl_t * p_ctrl)
{
    goto_ctrl_hop_result_t result;

    while (p_ctrl->io.take_result(p_ctrl->io.p_context, &result))
    {
        /* The result belongs to a route that has ended. Discard it. */
    }
}

/* end of file goto_ctrl.c */
