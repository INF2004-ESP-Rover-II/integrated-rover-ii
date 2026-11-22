#include "goto_ctrl.h"
#include <stdio.h>
#include <string.h>

const char *goto_result_name(goto_result_t r)
{
    switch (r) {
    case GOTO_RES_NONE: return "NONE";
    case GOTO_RES_REACHED: return "REACHED";
    case GOTO_RES_NO_ROUTE: return "NO_ROUTE";
    case GOTO_RES_UNKNOWN_TARGET: return "UNKNOWN_TARGET";
    case GOTO_RES_NOT_READY: return "NOT_READY";
    case GOTO_RES_HOP_FAILED: return "HOP_FAILED";
    case GOTO_RES_CANCELLED: return "CANCELLED";
    }
    return "?";
}

void goto_init(goto_ctrl_t *g, const goto_io_t *io)
{
    memset(g, 0, sizeof(*g));
    g->io = *io;
    g->heading = ROUTE_HEADING_UNKNOWN;
}

static void finish(goto_ctrl_t *g, goto_result_t r)
{
    g->state = GOTO_IDLE;
    g->hop_in_flight = false;
    g->last_result = r;
    g->changed = true;
}

static void fail_hop(goto_ctrl_t *g, const char *why)
{
    snprintf(g->fail_reason, sizeof(g->fail_reason), "%s", why ? why : "?");
    finish(g, GOTO_RES_HOP_FAILED);
}

/* Plan from the authoritative current node and try to start the next hop. */
static void advance(goto_ctrl_t *g, uint32_t now)
{
    const rover_map_t *m = g->io.map(g->io.ctx);
    route_t r;
    route_status_t st = route_plan(m, m->current, g->target, g->heading, &r);
    if (st == ROUTE_ALREADY_THERE) { finish(g, GOTO_RES_REACHED); return; }
    if (st == ROUTE_UNKNOWN_TARGET) { finish(g, GOTO_RES_UNKNOWN_TARGET); return; }
    if (st != ROUTE_OK) { finish(g, GOTO_RES_NO_ROUTE); return; }
    g->hops_total = (uint8_t)(g->hops_done + r.hops);

    if (!g->io.ready(g->io.ctx)) {
        if (now - g->wait_since_ms > GOTO_ADMIT_TIMEOUT_MS) finish(g, GOTO_RES_NOT_READY);
        return;                                  /* try again next tick */
    }
    uint32_t seq = ++g->seq;
    bool started = g->io.begin_hop(g->io.ctx, r.ids[1], r.headings[0], seq, now);
    goto_hop_result_t res;
    bool have = g->io.take_result(g->io.ctx, &res);   /* a refused start can still have queued a failure */
    if (have && res.sequence == seq) {
        if (!res.arrived) { fail_hop(g, res.reason); return; }
    } else if (!started) {
        if (now - g->wait_since_ms > GOTO_ADMIT_TIMEOUT_MS) finish(g, GOTO_RES_NOT_READY);
        return;                                  /* refused before moving: retry */
    }
    g->hop_in_flight = true;
    g->hop_seq = seq;
    g->hop_target = r.ids[1];
    g->heading = r.headings[0];
    g->changed = true;
    if (have && res.sequence == seq && res.arrived) {   /* instant completion */
        g->hop_in_flight = false;
        ++g->hops_done;
        g->wait_since_ms = now;
    }
}

bool goto_start(goto_ctrl_t *g, uint8_t target_id, uint32_t now)
{
    if (g->state == GOTO_RUNNING) return false;       /* one route at a time */
    g->target = target_id;
    g->state = GOTO_RUNNING;
    g->hop_in_flight = false;
    g->hops_done = g->hops_total = 0;
    g->fail_reason[0] = '\0';
    g->heading = ROUTE_HEADING_UNKNOWN;
    g->started_ms = g->wait_since_ms = now;
    g->last_result = GOTO_RES_NONE;
    g->changed = true;
    advance(g, now);
    return true;
}

void goto_cancel(goto_ctrl_t *g)
{
    if (g->state == GOTO_RUNNING) finish(g, GOTO_RES_CANCELLED);
}

void goto_tick(goto_ctrl_t *g, uint32_t now)
{
    goto_hop_result_t res;
    if (g->state != GOTO_RUNNING) {
        while (g->io.take_result(g->io.ctx, &res)) { }    /* drop late results of a cancelled route */
        return;
    }
    if (g->hop_in_flight) {
        if (!g->io.take_result(g->io.ctx, &res)) return;
        if (res.sequence != g->hop_seq) return;           /* stale or duplicate: ignore */
        g->hop_in_flight = false;
        if (!res.arrived) { fail_hop(g, res.reason); return; }
        const rover_map_t *m = g->io.map(g->io.ctx);
        if (m->current != g->hop_target) { fail_hop(g, "ARRIVED_AT_UNEXPECTED_NODE"); return; }
        ++g->hops_done;
        g->wait_since_ms = now;
        g->changed = true;
    }
    if (!g->hop_in_flight) advance(g, now);
}

size_t goto_status(const goto_ctrl_t *g, char *buf, size_t cap)
{
    int n;
    if (g->state == GOTO_RUNNING)
        n = snprintf(buf, cap, "GOTO RUNNING target=%u hop=%u/%u next=%u last=%s",
                     (unsigned)g->target, (unsigned)g->hops_done, (unsigned)g->hops_total,
                     (unsigned)(g->hop_in_flight ? g->hop_target : 0u), goto_result_name(g->last_result));
    else if (g->last_result == GOTO_RES_HOP_FAILED)
        n = snprintf(buf, cap, "GOTO IDLE last=HOP_FAILED target=%u reason=%s",
                     (unsigned)g->target, g->fail_reason);
    else
        n = snprintf(buf, cap, "GOTO IDLE last=%s target=%u", goto_result_name(g->last_result), (unsigned)g->target);
    return n < 0 ? 0u : (size_t)n;
}

bool goto_take_event(goto_ctrl_t *g, char *buf, size_t cap)
{
    if (!g->changed) return false;
    g->changed = false;
    goto_status(g, buf, cap);
    return true;
}
