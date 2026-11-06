/* map_cmd.c - see map_cmd.h. */
#include "map_cmd.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static map_fsm_t g_fsm;
static uint16_t  g_last_published_version = 0xFFFF;
static uint32_t  g_last_snapshot_ms = 0;
static bool      g_dump_requested = false;
static uint32_t (*g_clock_us)(void) = NULL;
static map_fsm_t g_test_fsm;                         /* scratch FSM for the self test (no heap on the Pico) */

void map_cmd_set_clock_us(uint32_t (*clock_us)(void)) { g_clock_us = clock_us; }

map_fsm_t *map_cmd_fsm(void) { return &g_fsm; }

void map_cmd_init(void)
{
    mapfsm_init(&g_fsm);
    g_last_published_version = 0xFFFF;
}

/* ---- tiny bounded writer ---- */
typedef struct { char *p; size_t cap, len; bool over; } jw_t;

static void jw(jw_t *w, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void jw(jw_t *w, const char *fmt, ...)
{
    if (w->over) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(w->p + w->len, w->cap - w->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= w->cap - w->len) { w->over = true; return; }
    w->len += (size_t)n;
}

static const char *result_name(route_result_t r)
{
    return r == RESULT_REACHED ? "\"REACHED\"" : r == RESULT_NO_ROUTE ? "\"NO_ROUTE\"" : "null";
}

size_t map_snapshot_json(const map_fsm_t *f, char *out, size_t out_size)
{
    jw_t w = { out, out_size, 0, false };
    const rover_map_t *g = &f->graph;
    int cur = g->current == MAP_NONE ? -1 : g->markers[g->current].id;
    jw(&w, "{\"state\":\"%s\",\"n\":%d,\"count\":%u,\"start\":%d,\"current\":%d,\"confirmed\":%s,\"target\":%d,",
       mapfsm_state_name(f->state), f->n ? f->n : -1, g->count, g->start_id, cur,
       f->position_confirmed ? "true" : "false", f->target_id);
    jw(&w, "\"route\":[");
    for (uint8_t i = 0; i < f->path_len; i++) jw(&w, i ? ",%u" : "%u", f->path[i]);
    jw(&w, "],\"hop\":%u,\"markers\":[", f->hop);
    for (uint8_t i = 0; i < g->count; i++) {
        const map_marker_t *m = &g->markers[i];
        jw(&w, "%s{\"id\":%u,\"x\":%d,\"y\":%d,\"junction\":true,\"exits\":[%u,%u,%u,%u]}",
           i ? "," : "", m->id, m->x, m->y, m->exits[0], m->exits[1], m->exits[2], m->exits[3]);
    }
    jw(&w, "],\"edges\":[");
    bool first = true;
    for (uint8_t i = 0; i < g->count; i++)
        for (uint8_t d = 0; d < 4; d++) {
            uint8_t n = g->markers[i].nbr[d];
            if (n != MAP_NONE && g->markers[i].id < g->markers[n].id) {
                jw(&w, "%s[%u,%u,%s]", first ? "" : ",", g->markers[i].id, g->markers[n].id,
                   g->markers[i].blocked[d] ? "true" : "false");
                first = false;
            }
        }
    jw(&w, "],\"probable\":[");
    first = true;
    for (uint8_t i = 0; i < g->count; i++)
        for (uint8_t d = 0; d < 4; d++) {
            int p = map_is_probable(g, i, d);
            if (p >= 0) {
                uint8_t n = map_neighbour_at(g, i, d);
                if (g->markers[i].id < g->markers[n].id) {
                    jw(&w, "%s[%u,%u,%s]", first ? "" : ",", g->markers[i].id, g->markers[n].id, p ? "true" : "false");
                    first = false;
                }
            }
        }
    jw(&w, "],\"last_result\":%s,\"inconsistencies\":%u}", result_name(f->last_result), g->inconsistencies);
    return w.over ? 0 : w.len;
}

bool map_cmd_snapshot_due(uint32_t now_ms, uint32_t min_interval_ms, char *out, size_t out_size)
{
    bool changed = g_fsm.version != g_last_published_version;
    if (!g_dump_requested && !(changed && now_ms - g_last_snapshot_ms >= min_interval_ms)) return false;
    size_t n = map_snapshot_json(&g_fsm, out, out_size);
    if (n == 0) return false;                      /* did not fit: skip rather than send half a message */
    g_dump_requested = false;
    g_last_published_version = g_fsm.version;
    g_last_snapshot_ms = now_ms;
    return true;
}

void map_cmd_snapshot_retry(void)
{
    g_last_published_version = (uint16_t)(g_fsm.version + 1u);   /* differs from the current version: send again */
}

/* ---- command parsing ---- */
static bool parse_ints(const char *s, int *v, int max)
{
    int n = 0;
    while (*s && n < max) {
        char *end;
        long x = strtol(s, &end, 10);
        if (end == s) return false;
        v[n++] = (int)x;
        s = end;
        if (*s == ',') s++;
        else if (*s) return false;
    }
    return n == max && *s == 0;
}

static void hop_text(const map_fsm_t *f, char *out, size_t cap)
{
    map_hop_t h;
    if (mapfsm_current_hop(f, &h))
        snprintf(out, cap, " HOP seq=%u heading=%u target=%u last=%d", h.seq, h.heading, h.target_id, h.last);
    else if (f->state == MS_MAP_COMPLETE && f->last_result != RESULT_NONE)
        snprintf(out, cap, " RESULT=%s", f->last_result == RESULT_REACHED ? "REACHED" : "NO_ROUTE");
    else
        out[0] = 0;
}

static void say(const map_fsm_t *f, bool ok, const char *what, char *reply, size_t cap)
{
    char hop[64];
    hop_text(f, hop, sizeof hop);
    int cur = f->graph.current == MAP_NONE ? -1 : f->graph.markers[f->graph.current].id;
    snprintf(reply, cap, "MAP_%s %s state=%s count=%u cur=%d%s", ok ? "OK" : "REFUSED", what,
             mapfsm_state_name(f->state), map_count(&f->graph), cur, hop);
}

bool map_cmd_handle(const char *command, char *reply, size_t cap)
{
    if (strncmp(command, "MAP_", 4) != 0) return false;
    map_fsm_t *f = &g_fsm;
    const char *c = command + 4;
    const char *arg = strchr(command, ':');
    arg = arg ? arg + 1 : "";
    int v[5];
    bool ok = true;
    const char *what = c;

    if (!strncmp(c, "START:", 6)) {
        what = "START"; ok = parse_ints(arg, v, 2) && mapfsm_start(f, (uint8_t)v[0], (uint8_t)v[1]);
    } else if (!strncmp(c, "REACHED:", 8)) {
        what = "REACHED";
        ok = parse_ints(arg, v, 2) && v[0] >= 0 && v[1] >= 0 && v[1] <= 3;
        if (ok) {
            map_reached_t r = mapfsm_on_marker_reached(f, (uint8_t)v[0], (uint8_t)v[1]);
            ok = r >= 0;
            if (r == MAP_REACHED_CONFLICT) what = "REACHED(position conflict, map unchanged)";
        }
    } else if (!strncmp(c, "EXITS:", 6)) {
        what = "EXITS"; ok = parse_ints(arg, v, 5);
        if (ok) { uint8_t e[4] = { (uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3], (uint8_t)v[4] };
                  for (int i = 0; i < 4; i++) if (e[i] > 3) ok = false;
                  ok = ok && mapfsm_on_exit_report(f, (uint8_t)v[0], e); }
    } else if (!strncmp(c, "EXPLORED:", 9)) {
        what = "EXPLORED"; ok = parse_ints(arg, v, 2) && v[1] >= 0 && v[1] <= 3 && mapfsm_on_exit_explored(f, (uint8_t)v[0], (uint8_t)v[1]);
    } else if (!strncmp(c, "BLOCKED:", 8)) {
        what = "BLOCKED"; ok = parse_ints(arg, v, 2) && v[1] >= 0 && v[1] <= 3 && mapfsm_on_exit_blocked(f, (uint8_t)v[0], (uint8_t)v[1]);
    } else if (!strcmp(c, "DONE")) {
        ok = mapfsm_on_explore_done(f);
    } else if (!strcmp(c, "STOP")) { mapfsm_stop(f);
    } else if (!strcmp(c, "FAULT")) { mapfsm_fault(f);
    } else if (!strcmp(c, "RESUME")) { ok = mapfsm_resume(f);
    } else if (!strcmp(c, "RESET_FAULT")) { ok = mapfsm_reset_fault(f);
    } else if (!strcmp(c, "RESET")) { ok = mapfsm_reset_map(f);
    } else if (!strcmp(c, "FINISH")) { ok = mapfsm_finish_early(f);
    } else if (!strncmp(c, "CONFIRM:", 8)) {
        what = "CONFIRM"; ok = parse_ints(arg, v, 1) && mapfsm_confirm_marker(f, (uint8_t)v[0]);
    } else if (!strncmp(c, "RESUME_START:", 13)) {
        what = "RESUME_START"; ok = parse_ints(arg, v, 2) && mapfsm_resume_start(f, (uint8_t)v[0], (uint8_t)v[1]);
    } else if (!strncmp(c, "GOTO:", 5)) {
        what = "GOTO"; ok = parse_ints(arg, v, 1) && mapfsm_goto(f, (uint8_t)v[0]);
    } else if (!strcmp(c, "HOP_DONE")) {
        map_hop_t h; ok = mapfsm_current_hop(f, &h) && mapfsm_hop_done(f, h.seq);
    } else if (!strncmp(c, "HOP_BLOCKED:", 12)) {
        what = "HOP_BLOCKED"; map_hop_t h;
        ok = parse_ints(arg, v, 2) && v[1] >= 0 && v[1] <= 3 && mapfsm_current_hop(f, &h) &&
             mapfsm_hop_blocked(f, h.seq, (uint8_t)v[0], (uint8_t)v[1]);
    } else if (!strcmp(c, "STATUS")) {
        what = "STATUS";
    } else if (!strcmp(c, "DUMP")) {
        what = "DUMP"; g_dump_requested = true;
    } else if (!strcmp(c, "SELFTEST")) {
        (void)map_selftest(reply, cap, g_clock_us);
        return true;
    } else {
        snprintf(reply, cap, "MAP_REFUSED unknown command %s", command);
        return true;
    }
    say(f, ok, what, reply, cap);
    return true;
}

/* ---- built-in self test ---- */
static bool t_corridor(char *why, size_t cap)
{
    /* 12 - 18 - 14 - 15 - 9 - 2 going north (the real 6-marker run), route 2 -> 15 must be 2 hops south-ish */
    map_fsm_t *f = &g_test_fsm;
    bool ok = true;
    mapfsm_init(f);
    ok = ok && mapfsm_start(f, 6, 12);
    static const uint8_t ids[5] = { 18, 14, 15, 9, 2 };
    for (int i = 0; i < 5; i++) (void)mapfsm_on_marker_reached(f, ids[i], DIR_N);
    ok = ok && f->state == MS_MAP_COMPLETE && map_count(&f->graph) == 6;
    ok = ok && mapfsm_goto(f, 15);
    map_hop_t h;
    ok = ok && mapfsm_current_hop(f, &h) && h.heading == DIR_S && h.target_id == 9 && !h.last;
    ok = ok && mapfsm_hop_done(f, h.seq);
    ok = ok && mapfsm_current_hop(f, &h) && h.target_id == 15 && h.last;
    ok = ok && mapfsm_hop_done(f, h.seq) && f->state == MS_MAP_COMPLETE && f->last_result == RESULT_REACHED;
    if (!ok) snprintf(why, cap, "corridor route wrong");
    return ok;
}

static bool t_replan(char *why, size_t cap)
{
    /* square 1-2 / 4-3 ; ids: 1(0,0) 2(1,0) 3(1,-1) 4(0,-1). Block 1->2, route 1->3 must go via 4. */
    map_fsm_t *f = &g_test_fsm;
    bool ok = true;
    mapfsm_init(f);
    ok = ok && mapfsm_start(f, 4, 1);
    (void)mapfsm_on_marker_reached(f, 2, DIR_E);
    (void)mapfsm_on_marker_reached(f, 3, DIR_S);
    (void)mapfsm_on_marker_reached(f, 4, DIR_W);
    ok = ok && f->state == MS_MAP_COMPLETE;
    ok = ok && mapfsm_confirm_marker(f, 1) && mapfsm_goto(f, 3);
    map_hop_t h;
    ok = ok && mapfsm_current_hop(f, &h) && h.target_id == 2;            /* via 2 (3 first by ID order) */
    ok = ok && mapfsm_hop_blocked(f, h.seq, 1, DIR_E);                   /* wall between 1 and 2 */
    ok = ok && mapfsm_current_hop(f, &h) && h.target_id == 4 && h.heading == DIR_S;
    ok = ok && mapfsm_hop_done(f, h.seq) && mapfsm_current_hop(f, &h) && h.target_id == 3 && h.last;
    ok = ok && mapfsm_hop_done(f, h.seq) && f->last_result == RESULT_REACHED;
    if (!ok) snprintf(why, cap, "blocked-hop replan wrong");
    return ok;
}

static bool t_worst_case(char *why, size_t cap, uint32_t (*clock_us)(void), uint32_t *us)
{
    /* full 6x6 grid, snake-driven, then plan corner to corner using probable links */
    map_fsm_t *f = &g_test_fsm;
    bool ok = true;
    mapfsm_init(f);
    ok = ok && mapfsm_start(f, 36, 1);
    uint8_t id = 1;
    for (int row = 0; row < 6; row++) {
        for (int col = 1; col < 6; col++) { id++; (void)mapfsm_on_marker_reached(f, id, row % 2 == 0 ? DIR_E : DIR_W); }
        if (row < 5) { id++; (void)mapfsm_on_marker_reached(f, id, DIR_S); }
    }
    ok = ok && f->state == MS_MAP_COMPLETE && map_count(&f->graph) == 36;
    uint8_t path[PLAN_MAX_PATH];
    uint32_t t0 = clock_us ? clock_us() : 0;
    int n = plan_route(&f->graph, 1, 36, DIR_N, true, path, (int)PLAN_MAX_PATH);
    uint32_t t1 = clock_us ? clock_us() : 0;
    *us = t1 - t0;
    ok = ok && n == 6 && path[0] == 1 && path[5] == 36;   /* 5 hops down the never-driven column (probable links), not 35 driven hops */
    if (!ok) snprintf(why, cap, "36-marker plan wrong (len %d)", n);
    return ok;
}

bool map_selftest(char *reply, size_t cap, uint32_t (*clock_us)(void))
{
    char why[64] = "";
    uint32_t us = 0;
    bool a = t_corridor(why, sizeof why);
    bool b = a && t_replan(why, sizeof why);
    bool c = b && t_worst_case(why, sizeof why, clock_us, &us);
    if (c) snprintf(reply, cap, "MAP_SELFTEST PASS corridor, replan, 36-marker plan (%lu us)", (unsigned long)us);
    else   snprintf(reply, cap, "MAP_SELFTEST FAIL %s", why);
    return c;
}
