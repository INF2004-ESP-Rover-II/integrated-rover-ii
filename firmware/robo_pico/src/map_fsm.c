/* map_fsm.c - see map_fsm.h. Port of rover_mapping/mapping/controller.py. */
#include "map_fsm.h"

#include <string.h>

static void bump(map_fsm_t *f) { f->version++; }

static void goto_state(map_fsm_t *f, map_state_t s)
{
    f->state = s;
    bump(f);
}

static void reset_route(map_fsm_t *f)
{
    f->target_id = -1;
    f->path_len = 0;
    f->hop = 0;
    f->hop_valid = false;
}

static void clear_map(map_fsm_t *f)
{
    map_init(&f->graph);
    f->n = 0;
    f->heading = 0;
    f->position_confirmed = false;
    reset_route(f);
    f->seg_seq = 0;
}

void mapfsm_init(map_fsm_t *f)
{
    memset(f, 0, sizeof *f);
    f->state = MS_IDLE;
    f->last_result = RESULT_NONE;
    f->last_result_target = -1;
    clear_map(f);
}

const char *mapfsm_state_name(map_state_t s)
{
    switch (s) {
    case MS_IDLE:          return "IDLE";
    case MS_MAPPING:       return "MAPPING";
    case MS_PAUSED:        return "PAUSED";
    case MS_MAP_COMPLETE:  return "MAP_COMPLETE";
    case MS_MAP_EXHAUSTED: return "MAP_EXHAUSTED";
    case MS_PLANNING:      return "PLANNING";
    case MS_WAITING_HOP:   return "WAITING_SEGMENT";   /* same name the dashboard already knows */
    case MS_REPLANNING:    return "REPLANNING";
    case MS_FAULT_WAIT:    return "FAULT_WAIT";
    }
    return "?";
}

static bool in_phase2(const map_fsm_t *f)
{
    return f->state == MS_PLANNING || f->state == MS_WAITING_HOP || f->state == MS_REPLANNING;
}

static void check_count(map_fsm_t *f)
{
    if (f->state == MS_MAPPING && f->n > 0 && map_count(&f->graph) == f->n)
        goto_state(f, MS_MAP_COMPLETE);
}

/* ---- operator / mission ---- */
bool mapfsm_start(map_fsm_t *f, uint8_t n, uint8_t start_marker_id)
{
    if (f->state != MS_IDLE || n < 1 || n > MAP_MAX_MARKERS) return false;
    if (!map_start(&f->graph, start_marker_id)) return false;
    f->n = n;
    f->position_confirmed = true;
    goto_state(f, MS_MAPPING);
    check_count(f);
    return true;
}

bool mapfsm_resume_start(map_fsm_t *f, uint8_t n, uint8_t marker_id)
{
    if ((f->state == MS_PAUSED || f->state == MS_FAULT_WAIT) && n >= 1 && n <= MAP_MAX_MARKERS &&
        map_index(&f->graph, marker_id) != MAP_NONE) {
        f->n = n;
        f->graph.current = map_index(&f->graph, marker_id);
        f->position_confirmed = true;
        reset_route(f);
        goto_state(f, MS_MAPPING);
        check_count(f);
        return true;
    }
    return false;
}

void mapfsm_stop(map_fsm_t *f)
{
    if (f->state == MS_MAPPING) {
        f->position_confirmed = false;
        goto_state(f, MS_PAUSED);
    } else if (in_phase2(f)) {
        reset_route(f);
        f->position_confirmed = false;
        goto_state(f, MS_MAP_COMPLETE);
    }
}

void mapfsm_fault(map_fsm_t *f)
{
    if (f->state == MS_MAPPING) {
        f->position_confirmed = false;
        goto_state(f, MS_PAUSED);
    } else if (in_phase2(f)) {
        reset_route(f);
        f->position_confirmed = false;
        goto_state(f, MS_FAULT_WAIT);
    }
}

bool mapfsm_resume(map_fsm_t *f)
{
    if (f->state == MS_PAUSED && f->position_confirmed) {
        goto_state(f, MS_MAPPING);
        return true;
    }
    return false;
}

bool mapfsm_reset_fault(map_fsm_t *f)
{
    if (f->state == MS_FAULT_WAIT) {
        goto_state(f, MS_MAP_COMPLETE);
        return true;
    }
    if (f->state == MS_MAP_EXHAUSTED) {
        clear_map(f);
        goto_state(f, MS_IDLE);
        return true;
    }
    return false;
}

bool mapfsm_reset_map(map_fsm_t *f)
{
    if (f->state == MS_PAUSED || f->state == MS_MAP_COMPLETE) {
        clear_map(f);
        goto_state(f, MS_IDLE);
        return true;
    }
    return false;
}

bool mapfsm_finish_early(map_fsm_t *f)
{
    if ((f->state == MS_MAPPING || f->state == MS_PAUSED) && map_count(&f->graph) >= 2) {
        f->n = map_count(&f->graph);
        goto_state(f, MS_MAP_COMPLETE);
        return true;
    }
    return false;
}

bool mapfsm_confirm_marker(map_fsm_t *f, uint8_t marker_id)
{
    uint8_t i = map_index(&f->graph, marker_id);
    if (i == MAP_NONE) return false;
    f->graph.current = i;
    f->position_confirmed = true;
    bump(f);
    return true;
}

/* ---- phase 1 events ---- */
map_reached_t mapfsm_on_marker_reached(map_fsm_t *f, uint8_t marker_id, uint8_t heading)
{
    if (f->state != MS_MAPPING) return MAP_REACHED_ERROR;
    map_reached_t r = map_marker_reached(&f->graph, marker_id, heading);
    f->heading = heading;
    f->position_confirmed = true;
    bump(f);
    check_count(f);
    return r;
}

bool mapfsm_on_exit_report(map_fsm_t *f, uint8_t marker_id, const uint8_t exits[4])
{
    if (f->state != MS_MAPPING) return false;
    map_apply_exit_report(&f->graph, marker_id, exits);
    bump(f);
    return true;
}

bool mapfsm_on_exit_explored(map_fsm_t *f, uint8_t marker_id, uint8_t dir)
{
    if (f->state != MS_MAPPING) return false;
    map_set_exit_status(&f->graph, marker_id, dir, EXIT_EXPLORED);
    bump(f);
    return true;
}

bool mapfsm_on_exit_blocked(map_fsm_t *f, uint8_t marker_id, uint8_t dir)
{
    if (f->state != MS_MAPPING) return false;
    map_mark_blocked(&f->graph, marker_id, dir);
    bump(f);
    return true;
}

bool mapfsm_on_explore_done(map_fsm_t *f)
{
    if (f->state != MS_MAPPING) return false;
    if (map_count(&f->graph) < f->n) goto_state(f, MS_MAP_EXHAUSTED);
    return true;
}

/* ---- phase 2 ---- */
static void finish(map_fsm_t *f, route_result_t r)
{
    f->last_result = r;
    f->last_result_target = f->target_id;
    reset_route(f);
    goto_state(f, MS_MAP_COMPLETE);
}

static void send_next(map_fsm_t *f)
{
    uint8_t headings[PLAN_MAX_PATH], targets[PLAN_MAX_PATH];
    int hops = plan_path_headings(&f->graph, f->path, f->path_len, headings, targets);
    if (hops < 1 || f->hop >= hops) { finish(f, RESULT_NO_ROUTE); return; }
    f->seg_seq++;
    f->cur_hop = (map_hop_t){ f->seg_seq, headings[f->hop], targets[f->hop], f->hop == hops - 1 };
    f->hop_valid = true;
    goto_state(f, MS_WAITING_HOP);
}

static void plan(map_fsm_t *f)
{
    uint8_t cur_id = f->graph.markers[f->graph.current].id;
    int n = plan_route(&f->graph, cur_id, (uint8_t)f->target_id, f->heading, true, f->path, (int)PLAN_MAX_PATH);
    if (n == 0) { finish(f, RESULT_NO_ROUTE); return; }
    f->path_len = (uint8_t)n;
    f->hop = 0;
    if (n == 1) { finish(f, RESULT_REACHED); return; }
    send_next(f);
}

bool mapfsm_goto(map_fsm_t *f, uint8_t target_id)
{
    if (f->state != MS_MAP_COMPLETE || !f->position_confirmed ||
        map_index(&f->graph, target_id) == MAP_NONE || f->graph.current == MAP_NONE)
        return false;
    f->target_id = target_id;
    goto_state(f, MS_PLANNING);
    plan(f);
    return true;
}

bool mapfsm_current_hop(const map_fsm_t *f, map_hop_t *out)
{
    if (f->state != MS_WAITING_HOP || !f->hop_valid) return false;
    *out = f->cur_hop;
    return true;
}

bool mapfsm_hop_done(map_fsm_t *f, uint8_t seq)
{
    if (f->state != MS_WAITING_HOP || !f->hop_valid || seq != f->cur_hop.seq) return false;   /* stale or repeated */
    uint8_t cur_id = f->graph.markers[f->graph.current].id;
    if (map_direction_between(&f->graph, cur_id, f->cur_hop.target_id) >= 0)
        map_link(&f->graph, cur_id, f->cur_hop.heading, f->cur_hop.target_id);   /* a probable edge was driven: now real */
    f->graph.current = map_index(&f->graph, f->cur_hop.target_id);
    f->heading = f->cur_hop.heading;
    f->position_confirmed = true;
    f->hop_valid = false;
    if (f->cur_hop.last) {
        finish(f, RESULT_REACHED);
    } else {
        f->hop++;
        send_next(f);
    }
    return true;
}

bool mapfsm_hop_blocked(map_fsm_t *f, uint8_t seq, uint8_t marker_id, uint8_t dir)
{
    if (f->state != MS_WAITING_HOP || !f->hop_valid || seq != f->cur_hop.seq) return false;
    map_mark_blocked(&f->graph, marker_id, dir);
    f->hop_valid = false;
    goto_state(f, MS_REPLANNING);
    plan(f);
    return true;
}
