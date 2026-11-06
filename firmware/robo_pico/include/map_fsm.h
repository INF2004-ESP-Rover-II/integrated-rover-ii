/* map_fsm.h - mapping + path-planning state machine (Subsystem 4), runs on the RoboPico.
 * Port of rover_mapping/mapping/controller.py. Plain C11, no malloc, no Pico SDK includes.
 *
 * Phase 1 (MAPPING): the navigator reports every marker the rover reaches and every exit it sees;
 *                    the FSM builds the marker graph and moves to MAP_COMPLETE when n markers are found.
 * Phase 2 (routing): mapfsm_goto(target) plans a route with A*; the navigator polls mapfsm_current_hop(),
 *                    drives that one hop, then reports mapfsm_hop_done() or mapfsm_hop_blocked().
 *
 * Everything is called from ONE context (the main loop / one task); there is no locking. */
#ifndef MAP_FSM_H
#define MAP_FSM_H

#include <stdbool.h>
#include <stdint.h>

#include "rover_map.h"
#include "route_planner.h"

typedef enum {
    MS_IDLE = 0,        /* no mission, empty map */
    MS_MAPPING,         /* phase 1: exploring */
    MS_PAUSED,          /* phase 1 stopped by the operator or a fault (map kept) */
    MS_MAP_COMPLETE,    /* map finished, ready to route */
    MS_MAP_EXHAUSTED,   /* exploration ended with fewer markers than requested */
    MS_PLANNING,        /* transient: running A* */
    MS_WAITING_HOP,     /* a hop is outstanding; waiting for hop_done / hop_blocked */
    MS_REPLANNING,      /* transient: re-running A* after a blocked hop */
    MS_FAULT_WAIT       /* fault during routing; reset_fault returns to MAP_COMPLETE */
} map_state_t;

typedef enum { RESULT_NONE = 0, RESULT_REACHED, RESULT_NO_ROUTE } route_result_t;

typedef struct {
    uint8_t seq;        /* identifies this hop; hop_done / hop_blocked must quote it */
    uint8_t heading;    /* absolute grid direction to drive, 0=N 1=E 2=S 3=W */
    uint8_t target_id;  /* marker the hop ends at */
    bool    last;       /* final hop of the route */
} map_hop_t;

typedef struct {
    rover_map_t graph;
    map_state_t state;
    uint8_t  n;                       /* markers wanted (0 = not set) */
    uint8_t  heading;                 /* heading of the last hop / arrival */
    bool     position_confirmed;      /* camera has confirmed the current marker */
    int16_t  target_id;               /* -1 when no route is active */
    uint8_t  path[PLAN_MAX_PATH];
    uint8_t  path_len;
    uint8_t  hop;                     /* index of the outstanding hop in path */
    bool     hop_valid;
    map_hop_t cur_hop;
    uint8_t  seg_seq;
    route_result_t last_result;
    int16_t  last_result_target;
    uint16_t version;                 /* bumps on every change; publish a snapshot when it moves */
} map_fsm_t;

void mapfsm_init(map_fsm_t *f);

/* ---- operator / mission inputs ---- */
bool mapfsm_start(map_fsm_t *f, uint8_t n, uint8_t start_marker_id);     /* IDLE -> MAPPING */
bool mapfsm_resume_start(map_fsm_t *f, uint8_t n, uint8_t marker_id);    /* PAUSED/FAULT_WAIT -> MAPPING, keeps the map */
void mapfsm_stop(map_fsm_t *f);                                          /* MAPPING -> PAUSED; routing -> MAP_COMPLETE */
void mapfsm_fault(map_fsm_t *f);                                         /* MAPPING -> PAUSED; routing -> FAULT_WAIT */
bool mapfsm_resume(map_fsm_t *f);                                        /* PAUSED -> MAPPING (needs a confirmed marker) */
bool mapfsm_reset_fault(map_fsm_t *f);                                   /* FAULT_WAIT -> MAP_COMPLETE; MAP_EXHAUSTED -> IDLE */
bool mapfsm_reset_map(map_fsm_t *f);                                     /* PAUSED / MAP_COMPLETE -> IDLE, map cleared */
bool mapfsm_finish_early(map_fsm_t *f);                                  /* MAPPING/PAUSED with >= 2 markers -> MAP_COMPLETE */
bool mapfsm_confirm_marker(map_fsm_t *f, uint8_t marker_id);             /* camera sees marker_id: it is the current one */

/* ---- phase 1 events from the navigator ---- */
map_reached_t mapfsm_on_marker_reached(map_fsm_t *f, uint8_t marker_id, uint8_t heading);
bool mapfsm_on_exit_report(map_fsm_t *f, uint8_t marker_id, const uint8_t exits[4]);
bool mapfsm_on_exit_explored(map_fsm_t *f, uint8_t marker_id, uint8_t dir);
bool mapfsm_on_exit_blocked(map_fsm_t *f, uint8_t marker_id, uint8_t dir);
bool mapfsm_on_explore_done(map_fsm_t *f);                               /* nothing left to explore */

/* ---- phase 2 ---- */
bool mapfsm_goto(map_fsm_t *f, uint8_t target_id);                       /* MAP_COMPLETE -> plan, first hop */
bool mapfsm_current_hop(const map_fsm_t *f, map_hop_t *out);             /* the outstanding hop, if WAITING_HOP */
bool mapfsm_hop_done(map_fsm_t *f, uint8_t seq);                         /* hop finished at its target marker */
bool mapfsm_hop_blocked(map_fsm_t *f, uint8_t seq, uint8_t marker_id, uint8_t dir);  /* blocked at marker_id in dir; re-plans */

const char *mapfsm_state_name(map_state_t s);

#endif
