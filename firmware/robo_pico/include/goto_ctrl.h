#ifndef GOTO_CTRL_H
#define GOTO_CTRL_H
/* Subsystem 4: "go to marker X" on the Robo Pico. Plans with route_plan() over the rover map and
 * sends ONE verified adjacent hop at a time to the onboard executor (onboard_dfs_begin_hop), then waits
 * for its terminal result. It never touches motors and never edits the map.
 *
 * After every arrival it re-plans from the authoritative current node, so a topology change is picked up.
 * A failed hop STOPS the route (the executor is in FAULT, the rover may be between nodes): nothing is
 * replanned from a guessed position. Recover (DFS_RESET, DFS_RESUME), then send GOTO again. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "rover_map.h"
#include "route_planner.h"

typedef struct {
    uint32_t sequence;
    uint8_t target;
    bool arrived;
    char reason[40];
} goto_hop_result_t;

typedef struct {
    void *ctx;
    const rover_map_t *(*map)(void *ctx);
    /* true when the executor may take a hop right now (phase complete, position confirmed) */
    bool (*ready)(void *ctx);
    bool (*begin_hop)(void *ctx, uint8_t target, unsigned direction, uint32_t sequence, uint32_t now_ms);
    bool (*take_result)(void *ctx, goto_hop_result_t *out);
} goto_io_t;

typedef enum { GOTO_IDLE = 0, GOTO_RUNNING = 1 } goto_state_t;
typedef enum {
    GOTO_RES_NONE = 0, GOTO_RES_REACHED, GOTO_RES_NO_ROUTE, GOTO_RES_UNKNOWN_TARGET,
    GOTO_RES_NOT_READY, GOTO_RES_HOP_FAILED, GOTO_RES_CANCELLED
} goto_result_t;

#define GOTO_ADMIT_TIMEOUT_MS 3000u   /* how long a hop may be refused (stale camera/sensor) before giving up */

typedef struct {
    goto_io_t io;
    goto_state_t state;
    goto_result_t last_result;
    uint8_t target;
    uint8_t hop_target;           /* node the hop in flight is heading to */
    bool hop_in_flight;
    int heading;                  /* direction of the last hop, or ROUTE_HEADING_UNKNOWN */
    uint32_t seq, hop_seq, started_ms, wait_since_ms;
    uint8_t hops_done, hops_total;
    char fail_reason[40];
    bool changed;                 /* set whenever state/result/progress changed; see goto_take_event */
} goto_ctrl_t;

void goto_init(goto_ctrl_t *g, const goto_io_t *io);
/* Begin a route to target_id. Returns true if started or finished immediately (already there). */
bool goto_start(goto_ctrl_t *g, uint8_t target_id, uint32_t now_ms);
void goto_cancel(goto_ctrl_t *g);
void goto_tick(goto_ctrl_t *g, uint32_t now_ms);
/* One-line status, e.g. "GOTO RUNNING target=36 hop=1/3 next=9 last=NONE". */
size_t goto_status(const goto_ctrl_t *g, char *buf, size_t cap);
/* true once after every change; fills buf with goto_status text (for mqtt_publish_status) */
bool goto_take_event(goto_ctrl_t *g, char *buf, size_t cap);
const char *goto_result_name(goto_result_t r);
#endif
