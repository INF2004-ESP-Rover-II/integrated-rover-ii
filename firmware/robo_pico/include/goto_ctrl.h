/**
 * @file goto_ctrl.h
 * @brief Subsystem 4: "go to marker X" on the Robo Pico.
 *
 * Plans with route_planner_plan() over the rover map and sends ONE verified
 * adjacent hop at a time to the onboard executor (onboard_dfs_begin_hop),
 * then waits for its terminal result. It never touches the motors and never
 * edits the map.
 *
 * After every arrival it re-plans from the authoritative current node, so a
 * change in the map is picked up. A failed hop STOPS the route: the executor
 * is in FAULT and the rover may be between nodes, so nothing is re-planned
 * from a guessed position. Recover (DFS_RESET, DFS_RESUME), then send GOTO
 * again.
 */

#ifndef GOTO_CTRL_H
#define GOTO_CTRL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "rover_map.h"
#include "route_planner.h"

/** Size of the failure reason text, including the terminating zero. */
#define GOTO_CTRL_REASON_SIZE (40u)

/** How long a hop may be refused (stale camera or sensor) before giving up. */
#define GOTO_CTRL_ADMIT_TIMEOUT_MS (3000u)

/** Terminal result of one hop, as reported by the executor. */
typedef struct
{
    uint32_t sequence;                    /**< Sequence number of the hop. */
    uint8_t target;                       /**< Marker the hop was heading to. */
    bool arrived;                         /**< True if arrival was confirmed. */
    char reason[GOTO_CTRL_REASON_SIZE];   /**< Short reason text. */
} goto_ctrl_hop_result_t;

/** Return the map the executor is using. */
typedef const rover_map_t * (* goto_ctrl_map_fn_t)(void * p_context);

/** True when the executor may take a hop right now. */
typedef bool (* goto_ctrl_ready_fn_t)(void * p_context);

/** Start one hop. False if refused (a refusal queues no result). */
typedef bool (* goto_ctrl_begin_fn_t)(void * p_context,
                                      uint8_t target,
                                      uint32_t direction,
                                      uint32_t sequence,
                                      uint32_t now_ms);

/** Fetch the terminal result of a hop, once. False if none is waiting. */
typedef bool (* goto_ctrl_take_fn_t)(void * p_context,
                                     goto_ctrl_hop_result_t * p_result);

/** Connections to the executor. All four functions are required. */
typedef struct
{
    void * p_context;
    goto_ctrl_map_fn_t map;
    goto_ctrl_ready_fn_t ready;
    goto_ctrl_begin_fn_t begin_hop;
    goto_ctrl_take_fn_t take_result;
} goto_ctrl_io_t;

/** Whether a route is being driven. */
typedef enum
{
    GOTO_CTRL_IDLE = 0,
    GOTO_CTRL_RUNNING = 1
} goto_ctrl_state_t;

/** How the last route ended. */
typedef enum
{
    GOTO_CTRL_RES_NONE = 0,
    GOTO_CTRL_RES_REACHED,
    GOTO_CTRL_RES_NO_ROUTE,
    GOTO_CTRL_RES_UNKNOWN_TARGET,
    GOTO_CTRL_RES_NOT_READY,
    GOTO_CTRL_RES_HOP_FAILED,
    GOTO_CTRL_RES_CANCELLED
} goto_ctrl_result_t;

/** Controller state. Treat the fields as read-only outside goto_ctrl.c. */
typedef struct
{
    goto_ctrl_io_t io;
    goto_ctrl_state_t state;
    goto_ctrl_result_t last_result;
    uint8_t target;       /**< Marker the route is heading to. */
    uint8_t hop_target;   /**< Marker the hop in flight is heading to. */
    bool hop_in_flight;
    int32_t heading;      /**< Direction of the last hop, or unknown. */
    uint32_t seq;         /**< Last sequence number used. */
    uint32_t hop_seq;     /**< Sequence number of the hop in flight. */
    uint32_t started_ms;
    uint32_t wait_since_ms;
    uint8_t hops_done;
    uint8_t hops_total;
    char fail_reason[GOTO_CTRL_REASON_SIZE];
    bool changed;         /**< Set on every change, see take_event. */
} goto_ctrl_t;

/**
 * @brief Prepare a controller. Call once at start-up.
 * @param p_ctrl Controller to initialise.
 * @param p_io   Executor connections, copied.
 */
void goto_ctrl_init(goto_ctrl_t * p_ctrl, const goto_ctrl_io_t * p_io);

/**
 * @brief Begin a route to a marker.
 * @param p_ctrl    Controller.
 * @param target_id Marker to reach.
 * @param now_ms    Current time in milliseconds.
 * @return True if started or finished at once (already there). False if a
 *         route is already running.
 */
bool goto_ctrl_start(goto_ctrl_t * p_ctrl,
                     uint8_t target_id,
                     uint32_t now_ms);

/**
 * @brief Cancel a running route. Late hop results are then dropped.
 * @param p_ctrl Controller.
 */
void goto_ctrl_cancel(goto_ctrl_t * p_ctrl);

/**
 * @brief Run the controller. Call from the main loop after the executor's
 *        own tick.
 * @param p_ctrl Controller.
 * @param now_ms Current time in milliseconds.
 */
void goto_ctrl_tick(goto_ctrl_t * p_ctrl, uint32_t now_ms);

/**
 * @brief Write a one-line status, for example
 *        "GOTO RUNNING target=36 hop=1/3 next=9 last=NONE".
 * @param p_ctrl Controller.
 * @param p_buf  Output buffer.
 * @param cap    Size of the buffer in bytes.
 * @return Number of characters written (not counting the zero).
 */
size_t goto_ctrl_status(const goto_ctrl_t * p_ctrl, char * p_buf, size_t cap);

/**
 * @brief Report a change once, for publishing on MQTT.
 * @param p_ctrl Controller.
 * @param p_buf  Output buffer, filled with the status text.
 * @param cap    Size of the buffer in bytes.
 * @return True once after every change, otherwise false.
 */
bool goto_ctrl_take_event(goto_ctrl_t * p_ctrl, char * p_buf, size_t cap);

/**
 * @brief Text name of a result.
 * @param result Result to name.
 * @return Constant string, never NULL.
 */
const char * goto_ctrl_result_name(goto_ctrl_result_t result);

#endif /* GOTO_CTRL_H */

/* end of file goto_ctrl.h */
