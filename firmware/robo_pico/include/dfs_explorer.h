#ifndef DFS_EXPLORER_H
#define DFS_EXPLORER_H

#include <stdbool.h>
#include <stdint.h>

// Fixed storage, no allocation or Pico dependencies. Marker IDs may include 0.
#define DFS_MAX_MARKERS 64u
#define DFS_NO_NODE UINT16_MAX
#define DFS_ALL_EXITS 0x0Fu

// Map-frame directions, clockwise. The caller supplies calibrated heading;
// these are NOT the ToF left/front/right mask bits.
typedef enum { DFS_NORTH, DFS_EAST, DFS_SOUTH, DFS_WEST } dfs_direction_t;
typedef enum { DFS_PLAIN, DFS_JUNCTION } dfs_marker_type_t;
typedef enum {
    DFS_IDLE, DFS_READY, DFS_MOVING, DFS_PAUSED,
    DFS_COMPLETE, DFS_EXHAUSTED, DFS_FAULT
} dfs_state_t;
typedef enum {
    DFS_FAULT_NONE, DFS_FAULT_TIMEOUT, DFS_FAULT_TOPOLOGY,
    DFS_FAULT_RETURN_BLOCKED, DFS_FAULT_CAPACITY, DFS_FAULT_SEQUENCE
} dfs_fault_t;
typedef enum {
    DFS_MOVE_NONE, DFS_EXPLORE, DFS_BACKTRACK, DFS_LOOP_RETURN, DFS_BLOCKED_RETURN
} dfs_move_kind_t;
typedef enum {
    DFS_ACTION_WAIT, DFS_ACTION_SCAN, DFS_ACTION_MOVE,
    DFS_ACTION_PAUSED, DFS_ACTION_COMPLETE, DFS_ACTION_EXHAUSTED, DFS_ACTION_FAULT
} dfs_action_kind_t;

typedef struct {
    uint32_t session_id;       // Nonzero; use a new ID when resetting a mission.
    uint16_t target_markers;   // N distinct IDs, INCLUDING the starting marker.
    uint32_t segment_timeout_ms;
    uint8_t priority[4];       // Clockwise offsets: front=0, right=1, back=2, left=3.
} dfs_config_t;

typedef struct {
    uint32_t marker_id;
    dfs_marker_type_t type;
    uint8_t known_exits;
    uint8_t open_exits;
    uint8_t blocked_exits;
    uint8_t traversed_exits;
    uint16_t neighbor[4];      // B1 traversal bookkeeping; B2 owns the map/positions.
} dfs_node_t;

typedef struct {
    uint32_t session_id;
    uint32_t segment_id;
    uint32_t from_marker;
    bool from_confirmed;      // False when returning from a blockage mid-segment.
    uint32_t target_marker;
    bool target_known;        // Exploration discovers its destination on arrival.
    dfs_direction_t direction;
    dfs_move_kind_t kind;
} dfs_segment_t;

typedef struct {
    dfs_action_kind_t kind;
    dfs_fault_t fault;
    uint32_t marker_id;
    dfs_segment_t segment;
} dfs_action_t;

typedef struct {
    dfs_config_t config;
    dfs_state_t state;
    dfs_fault_t fault;
    dfs_node_t nodes[DFS_MAX_MARKERS];
    uint16_t node_count;
    uint16_t stack[DFS_MAX_MARKERS];
    uint16_t depth;
    uint16_t current;
    bool current_confirmed;
    dfs_direction_t heading;
    dfs_segment_t pending;
    uint32_t issued_ms;
    uint32_t next_segment_id;
    uint16_t return_node;
    dfs_direction_t return_direction;
    dfs_move_kind_t return_kind;
} dfs_explorer_t;

// Provisional 10 s travel deadline; calibrate on hardware before live use.
dfs_config_t dfs_default_config(uint32_t session_id, uint16_t target_markers);
bool dfs_start(dfs_explorer_t *dfs, const dfs_config_t *config, uint32_t marker_id,
               dfs_marker_type_t type, dfs_direction_t heading);
// Supply CONFIRMED map-frame open/blocked exits; omitted bits remain unknown.
// Blocked exits stay excluded for this mission. Reset starts a fresh graph.
bool dfs_observe_exits(dfs_explorer_t *dfs, uint32_t marker_id,
                       uint8_t open_exits, uint8_t blocked_exits);
// Produces one MOVE then WAIT until its result. No function here drives motors.
dfs_action_t dfs_next_action(dfs_explorer_t *dfs, uint32_t now_ms);
bool dfs_arrive(dfs_explorer_t *dfs, uint32_t session_id, uint32_t segment_id,
                uint32_t marker_id, dfs_marker_type_t type,
                dfs_direction_t heading, uint32_t now_ms);
// Call only for a NAVIGATION blockage. Emergency STOP must cancel/pause instead.
// Requires a subsequent return action and confirmed source-marker arrival.
bool dfs_segment_blocked(dfs_explorer_t *dfs, uint32_t session_id,
                         uint32_t segment_id, uint32_t now_ms);
void dfs_tick(dfs_explorer_t *dfs, uint32_t now_ms);
// Caller must stop/cancel actual motion. Progress is kept, pending results rejected.
bool dfs_pause(dfs_explorer_t *dfs);
// Only after external self-check/authorization and position confirmation at stack top.
bool dfs_resume(dfs_explorer_t *dfs, uint32_t marker_id, dfs_direction_t heading);
const dfs_node_t *dfs_find_marker(const dfs_explorer_t *dfs, uint32_t marker_id);

#endif
