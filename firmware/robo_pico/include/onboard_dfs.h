#ifndef ONBOARD_DFS_H
#define ONBOARD_DFS_H
#include <stddef.h>
#include "rover_map.h"
#include "vision_protocol.h"

/* Full node/edge JSON plus state. Also size the MQTT output ring for this. */
#define ONBOARD_SNAPSHOT_BYTES (1024u + 160u * ROVER_MAX_NODES)

typedef enum {
    DFS_IDLE, DFS_READY, DFS_SCAN, DFS_TURN, DFS_SETTLE, DFS_ACQUIRE,
    DFS_DRIVE, DFS_CONFIRM, DFS_HOLD, DFS_RECOVERY_TURN, DFS_RECOVERY_SETTLE,
    DFS_COMPLETE, DFS_PAUSED, DFS_FAULT
} onboard_phase_t;
typedef struct {
    float heading_deg;
    uint16_t tof_mm[3]; /* left/front/right; 0 unavailable; 1023 healthy beyond range */
    uint8_t tof_faults;
    bool imu_valid, rear_valid;
    uint16_t rear_mm;
    uint32_t received_ms;
} onboard_sensor_t;
typedef struct {
    void *context;
    bool (*drive)(void *, int percent);
    bool (*turn)(void *, int absolute_degrees);
    bool (*turn_done)(void *);
    void (*stop)(void *);
} onboard_motor_io_t;
typedef struct {
    float pitch_mm, camera_height_mm, camera_forward_mm, camera_down_deg;
    float position_tolerance_mm, bearing_limit_deg;
    uint16_t opening_mm, turn_clearance_mm;
    int cruise_percent, approach_percent;
    uint32_t sensor_max_ms, image_max_ms, settle_ms, leg_timeout_ms;
    bool camera_calibrated;
} onboard_config_t;
typedef struct {
    uint32_t sequence;
    uint8_t target;
    bool arrived;
    const char *reason;
} onboard_hop_result_t;
typedef struct {
    rover_map_t map;
    onboard_config_t config;
    onboard_motor_io_t motor;
    onboard_sensor_t sensor;
    vision_frame_t last_camera;
    bool camera_received;
    uint32_t clock_ms, last_camera_rx_ms;
    onboard_phase_t phase;
    const char *reason;
    uint8_t goal, stack[ROVER_MAX_NODES], depth;
    unsigned direction;
    float zero_heading, pose_x, pose_y;
    bool sensor_seen, image_seen, sequence_seen, pose_valid, current_confirmed;
    uint8_t last_sequence;
    uint32_t last_image_ms, last_pose_ms, state_since, leg_since, hold_since;
    uint32_t state_revision, last_scan_sensor_ms, last_motor_ms;
    int target_x, target_y;
    uint8_t source, target, leg_kind, loop_return;
    bool inspection, recovery_used, route_mode, result_pending, leg_active, wall_valid;
    float wall_coordinate;
    uint32_t last_wall_sensor_ms;
    unsigned wall_confirmations;
    float recovery_heading;
    unsigned recovery_angle, recovery_confirmations;
    unsigned scan_count[4];
    uint8_t scan_value[4], arrival_count;
    uint8_t candidate_count[ROVER_MAX_MARKER_ID + 1u];
    int16_t candidate_x[ROVER_MAX_MARKER_ID + 1u], candidate_y[ROVER_MAX_MARKER_ID + 1u];
    uint32_t hop_sequence, last_hop_sequence;
    onboard_hop_result_t result;
} onboard_dfs_t;

onboard_config_t onboard_dfs_default_config(void);
void onboard_dfs_init(onboard_dfs_t *, const onboard_motor_io_t *);
/* Set measured mounting geometry before anchoring a map. RAM only, per boot. */
bool onboard_dfs_set_geometry(onboard_dfs_t *, float height_mm, float forward_mm,
                              float downward_deg, uint32_t now_ms);
bool onboard_dfs_set_start(onboard_dfs_t *, uint8_t id, uint32_t now_ms);
bool onboard_dfs_start(onboard_dfs_t *, uint8_t count, uint16_t opening_mm, uint32_t now_ms);
void onboard_dfs_stop(onboard_dfs_t *, const char *reason, uint32_t now_ms);
void onboard_dfs_fault(onboard_dfs_t *, const char *reason, uint32_t now_ms);
bool onboard_dfs_resume(onboard_dfs_t *, uint32_t now_ms);
void onboard_dfs_sensor(onboard_dfs_t *, const onboard_sensor_t *);
void onboard_dfs_camera(onboard_dfs_t *, const vision_frame_t *, uint32_t received_ms, uint32_t now_ms);
void onboard_dfs_tick(onboard_dfs_t *, uint32_t now_ms);
bool onboard_dfs_busy(const onboard_dfs_t *);
const char *onboard_dfs_phase_name(onboard_phase_t);
/* B2's future LOCAL planner reads this map and submits a verified adjacent hop.
 * No A* implementation or speculative edges are hidden in this DFS module. */
const rover_map_t *onboard_dfs_map(const onboard_dfs_t *);
bool onboard_dfs_begin_hop(onboard_dfs_t *, uint8_t target, unsigned direction,
                          uint32_t sequence, uint32_t now_ms);
bool onboard_dfs_take_result(onboard_dfs_t *, onboard_hop_result_t *);
size_t onboard_dfs_snapshot(const onboard_dfs_t *, char *, size_t);
#endif
