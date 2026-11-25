/**
 * @file onboard_dfs.h
 * @brief Onboard dfs firmware module.
 */
#ifndef ROVER_ONBOARD_DFS_H
#define ROVER_ONBOARD_DFS_H
#include "onboard_drive.h"

#define DFS_OPEN_DISTANCE_MM 500u
#define DFS_SETTLE_MS 300u
#define DFS_SAMPLE_MS 800u
#define DFS_TARGET_TIMEOUT_MS 5000u
#define DFS_TURN_RETRY_MS 3000u
#define DFS_TURN_TIMEOUT_MS 5000u
typedef enum
{
    DFS_IDLE,
    DFS_AT_NODE,
    DFS_TURNING,
    DFS_SAMPLING,
    DFS_WAIT_TARGET,
    DFS_TRAVELLING,
    DFS_FINISHED,
    DFS_STOPPED
} dfs_stage_t;
typedef enum
{
    DFS_CHOOSE_EXPLORE,
    DFS_CHOOSE_SCAN,
    DFS_CHOOSE_BACKTRACK,
    DFS_CHOOSE_COMPLETE,
    DFS_CHOOSE_UNRESOLVED,
    DFS_CHOOSE_ERROR
} dfs_choice_kind_t;
typedef struct
{
    dfs_choice_kind_t kind;
    map_direction_t   direction;
    uint8_t           expected_id;
} dfs_choice_t;
typedef struct
{
    onboard_drive_inputs_t drive;
    uint16_t               tof_mm[3];
    uint8_t                fault_mask;
    float                  heading_offset_deg;
} onboard_dfs_inputs_t;
typedef struct
{
    bool            active;
    bool            backtracking;
    dfs_stage_t     stage;
    dfs_stage_t     after_turn;
    uint8_t         stack[ROVER_MAP_CAPACITY];
    uint8_t         scanned[ROVER_MAP_CAPACITY];
    uint8_t         seen[32];
    uint8_t         depth;
    uint8_t         expected_id;
    uint8_t         from_node;
    uint8_t         sample_code[4];
    uint8_t         sample_count[4];
    uint16_t        open_distance_mm;
    map_direction_t direction;
    uint32_t        stage_started_ms;
    uint32_t        ready_ms;
    uint32_t        last_sensor_ms;
    uint32_t        last_renew_ms;
    uint32_t        turn_attempt_started_ms;
    bool            b_turn_retry_waiting;
    const char *    detail;
    bool            input_waiting;
    leg_phase_t     input_resume_phase;
    uint32_t        input_wait_started_ms;
} onboard_dfs_t;


void onboard_dfs_init (onboard_dfs_t * p_dfs);


const char * onboard_dfs_start (onboard_dfs_t * p_dfs, mission_t * p_mission,
                                const rover_map_t *          p_map,
                                onboard_drive_t *            p_drive,
                                target_selector_t *          p_selection,
                                const onboard_dfs_inputs_t * p_inputs,
                                uint32_t                     now);


void onboard_dfs_tick (onboard_dfs_t * p_dfs, mission_t * p_mission,
                       rover_map_t * p_map, onboard_drive_t * p_drive,
                       target_selector_t *          p_selection,
                       const onboard_dfs_inputs_t * p_inputs,
                       const camera_observation_t * p_camera, uint32_t now);


void onboard_dfs_stop (onboard_dfs_t * p_dfs, mission_t * p_mission,
                       const char * p_detail, uint32_t now);


bool onboard_dfs_selecting (const onboard_dfs_t * p_dfs, uint32_t now);


dfs_choice_t onboard_dfs_choose (const onboard_dfs_t * p_dfs,
                                 const rover_map_t *   p_map,
                                 map_direction_t       facing,
                                 bool b_forward_camera_candidate);


bool onboard_dfs_note_arrival (onboard_dfs_t *     p_dfs,
                               const rover_map_t * p_map);


size_t onboard_dfs_format (const onboard_dfs_t * p_dfs,
                           const rover_map_t * p_map, char * p_out,
                           size_t capacity);

#endif

/* End of file. */
