/**
 * @file mission_control.h
 * @brief Mission control firmware module.
 */
#ifndef ROVER_MISSION_CONTROL_H
#define ROVER_MISSION_CONTROL_H

#include "travel_leg.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MISSION_IDLE,
    MISSION_READY,
    MISSION_EXPLORING,
    MISSION_PAUSED,
    MISSION_COMPLETE,
    MISSION_FAULT
} mission_state_t;

typedef enum
{
    MISSION_REASON_BOOT,
    MISSION_REASON_TEST_READY,
    MISSION_REASON_TEST_START,
    MISSION_REASON_OPERATOR_STOP,
    MISSION_REASON_TEST_COMPLETE,
    MISSION_REASON_TEST_FAULT,
    MISSION_REASON_ESTOP,
    MISSION_REASON_FRONT_STOP,
    MISSION_REASON_LINK_LOST,
    MISSION_REASON_RESET,
    MISSION_REASON_START_DFS,
    MISSION_REASON_INPUT_LOST,
    MISSION_REASON_APPROACH_LIMIT,
    MISSION_REASON_DRIVE_TIMEOUT,
    MISSION_REASON_MOTOR_STOPPED,
    MISSION_REASON_TARGET_ARRIVED,
    MISSION_REASON_DFS_COMPLETE,
    MISSION_REASON_NODE_LIMIT_REACHED
} mission_reason_t;

typedef enum
{
    MISSION_ACCEPTED,
    MISSION_REJECTED_COMMAND,
    MISSION_REJECTED_STATE,
    MISSION_REJECTED_SAFETY
} mission_result_t;

typedef struct
{
    mission_state_t  state;
    mission_reason_t reason;
    uint32_t         revision;
    uint32_t         changed_ms;
    bool             test_mode;
    bool             dfs_active;
    travel_leg_t     leg;
} mission_t;

// Cooperative main-loop owner only. No hardware calls, allocation or waits.

void mission_init (mission_t * p_mission, uint32_t now_ms);

void mission_clear_leg (mission_t * p_mission, uint32_t now_ms);

bool mission_begin_drive (mission_t * p_mission, uint32_t now_ms);

bool mission_motion_enabled (const mission_t * p_mission);

bool mission_arrival_phase (mission_t * p_mission, leg_phase_t phase,
                            uint32_t now_ms);

bool mission_complete_target (mission_t * p_mission, uint32_t now_ms);

bool mission_camera_hold (mission_t * p_mission, bool b_holding,
                          uint32_t now_ms);

bool mission_reference_phase (mission_t * p_mission, leg_phase_t phase,
                              uint32_t now_ms);

bool mission_search_phase (mission_t * p_mission, leg_phase_t phase,
                           uint32_t now_ms);

bool mission_alignment_phase (mission_t * p_mission, bool b_aligning,
                              uint32_t now_ms);

bool mission_camera_wait_phase (mission_t * p_mission, leg_phase_t phase,
                                uint32_t now_ms);

bool mission_recovery_phase (mission_t * p_mission, leg_phase_t phase,
                             uint32_t now_ms);

bool mission_start_dfs (mission_t * p_mission, uint32_t now_ms);

bool mission_finish_dfs (mission_t * p_mission, uint32_t now_ms);

bool mission_finish_node_limit (mission_t * p_mission, uint32_t now_ms);

// Diagnostic transitions only; READY/COMPLETE do not validate a real DFS
// mission. Exact events: READY, START, PAUSE, COMPLETE, FAULT, RESET.

mission_result_t mission_test_event (mission_t *  p_mission,
                                     const char * p_event,
                                     bool b_safety_latched, uint32_t now_ms);

void mission_pause (mission_t * p_mission, mission_reason_t reason,
                    uint32_t now_ms);

void mission_fault (mission_t * p_mission, mission_reason_t reason,
                    uint32_t now_ms);

void mission_tick (mission_t * p_mission, bool b_safety_latched, bool b_link_ok,
                   uint32_t now_ms);

// Diagnostic ownership persists through PAUSED/COMPLETE/FAULT until RESET.

bool mission_command_allowed (const mission_t * p_mission,
                              const char *      p_command);

const char * mission_state_name (mission_state_t state);

const char * mission_reason_name (mission_reason_t reason);

const char * mission_result_name (mission_result_t result);

#endif

/* End of file. */
