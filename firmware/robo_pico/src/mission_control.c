/**
 * @file mission_control.c
 * @brief Mission control firmware module.
 */
#include "mission_control.h"

#include <string.h>

static void
transition (mission_t * p_mission, mission_state_t state,
            mission_reason_t reason, bool b_test_mode, uint32_t now_ms)
{
    if (p_mission->state == state && p_mission->reason == reason &&
        p_mission->test_mode == b_test_mode)
    {
        return;
    }
    p_mission->state      = state;
    p_mission->reason     = reason;
    p_mission->test_mode  = b_test_mode;
    p_mission->changed_ms = now_ms;
    p_mission->revision++;
}

void
mission_init (mission_t * p_mission, uint32_t now_ms)
{

    *p_mission = (mission_t) { .state      = MISSION_IDLE,
                               .reason     = MISSION_REASON_BOOT,
                               .changed_ms = now_ms };

    travel_leg_init(&p_mission->leg);
}

void
mission_clear_leg (mission_t * p_mission, uint32_t now_ms)
{
    if (travel_leg_clear(&p_mission->leg))
    {
        p_mission->changed_ms = now_ms;
        p_mission->revision++;
    }
}

void
mission_pause (mission_t * p_mission, mission_reason_t reason, uint32_t now_ms)
{
    p_mission->dfs_active = false;
    mission_clear_leg(p_mission, now_ms);
    if (p_mission->state == MISSION_READY ||
        p_mission->state == MISSION_EXPLORING)
    {
        transition(p_mission, MISSION_PAUSED, reason, p_mission->test_mode,
                   now_ms);
    }
}

bool
mission_begin_drive (mission_t * p_mission, uint32_t now_ms)
{
    if (p_mission->test_mode ||
        (p_mission->state != MISSION_IDLE &&
         p_mission->state != MISSION_READY &&
         !(p_mission->state == MISSION_EXPLORING && p_mission->dfs_active)) ||
        !p_mission->leg.configured || !p_mission->leg.next_target_node ||
        p_mission->leg.phase != LEG_CONFIGURED)
    {
        return false;
    }
    p_mission->leg.phase = LEG_APPROACHING;
    uint32_t revision    = p_mission->revision;
    transition(p_mission, MISSION_EXPLORING, MISSION_REASON_START_DFS, false,
               now_ms);
    if (p_mission->revision == revision)
    {
        p_mission->revision++;
        p_mission->changed_ms = now_ms;
    }
    return true;
}

bool
mission_motion_enabled (const mission_t * p_mission)
{
    return !p_mission->test_mode && p_mission->state == MISSION_EXPLORING &&
           (p_mission->leg.phase == LEG_APPROACHING ||
            p_mission->leg.phase == LEG_BACKING_UP ||
            p_mission->leg.phase == LEG_SEARCHING_REFERENCE ||
            p_mission->leg.phase == LEG_ALIGNING ||
            p_mission->leg.phase == LEG_CORRECTING ||
            (p_mission->dfs_active && p_mission->leg.phase == LEG_TURNING));
}

bool
mission_arrival_phase (mission_t * p_mission, leg_phase_t phase,
                       uint32_t now_ms)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        !p_mission->leg.configured ||
        !((phase == LEG_BRAKING &&
           (p_mission->leg.phase == LEG_APPROACHING ||
            p_mission->leg.phase == LEG_HOLDING ||
            p_mission->leg.phase == LEG_SEARCHING_REFERENCE)) ||
          (phase == LEG_VERIFYING && p_mission->leg.phase == LEG_BRAKING)))
    {
        return false;
    }
    p_mission->leg.phase  = phase;
    p_mission->changed_ms = now_ms;
    p_mission->revision++;
    return true;
}

bool
mission_camera_hold (mission_t * p_mission, bool b_holding, uint32_t now_ms)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        !p_mission->leg.configured ||
        p_mission->leg.phase != (b_holding ? LEG_APPROACHING : LEG_HOLDING))
    {
        return false;
    }
    p_mission->leg.phase  = b_holding ? LEG_HOLDING : LEG_APPROACHING;
    p_mission->changed_ms = now_ms;
    p_mission->revision++;
    return true;
}

bool
mission_alignment_phase (mission_t * p_mission, bool b_aligning, uint32_t now)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        !p_mission->leg.configured)
    {
        return false;
    }
    if (b_aligning ? (p_mission->leg.phase != LEG_APPROACHING &&
                      p_mission->leg.phase != LEG_HOLDING &&
                      p_mission->leg.phase != LEG_SEARCHING_REFERENCE &&
                      p_mission->leg.phase != LEG_BRAKING &&
                      p_mission->leg.phase != LEG_VERIFYING &&
                      p_mission->leg.phase != LEG_REACQUIRING)
                   : p_mission->leg.phase != LEG_ALIGNING)
    {
        return false;
    }
    p_mission->leg.phase  = b_aligning ? LEG_ALIGNING : LEG_APPROACHING;
    p_mission->changed_ms = now;
    p_mission->revision++;
    return true;
}

static bool
camera_waitable_phase (leg_phase_t phase)
{
    return phase == LEG_APPROACHING || phase == LEG_HOLDING ||
           phase == LEG_BRAKING || phase == LEG_VERIFYING ||
           phase == LEG_REACQUIRING || phase == LEG_SEARCHING_REFERENCE ||
           phase == LEG_ALIGNING || phase == LEG_CORRECTING ||
           phase == LEG_OBSTACLE_VERIFYING;
}

bool
mission_camera_wait_phase (mission_t * p_mission, leg_phase_t phase,
                           uint32_t now)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        !p_mission->leg.configured)
    {
        return false;
    }
    if (phase == LEG_WAITING_CAMERA
            ? !camera_waitable_phase(p_mission->leg.phase)
            : (p_mission->leg.phase != LEG_WAITING_CAMERA ||
               !camera_waitable_phase(phase)))
    {
        return false;
    }
    p_mission->leg.phase  = phase;
    p_mission->changed_ms = now;
    p_mission->revision++;
    return true;
}

bool
mission_recovery_phase (mission_t * p_mission, leg_phase_t phase, uint32_t now)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        (!p_mission->leg.configured && !p_mission->dfs_active) ||
        phase == LEG_NONE || phase == LEG_COMPLETE || phase == LEG_CANCELLED ||
        (unsigned) phase > LEG_CORRECTING)
    {
        return false;
    }
    p_mission->leg.phase  = phase;
    p_mission->changed_ms = now;
    p_mission->revision++;
    return true;
}

bool
mission_complete_target (mission_t * p_mission, uint32_t now_ms)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        (p_mission->leg.phase != LEG_VERIFYING &&
         p_mission->leg.phase != LEG_OBSTACLE_VERIFYING))
    {
        return false;
    }
    p_mission->leg.phase = LEG_COMPLETE;
    transition(p_mission,
               p_mission->dfs_active ? MISSION_EXPLORING : MISSION_READY,
               MISSION_REASON_TARGET_ARRIVED, false, now_ms);
    return true;
}

bool
mission_reference_phase (mission_t * p_mission, leg_phase_t phase, uint32_t now)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        !p_mission->leg.configured)
    {
        return false;
    }
    leg_phase_t from = p_mission->leg.phase;

    bool b_allowed = (phase == LEG_HOLDING &&
                      (from == LEG_APPROACHING || from == LEG_BRAKING ||
                       from == LEG_VERIFYING)) ||
                     (phase == LEG_BACKING_UP &&
                      (from == LEG_HOLDING || from == LEG_REACQUIRING)) ||
                     (phase == LEG_REACQUIRING && from == LEG_BACKING_UP) ||
                     (phase == LEG_APPROACHING &&
                      (from == LEG_HOLDING || from == LEG_REACQUIRING));

    if (!b_allowed)
    {
        return false;
    }
    p_mission->leg.phase  = phase;
    p_mission->changed_ms = now;
    p_mission->revision++;
    return true;
}

bool
mission_search_phase (mission_t * p_mission, leg_phase_t phase, uint32_t now)
{
    if (p_mission->test_mode || p_mission->state != MISSION_EXPLORING ||
        !p_mission->leg.configured)
    {
        return false;
    }
    leg_phase_t from = p_mission->leg.phase;

    bool b_allowed =
        (phase == LEG_SEARCHING_REFERENCE &&
         (from == LEG_APPROACHING || from == LEG_HOLDING)) ||
        (phase == LEG_HOLDING && from == LEG_SEARCHING_REFERENCE) ||
        (phase == LEG_APPROACHING && from == LEG_SEARCHING_REFERENCE) ||
        (phase == LEG_OBSTACLE_VERIFYING &&
         (from == LEG_APPROACHING || from == LEG_SEARCHING_REFERENCE ||
          from == LEG_HOLDING || from == LEG_BRAKING || from == LEG_VERIFYING));

    if (!b_allowed)
    {
        return false;
    }
    p_mission->leg.phase  = phase;
    p_mission->changed_ms = now;
    p_mission->revision++;
    return true;
}

bool
mission_start_dfs (mission_t * p_mission, uint32_t now_ms)
{
    if (p_mission->test_mode ||
        (p_mission->state != MISSION_IDLE && p_mission->state != MISSION_READY))
    {
        return false;
    }
    mission_clear_leg(p_mission, now_ms);
    p_mission->dfs_active = true;
    transition(p_mission, MISSION_EXPLORING, MISSION_REASON_START_DFS, false,
               now_ms);
    return true;
}

bool
mission_finish_dfs (mission_t * p_mission, uint32_t now_ms)
{
    if (!p_mission->dfs_active || p_mission->test_mode ||
        p_mission->state != MISSION_EXPLORING)
    {
        return false;
    }
    mission_clear_leg(p_mission, now_ms);
    p_mission->dfs_active = false;
    transition(p_mission, MISSION_COMPLETE, MISSION_REASON_DFS_COMPLETE, false,
               now_ms);
    return true;
}

void
mission_fault (mission_t * p_mission, mission_reason_t reason, uint32_t now_ms)
{
    p_mission->dfs_active = false;
    mission_clear_leg(p_mission, now_ms);
    // Preserve the first fault; an actual ESTOP overrides a diagnostic fault.
    if (p_mission->state != MISSION_FAULT || reason == MISSION_REASON_ESTOP)
    {
        transition(p_mission, MISSION_FAULT, reason, p_mission->test_mode,
                   now_ms);
    }
}

mission_result_t
mission_test_event (mission_t * p_mission, const char * p_event,
                    bool b_safety_latched, uint32_t now_ms)
{
    bool b_ready    = strcmp(p_event, "READY") == 0;
    bool b_start    = strcmp(p_event, "START") == 0;
    bool b_pause    = strcmp(p_event, "PAUSE") == 0;
    bool b_complete = strcmp(p_event, "COMPLETE") == 0;
    bool b_fault    = strcmp(p_event, "FAULT") == 0;
    bool b_reset    = strcmp(p_event, "RESET") == 0;
    if (!(b_ready || b_start || b_pause || b_complete || b_fault || b_reset))
    {
        return MISSION_REJECTED_COMMAND;
    }
    if (b_safety_latched)
    {
        mission_fault(p_mission, MISSION_REASON_ESTOP, now_ms);
        return MISSION_REJECTED_SAFETY;
    }
    if (b_reset)
    {
        p_mission->dfs_active = false;
        mission_clear_leg(p_mission, now_ms);
        // Reset does not start/resume a mission or clear the motor safety
        // latch.
        if (p_mission->state != MISSION_IDLE || p_mission->test_mode)
        {
            transition(p_mission, MISSION_IDLE, MISSION_REASON_RESET, false,
                       now_ms);
        }
        return MISSION_ACCEPTED;
    }
    if (b_fault)
    {
        mission_clear_leg(p_mission, now_ms);
        transition(p_mission, MISSION_FAULT,
                   p_mission->state == MISSION_FAULT
                       ? p_mission->reason
                       : MISSION_REASON_TEST_FAULT,
                   true, now_ms);
        return MISSION_ACCEPTED;
    }
    if (b_ready && p_mission->state == MISSION_IDLE)
    {
        transition(p_mission, MISSION_READY, MISSION_REASON_TEST_READY, true,
                   now_ms);
        return MISSION_ACCEPTED;
    }
    if (b_start && p_mission->state == MISSION_READY)
    {
        transition(p_mission, MISSION_EXPLORING, MISSION_REASON_TEST_START,
                   true, now_ms);
        return MISSION_ACCEPTED;
    }
    if (b_pause && (p_mission->state == MISSION_READY ||
                    p_mission->state == MISSION_EXPLORING))
    {
        mission_pause(p_mission, MISSION_REASON_OPERATOR_STOP, now_ms);
        return MISSION_ACCEPTED;
    }
    if (b_complete && p_mission->state == MISSION_EXPLORING)
    {
        transition(p_mission, MISSION_COMPLETE, MISSION_REASON_TEST_COMPLETE,
                   true, now_ms);
        return MISSION_ACCEPTED;
    }
    // Re-delivery of an already applied event is harmless and changes no
    // revision.
    if (p_mission->test_mode &&
        ((b_ready && p_mission->state == MISSION_READY) ||
         (b_start && p_mission->state == MISSION_EXPLORING) ||
         (b_pause && p_mission->state == MISSION_PAUSED) ||
         (b_complete && p_mission->state == MISSION_COMPLETE)))
    {
        return MISSION_ACCEPTED;
    }
    return MISSION_REJECTED_STATE;
}

void
mission_tick (mission_t * p_mission, bool b_safety_latched, bool b_link_ok,
              uint32_t now_ms)
{
    if (b_safety_latched)
    {
        mission_fault(p_mission, MISSION_REASON_ESTOP, now_ms);
    }
    else if (p_mission->test_mode && !b_link_ok)
    {
        mission_pause(p_mission, MISSION_REASON_LINK_LOST, now_ms);
    }
    // Real DFS executors stop and retain their context while waiting for
    // inputs. Diagnostic EXPLORING remains stationary; real motion belongs to
    // the executor.
}

bool
mission_command_allowed (const mission_t * p_mission, const char * p_command)
{
    if (!p_mission->test_mode && p_mission->state != MISSION_EXPLORING)
    {
        return true;
    }
    return strcmp(p_command, "STOP") == 0 ||
           strcmp(p_command, "AUTO_STOP") == 0 ||
           strcmp(p_command, "MOVE_STOP") == 0 ||
           strcmp(p_command, "ESTOP") == 0 ||
           strcmp(p_command, "RESET_SAFETY") == 0;
}

const char *
mission_state_name (mission_state_t state)
{
    switch (state)
    {
        case MISSION_IDLE:
            return "IDLE";
        case MISSION_READY:
            return "READY";
        case MISSION_EXPLORING:
            return "EXPLORING";
        case MISSION_PAUSED:
            return "PAUSED";
        case MISSION_COMPLETE:
            return "COMPLETE";
        case MISSION_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

const char *
mission_reason_name (mission_reason_t reason)
{
    switch (reason)
    {
        case MISSION_REASON_BOOT:
            return "BOOT";
        case MISSION_REASON_TEST_READY:
            return "TEST_READY";
        case MISSION_REASON_TEST_START:
            return "TEST_START";
        case MISSION_REASON_OPERATOR_STOP:
            return "OPERATOR_STOP";
        case MISSION_REASON_TEST_COMPLETE:
            return "TEST_COMPLETE";
        case MISSION_REASON_TEST_FAULT:
            return "TEST_FAULT";
        case MISSION_REASON_ESTOP:
            return "ESTOP";
        case MISSION_REASON_FRONT_STOP:
            return "FRONT_STOP";
        case MISSION_REASON_LINK_LOST:
            return "LINK_LOST";
        case MISSION_REASON_RESET:
            return "RESET";
        case MISSION_REASON_START_DFS:
            return "START_DFS";
        case MISSION_REASON_INPUT_LOST:
            return "INPUT_LOST";
        case MISSION_REASON_APPROACH_LIMIT:
            return "APPROACH_LIMIT";
        case MISSION_REASON_DRIVE_TIMEOUT:
            return "DRIVE_TIMEOUT";
        case MISSION_REASON_MOTOR_STOPPED:
            return "MOTOR_STOPPED";
        case MISSION_REASON_TARGET_ARRIVED:
            return "TARGET_ARRIVED";
        case MISSION_REASON_DFS_COMPLETE:
            return "DFS_COMPLETE";
        default:
            return "UNKNOWN";
    }
}

const char *
mission_result_name (mission_result_t result)
{
    switch (result)
    {
        case MISSION_ACCEPTED:
            return "ACCEPTED";
        case MISSION_REJECTED_COMMAND:
            return "UNKNOWN_EVENT";
        case MISSION_REJECTED_STATE:
            return "WRONG_STATE";
        case MISSION_REJECTED_SAFETY:
            return "SAFETY_LOCKED";
        default:
            return "UNKNOWN";
    }
}

/* End of file. */
