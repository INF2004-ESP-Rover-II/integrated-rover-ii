#include "mission_control.h"

#include <string.h>

static void transition(mission_t *mission, mission_state_t state,
                       mission_reason_t reason, bool test_mode, uint32_t now_ms)
{
    if (mission->state == state && mission->reason == reason &&
        mission->test_mode == test_mode) return;
    mission->state = state;
    mission->reason = reason;
    mission->test_mode = test_mode;
    mission->changed_ms = now_ms;
    mission->revision++;
}

void mission_init(mission_t *mission, uint32_t now_ms)
{
    *mission = (mission_t){
        .state = MISSION_IDLE,
        .reason = MISSION_REASON_BOOT,
        .changed_ms = now_ms
    };
}

void mission_pause(mission_t *mission, mission_reason_t reason, uint32_t now_ms)
{
    if (mission->state == MISSION_READY || mission->state == MISSION_EXPLORING)
        transition(mission, MISSION_PAUSED, reason, mission->test_mode, now_ms);
}

void mission_fault(mission_t *mission, mission_reason_t reason, uint32_t now_ms)
{
    // Preserve the first fault; an actual ESTOP overrides a diagnostic fault.
    if (mission->state != MISSION_FAULT || reason == MISSION_REASON_ESTOP)
        transition(mission, MISSION_FAULT, reason, mission->test_mode, now_ms);
}

mission_result_t mission_test_event(mission_t *mission, const char *event,
                                   bool safety_latched, uint32_t now_ms)
{
    bool ready = strcmp(event, "READY") == 0;
    bool start = strcmp(event, "START") == 0;
    bool pause = strcmp(event, "PAUSE") == 0;
    bool complete = strcmp(event, "COMPLETE") == 0;
    bool fault = strcmp(event, "FAULT") == 0;
    bool reset = strcmp(event, "RESET") == 0;
    if (!(ready || start || pause || complete || fault || reset))
        return MISSION_REJECTED_COMMAND;
    if (safety_latched) {
        mission_fault(mission, MISSION_REASON_ESTOP, now_ms);
        return MISSION_REJECTED_SAFETY;
    }
    if (reset) {
        // Reset does not start/resume a mission or clear the motor safety latch.
        if (mission->state != MISSION_IDLE || mission->test_mode)
            transition(mission, MISSION_IDLE, MISSION_REASON_RESET, false, now_ms);
        return MISSION_ACCEPTED;
    }
    if (fault) {
        transition(mission, MISSION_FAULT,
                   mission->state == MISSION_FAULT ? mission->reason : MISSION_REASON_TEST_FAULT,
                   true, now_ms);
        return MISSION_ACCEPTED;
    }
    if (ready && mission->state == MISSION_IDLE) {
        transition(mission, MISSION_READY, MISSION_REASON_TEST_READY, true, now_ms);
        return MISSION_ACCEPTED;
    }
    if (start && mission->state == MISSION_READY) {
        transition(mission, MISSION_EXPLORING, MISSION_REASON_TEST_START, true, now_ms);
        return MISSION_ACCEPTED;
    }
    if (pause && (mission->state == MISSION_READY || mission->state == MISSION_EXPLORING)) {
        mission_pause(mission, MISSION_REASON_OPERATOR_STOP, now_ms);
        return MISSION_ACCEPTED;
    }
    if (complete && mission->state == MISSION_EXPLORING) {
        transition(mission, MISSION_COMPLETE, MISSION_REASON_TEST_COMPLETE, true, now_ms);
        return MISSION_ACCEPTED;
    }
    // Re-delivery of an already applied event is harmless and changes no revision.
    if (mission->test_mode &&
        ((ready && mission->state == MISSION_READY) ||
         (start && mission->state == MISSION_EXPLORING) ||
         (pause && mission->state == MISSION_PAUSED) ||
         (complete && mission->state == MISSION_COMPLETE))) return MISSION_ACCEPTED;
    return MISSION_REJECTED_STATE;
}

void mission_tick(mission_t *mission, bool safety_latched, bool link_ok, uint32_t now_ms)
{
    if (safety_latched) mission_fault(mission, MISSION_REASON_ESTOP, now_ms);
    else if (mission->test_mode && !link_ok)
        mission_pause(mission, MISSION_REASON_LINK_LOST, now_ms);
    // No DFS/executor yet: EXPLORING is stationary and never auto-completes.
}

bool mission_command_allowed(const mission_t *mission, const char *command)
{
    if (!mission->test_mode) return true; // Preserve existing laptop/manual mode.
    return strcmp(command, "STOP") == 0 || strcmp(command, "AUTO_STOP") == 0 ||
           strcmp(command, "MOVE_STOP") == 0 || strcmp(command, "ESTOP") == 0 ||
           strcmp(command, "RESET_SAFETY") == 0;
}

const char *mission_state_name(mission_state_t state)
{
    switch (state) {
    case MISSION_IDLE: return "IDLE";
    case MISSION_READY: return "READY";
    case MISSION_EXPLORING: return "EXPLORING";
    case MISSION_PAUSED: return "PAUSED";
    case MISSION_COMPLETE: return "COMPLETE";
    case MISSION_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

const char *mission_reason_name(mission_reason_t reason)
{
    switch (reason) {
    case MISSION_REASON_BOOT: return "BOOT";
    case MISSION_REASON_TEST_READY: return "TEST_READY";
    case MISSION_REASON_TEST_START: return "TEST_START";
    case MISSION_REASON_OPERATOR_STOP: return "OPERATOR_STOP";
    case MISSION_REASON_TEST_COMPLETE: return "TEST_COMPLETE";
    case MISSION_REASON_TEST_FAULT: return "TEST_FAULT";
    case MISSION_REASON_ESTOP: return "ESTOP";
    case MISSION_REASON_FRONT_STOP: return "FRONT_STOP";
    case MISSION_REASON_LINK_LOST: return "LINK_LOST";
    case MISSION_REASON_RESET: return "RESET";
    default: return "UNKNOWN";
    }
}

const char *mission_result_name(mission_result_t result)
{
    switch (result) {
    case MISSION_ACCEPTED: return "ACCEPTED";
    case MISSION_REJECTED_COMMAND: return "UNKNOWN_EVENT";
    case MISSION_REJECTED_STATE: return "WRONG_STATE";
    case MISSION_REJECTED_SAFETY: return "SAFETY_LOCKED";
    default: return "UNKNOWN";
    }
}
