#ifndef ROVER_MISSION_CONTROL_H
#define ROVER_MISSION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MISSION_IDLE,
    MISSION_READY,
    MISSION_EXPLORING,
    MISSION_PAUSED,
    MISSION_COMPLETE,
    MISSION_FAULT
} mission_state_t;

typedef enum {
    MISSION_REASON_BOOT,
    MISSION_REASON_TEST_READY,
    MISSION_REASON_TEST_START,
    MISSION_REASON_OPERATOR_STOP,
    MISSION_REASON_TEST_COMPLETE,
    MISSION_REASON_TEST_FAULT,
    MISSION_REASON_ESTOP,
    MISSION_REASON_FRONT_STOP,
    MISSION_REASON_LINK_LOST,
    MISSION_REASON_RESET
} mission_reason_t;

typedef enum {
    MISSION_ACCEPTED,
    MISSION_REJECTED_COMMAND,
    MISSION_REJECTED_STATE,
    MISSION_REJECTED_SAFETY
} mission_result_t;

typedef struct {
    mission_state_t state;
    mission_reason_t reason;
    uint32_t revision;
    uint32_t changed_ms;
    bool test_mode;
} mission_t;

// Cooperative main-loop owner only. No hardware calls, allocation or waits.
void mission_init(mission_t *mission, uint32_t now_ms);
// Diagnostic transitions only; READY/COMPLETE do not validate a real DFS mission.
// Exact events: READY, START, PAUSE, COMPLETE, FAULT, RESET.
mission_result_t mission_test_event(mission_t *mission, const char *event,
                                   bool safety_latched, uint32_t now_ms);
void mission_pause(mission_t *mission, mission_reason_t reason, uint32_t now_ms);
void mission_fault(mission_t *mission, mission_reason_t reason, uint32_t now_ms);
void mission_tick(mission_t *mission, bool safety_latched, bool link_ok, uint32_t now_ms);
// Diagnostic ownership persists through PAUSED/COMPLETE/FAULT until RESET.
bool mission_command_allowed(const mission_t *mission, const char *command);
const char *mission_state_name(mission_state_t state);
const char *mission_reason_name(mission_reason_t reason);
const char *mission_result_name(mission_result_t result);

#endif
