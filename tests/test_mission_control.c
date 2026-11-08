#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_control.h"

static void event(mission_t *mission, const char *name, uint32_t now)
{
    assert(mission_test_event(mission, name, false, now) == MISSION_ACCEPTED);
}

static void assert_movement_locked(const mission_t *mission)
{
    const char *moves[] = {
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "ROTATE_LEFT", "ROTATE_RIGHT",
        "FRONT_LEFT", "FRONT_RIGHT", "BACK_LEFT", "BACK_RIGHT",
        "MOVE_FORWARD:30", "MOVE_BACKWARD:30", "TURN_TO:90", "TURN_BY:-90",
        "ARM_UP", "ARM_DOWN", "CLAW_OPEN", "CLAW_CLOSE", "ZERO_HEADING", "AUTO_START"
    };
    const char *stops[] = {"STOP", "AUTO_STOP", "MOVE_STOP", "ESTOP", "RESET_SAFETY"};
    assert(mission->test_mode);
    for (size_t i = 0; i < sizeof(moves) / sizeof(moves[0]); ++i)
        assert(!mission_command_allowed(mission, moves[i]));
    for (size_t i = 0; i < sizeof(stops) / sizeof(stops[0]); ++i)
        assert(mission_command_allowed(mission, stops[i]));
    assert(!mission_command_allowed(mission, "STOP:FORWARD"));
    assert(!mission_command_allowed(mission, "STOP_EXTRA"));
}

int main(void)
{
    mission_t mission;
    mission_init(&mission, 10);
    assert(mission.state == MISSION_IDLE && !mission.test_mode);
    assert(mission.reason == MISSION_REASON_BOOT && mission.revision == 0);
    assert(mission_command_allowed(&mission, "FORWARD"));

    // Can't start or claim completion before preparing the stationary test.
    assert(mission_test_event(&mission, "START", false, 20) == MISSION_REJECTED_STATE);
    assert(mission_test_event(&mission, "COMPLETE", false, 20) == MISSION_REJECTED_STATE);
    assert(mission_test_event(&mission, "READY:extra", false, 20) == MISSION_REJECTED_COMMAND);
    assert(mission_test_event(&mission, "ready", false, 20) == MISSION_REJECTED_COMMAND);
    assert(mission.state == MISSION_IDLE && mission.revision == 0);

    event(&mission, "READY", 30);
    assert(mission.state == MISSION_READY && mission.changed_ms == 30);
    assert_movement_locked(&mission);
    uint32_t revision = mission.revision;
    event(&mission, "READY", 40);
    assert(mission.revision == revision && mission.changed_ms == 30);

    event(&mission, "START", 50);
    assert(mission.state == MISSION_EXPLORING);
    assert_movement_locked(&mission);
    revision = mission.revision;
    event(&mission, "START", 60);
    mission_tick(&mission, false, true, 100000);
    assert(mission.revision == revision && mission.state == MISSION_EXPLORING);
    assert(mission.changed_ms == 50); // No invented completion or background work.
    assert(mission_test_event(&mission, "READY", false, 100001) == MISSION_REJECTED_STATE);

    event(&mission, "COMPLETE", 100002);
    assert(mission.state == MISSION_COMPLETE);
    assert_movement_locked(&mission);
    revision = mission.revision;
    event(&mission, "COMPLETE", 100003);
    assert(mission.revision == revision);
    assert(mission_test_event(&mission, "START", false, 100004) == MISSION_REJECTED_STATE);
    mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, 100005);
    assert(mission.state == MISSION_COMPLETE);
    event(&mission, "RESET", 100006);
    assert(mission.state == MISSION_IDLE && !mission.test_mode);
    assert(mission_command_allowed(&mission, "TURN_TO:90"));

    // STOP retains stationary ownership; READY/START cannot silently resume.
    event(&mission, "READY", 200000);
    event(&mission, "START", 200001);
    mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, 200002);
    assert(mission.state == MISSION_PAUSED);
    assert_movement_locked(&mission);
    assert(mission_test_event(&mission, "START", false, 200003) == MISSION_REJECTED_STATE);
    assert(mission_test_event(&mission, "READY", false, 200003) == MISSION_REJECTED_STATE);
    revision = mission.revision;
    event(&mission, "PAUSE", 200004);
    mission_tick(&mission, false, true, 200005);
    assert(mission.state == MISSION_PAUSED && mission.revision == revision);

    // Input/link restoration cannot resurrect an interrupted test.
    event(&mission, "RESET", 300000);
    event(&mission, "READY", 300001);
    mission_tick(&mission, false, false, 300002);
    assert(mission.state == MISSION_PAUSED && mission.reason == MISSION_REASON_LINK_LOST);
    mission_tick(&mission, false, true, 300003);
    assert(mission.state == MISSION_PAUSED);
    event(&mission, "RESET", 300004);
    event(&mission, "READY", 300005);
    event(&mission, "START", 300006);
    mission_pause(&mission, MISSION_REASON_FRONT_STOP, 300007);
    assert(mission.state == MISSION_PAUSED && mission.reason == MISSION_REASON_FRONT_STOP);

    // Diagnostic FAULT persists, and a real ESTOP takes precedence.
    event(&mission, "FAULT", 400000);
    assert(mission.state == MISSION_FAULT && mission.reason == MISSION_REASON_TEST_FAULT);
    assert_movement_locked(&mission);
    assert(mission_test_event(&mission, "COMPLETE", false, 400001) == MISSION_REJECTED_STATE);
    mission_tick(&mission, true, true, 400002);
    assert(mission.reason == MISSION_REASON_ESTOP && mission.state == MISSION_FAULT);
    revision = mission.revision;
    mission_tick(&mission, true, true, 400003);
    assert(mission.revision == revision);
    assert(mission_test_event(&mission, "RESET", true, 400004) == MISSION_REJECTED_SAFETY);
    assert(mission.state == MISSION_FAULT && mission.test_mode);
    mission_tick(&mission, false, true, 400005); // Motor reset alone is not mission reset.
    assert(mission.state == MISSION_FAULT);
    event(&mission, "RESET", 400006);
    assert(mission.state == MISSION_IDLE && !mission.test_mode);

    // An ESTOP outside diagnostics still reports a fault; never enables motion.
    mission_tick(&mission, true, true, UINT32_MAX);
    assert(mission.state == MISSION_FAULT && !mission.test_mode);
    assert(mission_test_event(&mission, "READY", true, 1) == MISSION_REJECTED_SAFETY);
    assert(mission.state == MISSION_FAULT);
    event(&mission, "FAULT", 2); // Take diagnostic ownership after motor safety clears.
    assert_movement_locked(&mission);
    assert(mission.reason == MISSION_REASON_ESTOP);
    event(&mission, "RESET", 3);
    revision = mission.revision;
    event(&mission, "RESET", 4);
    mission_tick(&mission, false, false, 5);
    assert(mission.state == MISSION_IDLE && mission.revision == revision);

    // All six public names and acknowledgements are stable wire strings.
    const char *states[] = {"IDLE", "READY", "EXPLORING", "PAUSED", "COMPLETE", "FAULT"};
    for (unsigned i = 0; i < 6; ++i)
        assert(strcmp(mission_state_name((mission_state_t)i), states[i]) == 0);
    assert(strcmp(mission_result_name(MISSION_REJECTED_SAFETY), "SAFETY_LOCKED") == 0);
    puts("Mission control tests passed");
    return 0;
}
