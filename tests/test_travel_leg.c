#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mission_control.h"
#include "rover_map.h"

static travel_leg_t fixture(void)
{
    return (travel_leg_t){.leg_id = 42, .departure_node = 7, .next_target_node = 12,
        .arrival_reference_node = 3, .target_x = 1, .target_y = 0, .target_cell_valid = true,
        .configured = true, .direction = LEG_EAST, .phase = LEG_CONFIGURED};
}

static void output(const travel_leg_t *leg)
{
    char json[384], status[256];
    size_t json_length = travel_leg_format_json(leg, json, sizeof(json));
    size_t status_length = travel_leg_format_status(leg, status, sizeof(status));
    assert(json_length && json_length == strlen(json));
    assert(status_length && status_length == strlen(status));
    puts(json);
    puts(status);
}

static void assert_cleared(const travel_leg_t *leg)
{
    assert(!leg->configured && leg->phase == LEG_NONE);
    assert(!leg->departure_node && !leg->next_target_node && !leg->arrival_reference_node);
    assert(!leg->target_cell_valid && leg->direction == LEG_DIRECTION_UNKNOWN);
}

int main(void)
{
    mission_t mission;
    rover_map_t map;
    mission_init(&mission, 10);
    rover_map_init(&map, 123);
    assert(rover_map_start(&map, 7) == MAP_OK);
    uint32_t map_revision = map.revision;
    assert_cleared(&mission.leg);
    assert(!mission.leg.leg_id);
    output(&mission.leg);

    // Independent reporting fixture; production selection is tested separately.
    mission.leg = fixture();
    output(&mission.leg);
    assert(mission_test_event(&mission, "RESET", false, 20) == MISSION_ACCEPTED);
    assert_cleared(&mission.leg);
    assert(mission.leg.leg_id == 42); // No reuse of a previous identity on clear.
    assert(map.current_node == 7 && map.count == 1 && map.revision == map_revision);
    output(&mission.leg);
    uint32_t revision = mission.revision;
    mission_clear_leg(&mission, 21);
    assert(mission.revision == revision); // Idempotent clear.

    mission.leg = fixture();
    mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, 30);
    assert_cleared(&mission.leg);
    assert(mission.state == MISSION_IDLE && map.current_node == 7);
    mission.leg = fixture();
    mission_fault(&mission, MISSION_REASON_ESTOP, 40);
    assert_cleared(&mission.leg);
    assert(mission.state == MISSION_FAULT && map.current_node == 7);

    travel_leg_t origin = fixture();
    origin.target_x = origin.target_y = 0;
    output(&origin); // A valid (0,0) target differs from an unset target cell.
    travel_leg_t maximum = fixture();
    maximum.leg_id = UINT32_MAX;
    maximum.departure_node = maximum.next_target_node = maximum.arrival_reference_node = UINT8_MAX;
    maximum.target_x = INT16_MIN;
    maximum.target_y = INT16_MAX;
    maximum.direction = LEG_WEST;
    maximum.phase = LEG_APPROACHING;
    output(&maximum);
    char small[8];
    assert(!travel_leg_format_json(&maximum, small, sizeof(small)) && !small[0]);
    assert(!travel_leg_format_status(&maximum, small, sizeof(small)) && !small[0]);
    assert(!travel_leg_format_json(&maximum, NULL, 0));
    assert(map.current_node == 7 && map.nodes[0].visited && map.revision == map_revision);
    printf("Travel-leg storage/reporting/reset tests passed (%u bytes)\n", (unsigned)sizeof(travel_leg_t));
    return 0;
}
