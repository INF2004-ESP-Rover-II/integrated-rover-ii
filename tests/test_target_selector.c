#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "target_selector.h"

static mission_t mission;
static rover_map_t map;
static target_selector_t selector;
static camera_observation_t image;
static uint32_t now;

static camera_marker_t marker(uint8_t id, uint16_t range, int16_t bearing)
{
    return (camera_marker_t){.id = id, .range_mm = range, .bearing_cdeg = bearing};
}

static void setup(void)
{
    now = 1000;
    mission_init(&mission, now);
    rover_map_init(&map, 123);
    assert(rover_map_start(&map, 7) == MAP_OK);
    target_selector_init(&selector);
    image = (camera_observation_t){.generation = 1, .received_ms = now,
        .frame = {.seq = 250, .status = VISION_STATUS_OK, .age_ms = 100, .count = 2}};
    image.frame.markers[0] = marker(2, 480, 100);
    image.frame.markers[1] = marker(9, 900, 150);
}

static void tick(bool safe)
{
    target_selector_tick(&selector, &mission, &map, &image, safe, now);
}

static void arm(void)
{
    tick(true);
    assert(mission.leg.phase == LEG_SELECTING && !mission.leg.configured);
    assert(selector.state == TARGET_WAITING && mission.leg.departure_node == 7);
}

static void frame(void)
{
    now += 200;
    image.received_ms = now;
    image.generation++;
    image.frame.seq++;
    tick(true);
}

static void confirm(void) { frame(); frame(); frame(); }

static void output(void)
{
    char status[768], leg[384];
    size_t size = target_selector_format(&selector, status, sizeof(status));
    assert(size && size == strlen(status));
    assert(travel_leg_format_json(&mission.leg, leg, sizeof(leg)));
    puts(status); puts(leg);
}

static void test_automatic_two_and_nine(void)
{
    setup();
    rover_map_t before = map;
    arm();
    frame();
    assert(selector.marker == 2 && selector.confirmations == 1);
    assert(!mission.leg.next_target_node && !mission.leg.arrival_reference_node);
    for (int i = 0; i < 20; ++i) tick(true);
    assert(selector.confirmations == 1 && selector.reference_confirmations == 1);
    frame(); frame();
    assert(selector.state == TARGET_LOCKED && mission.leg.configured);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    assert(selector.reference_marker == 9 && selector.reference_available);
    assert(mission.leg.direction == LEG_DIRECTION_UNKNOWN && !mission.leg.target_cell_valid);
    assert(memcmp(&before, &map, sizeof(map)) == 0); // no fabricated coords/visit/arrival
    output();
    uint32_t revision = mission.revision;
    image.frame.count = 4;
    image.frame.markers[2] = marker(1, 100, 0); // new closer target cannot replace 2
    image.frame.markers[3] = marker(3, 700, 110); // new nearer reference cannot replace 9
    image.frame.markers[0].range_mm = 450;
    frame();
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    assert(selector.range_mm == 450 && mission.revision == revision);
    image.frame.count = 1; frame();
    assert(selector.available && !selector.reference_available);
    assert(mission.leg.arrival_reference_node == 9 && selector.reference_marker == 9);
    output();
    image.frame.markers[0] = marker(9, 800, 150); frame();
    assert(!selector.available && selector.reference_available);
    assert(mission.leg.next_target_node == 2 && map.current_node == 7);
    now += 2000; tick(true);
    assert(!selector.available && !selector.reference_available);
    assert(selector.marker == 2 && selector.reference_marker == 9);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    assert(memcmp(&before, &map, sizeof(map)) == 0);
}

static void test_late_reference_and_ordering(void)
{
    setup(); image.frame.count = 1; arm(); confirm();
    assert(mission.leg.next_target_node == 2 && !mission.leg.arrival_reference_node);
    output();
    image.frame.count = 2;
    frame(); frame();
    assert(!mission.leg.arrival_reference_node && selector.reference_confirmations == 2);
    frame(); assert(mission.leg.arrival_reference_node == 9); output();
    setup(); image.frame.markers[0] = marker(9, 400, 0);
    image.frame.markers[1] = marker(2, 800, 0); arm(); confirm();
    assert(mission.leg.next_target_node == 9 && mission.leg.arrival_reference_node == 2); // distance, not ID
    setup(); image.frame.markers[1].bearing_cdeg = 700; arm(); confirm();
    assert(mission.leg.next_target_node == 2 && !mission.leg.arrival_reference_node); // bearing delta >5deg
    image.frame.markers[1] = marker(9, 550, 100); confirm();
    assert(!mission.leg.arrival_reference_node); // insufficient depth separation
    image.frame.markers[1] = marker(9, 580, 100); confirm();
    assert(mission.leg.arrival_reference_node == 9); // minimum gap inclusive
    setup(); image.frame.count = 1; arm(); confirm();
    image.frame.markers[0] = marker(9, 900, 150); confirm();
    assert(!mission.leg.arrival_reference_node); // no simultaneous target: depth order unknown
    setup();
    assert(rover_map_add(&map, 2, 1, 0, false) == MAP_OK);
    assert(rover_map_add(&map, 9, 2, 0, false) == MAP_OK);
    image.frame.count = 1; arm(); confirm();
    image.frame.markers[0] = marker(9, 900, 150); confirm();
    assert(mission.leg.arrival_reference_node == 9 && !selector.available); // map establishes order
}

static void test_filtering_and_sequence(void)
{
    setup(); image.frame.count = 4;
    image.frame.markers[0] = marker(7, 50, 0); // current node
    image.frame.markers[1] = marker(1, 100, 1001); // off axis
    image.frame.markers[2] = marker(3, 200, 0); image.frame.markers[2].quality = 1;
    image.frame.markers[3] = marker(2, 480, -1000);
    arm(); confirm(); assert(mission.leg.next_target_node == 2 && !mission.leg.arrival_reference_node);
    setup(); arm(); image.frame.markers[0].range_mm = 900; confirm();
    assert(!selector.marker && !mission.leg.next_target_node); // tied ranges
    setup(); image.frame.count = 1; arm();
    for (uint8_t quality = 1; quality <= 4; ++quality) {
        image.frame.markers[0].quality = quality; frame(); assert(!selector.marker);
    }
    image.frame.markers[0].quality = 0;
    image.frame.markers[0].range_mm = 0; frame(); assert(!selector.marker);
    image.frame.markers[0].range_mm = UINT16_MAX; frame(); assert(!selector.marker);
    image.frame.markers[0].range_mm = 480; frame(); assert(selector.confirmations == 1);
    image.generation++; tick(true); assert(selector.confirmations == 1); // repeated seq
    image.frame.age_ms = 1501; frame(); assert(!selector.confirmations);
    image.frame.age_ms = 100; frame(); assert(selector.confirmations == 1);
    image.frame.status = VISION_STATUS_STALLED; frame(); assert(!selector.confirmations);
    image.frame.status = VISION_STATUS_OK; frame(); assert(selector.confirmations == 1);
    image.frame.markers[0].id = 3; frame(); assert(selector.confirmations == 1);
    image.frame.seq -= 2; frame(); assert(!selector.confirmations); // backward jump
    confirm(); assert(mission.leg.next_target_node == 3);
    setup(); image.frame.seq = 254; arm(); confirm(); // 255 -> 0 -> 1
    assert(mission.leg.next_target_node == 2);
    setup(); arm(); now += 50;
    image.received_ms = now; image.generation++; image.frame.seq++; tick(true);
    assert(!selector.confirmations); // image predates automatic arm
    frame(); assert(selector.confirmations == 1);
    now += TARGET_UART_MAX_AGE_MS + 1; tick(true); assert(!selector.confirmations);
    setup(); arm(); frame();
    // A restarted sender repeating the same seq with new packets must go stale.
    for (int i = 0; i < 10; ++i) {
        now += 200; image.received_ms = now; image.generation++; tick(true);
    }
    assert(!selector.confirmations && !mission.leg.next_target_node);
}

static void test_map_geometry_and_lifecycle(void)
{
    const int x[] = {0,1,0,-1}, y[] = {1,0,-1,0};
    for (unsigned d = 0; d < 4; ++d) {
        setup();
        assert(rover_map_add(&map, 2, (int16_t)x[d], (int16_t)y[d], false) == MAP_OK);
        assert(rover_map_add(&map, 9, (int16_t)(2*x[d]), (int16_t)(2*y[d]), false) == MAP_OK);
        rover_map_t before = map;
        arm(); confirm();
        assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
        assert(mission.leg.direction == (leg_direction_t)d && mission.leg.target_cell_valid);
        assert(mission.leg.target_x == x[d] && mission.leg.target_y == y[d]);
        assert(memcmp(&before, &map, sizeof(map)) == 0);
        if (d == 1) output();
    }
    setup();
    assert(rover_map_add(&map, 2, 2, 0, false) == MAP_OK);
    image.frame.count = 1; arm(); confirm(); assert(!selector.marker); // not adjacent
    setup();
    assert(rover_map_add(&map, 2, 1, 0, false) == MAP_OK);
    assert(rover_map_add(&map, 9, 0, 1, false) == MAP_OK);
    arm(); confirm();
    assert(mission.leg.next_target_node == 2 && !mission.leg.arrival_reference_node); // reference off grid line
    setup(); assert(rover_map_add(&map, 2, 1, 0, false) == MAP_OK);
    assert(rover_map_set_exit(&map, 7, MAP_EAST, MAP_EXIT_BLOCKED) == MAP_OK);
    image.frame.count = 1; arm(); confirm(); assert(!selector.marker);
    setup(); arm(); confirm(); tick(false);
    assert(selector.state == TARGET_CANCELLED && mission.leg.phase == LEG_NONE && map.current_node == 7);
    tick(true); confirm(); assert(mission.leg.leg_id == 2 && mission.leg.next_target_node == 2);
    mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, now); tick(true);
    assert(mission.leg.phase == LEG_SELECTING && !mission.leg.next_target_node); // automatic, fresh re-selection
    confirm(); assert(mission.leg.next_target_node == 2);
    rover_map_depart(&map); tick(true);
    assert(mission.leg.phase == LEG_NONE && !mission.leg.next_target_node && map.current_node == 0);
    setup(); mission.test_mode = true; tick(true); assert(mission.leg.phase == LEG_NONE);
    setup(); mission.state = MISSION_PAUSED; tick(true); assert(mission.leg.phase == LEG_NONE);
    setup(); mission.state = MISSION_FAULT; tick(true); assert(mission.leg.phase == LEG_NONE);
    setup(); tick(false); assert(mission.leg.phase == LEG_NONE);
    setup(); mission.leg.leg_id = UINT32_MAX; tick(true); assert(mission.leg.phase == LEG_NONE);
    setup(); now = UINT32_MAX - 100; arm(); confirm(); assert(mission.leg.next_target_node == 2);
    char tiny[4];
    assert(!target_selector_format(&selector, tiny, sizeof(tiny)) && !tiny[0]);
    assert(!target_selector_format(&selector, NULL, 0));
}

static void test_dfs_route_filters_wrong_marker_and_side_cell(void)
{
    setup();
    assert(rover_map_add(&map,2,0,1,false)==MAP_OK);
    assert(rover_map_add(&map,3,1,0,false)==MAP_OK);
    target_selector_set_route(&selector,(target_route_t){.enabled=true,.expected_id=2,.x=0,.y=1,.direction=LEG_NORTH});
    image.frame.count=3;
    image.frame.markers[0]=marker(9,100,0); // Closer unrelated unknown ID must not replace the known parent.
    image.frame.markers[1]=marker(3,200,0); // A known adjacent side cell is also wrong for the chosen route.
    image.frame.markers[2]=marker(2,500,0);
    arm();confirm();
    assert(mission.leg.next_target_node==2 && mission.leg.target_cell_valid);
    assert(mission.leg.target_x==0 && mission.leg.target_y==1);
    assert(selector.route.enabled && selector.route.expected_id==2);
}

int main(void)
{
    test_automatic_two_and_nine();
    test_late_reference_and_ordering();
    test_filtering_and_sequence();
    test_map_geometry_and_lifecycle();
    test_dfs_route_filters_wrong_marker_and_side_cell();
    puts("Automatic target/reference, late reference, freshness, locks and no-map-write tests passed");
    return 0;
}
