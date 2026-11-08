#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "onboard_drive.h"
#include "motor_driver.h"
#include "motor_lease.h"

static mission_t mission;
static rover_map_t map;
static target_selector_t selection;
static onboard_drive_t drive;
static onboard_drive_inputs_t inputs;
static camera_observation_t camera;
static uint32_t now;
static unsigned starts, stops, renews;
static unsigned reverse_starts;
static int last_speed;
static motor_error_t start_result;
static motor_auto_status_t status;
static bool renew_ok;
static motor_lease_t lease;

motor_error_t motor_auto_drive_leased(int speed, uint16_t lease_ms)
{
    assert(speed && speed >= -100 && speed <= 100 && lease_ms == ONBOARD_DRIVE_LEASE_MS);
    last_speed=speed;
    if (speed<0) reverse_starts++;
    starts++;
    if (start_result == MOTOR_OK) {
        status = MOTOR_AUTO_RUNNING;
        lease = (motor_lease_t){.duration_ms = lease_ms, .refreshed_ms = now};
    }
    return start_result;
}
motor_error_t motor_emergency_stop(void) { stops++; status = MOTOR_AUTO_STOPPED; return MOTOR_OK; }
motor_auto_status_t motor_auto_status(void) { return status; }
bool motor_auto_renew_lease(void)
{
    renews++;
    if (!renew_ok || motor_lease_expired(&lease, now)) return false;
    lease.refreshed_ms = now;
    return true;
}

static void setup(void)
{
    now = 1000; starts = stops = renews = 0;
    reverse_starts=0; last_speed=0;
    start_result = MOTOR_OK; status = MOTOR_AUTO_IDLE; renew_ok = true;
    mission_init(&mission, now); rover_map_init(&map, 123);
    assert(rover_map_start(&map, 7) == MAP_OK);
    target_selector_init(&selection); onboard_drive_init(&drive);
    camera = (camera_observation_t){.generation = 1, .received_ms = now,
        .frame = {.seq = 250, .status = VISION_STATUS_OK, .age_ms = 100, .count = 2,
            .markers = {{.id = 2, .range_mm = 480, .bearing_cdeg = 100},
                        {.id = 9, .range_mm = 900, .bearing_cdeg = 150}}}};
    target_selector_tick(&selection, &mission, &map, &camera, true, now);
    for (int i = 0; i < 3; ++i) {
        now += 200; camera.received_ms = now; camera.generation++; camera.frame.seq++;
        target_selector_tick(&selection, &mission, &map, &camera, true, now);
    }
    inputs = (onboard_drive_inputs_t){.stopped = true, .link_ok = true,
        .sensor_valid = true, .imu_ok = true, .camera_fresh = true,
        .front_mm = 1023, .sensor_received_ms = now, .camera_present = true, .camera_status = VISION_STATUS_OK,
        .camera_image_age_ms = 100};
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
}
static const char *start(void) { return onboard_drive_start(&drive, &mission, &map, &selection, &inputs, now); }
static void tick(uint32_t elapsed)
{
    now += elapsed; inputs.sensor_received_ms = now;
    selection.sample_generation++; selection.sample_received_ms = now; selection.sample_age_ms = 100;
    onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
}
static void assert_start_refused(const char *code)
{
    rover_map_t before = map;
    assert(strcmp(start(), code) == 0);
    assert(!drive.active && !mission_motion_enabled(&mission));
    assert(memcmp(&before, &map, sizeof(map)) == 0);
}

static void test_start_and_drive_preserve_locked_leg(void)
{
    setup();
    assert(!strcmp(start(), "STARTED"));
    assert(starts == 1 && drive.active && mission_motion_enabled(&mission));
    assert(mission.state == MISSION_EXPLORING && mission.leg.phase == LEG_APPROACHING);
    char leg_json[384];
    assert(travel_leg_format_json(&mission.leg, leg_json, sizeof(leg_json)));
    printf("MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":true,\"revision\":%lu,\"leg\":%s}\n",
        (unsigned long)mission.revision, leg_json);
    assert(map.current_node == 0 && map.count == 1 && map.nodes[0].visited);
    uint32_t started = drive.started_ms;
    assert(!strcmp(start(), "ALREADY_RUNNING") && starts == 1 && drive.started_ms == started);
    assert(!mission_command_allowed(&mission, "FORWARD"));
    assert(mission_command_allowed(&mission, "STOP"));
    now += 100; camera.received_ms = now; camera.generation++; camera.frame.seq++;
    target_selector_tick(&selection, &mission, &map, &camera, false, now);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    assert(mission.leg.departure_node == 7 && mission.leg.phase == LEG_APPROACHING && selection.available);
    inputs.sensor_received_ms = now;
    onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(renews == 1 && drive.active);
    // Safety stop does not mark an obstacle as an arrival.
    inputs.front_urgent = true; tick(20);
    assert(!drive.active && mission.state == MISSION_PAUSED && mission.reason == MISSION_REASON_FRONT_STOP);
    assert(map.current_node == 0 && map.count == 1 && !mission.leg.next_target_node);
    assert(stops == 1);
    tick(100); assert(stops == 1); // no repeated output/restart after cancellation
    assert_start_refused("CURRENT_NODE_UNKNOWN");
}

static void test_refusals_have_no_map_or_motion_side_effects(void)
{
    setup(); rover_map_init(&map, 123); assert_start_refused("START_NODE_NOT_SET"); assert(!starts);
    printf("DFS_REFUSED:%s\n", start());
    setup(); rover_map_depart(&map); assert_start_refused("CURRENT_NODE_UNKNOWN");
    setup(); mission.test_mode = true; assert_start_refused("MISSION_NOT_IDLE");
    setup(); mission.state = MISSION_PAUSED; assert_start_refused("MISSION_NOT_IDLE");
    setup(); inputs.stopped = false; assert_start_refused("MOTOR_BUSY");
    setup(); mission_clear_leg(&mission, now); assert_start_refused("NO_LOCKED_TARGET");
    setup(); inputs.safety_latched = true; assert_start_refused("SAFETY_LOCKED");
    setup(); inputs.link_ok = false; assert_start_refused("LINK_LOST");
    setup(); inputs.sensor_valid = false; assert_start_refused("SENSOR_STALE");
    setup(); inputs.sensor_received_ms = now - 601; assert_start_refused("SENSOR_STALE");
    setup(); inputs.imu_ok = false; assert_start_refused("HEADING_UNAVAILABLE");
    setup(); inputs.front_fault = true; assert_start_refused("FRONT_SENSOR_FAULT");
    setup(); inputs.front_mm = 149; assert_start_refused("FRONT_BLOCKED");
    setup(); inputs.front_urgent = true; assert_start_refused("FRONT_BLOCKED");
    setup(); inputs.camera_fresh = false; assert_start_refused("CAMERA_DATA_UNUSABLE");
    setup(); inputs.camera_present = false; assert_start_refused("CAMERA_UART_MISSING");
    setup(); inputs.camera_status = VISION_STATUS_NO_MARKER; assert_start_refused("CAMERA_NO_MARKERS");
    setup(); inputs.camera_status = VISION_STATUS_STALLED; assert_start_refused("CAMERA_PROCESSING_STALLED");
    setup(); inputs.camera_status = VISION_STATUS_CAMERA_LOST; assert_start_refused("CAMERA_LOST");
    setup(); inputs.camera_packet_age_ms = 301; assert_start_refused("CAMERA_PACKET_STALE");
    setup(); inputs.camera_image_age_ms = 751; assert_start_refused("CAMERA_IMAGE_STALE");
    setup(); selection.sample_received_ms = now - 301; assert_start_refused("CAMERA_ACCEPTED_IMAGE_STALE");
    setup(); selection.available = false; assert_start_refused("TARGET_NOT_VISIBLE_AT_START");
    setup(); selection.bearing_cdeg = 501; assert_start_refused("TARGET_NOT_ALIGNED");
    setup(); selection.range_mm = 300; assert_start_refused("TARGET_TOO_CLOSE");
    setup(); start_result = MOTOR_ERR_NO_HEADING; assert_start_refused("HEADING_UNAVAILABLE");
    setup(); start_result = MOTOR_ERR_SAFETY; assert_start_refused("MOTOR_REFUSED");
    setup(); start_result = MOTOR_ERR_FRONT_BLOCKED; assert_start_refused("FRONT_BLOCKED");
}

static void test_stop_limits_and_lost_inputs(void)
{
    setup(); assert(!strcmp(start(), "STARTED")); selection.range_mm = 300; tick(20);
    assert(!drive.active && !strcmp(drive.result, "NEAR_TARGET_UNCONFIRMED") && map.current_node == 0);
    assert(mission.state == MISSION_PAUSED && mission.reason == MISSION_REASON_APPROACH_LIMIT);
    setup(); assert(!strcmp(start(), "STARTED")); tick(ONBOARD_DRIVE_MAX_MS);
    assert(!drive.active && !strcmp(drive.result, "DRIVE_TIME_LIMIT"));
    setup(); assert(!strcmp(start(), "STARTED")); inputs.camera_fresh = false; tick(20);
    assert(drive.active && mission.leg.phase == LEG_HOLDING && !mission_motion_enabled(&mission));
    tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(!drive.active && mission.reason == MISSION_REASON_INPUT_LOST);
    setup(); assert(!strcmp(start(), "STARTED")); inputs.link_ok = false; tick(20);
    assert(!drive.active && mission.reason == MISSION_REASON_LINK_LOST);
    setup(); assert(!strcmp(start(), "STARTED")); inputs.sensor_valid = false; tick(20);
    assert(!drive.active && map.current_node == 0);
    setup(); assert(!strcmp(start(), "STARTED")); selection.bearing_cdeg = -501; tick(20);
    assert(!drive.active && !strcmp(drive.result, "TARGET_NOT_ALIGNED"));
    setup(); assert(!strcmp(start(), "STARTED")); status = MOTOR_AUTO_STOPPED; tick(20);
    assert(!drive.active && mission.reason == MISSION_REASON_MOTOR_STOPPED);
    setup(); assert(!strcmp(start(), "STARTED")); renew_ok = false; tick(100);
    assert(!drive.active && !strcmp(drive.result, "DRIVE_LEASE_EXPIRED"));
    setup(); assert(!strcmp(start(), "STARTED")); mission_pause(&mission, MISSION_REASON_OPERATOR_STOP, now); tick(20);
    assert(!drive.active && stops == 1 && map.current_node == 0);
    setup(); assert(!strcmp(start(), "STARTED"));
    onboard_drive_stop(&drive, &mission, MISSION_REASON_OPERATOR_STOP, "OPERATOR_STOP", now);
    assert(!drive.active && map.current_node == 0);
}

static void test_reference_can_be_added_during_approach(void)
{
    setup(); mission.leg.arrival_reference_node = 0; selection.reference_marker = 0;
    selection.reference_candidate = selection.reference_confirmations = 0;
    assert(!strcmp(start(), "STARTED"));
    for (int i = 0; i < 3; ++i) {
        now += 100; camera.received_ms = now; camera.generation++; camera.frame.seq++;
        target_selector_tick(&selection, &mission, &map, &camera, false, now);
        inputs.sensor_received_ms = now;
        onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    }
    assert(drive.active && mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    assert(mission.leg.phase == LEG_APPROACHING && map.current_node == 0 && map.count == 1);
}

static void test_healthy_no_return_is_not_a_fault_or_an_obstacle(void)
{
    setup(); inputs.front_mm = 0;
    assert(!strcmp(start(), "STARTED"));
    tick(100);
    assert(drive.active && renews == 1 && mission_motion_enabled(&mission));
    assert(map.current_node == 0 && map.count == 1); // Not proof of arrival or a clear mapped exit.
    inputs.front_mm = 100; tick(20);
    assert(!drive.active && !strcmp(drive.result, "FRONT_BLOCKED"));
    assert(map.current_node == 0 && map.count == 1);

    setup(); inputs.front_mm = 0; inputs.front_fault = true;
    assert_start_refused("FRONT_SENSOR_FAULT");
    assert(!starts);
    setup(); inputs.front_mm = 0; inputs.sensor_received_ms = now - 601;
    assert_start_refused("SENSOR_STALE");
    assert(!starts);
    setup(); inputs.front_mm = 0;
    assert(!strcmp(start(), "STARTED"));
    inputs.front_fault = true; tick(20);
    assert(!drive.active && !strcmp(drive.result, "FRONT_SENSOR_FAULT"));
    setup(); inputs.front_mm = 0;
    assert(!strcmp(start(), "STARTED"));
    inputs.front_urgent = true; tick(20);
    assert(!drive.active && !strcmp(drive.result, "FRONT_BLOCKED"));
    setup(); inputs.front_mm = 0;
    assert(!strcmp(start(), "STARTED"));
    inputs.sensor_valid = false; tick(20);
    assert(!drive.active && !strcmp(drive.result, "SENSOR_STALE"));
    setup(); inputs.front_mm = 0;
    assert(!strcmp(start(), "STARTED"));
    selection.range_mm = 300; tick(20);
    assert(!drive.active && !strcmp(drive.result, "NEAR_TARGET_UNCONFIRMED"));
}

static void test_lease_and_clock_wrap(void)
{
    motor_lease_t l = {.refreshed_ms = UINT32_MAX - 100, .duration_ms = 250};
    assert(!motor_lease_expired(&l, 149));
    assert(motor_lease_expired(&l, 150));
    l.duration_ms = 0; assert(!motor_lease_expired(&l, 999));
    setup(); now = UINT32_MAX - 50; inputs.sensor_received_ms = now;
    selection.sample_received_ms = now;
    assert(!strcmp(start(), "STARTED")); tick(100); assert(drive.active && renews == 1);
    tick(251); assert(!drive.active && !strcmp(drive.result, "DRIVE_LEASE_EXPIRED"));
}

static void observe_reference(uint16_t range, uint32_t elapsed, uint16_t age)
{
    now += elapsed; inputs.sensor_received_ms = now;
    selection.available = false;
    selection.reference_available = true;
    selection.reference_range_mm = range;
    selection.sample_generation++;
    selection.sample_received_ms = now;
    selection.sample_age_ms = age;
    onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
}

static void start_500_mm_leg(void)
{
    setup(); selection.range_mm = 500;
    assert(!strcmp(start(), "STARTED"));
    assert(drive.reference_model_valid && drive.target_anchor_mm == 500 && drive.reference_anchor_mm == 900);
}

static void brake_near_target(void)
{
    observe_reference(800, 100, 0);
    assert(drive.active && drive.using_reference && drive.remaining_mm == 400);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    observe_reference(600, 100, 0);
    assert(drive.active && drive.remaining_mm == 200 && map.current_node == 0);
    observe_reference(450, 100, 0);
    assert(drive.active && mission.leg.phase == LEG_BRAKING && !mission_motion_enabled(&mission));
    assert(stops == 1 && map.current_node == 0 && map.count == 1);
}

static void test_reference_progress_brake_verify_and_commit_once(void)
{
    start_500_mm_leg(); brake_near_target();
    unsigned refreshes = renews;
    observe_reference(420, 100, 0);
    assert(!drive.verification_frames && mission.leg.phase == LEG_BRAKING);
    observe_reference(400, 200, 0);
    assert(drive.verification_frames == 1 && mission.leg.phase == LEG_VERIFYING);
    assert(!mission_command_allowed(&mission, "FORWARD"));
    assert(!strcmp(start(), "ALREADY_RUNNING")); // Must not restart a braked leg.
    for (unsigned i = 0; i < 10; ++i)
        onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(drive.verification_frames == 1 && map.current_node == 0);
    observe_reference(400, 100, 0);
    assert(drive.verification_frames == 2 && map.current_node == 0);
    observe_reference(400, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "ARRIVED"));
    assert(mission.state == MISSION_READY && mission.reason == MISSION_REASON_TARGET_ARRIVED);
    assert(mission.leg.phase == LEG_COMPLETE && mission.leg.next_target_node == 2);
    assert(map.current_node == 2 && map.count == 2 && rover_map_find_id(&map, 2)->visited);
    assert(rover_map_find_id(&map, 2)->x == 0 && rover_map_find_id(&map, 2)->y == 1);
    assert(rover_map_find_id(&map, 7)->exits[MAP_NORTH] == MAP_EXIT_TRAVERSED);
    assert(rover_map_find_id(&map, 2)->exits[MAP_SOUTH] == MAP_EXIT_TRAVERSED);
    assert(starts == 1 && stops == 1 && renews == refreshes);
    target_selector_tick(&selection, &mission, &map, &camera, true, now);
    assert(mission.leg.phase == LEG_COMPLETE && mission.leg.next_target_node == 2 && map.current_node == 2);
    uint32_t revision = map.revision;
    tick(100);
    assert(map.revision == revision && map.current_node == 2);
    char json[768];
    assert(onboard_drive_format(&drive, &mission, json, sizeof(json)));
    printf("TRACKING_JSON:%s\n", json);
    assert(rover_map_format(&map, "", json, sizeof(json)));
    printf("ARRIVAL_MAP_JSON:%s\n", json);
}

static void test_reference_failures_do_not_commit_arrival(void)
{
    start_500_mm_leg(); observe_reference(500, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "REFERENCE_RANGE_INCONSISTENT") && map.count == 1);
    start_500_mm_leg(); observe_reference(970, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "REFERENCE_RANGE_INCONSISTENT"));
    start_500_mm_leg(); selection.reference_marker = 3; observe_reference(800, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "TRACKING_ID_CHANGED"));
    start_500_mm_leg(); selection.marker = 9; observe_reference(800, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "TRACKING_ID_CHANGED"));
    start_500_mm_leg(); selection.reference_bearing_cdeg = 1001; observe_reference(800, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "REFERENCE_NOT_ALIGNED"));
    start_500_mm_leg();
    selection.available = selection.reference_available = false; tick(100);
    assert(drive.active && mission.leg.phase == LEG_HOLDING && map.current_node == 0);
    tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(!drive.active && !strcmp(drive.result, "CAMERA_RECOVERY_TIMEOUT"));
    setup(); mission.leg.arrival_reference_node = 0; selection.reference_marker = 0;
    assert(!strcmp(start(), "STARTED"));
    observe_reference(800, 100, 0);
    assert(drive.active && mission.leg.phase == LEG_HOLDING && !drive.reference_model_valid);
    observe_reference(800, ONBOARD_DRIVE_CAMERA_RECOVERY_MS, 0);
    assert(!drive.active && !strcmp(drive.result, "CAMERA_RECOVERY_TIMEOUT"));
    start_500_mm_leg(); observe_reference(700, 100, 0); observe_reference(500, 100, 0); observe_reference(250, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "TARGET_OVERSHOOT_UNCONFIRMED") && map.current_node == 0);
    start_500_mm_leg(); observe_reference(800, 100, 751);
    assert(drive.active && mission.leg.phase == LEG_HOLDING && !strcmp(drive.input_reason, "CAMERA_IMAGE_STALE"));
    observe_reference(800, ONBOARD_DRIVE_CAMERA_RECOVERY_MS, 751);
    assert(!drive.active && !strcmp(drive.result, "CAMERA_RECOVERY_TIMEOUT"));
    start_500_mm_leg(); brake_near_target();
    observe_reference(650, 400, 0); // Reference moves backwards beyond noise allowance.
    assert(!drive.active && map.count == 1 && map.current_node == 0);
    start_500_mm_leg(); brake_near_target();
    onboard_drive_stop(&drive, &mission, MISSION_REASON_OPERATOR_STOP, "OPERATOR_STOP", now);
    observe_reference(400, 500, 0); observe_reference(400, 100, 0); observe_reference(400, 100, 0);
    assert(!drive.active && map.count == 1 && map.current_node == 0);
}

static void test_verification_requires_post_settle_images_and_consistent_map(void)
{
    start_500_mm_leg(); brake_near_target();
    observe_reference(400, 400, 500); // Received after stopping; image still predates settling.
    assert(drive.active && !drive.verification_frames && map.count == 1);
    observe_reference(400, 400, 500);
    assert(drive.verification_frames == 1);
    observe_reference(400, 100, 500); observe_reference(400, 100, 500);
    assert(!drive.active && map.current_node == 2);
    start_500_mm_leg(); brake_near_target();
    observe_reference(400, ONBOARD_DRIVE_VERIFY_TIMEOUT_MS, 0);
    assert(!drive.active && !strcmp(drive.result, "ARRIVAL_VERIFICATION_TIMEOUT") && map.count == 1);
    start_500_mm_leg(); brake_near_target();
    assert(rover_map_set_exit(&map, 7, MAP_EAST, MAP_EXIT_OPEN) == MAP_OK);
    observe_reference(400, 400, 0);
    assert(!drive.active && !strcmp(drive.result, "MAP_CHANGED_DURING_LEG") && map.current_node == 0);
    start_500_mm_leg(); brake_near_target();
    inputs.front_urgent = true; observe_reference(400, 400, 0);
    assert(!drive.active && map.current_node == 0 && map.count == 1);
    start_500_mm_leg(); brake_near_target();
    mission_tick(&mission, false, false, now + 20); // Link loss must cancel even with wheels stopped for verification.
    observe_reference(400, 400, 0);
    assert(!drive.active && !strcmp(drive.result, "LINK_LOST") && map.current_node == 0 && map.count == 1);
}

static void test_grid_heading_planning(void)
{
    const int x[] = {0,1,0,-1}, y[] = {1,0,-1,0};
    for (unsigned d = 0; d < 4; ++d) {
        setup(); drive.frame_valid = true; drive.north_heading_deg = 359.0f;
        inputs.heading_deg = d ? (float)(90u*d - 1u) : 359.0f;
        assert(!strcmp(start(), "STARTED"));
        assert(mission.leg.direction == (leg_direction_t)d && mission.leg.target_cell_valid);
        assert(mission.leg.target_x == x[d] && mission.leg.target_y == y[d]);
    }
    setup(); drive.frame_valid = true; drive.north_heading_deg = 0; inputs.heading_deg = 30;
    assert_start_refused("GRID_DIRECTION_NOT_ALIGNED");
    setup(); drive.frame_valid = true; drive.north_heading_deg = 0;
    assert(rover_map_add(&map, 9, 0, 1, false) == MAP_OK);
    assert_start_refused("TARGET_MAP_CONFLICT");
    setup(); map.nodes[0].y = INT16_MAX;
    assert_start_refused("TARGET_CELL_OVERFLOW");
    setup(); inputs.heading_deg = -1;
    assert_start_refused("HEADING_UNAVAILABLE");
    setup(); drive.frame_valid = true;
    for (unsigned id = 10; map.count < ROVER_MAP_CAPACITY; ++id)
        assert(rover_map_add(&map, (uint8_t)id, (int16_t)(id-9), 0, false) == MAP_OK);
    assert_start_refused("MAP_FULL");
}

static void test_last_measured_distance_is_not_hardcoded_to_500(void)
{
    setup(); selection.range_mm = 620; selection.reference_range_mm = 1000;
    assert(!strcmp(start(), "STARTED"));
    observe_reference(800, 100, 0); assert(drive.remaining_mm == 420 && drive.active);
    observe_reference(600, 100, 0); assert(drive.remaining_mm == 220 && drive.active);
    observe_reference(420, 100, 0); assert(mission.leg.phase == LEG_BRAKING);
    observe_reference(380, 400, 0); observe_reference(380, 100, 0); observe_reference(380, 100, 0);
    assert(!drive.active && map.current_node == 2 && drive.reference_progress_mm == 620);
    char tiny[4];
    assert(!onboard_drive_format(&drive, &mission, tiny, sizeof(tiny)) && !tiny[0]);
    assert(!onboard_drive_format(&drive, &mission, NULL, 0));
}

static void camera_gap(void)
{
    inputs.camera_fresh = false;
    inputs.camera_packet_age_ms = 350;
    inputs.camera_image_age_ms = 880;
    selection.sample_age_ms = 530;
    now += 350; inputs.sensor_received_ms = now;
    onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(drive.active && mission.leg.phase == LEG_HOLDING && !mission_motion_enabled(&mission));
    assert(!strcmp(drive.input_reason, "CAMERA_PACKET_STALE") && stops == 1 && starts == 1);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9 && drive.reference_model_valid);
}

static void healthy_camera(void)
{
    inputs.camera_fresh = true;
    inputs.camera_packet_age_ms = 0;
    inputs.camera_image_age_ms = 100;
    inputs.camera_status = VISION_STATUS_OK;
}

static void test_camera_gap_brakes_retains_leg_and_resumes_on_distinct_images(void)
{
    start_500_mm_leg(); observe_reference(840, 100, 100);
    assert(drive.reference_progress_mm == 60); // Reproduces the reported progress before the camera stop.
    uint32_t id = mission.leg.leg_id;
    camera_gap();
    uint32_t driven = drive.driving_ms;
    char json[768];
    assert(onboard_drive_format(&drive, &mission, json, sizeof(json)));
    printf("HOLD_TRACKING_JSON:%s\n", json);
    healthy_camera(); observe_reference(820, 400, 100);
    assert(drive.recovery_frames == 1 && mission.leg.phase == LEG_HOLDING && starts == 1);
    for (int i = 0; i < 5; ++i) onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(drive.recovery_frames == 1 && starts == 1); // Same image never resumes the drive.
    observe_reference(800, 100, 100);
    assert(mission.leg.phase == LEG_APPROACHING && starts == 2 && mission_motion_enabled(&mission));
    assert(!strcmp(drive.result, "RESUMED_AFTER_CAMERA_GAP") && drive.driving_ms == driven);
    assert(mission.leg.leg_id == id && mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    observe_reference(600, 100, 100); observe_reference(450, 100, 100);
    assert(mission.leg.phase == LEG_BRAKING && map.current_node == 0);
    observe_reference(400, 400, 0); observe_reference(400, 100, 0); observe_reference(400, 100, 0);
    assert(!drive.active && map.current_node == 2 && map.count == 2 && starts == 2);
}

static void test_camera_hold_timeout_stop_and_fault_cannot_resume(void)
{
    start_500_mm_leg(); camera_gap(); tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(!drive.active && !strcmp(drive.result, "CAMERA_RECOVERY_TIMEOUT"));
    assert(!strcmp(drive.input_reason, "CAMERA_PACKET_STALE") && map.current_node == 0 && map.count == 1);
    healthy_camera(); observe_reference(800, 100, 0); observe_reference(800, 100, 0);
    assert(starts == 1 && !drive.active); // Recovery after timeout cannot restart a cancelled leg.
    start_500_mm_leg(); camera_gap();
    onboard_drive_stop(&drive, &mission, MISSION_REASON_OPERATOR_STOP, "OPERATOR_STOP", now);
    healthy_camera(); observe_reference(800, 100, 0); observe_reference(800, 100, 0);
    assert(starts == 1 && !drive.active && map.current_node == 0);
    start_500_mm_leg(); camera_gap(); inputs.front_urgent = true; tick(20);
    assert(!drive.active && !strcmp(drive.result, "FRONT_BLOCKED") && map.current_node == 0);
    start_500_mm_leg(); camera_gap(); inputs.sensor_valid = false; tick(20);
    assert(!drive.active && !strcmp(drive.result, "SENSOR_STALE"));
    start_500_mm_leg(); camera_gap(); healthy_camera(); drive.started_ms = now - ONBOARD_DRIVE_MAX_WALL_MS;
    observe_reference(800, 10, 0);
    assert(!drive.active && !strcmp(drive.result, "LEG_TIME_LIMIT") && starts == 1);
}

static void test_selector_keeps_locked_reference_while_holding(void)
{
    start_500_mm_leg(); camera_gap();
    now += 100; camera.received_ms = now; camera.generation++; camera.frame.seq++;
    camera.frame.status = VISION_STATUS_NO_MARKER; camera.frame.count = 0;
    target_selector_tick(&selection, &mission, &map, &camera, true, now);
    assert(mission.leg.phase == LEG_HOLDING && mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    healthy_camera(); camera.frame.status = VISION_STATUS_OK; camera.frame.count = 1;
    camera.frame.markers[0] = (camera_marker_t){.id = 9, .range_mm = 840, .bearing_cdeg = 150};
    for (int i = 0; i < 2; ++i) {
        now += 100; camera.received_ms = now; camera.generation++; camera.frame.seq++;
        target_selector_tick(&selection, &mission, &map, &camera, true, now);
        inputs.sensor_received_ms = now;
        onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    }
    assert(drive.active && mission.leg.phase == LEG_APPROACHING && starts == 2 && drive.using_reference);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
}

static void test_camera_gaps_during_verification_never_restart_wheels(void)
{
    start_500_mm_leg(); brake_near_target(); observe_reference(400, 400, 0);
    assert(drive.verification_frames == 1);
    inputs.camera_status = VISION_STATUS_NO_MARKER; inputs.camera_fresh = false; tick(100);
    assert(drive.active && mission.leg.phase == LEG_VERIFYING && drive.verification_frames == 1 && starts == 1);
    assert(!strcmp(drive.input_reason, "CAMERA_NO_MARKERS"));
    healthy_camera(); observe_reference(400, 100, 0); observe_reference(400, 100, 0); observe_reference(400, 100, 0);
    assert(!drive.active && map.current_node == 2 && starts == 1 && stops == 1);
}

static void test_recovery_ignores_delayed_pre_brake_images(void)
{
    start_500_mm_leg(); camera_gap(); healthy_camera(); inputs.camera_image_age_ms = 500;
    observe_reference(840, 100, 500);
    assert(mission.leg.phase == LEG_HOLDING && !drive.recovery_frames && starts == 1);
    assert(!strcmp(drive.result, "WAITING_FOR_POST_HOLD_IMAGE"));
    observe_reference(840, 400, 500);
    assert(drive.recovery_frames == 1 && starts == 1);
    observe_reference(840, 100, 500);
    assert(mission.leg.phase == LEG_APPROACHING && starts == 2);
}

static void expire_cached_image(void)
{
    now += 301; inputs.sensor_received_ms = now;
    inputs.camera_fresh = false; inputs.camera_packet_age_ms = 301;
    inputs.camera_image_age_ms = now - selection.sample_received_ms + selection.sample_age_ms;
    onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
}

static void test_verification_keeps_good_samples_across_cached_frame_expiry(void)
{
    start_500_mm_leg(); brake_near_target();
    observe_reference(400, 400, 100);
    assert(drive.verification_frames == 1 && starts == 1);
    expire_cached_image();
    assert(drive.active && drive.verification_frames == 1 && mission.leg.phase == LEG_VERIFYING);
    healthy_camera(); observe_reference(400, 100, 100);
    assert(drive.verification_frames == 2 && starts == 1);
    expire_cached_image();
    assert(drive.active && drive.verification_frames == 2 && map.current_node == 0);
    healthy_camera(); observe_reference(400, 100, 100);
    assert(!drive.active && map.current_node == 2 && drive.verification_frames == 3 && starts == 1);

    start_500_mm_leg(); brake_near_target(); observe_reference(400, 400, 100);
    assert(drive.verification_frames == 1);
    inputs.camera_fresh = false; tick(ONBOARD_DRIVE_VERIFY_TIMEOUT_MS);
    assert(!drive.active && !strcmp(drive.result, "ARRIVAL_VERIFICATION_TIMEOUT") && map.current_node == 0);
}

static void test_recovery_keeps_good_image_across_cached_frame_expiry(void)
{
    start_500_mm_leg(); camera_gap(); healthy_camera(); observe_reference(840, 400, 100);
    assert(drive.recovery_frames == 1 && starts == 1);
    expire_cached_image();
    assert(drive.active && drive.recovery_frames == 1 && mission.leg.phase == LEG_HOLDING && starts == 1);
    healthy_camera(); observe_reference(840, 100, 100);
    assert(mission.leg.phase == LEG_APPROACHING && starts == 2 && mission.leg.next_target_node == 2);

    start_500_mm_leg(); camera_gap(); healthy_camera(); observe_reference(840, 400, 100);
    assert(drive.recovery_frames == 1);
    inputs.camera_fresh = false; tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(!drive.active && !strcmp(drive.result, "CAMERA_RECOVERY_TIMEOUT") && starts == 1);
}

static void lose_reference(void)
{
    inputs.reference_absent=true; inputs.camera_status=VISION_STATUS_NO_MARKER;
    inputs.camera_fresh=false; inputs.camera_packet_age_ms=0; inputs.camera_image_age_ms=100;
    selection.available=selection.reference_available=false;
    tick(20);
    assert(drive.active && drive.reference_recovery && mission.leg.phase==LEG_HOLDING);
    assert(!mission_motion_enabled(&mission) && map.current_node==0 && map.count==1);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
}
static void start_backup(void)
{
    lose_reference(); tick(ONBOARD_DRIVE_REFERENCE_MISSING_MS-1u);
    assert(!reverse_starts && mission.leg.phase==LEG_HOLDING);
    tick(1);
    assert(reverse_starts==1 && drive.backup_used && mission.leg.phase==LEG_BACKING_UP);
    assert(last_speed==-ONBOARD_DRIVE_BACKUP_PCT && mission_motion_enabled(&mission));
    assert(!motor_lease_expired(&lease,now+ONBOARD_DRIVE_BACKUP_MS-1u));
    assert(motor_lease_expired(&lease,now+ONBOARD_DRIVE_BACKUP_MS+1u));
}
static void reference_returns(void)
{
    inputs.reference_absent=false; healthy_camera();
}
static void test_missing_reference_waits_two_seconds_then_one_bounded_reverse(void)
{
    start_500_mm_leg(); brake_near_target(); observe_reference(400,400,0);
    assert(drive.verification_frames==1);
    uint32_t leg=mission.leg.leg_id;
    start_backup(); assert(!drive.verification_frames);
    char json[768]; assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
    printf("BACKUP_TRACKING_JSON:%s\n",json);
    assert(travel_leg_format_json(&mission.leg,json,sizeof(json)));
    printf("BACKUP_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":true,\"revision\":%lu,\"leg\":%s}\n",
        (unsigned long)mission.revision,json);
    unsigned renewals=renews;
    tick(ONBOARD_DRIVE_BACKUP_MS-1u); assert(mission.leg.phase==LEG_BACKING_UP);
    tick(1); assert(mission.leg.phase==LEG_REACQUIRING && !mission_motion_enabled(&mission));
    assert(renews==renewals && drive.active && map.count==1 && map.current_node==0);
    reference_returns();
    observe_reference(600,300,100); // Delayed moving/pre-settle image cannot count.
    assert(!drive.recovery_frames && mission.leg.phase==LEG_REACQUIRING);
    observe_reference(600,100,0); // Reverse range increase of 200 mm is permitted only here.
    assert(drive.recovery_frames==1 && !drive.verification_frames && starts==2);
    for(unsigned i=0;i<5;++i) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(drive.recovery_frames==1 && map.current_node==0);
    observe_reference(590,100,0);
    assert(!drive.reference_recovery && mission.leg.phase==LEG_APPROACHING && starts==3);
    assert(last_speed>0 && mission.leg.leg_id==leg && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(!strcmp(drive.result,"RESUMED_AFTER_REFERENCE_RECOVERY"));
    observe_reference(450,100,0); assert(mission.leg.phase==LEG_BRAKING && !drive.verification_frames);
    observe_reference(400,400,0); observe_reference(400,100,0); assert(map.current_node==0);
    observe_reference(400,100,0);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(reverse_starts==1);
}
static void test_reference_return_before_two_seconds_never_reverses(void)
{
    start_500_mm_leg(); brake_near_target(); lose_reference(); reference_returns();
    observe_reference(400,500,0); observe_reference(400,100,0);
    assert(!reverse_starts && mission.leg.phase==LEG_BRAKING && !drive.verification_frames && map.current_node==0);
    observe_reference(400,400,0); observe_reference(400,100,0); observe_reference(400,100,0);
    assert(!drive.active && map.current_node==2);
}
static void test_reverse_without_rear_data_recovery_deadline_and_no_retry(void)
{
    // No rear reading is supplied anywhere in this fixture/input contract.
    start_500_mm_leg();start_backup();tick(20);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==1 && map.current_node==0);
    tick(ONBOARD_DRIVE_BACKUP_MS-20u);
    assert(drive.active && mission.leg.phase==LEG_REACQUIRING && !mission_motion_enabled(&mission));
    tick(ONBOARD_DRIVE_SETTLE_MS+ONBOARD_DRIVE_REACQUIRE_MS);
    assert(!drive.active && reverse_starts==1 && !strcmp(drive.result,"REFERENCE_REACQUISITION_TIMEOUT"));
    assert(map.count==1 && map.current_node==0);
    reference_returns();observe_reference(600,100,0);observe_reference(600,100,0);assert(starts==2);
    start_500_mm_leg();observe_reference(800,100,0);start_backup();tick(ONBOARD_DRIVE_BACKUP_MS);
    reference_returns();observe_reference(900,400,0);observe_reference(890,100,0);
    assert(mission.leg.phase==LEG_APPROACHING && starts==3);
    lose_reference();tick(ONBOARD_DRIVE_REFERENCE_MISSING_MS);
    assert(!drive.active && reverse_starts==1 && !strcmp(drive.result,"REFERENCE_RECOVERY_EXHAUSTED"));
}
static void test_recovery_cannot_override_faults_cancel_or_camera_failure(void)
{
    start_500_mm_leg();start_backup();inputs.link_ok=false;tick(20);
    assert(!drive.active && !strcmp(drive.result,"LINK_LOST") && map.current_node==0);
    start_500_mm_leg();start_backup();inputs.safety_latched=true;tick(20);
    assert(!drive.active && !strcmp(drive.result,"SAFETY_LOCKED"));
    start_500_mm_leg();start_backup();inputs.sensor_valid=false;tick(20);
    assert(!drive.active && !strcmp(drive.result,"SENSOR_STALE"));
    start_500_mm_leg();start_backup();mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);tick(20);
    reference_returns();observe_reference(800,400,0);observe_reference(800,100,0);
    assert(!drive.active && starts==2 && map.count==1);
    start_500_mm_leg();lose_reference();inputs.camera_status=VISION_STATUS_STALLED;tick(ONBOARD_DRIVE_REFERENCE_MISSING_MS);
    assert(!drive.active && !reverse_starts && !strcmp(drive.result,"CAMERA_RECOVERY_TIMEOUT"));
    start_500_mm_leg();start_backup();tick(ONBOARD_DRIVE_BACKUP_MS);reference_returns();
    observe_reference(1200,400,0); // Implausible reverse jump cannot confirm/restart.
    assert(!drive.active && !strcmp(drive.result,"REFERENCE_RANGE_INCONSISTENT") && map.current_node==0);
}
static void test_backup_stops_early_on_new_reference_image(void)
{
    start_500_mm_leg();start_backup();reference_returns();observe_reference(950,40,0);
    assert(mission.leg.phase==LEG_REACQUIRING && !mission_motion_enabled(&mission) && starts==2);
    assert(!drive.recovery_frames && !drive.verification_frames && map.current_node==0);
}

static void test_brief_reference_return_restarts_missing_timer(void)
{
    start_500_mm_leg(); lose_reference(); reference_returns(); observe_reference(900,1000,0);
    assert(drive.recovery_frames==1 && !drive.reference_missing && !reverse_starts);
    lose_reference(); tick(ONBOARD_DRIVE_REFERENCE_MISSING_MS-1u);
    assert(!reverse_starts && drive.active && mission.leg.phase==LEG_HOLDING);
    tick(1); assert(reverse_starts==1 && mission.leg.phase==LEG_BACKING_UP);
}

static void test_selector_keeps_reference_locked_during_reverse_and_reacquisition(void)
{
    start_500_mm_leg();start_backup();
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;
    camera.received_ms=now;camera.generation++;camera.frame.seq++;
    target_selector_tick(&selection,&mission,&map,&camera,false,now);
    assert(mission.leg.phase==LEG_BACKING_UP && selection.state==TARGET_LOCKED);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    tick(ONBOARD_DRIVE_BACKUP_MS);
    camera.frame.status=VISION_STATUS_OK;camera.frame.count=1;
    camera.frame.markers[0]=(camera_marker_t){.id=9,.range_mm=950};
    for(unsigned i=0;i<3;++i) {
        now+=200;camera.received_ms=now;camera.generation++;camera.frame.seq++;
        target_selector_tick(&selection,&mission,&map,&camera,true,now);
    }
    assert(mission.leg.phase==LEG_REACQUIRING && selection.reference_available && selection.reference_marker==9);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node);
}

int main(void)
{
    test_start_and_drive_preserve_locked_leg();
    test_refusals_have_no_map_or_motion_side_effects();
    test_stop_limits_and_lost_inputs();
    test_reference_can_be_added_during_approach();
    test_healthy_no_return_is_not_a_fault_or_an_obstacle();
    test_lease_and_clock_wrap();
    test_reference_progress_brake_verify_and_commit_once();
    test_reference_failures_do_not_commit_arrival();
    test_verification_requires_post_settle_images_and_consistent_map();
    test_grid_heading_planning();
    test_last_measured_distance_is_not_hardcoded_to_500();
    test_camera_gap_brakes_retains_leg_and_resumes_on_distinct_images();
    test_camera_hold_timeout_stop_and_fault_cannot_resume();
    test_selector_keeps_locked_reference_while_holding();
    test_camera_gaps_during_verification_never_restart_wheels();
    test_recovery_ignores_delayed_pre_brake_images();
    test_verification_keeps_good_samples_across_cached_frame_expiry();
    test_recovery_keeps_good_image_across_cached_frame_expiry();
    test_missing_reference_waits_two_seconds_then_one_bounded_reverse();
    test_reference_return_before_two_seconds_never_reverses();
    test_reverse_without_rear_data_recovery_deadline_and_no_retry();
    test_recovery_cannot_override_faults_cancel_or_camera_failure();
    test_backup_stops_early_on_new_reference_image();
    test_brief_reference_return_restarts_missing_timer();
    test_selector_keeps_reference_locked_during_reverse_and_reacquisition();
    puts("Onboard reference progress, brake/verify, single arrival commit, bounded drive and failure tests passed");
    return 0;
}
