#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
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
static unsigned alignment_turns;
static int alignment_heading;
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

motor_error_t motor_auto_rotate_leased(int heading, uint16_t lease_ms)
{
    assert(heading >= 0 && heading < 360 && lease_ms == ONBOARD_DRIVE_LEASE_MS);
    alignment_turns++; alignment_heading = heading;
    if (start_result == MOTOR_OK) {
        status = MOTOR_AUTO_RUNNING;
        lease = (motor_lease_t){.duration_ms = lease_ms, .refreshed_ms = now};
    }
    return start_result;
}

static void setup(void)
{
    now = 1000; starts = stops = renews = 0;
    reverse_starts=0; last_speed=0;
    alignment_turns=0; alignment_heading=-1;
    start_result = MOTOR_OK; status = MOTOR_AUTO_IDLE; renew_ok = true;
    mission_init(&mission, now); rover_map_init(&map, 123);
    assert(rover_map_start(&map, 7) == MAP_OK);
    target_selector_init(&selection); onboard_drive_init(&drive);
    camera = (camera_observation_t){.generation = 1, .received_ms = now,
        .frame = {.seq = 250, .status = VISION_STATUS_OK, .age_ms = 100, .count = 2,
            .markers = {{.id = 2, .range_mm = 480, .bearing_cdeg = 100},
                        {.id = 9, .range_mm = 1100, .bearing_cdeg = 150}}}};
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
    setup(); selection.bearing_cdeg = 3001; assert_start_refused("TARGET_NOT_ALIGNED");
    setup(); selection.range_mm = 300; assert_start_refused("TARGET_TOO_CLOSE");
    setup(); start_result = MOTOR_ERR_NO_HEADING; assert_start_refused("HEADING_UNAVAILABLE");
    setup(); start_result = MOTOR_ERR_SAFETY; assert_start_refused("MOTOR_REFUSED");
    setup(); start_result = MOTOR_ERR_FRONT_BLOCKED; assert_start_refused("FRONT_BLOCKED");
}

static void test_stop_limits_and_lost_inputs(void)
{
    setup(); selection.reference_available=false; mission.leg.arrival_reference_node=0;
    assert(!strcmp(start(), "STARTED")); selection.range_mm = 300; tick(20);
    assert(!drive.active && !strcmp(drive.result, "NEAR_TARGET_UNCONFIRMED") && map.current_node == 0);
    assert(mission.state == MISSION_PAUSED && mission.reason == MISSION_REASON_APPROACH_LIMIT);
    setup(); assert(!strcmp(start(), "STARTED")); tick(ONBOARD_DRIVE_MAX_MS);
    assert(!drive.active && !strcmp(drive.result, "DRIVE_TIME_LIMIT"));
    setup(); assert(!strcmp(start(), "STARTED")); inputs.camera_fresh = false; tick(20);
    assert(drive.active && mission.leg.phase == LEG_WAITING_CAMERA && !mission_motion_enabled(&mission));
    tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(drive.active && drive.camera_waiting && mission.state == MISSION_EXPLORING);
    setup(); assert(!strcmp(start(), "STARTED")); inputs.link_ok = false; tick(20);
    assert(drive.active && drive.input_waiting && mission.leg.phase==LEG_WAITING_INPUT);
    setup(); assert(!strcmp(start(), "STARTED")); inputs.sensor_valid = false; tick(20);
    assert(drive.active && drive.input_waiting && map.current_node == 0);
    setup(); assert(!strcmp(start(), "STARTED")); selection.bearing_cdeg = -1001; tick(20);
    assert(drive.active && mission.state == MISSION_EXPLORING && mission.leg.phase == LEG_ALIGNING);
    assert(!strcmp(drive.result, "TARGET_NOT_ALIGNED") && !alignment_turns);
    setup(); assert(!strcmp(start(), "STARTED")); status = MOTOR_AUTO_STOPPED; tick(20);
    assert(drive.active && drive.input_waiting && mission.leg.phase==LEG_WAITING_INPUT);
    setup(); assert(!strcmp(start(), "STARTED")); renew_ok = false; tick(100);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason, "DRIVE_LEASE_EXPIRED"));
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
    assert(drive.active && mission.leg.phase==LEG_OBSTACLE_VERIFYING);
    tick(300); tick(100);
    assert(!drive.active && map.current_node==2 && map.count==2);

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
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason, "SENSOR_STALE"));
    setup(); inputs.front_mm = 0; selection.reference_available=false; mission.leg.arrival_reference_node=0;
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
    tick(251); assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason, "DRIVE_LEASE_EXPIRED"));
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
    assert(drive.reference_model_valid && drive.target_anchor_mm == 500 && drive.reference_anchor_mm == 1100);
}

static void brake_near_target(void)
{
    observe_reference(1000, 100, 0);
    assert(drive.active && drive.using_reference && drive.remaining_mm == 400);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    observe_reference(800, 100, 0);
    assert(drive.active && drive.remaining_mm == 200 && map.current_node == 0);
    observe_reference(650, 100, 0);
    assert(drive.active && mission.leg.phase == LEG_BRAKING && !mission_motion_enabled(&mission));
    assert(stops == 1 && map.current_node == 0 && map.count == 1);
}

static void test_reference_progress_brake_verify_and_commit_once(void)
{
    start_500_mm_leg(); brake_near_target();
    unsigned refreshes = renews;
    observe_reference(620, 100, 0);
    assert(!drive.verification_frames && mission.leg.phase == LEG_BRAKING);
    observe_reference(600, 200, 0);
    assert(drive.verification_frames == 1 && mission.leg.phase == LEG_VERIFYING);
    assert(!mission_command_allowed(&mission, "FORWARD"));
    assert(!strcmp(start(), "ALREADY_RUNNING")); // Must not restart a braked leg.
    for (unsigned i = 0; i < 10; ++i)
        onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(drive.verification_frames == 1 && map.current_node == 0);
    observe_reference(600, 100, 0);
    assert(drive.verification_frames == 2 && map.current_node == 0);
    observe_reference(600, 100, 0);
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
    start_500_mm_leg(); selection.reference_marker = 3; observe_reference(1000, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "TRACKING_ID_CHANGED"));
    start_500_mm_leg(); selection.marker = 9; observe_reference(1000, 100, 0);
    assert(!drive.active && !strcmp(drive.result, "TRACKING_ID_CHANGED"));
    start_500_mm_leg(); selection.reference_bearing_cdeg = 1001; observe_reference(1000, 100, 0);
    assert(drive.active && drive.aligning && drive.b_alignment_reference);
    assert(!strcmp(drive.result, "REFERENCE_NOT_ALIGNED") && !map.current_node);
    start_500_mm_leg();
    selection.available = selection.reference_available = false; tick(100);
    assert(drive.active && mission.leg.phase == LEG_HOLDING && map.current_node == 0);
    tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(drive.active && drive.reference_recovery && mission.state==MISSION_EXPLORING);
    tick(ONBOARD_DRIVE_REFERENCE_MISSING_MS-ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(mission.leg.phase==LEG_BACKING_UP && reverse_starts==1 && !map.current_node);
    start_500_mm_leg(); observe_reference(900, 100, 0); observe_reference(700, 100, 0); observe_reference(450, 100, 0);
    assert(drive.active && mission.leg.phase==LEG_BRAKING && map.current_node == 0);
    start_500_mm_leg(); observe_reference(1000, 100, 751);
    assert(drive.active && mission.leg.phase == LEG_WAITING_CAMERA && !strcmp(drive.input_reason, "CAMERA_IMAGE_STALE"));
    observe_reference(1000, ONBOARD_DRIVE_CAMERA_RECOVERY_MS, 751);
    assert(drive.active && mission.leg.phase == LEG_WAITING_CAMERA && !strcmp(drive.result, "WAITING_FOR_FRESH_CAMERA"));
    start_500_mm_leg(); brake_near_target();
    observe_reference(850, 400, 0); // Post-brake position is outside the arrival tolerance.
    assert(drive.active && drive.correcting && mission.leg.phase==LEG_CORRECTING && map.count == 1 && map.current_node == 0);
    start_500_mm_leg(); brake_near_target();
    onboard_drive_stop(&drive, &mission, MISSION_REASON_OPERATOR_STOP, "OPERATOR_STOP", now);
    observe_reference(600, 500, 0); observe_reference(600, 100, 0); observe_reference(600, 100, 0);
    assert(!drive.active && map.count == 1 && map.current_node == 0);
}

static void test_verification_requires_post_settle_images_and_consistent_map(void)
{
    start_500_mm_leg(); brake_near_target();
    observe_reference(600, 400, 500); // Received after stopping; image still predates settling.
    assert(drive.active && !drive.verification_frames && map.count == 1);
    observe_reference(600, 400, 500);
    assert(drive.verification_frames == 1);
    observe_reference(600, 100, 500); observe_reference(600, 100, 500);
    assert(!drive.active && map.current_node == 2);
    start_500_mm_leg(); brake_near_target();
    observe_reference(600, ONBOARD_DRIVE_VERIFY_TIMEOUT_MS, 0);
    assert(drive.active && mission.leg.phase==LEG_VERIFYING && map.count==1 && !map.current_node);
    start_500_mm_leg(); brake_near_target();
    assert(rover_map_set_exit(&map, 7, MAP_EAST, MAP_EXIT_OPEN) == MAP_OK);
    observe_reference(600, 400, 0);
    assert(!drive.active && !strcmp(drive.result, "MAP_CHANGED_DURING_LEG") && map.current_node == 0);
    start_500_mm_leg(); brake_near_target();
    inputs.front_urgent = true; observe_reference(600, 400, 0);
    assert(!drive.active && map.current_node == 0 && map.count == 1);
    start_500_mm_leg(); brake_near_target();
    mission_tick(&mission, false, false, now + 20); inputs.link_ok=false; // Executor retains the stopped verification leg.
    observe_reference(600, 400, 0);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason, "LINK_LOST") && map.current_node == 0 && map.count == 1);
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

static void test_reference_arrival_uses_600_mm_instead_of_last_target_range(void)
{
    setup(); selection.range_mm = 620; selection.reference_range_mm = 1000;
    assert(!strcmp(start(), "STARTED"));
    observe_reference(1000, 100, 0); assert(drive.remaining_mm == 400 && drive.active);
    observe_reference(800, 100, 0); assert(drive.remaining_mm == 200 && drive.active);
    observe_reference(620, 100, 0); assert(mission.leg.phase == LEG_BRAKING);
    observe_reference(580, 400, 0); observe_reference(580, 100, 0); observe_reference(580, 100, 0);
    assert(!drive.active && map.current_node == 2 && drive.reference_progress_mm == 420);
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
    assert(drive.active && mission.leg.phase == LEG_WAITING_CAMERA && !mission_motion_enabled(&mission));
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
    start_500_mm_leg(); observe_reference(1040, 100, 100);
    assert(drive.reference_progress_mm == 60); // Reproduces the reported progress before the camera stop.
    uint32_t id = mission.leg.leg_id;
    camera_gap();
    uint32_t driven = drive.driving_ms;
    char json[768];
    assert(onboard_drive_format(&drive, &mission, json, sizeof(json)));
    printf("HOLD_TRACKING_JSON:%s\n", json);
    healthy_camera(); observe_reference(1020, 400, 100);
    assert(drive.camera_wait_frames == 1 && mission.leg.phase == LEG_WAITING_CAMERA && starts == 1);
    for (int i = 0; i < 5; ++i) onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(drive.camera_wait_frames == 1 && starts == 1); // Same image never resumes the drive.
    observe_reference(1000, 100, 100);
    assert(mission.leg.phase == LEG_APPROACHING && starts == 2 && mission_motion_enabled(&mission));
    assert(!strcmp(drive.result, "RESUMED_AFTER_CAMERA_WAIT") && drive.driving_ms == driven);
    assert(mission.leg.leg_id == id && mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    observe_reference(800, 100, 100); observe_reference(650, 100, 100);
    assert(mission.leg.phase == LEG_BRAKING && map.current_node == 0);
    observe_reference(600, 400, 0); observe_reference(600, 100, 0); observe_reference(600, 100, 0);
    assert(!drive.active && map.current_node == 2 && map.count == 2 && starts == 2);
}

static void test_camera_hold_timeout_stop_and_fault_cannot_resume(void)
{
    start_500_mm_leg(); camera_gap(); tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(drive.active && mission.leg.phase == LEG_WAITING_CAMERA && !strcmp(drive.result, "WAITING_FOR_FRESH_CAMERA"));
    assert(!strcmp(drive.input_reason, "CAMERA_PACKET_STALE") && map.current_node == 0 && map.count == 1);
    healthy_camera(); observe_reference(1000, 100, 0); observe_reference(1000, 100, 0);
    assert(starts == 2 && drive.active); // Camera age alone no longer cancels the leg.
    start_500_mm_leg(); camera_gap();
    onboard_drive_stop(&drive, &mission, MISSION_REASON_OPERATOR_STOP, "OPERATOR_STOP", now);
    healthy_camera(); observe_reference(1000, 100, 0); observe_reference(1000, 100, 0);
    assert(starts == 1 && !drive.active && map.current_node == 0);
    start_500_mm_leg(); camera_gap(); inputs.front_urgent = true; tick(20);
    assert(!drive.active && !strcmp(drive.result, "FRONT_BLOCKED") && map.current_node == 0);
    start_500_mm_leg(); camera_gap(); inputs.sensor_valid = false; tick(20);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason, "SENSOR_STALE"));
    start_500_mm_leg(); camera_gap(); healthy_camera(); drive.started_ms = now - ONBOARD_DRIVE_MAX_WALL_MS;
    observe_reference(1000, 10, 0);
    assert(!drive.active && !strcmp(drive.result, "LEG_TIME_LIMIT") && starts == 1);
}

static void test_selector_keeps_locked_reference_while_holding(void)
{
    start_500_mm_leg(); camera_gap();
    now += 100; camera.received_ms = now; camera.generation++; camera.frame.seq++;
    camera.frame.status = VISION_STATUS_NO_MARKER; camera.frame.count = 0;
    target_selector_tick(&selection, &mission, &map, &camera, true, now);
    assert(mission.leg.phase == LEG_WAITING_CAMERA && mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    healthy_camera(); camera.frame.status = VISION_STATUS_OK; camera.frame.count = 1;
    camera.frame.markers[0] = (camera_marker_t){.id = 9, .range_mm = 1040, .bearing_cdeg = 150};
    for (int i = 0; i < 3; ++i) {
        now += 200; camera.received_ms = now; camera.generation++; camera.frame.seq++;
        target_selector_tick(&selection, &mission, &map, &camera, true, now);
        inputs.sensor_received_ms = now;
        onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    }
    assert(drive.active && mission.leg.phase == LEG_APPROACHING && starts == 2 && drive.using_reference);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
}

static void test_camera_gaps_during_verification_never_restart_wheels(void)
{
    start_500_mm_leg(); brake_near_target(); observe_reference(600, 400, 0);
    assert(drive.verification_frames == 1);
    inputs.camera_status = VISION_STATUS_NO_MARKER; inputs.camera_fresh = false; tick(100);
    assert(drive.active && mission.leg.phase == LEG_VERIFYING && drive.verification_frames == 1 && starts == 1);
    assert(!strcmp(drive.input_reason, "CAMERA_NO_MARKERS"));
    healthy_camera(); observe_reference(600, 100, 0); observe_reference(600, 100, 0); observe_reference(600, 100, 0);
    assert(!drive.active && map.current_node == 2 && starts == 1 && stops == 1);
}

static void test_recovery_ignores_delayed_pre_brake_images(void)
{
    start_500_mm_leg(); camera_gap(); healthy_camera(); inputs.camera_image_age_ms = 500;
    observe_reference(1040, 100, 500);
    assert(mission.leg.phase == LEG_WAITING_CAMERA && !drive.camera_wait_frames && starts == 1);
    assert(!strcmp(drive.result, "WAITING_FOR_POST_STOP_CAMERA"));
    observe_reference(1040, 400, 500);
    assert(!drive.camera_wait_frames && starts == 1);
    observe_reference(1040, 300, 500);
    assert(drive.camera_wait_frames == 1 && starts == 1);
    observe_reference(1040, 100, 500);
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
    observe_reference(600, 400, 100);
    assert(drive.verification_frames == 1 && starts == 1);
    expire_cached_image();
    assert(drive.active && drive.verification_frames == 1 && mission.leg.phase == LEG_WAITING_CAMERA);
    healthy_camera(); observe_reference(600, 100, 100);
    assert(drive.verification_frames == 1 && drive.camera_wait_frames == 1 && starts == 1);
    expire_cached_image();
    assert(drive.active && drive.verification_frames == 1 && drive.camera_wait_frames == 1 && map.current_node == 0);
    healthy_camera(); observe_reference(600, 100, 100);
    assert(drive.active && drive.verification_frames == 2 && starts == 1);
    observe_reference(600,100,0);
    assert(!drive.active && map.current_node == 2 && drive.verification_frames == 3 && starts == 1);

    start_500_mm_leg(); brake_near_target(); observe_reference(600, 400, 100);
    assert(drive.verification_frames == 1);
    inputs.camera_fresh = false; tick(ONBOARD_DRIVE_VERIFY_TIMEOUT_MS);
    assert(drive.active && drive.camera_waiting && !strcmp(drive.result, "WAITING_FOR_FRESH_CAMERA") && map.current_node == 0);
}

static void test_recovery_keeps_good_image_across_cached_frame_expiry(void)
{
    start_500_mm_leg(); camera_gap(); healthy_camera(); observe_reference(1040, 400, 100);
    assert(drive.camera_wait_frames == 1 && starts == 1);
    expire_cached_image();
    assert(drive.active && drive.camera_wait_frames == 1 && mission.leg.phase == LEG_WAITING_CAMERA && starts == 1);
    healthy_camera(); observe_reference(1040, 100, 100);
    assert(mission.leg.phase == LEG_APPROACHING && starts == 2 && mission.leg.next_target_node == 2);

    start_500_mm_leg(); camera_gap(); healthy_camera(); observe_reference(1040, 400, 100);
    assert(drive.camera_wait_frames == 1);
    inputs.camera_fresh = false; tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(drive.active && drive.camera_waiting && !strcmp(drive.input_reason, "CAMERA_DATA_UNUSABLE") && starts == 1);
}

static void lose_reference(void)
{
    size_t count=map.count;
    inputs.reference_absent=true; inputs.camera_status=VISION_STATUS_NO_MARKER;
    inputs.camera_fresh=false; inputs.camera_packet_age_ms=0; inputs.camera_image_age_ms=100;
    selection.available=selection.reference_available=false;
    tick(20);
    assert(drive.active && drive.reference_recovery && mission.leg.phase==LEG_HOLDING);
    assert(!mission_motion_enabled(&mission) && map.current_node==0 && map.count==count);
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
static void test_missing_reference_waits_two_seconds_then_continuous_reverse(void)
{
    start_500_mm_leg(); brake_near_target(); observe_reference(600,400,0);
    assert(drive.verification_frames==1);
    uint32_t leg=mission.leg.leg_id;
    start_backup(); assert(!drive.verification_frames);
    char json[768]; assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
    printf("BACKUP_TRACKING_JSON:%s\n",json);
    assert(travel_leg_format_json(&mission.leg,json,sizeof(json)));
    printf("BACKUP_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":true,\"revision\":%lu,\"leg\":%s}\n",
        (unsigned long)mission.revision,json);
    unsigned renewals=renews;
    for(unsigned i=0;i<5;++i) tick(100);
    assert(mission.leg.phase==LEG_BACKING_UP && renews>renewals && drive.active && !map.current_node);
    reference_returns(); observe_reference(800,100,0);
    assert(mission.leg.phase==LEG_REACQUIRING && !mission_motion_enabled(&mission));
    observe_reference(800,300,100); // Delayed moving/pre-settle image cannot count.
    assert(!drive.recovery_frames && mission.leg.phase==LEG_REACQUIRING);
    observe_reference(800,100,0); // Reverse range increase of 200 mm is permitted only here.
    assert(drive.recovery_frames==1 && !drive.verification_frames && starts==2);
    for(unsigned i=0;i<5;++i) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(drive.recovery_frames==1 && map.current_node==0);
    observe_reference(790,100,0);
    assert(!drive.reference_recovery && mission.leg.phase==LEG_APPROACHING && starts==3);
    assert(last_speed>0 && mission.leg.leg_id==leg && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(!strcmp(drive.result,"RESUMED_AFTER_REFERENCE_RECOVERY"));
    observe_reference(650,100,0); assert(mission.leg.phase==LEG_BRAKING && !drive.verification_frames);
    observe_reference(600,400,0); observe_reference(600,100,0); assert(map.current_node==0);
    observe_reference(600,100,0);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(reverse_starts==1);
}
static void test_reference_return_before_two_seconds_never_reverses(void)
{
    start_500_mm_leg(); brake_near_target(); lose_reference(); reference_returns();
    observe_reference(600,500,0); observe_reference(600,100,0);
    assert(!reverse_starts && mission.leg.phase==LEG_BRAKING && !drive.verification_frames && map.current_node==0);
    observe_reference(600,400,0); observe_reference(600,100,0); observe_reference(600,100,0);
    assert(!drive.active && map.current_node==2);
}
static void test_reverse_without_rear_data_overall_deadline_and_retry(void)
{
    // No rear reading is supplied anywhere in this fixture/input contract.
    start_500_mm_leg();start_backup();tick(20);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==1 && map.current_node==0);
    for(unsigned i=0;i<20;++i) tick(100);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && map.current_node==0);
    drive.started_ms=now-ONBOARD_DRIVE_MAX_WALL_MS-ONBOARD_DRIVE_RECOVERY_WALL_MS;
    tick(1);
    assert(!drive.active && !strcmp(drive.result,"LEG_TIME_LIMIT") && map.current_node==0);
    start_500_mm_leg();start_backup();reference_returns();observe_reference(1100,100,0);
    observe_reference(1100,400,0);observe_reference(1090,100,0);
    assert(mission.leg.phase==LEG_APPROACHING && starts==3);
    lose_reference();tick(ONBOARD_DRIVE_REFERENCE_MISSING_MS);
    assert(drive.active && reverse_starts==2 && mission.leg.phase==LEG_BACKING_UP);

}
static void test_recovery_cannot_override_faults_cancel_or_camera_failure(void)
{
    start_500_mm_leg();start_backup();inputs.link_ok=false;tick(20);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason,"LINK_LOST") && map.current_node==0);
    start_500_mm_leg();start_backup();inputs.safety_latched=true;tick(20);
    assert(!drive.active && !strcmp(drive.result,"SAFETY_LOCKED"));
    start_500_mm_leg();start_backup();inputs.sensor_valid=false;tick(20);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason,"SENSOR_STALE"));
    start_500_mm_leg();start_backup();mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);tick(20);
    reference_returns();observe_reference(1000,400,0);observe_reference(1000,100,0);
    assert(!drive.active && starts==2 && map.count==1);
    start_500_mm_leg();lose_reference();inputs.camera_status=VISION_STATUS_STALLED;tick(ONBOARD_DRIVE_REFERENCE_MISSING_MS);
    assert(drive.active && drive.camera_waiting && !reverse_starts && !strcmp(drive.input_reason,"CAMERA_PROCESSING_STALLED"));
}

static void test_large_reference_range_changes_keep_the_locked_journey(void)
{
    start_500_mm_leg();
    uint32_t leg_id=mission.leg.leg_id, revision=map.revision;
    observe_reference(700,100,0); // 400 mm closer is now accepted.
    assert(drive.active && mission.leg.phase==LEG_APPROACHING && drive.remaining_mm==100);
    observe_reference(1170,100,0); // More than 60 mm farther is accepted too.
    assert(drive.active && mission.leg.phase==LEG_APPROACHING && drive.remaining_mm==570);
    assert(mission.leg.leg_id==leg_id && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(map.revision==revision && map.current_node==0 && map.count==1);
    start_500_mm_leg(); observe_reference(600,100,0); // A large jump to arrival range only brakes.
    assert(drive.active && mission.leg.phase==LEG_BRAKING && drive.verification_frames==0);
    assert(map.current_node==0 && map.count==1);
    observe_reference(600,400,0);
    assert(drive.active && drive.verification_frames==1 && map.current_node==0);
    for (unsigned i=0;i<5;++i) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(drive.verification_frames==1 && map.count==1); // Existing distinct-frame verification remains.
    start_500_mm_leg(); observe_reference(450,100,0);
    assert(drive.active && mission.leg.phase==LEG_BRAKING && map.current_node==0);
    start_500_mm_leg();start_backup();reference_returns();observe_reference(1400,100,0);
    observe_reference(1400,400,0);
    assert(drive.active && mission.leg.phase==LEG_REACQUIRING && drive.recovery_frames==1);
    observe_reference(1400,100,0);
    assert(drive.active && mission.leg.phase==LEG_APPROACHING && starts==3);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && map.current_node==0 && map.count==1);
}
static void test_backup_stops_early_on_new_reference_image(void)
{
    start_500_mm_leg();start_backup();reference_returns();observe_reference(1150,40,0);
    assert(mission.leg.phase==LEG_REACQUIRING && !mission_motion_enabled(&mission) && starts==2);
    assert(!drive.recovery_frames && !drive.verification_frames && map.current_node==0);
}

static void test_brief_reference_return_restarts_missing_timer(void)
{
    start_500_mm_leg(); lose_reference(); reference_returns(); observe_reference(1100,1000,0);
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
    tick(100); reference_returns(); observe_reference(950,100,0);
    camera.frame.status=VISION_STATUS_OK;camera.frame.count=1;
    camera.frame.markers[0]=(camera_marker_t){.id=9,.range_mm=950};
    for(unsigned i=0;i<3;++i) {
        now+=200;camera.received_ms=now;camera.generation++;camera.frame.seq++;
        target_selector_tick(&selection,&mission,&map,&camera,true,now);
    }
    assert(mission.leg.phase==LEG_REACQUIRING && selection.reference_available && selection.reference_marker==9);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node);
}

static void reference_image_gap(uint32_t elapsed)
{
    now+=elapsed; inputs.sensor_received_ms=now;
    // Reported failure: a recent UART packet carries an old NO_MARKER image.
    // Main must not label this packet as fresh evidence of reference absence.
    inputs.reference_absent=false; inputs.camera_status=VISION_STATUS_NO_MARKER;
    inputs.camera_fresh=false; inputs.camera_packet_age_ms=43; inputs.camera_image_age_ms=1051;
    onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
}

static void test_reference_reverse_waits_for_fresh_absence_after_two_second_boundary(void)
{
    setup(); selection.range_mm=548; selection.reference_range_mm=1181;
    assert(!strcmp(start(),"STARTED"));
    observe_reference(1100,100,0); observe_reference(1000,100,0); observe_reference(800,100,0); observe_reference(653,100,0);
    assert(drive.remaining_mm==53 && mission.leg.phase==LEG_APPROACHING); // Stop boundary just before reference arrival.
    lose_reference();
    uint32_t leg=mission.leg.leg_id;
    reference_image_gap(ONBOARD_DRIVE_REFERENCE_MISSING_MS);
    assert(drive.active && drive.reference_recovery && !reverse_starts);
    assert(mission.leg.phase==LEG_WAITING_CAMERA && !mission_motion_enabled(&mission) && map.current_node==0);
    assert(!strcmp(drive.result,"WAITING_FOR_FRESH_CAMERA"));
    assert(!strcmp(drive.input_reason,"CAMERA_IMAGE_STALE"));
    char json[768]; assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
    printf("REFERENCE_FRAME_WAIT_JSON:%s\n",json);
    // A subsequent fresh no-marker image authorises the single pulse, even
    // though no accepted marker measurement has arrived since the reference loss.
    now+=250; inputs.sensor_received_ms=now;
    inputs.reference_absent=true; inputs.camera_packet_age_ms=15; inputs.camera_image_age_ms=430;
    inputs.camera_generation++;
    onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(!reverse_starts && drive.camera_wait_frames==1);
    now+=200; inputs.sensor_received_ms=now; inputs.camera_generation++;
    onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(reverse_starts==1 && mission.leg.phase==LEG_BACKING_UP && mission_motion_enabled(&mission));
    assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(!drive.verification_frames && !map.current_node && map.count==1);
    reference_returns(); observe_reference(800,100,0);
    observe_reference(800,400,0); observe_reference(800,100,0);
    assert(mission.leg.phase==LEG_APPROACHING && map.current_node==0);
    observe_reference(650,100,0); assert(mission.leg.phase==LEG_BRAKING);
    observe_reference(600,400,0); observe_reference(600,100,0); assert(map.current_node==0);
    observe_reference(600,100,0);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(drive.verification_frames==3 && reverse_starts==1);
}

static void test_reference_frame_wait_retains_leg_and_respects_cancellation(void)
{
    start_500_mm_leg(); lose_reference(); reference_image_gap(ONBOARD_DRIVE_REFERENCE_MISSING_MS);
    assert(drive.active && !reverse_starts);
    reference_image_gap(ONBOARD_DRIVE_REACQUIRE_MS);
    assert(drive.active && !reverse_starts && mission.leg.phase==LEG_WAITING_CAMERA);
    assert(mission.state==MISSION_EXPLORING && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(map.current_node==0 && map.count==1);
    reference_image_gap(ONBOARD_DRIVE_MAX_WALL_MS+ONBOARD_DRIVE_RECOVERY_WALL_MS);
    assert(drive.active && drive.camera_waiting && !reverse_starts);
    mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
    inputs.reference_absent=true;tick(100);assert(!drive.active && !reverse_starts); // Operator stop remains final.
    start_500_mm_leg(); lose_reference(); reference_image_gap(ONBOARD_DRIVE_REFERENCE_MISSING_MS);
    inputs.link_ok=false; tick(10);
    assert(drive.active && drive.input_waiting && !reverse_starts && !strcmp(drive.input_reason,"LINK_LOST"));
}

static void raw_bearing_frame(uint32_t elapsed,uint8_t status_code,uint8_t count,uint8_t id,uint16_t range,int16_t bearing)
{
    now+=elapsed;inputs.sensor_received_ms=now;
    camera.received_ms=now;camera.generation++;camera.frame.seq++;camera.frame.age_ms=0;
    camera.frame.status=status_code;camera.frame.count=count;
    camera.frame.markers[0]=(camera_marker_t){.id=id,.range_mm=range,.bearing_cdeg=bearing};
    target_selector_tick(&selection,&mission,&map,&camera,false,now);
    inputs.camera_present=true;inputs.camera_status=status_code;inputs.camera_generation=camera.generation;
    inputs.camera_packet_age_ms=inputs.camera_image_age_ms=0;inputs.camera_fresh=status_code==VISION_STATUS_OK;
    inputs.camera_seq=camera.frame.seq; inputs.recovery_marker_count=count;
    memcpy(inputs.recovery_markers,camera.frame.markers,sizeof(inputs.recovery_markers));
    inputs.reference_absent=mission.leg.arrival_reference_node!=0 && count<=VISION_MAX_MARKERS;
    for(unsigned i=0;i<count && i<VISION_MAX_MARKERS;++i) {
        const camera_marker_t *marker=&camera.frame.markers[i];
        if(marker->id==mission.leg.arrival_reference_node && !marker->quality && marker->range_mm && marker->range_mm!=UINT16_MAX)
            inputs.reference_absent=false;
    }
    onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
}
static void raw_frame(uint32_t elapsed,uint8_t status_code,uint8_t count,uint8_t id,uint16_t range)
{
    raw_bearing_frame(elapsed,status_code,count,id,range,0);
}

static void start_without_reference(void)
{
    setup();mission.leg.arrival_reference_node=0;
    selection.reference_marker=selection.reference_candidate=selection.reference_confirmations=0;
    selection.reference_available=false;selection.range_mm=496;
    assert(!strcmp(start(),"STARTED"));
    assert(!drive.reference_model_valid && drive.last_target_range_mm==496);
}

static void test_slow_empty_camera_search_late_reference_600_mm_arrival(void)
{
    start_without_reference();uint32_t leg=mission.leg.leg_id;
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && mission.leg.phase==LEG_SEARCHING_REFERENCE && last_speed==ONBOARD_DRIVE_SEARCH_PCT);
    assert(selection.allow_late_reference && drive.search_used && map.current_node==0);
    for(unsigned i=0;i<35;++i) raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && drive.search_driving_ms>ONBOARD_DRIVE_MAX_MS && !mission.leg.arrival_reference_node);
    char json[768];assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
    printf("SEARCH_TRACKING_JSON:%s\n",json);
    raw_frame(100,VISION_STATUS_OK,1,2,480);
    raw_frame(100,VISION_STATUS_OK,1,2,480); // First image after a long empty interval re-synchronises sequence timing.
    assert(!mission.leg.arrival_reference_node && drive.search_active); // Target itself is never a reference.
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    for(unsigned i=0;i<3;++i) raw_frame(100,VISION_STATUS_OK,1,9,800);
    assert(drive.active && !drive.search_active && drive.reference_model_valid && drive.remaining_mm==200);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && mission.leg.leg_id==leg);
    assert(map.current_node==0 && map.count==1 && drive.last_target_range_mm==480);
    raw_frame(100,VISION_STATUS_OK,1,9,700);raw_frame(100,VISION_STATUS_OK,1,9,650);
    assert(mission.leg.phase==LEG_BRAKING && !mission_motion_enabled(&mission));
    raw_frame(300,VISION_STATUS_OK,1,9,600);raw_frame(100,VISION_STATUS_OK,1,9,600);
    assert(map.current_node==0 && drive.verification_frames==2);
    raw_frame(100,VISION_STATUS_OK,1,9,600);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
}

static void test_front_motor_stop_confirms_terminal_target_without_camera_reference(void)
{
    start_without_reference();raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    inputs.front_mm=100;inputs.front_urgent=true;
    motor_emergency_stop(); // Same independent brake as the UART ISR.
    assert(onboard_drive_note_front_stop(&drive,&mission));
    assert(mission.leg.configured && mission.leg.next_target_node==2);
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && mission.leg.phase==LEG_OBSTACLE_VERIFYING && !mission_motion_enabled(&mission));
    assert(map.current_node==0 && !drive.verification_frames);
    raw_frame(300,VISION_STATUS_STALLED,0,0,0);
    assert(drive.verification_frames==1 && map.current_node==0);
    for(unsigned i=0;i<5;++i) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(drive.verification_frames==1);
    raw_frame(100,VISION_STATUS_STALLED,0,0,0);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(rover_map_find_id(&map,2)->exits[MAP_NORTH]==MAP_EXIT_BLOCKED);
    assert(rover_map_find_id(&map,2)->exits[MAP_SOUTH]==MAP_EXIT_TRAVERSED);
    char json[768];assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
    printf("OBSTACLE_TRACKING_JSON:%s\n",json);
}

static void test_search_clearance_fault_cancel_and_budgets_do_not_fake_arrival(void)
{
    start_without_reference();inputs.front_mm=0;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && drive.search_active && mission.leg.phase==LEG_SEARCHING_REFERENCE && map.current_node==0);
    assert(last_speed==ONBOARD_DRIVE_SEARCH_PCT && !strcmp(drive.result,"SEARCHING_WITH_NO_FRONT_RETURN"));
    assert(!strcmp(drive.input_reason,"FRONT_DISTANCE_UNKNOWN"));
    assert(rover_map_find_id(&map,7)->exits[MAP_NORTH]==MAP_EXIT_UNKNOWN && map.count==1);
    start_without_reference();inputs.front_mm=0;inputs.front_fault=true;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && !strcmp(drive.result,"FRONT_SENSOR_FAULT") && map.current_node==0);
    start_without_reference();inputs.front_mm=599;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && !strcmp(drive.result,"SEARCH_FRONT_NOT_CLEAR"));
    start_without_reference();inputs.front_mm=600;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.search_active);
    inputs.front_mm=300;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);assert(drive.search_active);
    inputs.front_mm=100;inputs.front_fault=true;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && map.current_node==0 && !strcmp(drive.result,"FRONT_SENSOR_FAULT"));
    start_without_reference();raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    inputs.safety_latched=true;inputs.front_mm=100;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && map.current_node==0 && !strcmp(drive.result,"SAFETY_LOCKED"));
    start_without_reference();raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
    assert(!onboard_drive_note_front_stop(&drive,&mission));
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);assert(!drive.active && map.current_node==0);
    start_without_reference();
    for(unsigned i=0;i<110 && drive.active;++i) raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && !strcmp(drive.result,"REFERENCE_SEARCH_LIMIT") && map.current_node==0);
}

static void test_search_camera_gap_brakes_and_recovers_on_new_empty_images(void)
{
    start_without_reference();raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    uint32_t used=drive.search_driving_ms;
    raw_frame(100,VISION_STATUS_STALLED,0,0,0);
    assert(drive.search_active && mission.leg.phase==LEG_WAITING_CAMERA && !mission_motion_enabled(&mission));
    raw_frame(60000,VISION_STATUS_STALLED,0,0,0);
    assert(drive.active && drive.camera_waiting && drive.search_driving_ms==used+100);
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.camera_waiting && drive.camera_wait_frames==1 && !mission_motion_enabled(&mission));
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(mission.leg.phase==LEG_SEARCHING_REFERENCE && drive.active && status==MOTOR_AUTO_RUNNING);
    raw_frame(100,VISION_STATUS_STALLED,0,0,0);
    raw_frame(ONBOARD_DRIVE_CAMERA_RECOVERY_MS,VISION_STATUS_STALLED,0,0,0);
    assert(drive.active && drive.camera_waiting && map.current_node==0 && !strcmp(drive.input_reason,"CAMERA_PROCESSING_STALLED"));
}

static void test_unconfirmed_front_pulse_and_manual_estop_never_commit_arrival(void)
{
    start_without_reference();raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    inputs.front_mm=100;assert(onboard_drive_note_front_stop(&drive,&mission));
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    inputs.front_mm=300;raw_frame(300,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.verification_frames && map.current_node==0);
    raw_frame(ONBOARD_DRIVE_VERIFY_TIMEOUT_MS,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && mission.leg.phase==LEG_OBSTACLE_VERIFYING && map.current_node==0);
    start_without_reference();raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    inputs.front_mm=100;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    raw_frame(300,VISION_STATUS_NO_MARKER,0,0,0);assert(drive.verification_frames==1);
    inputs.safety_latched=true;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && !strcmp(drive.result,"SAFETY_LOCKED") && map.current_node==0);
}

static void test_no_front_return_still_has_time_sensor_and_obstacle_guards(void)
{
    start_without_reference();inputs.front_mm=0;
    for(unsigned i=0;i<110 && drive.active;++i) raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && !strcmp(drive.result,"REFERENCE_SEARCH_LIMIT"));
    assert(map.count==1 && map.current_node==0 && !rover_map_find_id(&map,2));
    start_without_reference();inputs.front_mm=0;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    inputs.sensor_received_ms=now-ONBOARD_DRIVE_SENSOR_MAX_AGE_MS-1u;
    onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason,"SENSOR_STALE") && map.current_node==0);
    start_without_reference();inputs.front_mm=0;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.search_active && !strcmp(drive.input_reason,"FRONT_DISTANCE_UNKNOWN"));
    inputs.front_mm=100;raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(mission.leg.phase==LEG_OBSTACLE_VERIFYING && !mission_motion_enabled(&mission) && map.current_node==0);
    raw_frame(300,VISION_STATUS_NO_MARKER,0,0,0);raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
}

static void enter_alignment(int bearing)
{
    setup();
    assert(!strcmp(start(), "STARTED"));
    selection.bearing_cdeg = (int16_t)bearing;
    tick(20);
    assert(drive.active && drive.aligning && mission.state == MISSION_EXPLORING);
    assert(mission.leg.phase == LEG_ALIGNING && starts == 1 && stops == 1 && !alignment_turns);
    assert(mission.leg.next_target_node == 2 && map.current_node == 0 && map.count == 1);
}

static void test_start_and_approach_share_ten_degree_selection_threshold(void)
{
    static const int16_t bearings[]={-1000,-531,0,531,1000};
    for(unsigned i=0;i<sizeof(bearings)/sizeof(bearings[0]);++i) {
        setup();selection.bearing_cdeg=bearings[i];
        selection.reference_available=false;mission.leg.arrival_reference_node=0;
        assert(!strcmp(start(),"STARTED"));tick(20);
        assert(drive.active && !drive.aligning && !drive.b_departure_pending);
        assert(mission.leg.phase==LEG_APPROACHING && starts==1 && !alignment_turns);
        assert(!map.current_node && map.count==1);
    }
    for(unsigned negative=0;negative<2;++negative) {
        setup();assert(!strcmp(start(),"STARTED"));
        selection.bearing_cdeg=negative?-1001:1001;tick(20);
        assert(drive.active && drive.aligning && mission.leg.phase==LEG_ALIGNING);
        assert(status==MOTOR_AUTO_STOPPED && starts==1);
    }
}
static void test_predeparture_alignment_confirms_images_before_starting_journey(void)
{
    for(unsigned negative=0;negative<2;++negative) {
        setup();mission.dfs_active=true;
        int16_t bearing=negative?-1200:1200;
        selection.bearing_cdeg=selection.reference_bearing_cdeg=bearing;
        uint32_t revision=map.revision,leg=mission.leg.leg_id;
        assert(!strcmp(start(),"STARTED"));
        assert(drive.active && drive.aligning && drive.b_departure_pending && !starts);
        assert(mission.state==MISSION_EXPLORING && mission.leg.phase==LEG_ALIGNING);
        assert(map.current_node==7 && map.revision==revision && map.count==1);
        // Repeated stationary corrections do not consume the journey timer.
        for(unsigned i=0;i<27;++i) {
            tick(400);assert(drive.alignment_rotating && alignment_turns==i+1);
            assert(alignment_heading==(negative?350:10) && !starts && map.current_node==7);
            status=MOTOR_AUTO_HEADING_REACHED;tick(10);
        }
        assert(drive.active && now-drive.started_ms>ONBOARD_DRIVE_MAX_WALL_MS);
        assert(!drive.driving_ms && !drive.search_driving_ms && map.revision==revision);
        selection.bearing_cdeg=selection.reference_bearing_cdeg=negative?-1000:1000;
        tick(400);assert(drive.alignment_frames==1 && !starts && map.current_node==7);
        for(unsigned i=0;i<5;++i) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
        assert(drive.alignment_frames==1 && !starts);
        tick(100);assert(drive.alignment_frames==2 && !starts);
        tick(100);
        assert(drive.active && !drive.aligning && !drive.b_departure_pending && starts==1);
        assert(drive.started_ms==now && !drive.driving_ms && !drive.camera_waited_ms);
        assert(mission.state==MISSION_EXPLORING && mission.leg.phase==LEG_APPROACHING);
        assert(mission.leg.leg_id==leg && mission.leg.departure_node==7 && mission.leg.next_target_node==2);
        assert(mission.leg.arrival_reference_node==9 && !map.current_node && map.count==1);
        assert(map.revision!=revision && drive.map_revision==map.revision && !drive.verification_frames);
        tick(100);assert(drive.driving_ms==100 && drive.active);
    }
}
static void test_predeparture_alignment_waits_for_camera_and_keeps_stop_authoritative(void)
{
    for(unsigned cause=0;cause<3;++cause) {
        setup();selection.bearing_cdeg=selection.reference_bearing_cdeg=1200;
        assert(!strcmp(start(),"STARTED"));tick(400);
        if(cause==0) inputs.camera_status=VISION_STATUS_STALLED;
        if(cause==1) inputs.sensor_valid=false;
        if(cause==2) inputs.link_ok=false;
        tick(20);tick(60000);
        assert(drive.active && drive.b_departure_pending && status==MOTOR_AUTO_STOPPED);
        assert(!starts && map.current_node==7 && map.count==1 && !drive.verification_frames);
        mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
        healthy_camera();inputs.sensor_valid=inputs.link_ok=true;tick(100);
        assert(!drive.active && status==MOTOR_AUTO_STOPPED && !starts && map.current_node==7);
    }
    setup();selection.bearing_cdeg=selection.reference_bearing_cdeg=1200;
    assert(!strcmp(start(),"STARTED"));tick(400);
    inputs.safety_latched=true;tick(20);
    assert(!drive.active && mission.reason==MISSION_REASON_ESTOP && map.current_node==7 && !starts);
}
static void test_predeparture_target_loss_reverses_and_records_actual_departure(void)
{
    setup();selection.bearing_cdeg=selection.reference_bearing_cdeg=1200;
    assert(!strcmp(start(),"STARTED"));
    uint32_t revision=map.revision,leg=mission.leg.leg_id;
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(map.current_node==7 && drive.b_departure_pending && !starts);
    raw_frame(ONBOARD_DRIVE_CAMERA_RECOVERY_MS,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==1);
    assert(!drive.b_departure_pending && drive.started_ms==now && drive.map_revision==map.revision);
    assert(!map.current_node && map.revision!=revision && map.count==1);
    assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && !drive.verification_frames);
    raw_frame(100,VISION_STATUS_OK,1,9,1000);
    for(unsigned i=0;i<10 && drive.reference_recovery;++i) raw_frame(100,VISION_STATUS_OK,1,9,1000);
    assert(drive.active && !drive.reference_recovery && mission.leg.phase==LEG_APPROACHING);
    assert(mission.leg.next_target_node==2 && !map.current_node && last_speed>0);
}

static void test_target_alignment_turns_then_verifies_fresh_stopped_images(void)
{
    enter_alignment(1200);
    char alignment_json[1024], alignment_leg[384];
    assert(onboard_drive_format(&drive,&mission,alignment_json,sizeof(alignment_json)));
    assert(travel_leg_format_json(&mission.leg,alignment_leg,sizeof(alignment_leg)));
    printf("ALIGNING_TRACKING_JSON:%s\n",alignment_json);
    printf("ALIGNING_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":true,\"revision\":%lu,\"leg\":%s}\n",
        (unsigned long)mission.revision,alignment_leg);
    uint32_t started = drive.started_ms, revision = map.revision;
    inputs.heading_deg = 357; inputs.heading_offset_deg = 20;
    tick(300); assert(!alignment_turns); // Image was captured before settling ended.
    tick(100); assert(alignment_turns == 1 && alignment_heading == 347 && drive.alignment_rotating);
    selection.bearing_cdeg = 100;
    tick(100); assert(drive.alignment_frames == 0 && starts == 1); // Centred while moving is insufficient.
    status = MOTOR_AUTO_HEADING_REACHED;
    tick(10); assert(!drive.alignment_rotating && drive.alignment_frames == 0 && starts == 1);
    tick(300); assert(drive.alignment_frames == 0);
    tick(100); assert(drive.alignment_frames == 1);
    now += 20; inputs.sensor_received_ms = now;
    onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(drive.alignment_frames == 1); // Cached frame cannot confirm twice.
    now += 300; inputs.sensor_received_ms = now;
    onboard_drive_tick(&drive, &mission, &map, &selection, &inputs, now);
    assert(drive.alignment_frames == 1 && drive.active && !drive.alignment_rotating);
    assert(mission.leg.phase==LEG_WAITING_CAMERA && !strcmp(drive.result,"WAITING_FOR_FRESH_CAMERA"));
    tick(200); assert(drive.alignment_frames == 1 && drive.camera_wait_frames == 1 && starts == 1);
    tick(200); assert(drive.alignment_frames == 2 && starts == 1);
    tick(200);
    assert(drive.active && !drive.aligning && mission.leg.phase == LEG_APPROACHING && starts == 2);
    assert(!strcmp(drive.result, "TARGET_ALIGNED") && drive.started_ms == started);
    assert(map.revision == revision && map.current_node == 0 && map.count == 1);
    assert(mission.leg.next_target_node == 2 && mission.leg.arrival_reference_node == 9);
    char json[1024];
    assert(onboard_drive_format(&drive,&mission,json,sizeof(json)) && strstr(json,"TARGET_ALIGNED"));
    enter_alignment(-1200); inputs.heading_deg=2; inputs.heading_offset_deg=20;
    tick(400); assert(alignment_heading == 332); // Left correction and heading wrap/manual zero.
    for (int bearing=-1000; bearing<=1000; bearing+=1000) {
        setup(); assert(!strcmp(start(),"STARTED"));
        selection.bearing_cdeg=(int16_t)bearing; selection.reference_available=false;
        tick(20); assert(!drive.aligning && drive.active && !alignment_turns);
    }
}

static void test_alignment_retries_past_old_limits_and_preserves_fault_checks(void)
{
    enter_alignment(2000);
    uint32_t leg=mission.leg.leg_id,started=drive.started_ms,revision=map.revision;
    // More than three corrections and six seconds must not pause alignment.
    for (unsigned i=0;i<20;++i) {
        tick(400); assert(alignment_turns==i+1 && drive.active && drive.alignment_rotating);
        assert(alignment_heading==10); // Each correction remains limited to ten degrees.
        status=MOTOR_AUTO_HEADING_REACHED; tick(10);
        assert(mission.state==MISSION_EXPLORING && mission.leg.phase==LEG_ALIGNING);
        assert(!drive.alignment_rotating && status==MOTOR_AUTO_STOPPED);
    }
    assert(now-started>6000 && map.revision==revision && !map.current_node && map.count==1);
    assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    selection.bearing_cdeg=0;tick(400);tick(100);tick(100);
    assert(drive.active && !drive.aligning && mission.leg.phase==LEG_APPROACHING && starts==2);
    assert(!strcmp(drive.result,"TARGET_ALIGNED") && drive.started_ms==started);
    assert(map.revision==revision && !map.current_node && map.count==1);
    enter_alignment(1200); tick(400); status=MOTOR_AUTO_STOPPED; tick(20);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason,"MOTOR_STOPPED"));
    enter_alignment(1200); start_result=MOTOR_ERR_SAFETY; tick(400);
    assert(!drive.active && !strcmp(drive.result,"TARGET_ALIGNMENT_MOTOR_REFUSED"));
    enter_alignment(1200); tick(400); renew_ok=false; tick(100);
    assert(drive.active && drive.input_waiting && !strcmp(drive.input_reason,"DRIVE_LEASE_EXPIRED"));
    enter_alignment(1200); tick(400); inputs.camera_fresh=false; tick(100);
    assert(drive.active && !drive.alignment_rotating && starts==1);
    tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(drive.active && drive.camera_waiting && mission.leg.phase==LEG_WAITING_CAMERA);
    enter_alignment(1200); tick(400); selection.available=false; tick(100);
    assert(drive.active && !drive.alignment_rotating && starts==1);
    tick(ONBOARD_DRIVE_CAMERA_RECOVERY_MS);
    assert(drive.active && drive.aligning && drive.b_alignment_reference);
    assert(mission.state==MISSION_EXPLORING && mission.leg.phase==LEG_ALIGNING && !reverse_starts);
    enter_alignment(1200); inputs.safety_latched=true; tick(20);
    assert(!drive.active && mission.reason==MISSION_REASON_ESTOP && map.count==1);
    enter_alignment(1200); inputs.link_ok=false; tick(20);
    assert(drive.active && drive.input_waiting && mission.leg.phase==LEG_WAITING_INPUT);
    enter_alignment(1200); inputs.front_mm=149; tick(20);
    assert(!drive.active && mission.state==MISSION_PAUSED && map.current_node==0 && map.count==1);
    enter_alignment(1200); selection.marker=3; tick(20);
    assert(!drive.active && !strcmp(drive.result,"TRACKING_ID_CHANGED"));
}

static void test_alignment_preserves_reference_search_budget(void)
{
    setup(); selection.reference_available=false; mission.leg.arrival_reference_node=0;
    assert(!strcmp(start(),"STARTED"));
    selection.available=false; tick(100);
    assert(drive.search_active && mission.leg.phase==LEG_SEARCHING_REFERENCE);
    selection.available=true; selection.bearing_cdeg=1200; tick(20);
    assert(drive.aligning && mission.leg.phase==LEG_ALIGNING);
    uint32_t search_ms=drive.search_driving_ms, started=drive.started_ms;
    tick(400); status=MOTOR_AUTO_HEADING_REACHED; tick(10);
    selection.bearing_cdeg=0; tick(400); tick(200); tick(200);
    assert(drive.active && !drive.aligning && drive.search_active && mission.leg.phase==LEG_SEARCHING_REFERENCE);
    assert(drive.search_driving_ms==search_ms && drive.started_ms==started);
    assert(map.current_node==0 && map.count==1 && mission.leg.next_target_node==2);
}

static void test_stale_camera_waits_without_cancelling_dfs_and_resumes_fresh(void)
{
    for (unsigned image=0;image<2;++image) {
        start_500_mm_leg(); mission.dfs_active=true;
        uint32_t id=mission.leg.leg_id, started=drive.started_ms;
        if (image) inputs.camera_image_age_ms=751;
        else inputs.camera_packet_age_ms=301;
        tick(20);
        uint32_t driving=drive.driving_ms;
        assert(drive.camera_waiting && mission.leg.phase==LEG_WAITING_CAMERA && stops==1);
        assert(!mission_motion_enabled(&mission) && mission.dfs_active);
        tick(2000);
        assert(drive.active && mission.state==MISSION_EXPLORING && mission.dfs_active && starts==1);
        assert(drive.driving_ms==driving && mission.leg.leg_id==id && mission.leg.next_target_node==2);
        assert(mission.leg.arrival_reference_node==9 && map.current_node==0 && map.count==1);
        char json[768]; assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
        if (!image) {
            printf("CAMERA_WAIT_JSON:%s\n",json);
            char leg_json[384]; assert(travel_leg_format_json(&mission.leg,leg_json,sizeof(leg_json)));
            printf("CAMERA_WAIT_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":false,\"revision\":%lu,\"leg\":%s}\n",
                (unsigned long)mission.revision,leg_json);
        }
        healthy_camera(); observe_reference(1000,400,0);
        assert(drive.camera_wait_frames==1 && starts==1);
        for (unsigned i=0;i<10;++i) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
        assert(drive.camera_wait_frames==1 && starts==1);
        observe_reference(1000,100,0);
        assert(drive.active && !drive.camera_waiting && mission.leg.phase==LEG_APPROACHING && starts==2);
        assert(drive.started_ms==started && drive.driving_ms==driving && mission.dfs_active && map.current_node==0);
    }
}

static void test_camera_wait_freezes_outage_time_without_resetting_spent_journey_budget(void)
{
    start_500_mm_leg(); brake_near_target(); observe_reference(600,400,0);
    assert(drive.verification_frames==1);
    inputs.camera_packet_age_ms=301; tick(100); tick(4000);
    assert(drive.active && mission.leg.phase==LEG_WAITING_CAMERA && drive.verification_frames==1);
    healthy_camera(); observe_reference(600,100,0); observe_reference(600,100,0);
    assert(drive.active && drive.verification_frames==2 && map.current_node==0 && starts==1);
    observe_reference(600,100,0);
    assert(!drive.active && map.current_node==2 && starts==1);

    enter_alignment(1200); tick(400); inputs.camera_packet_age_ms=301; tick(100);
    assert(drive.camera_waiting && !drive.alignment_rotating && alignment_turns==1);
    tick(6000);
    assert(drive.active && drive.aligning && mission.leg.phase==LEG_WAITING_CAMERA);
    healthy_camera(); selection.bearing_cdeg=0;
    tick(100); tick(100); tick(200); tick(200);
    assert(drive.active && !drive.aligning && mission.leg.phase==LEG_APPROACHING && starts==2);

    start_without_reference(); raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    inputs.camera_image_age_ms=751; tick(100);
    uint32_t search_ms=drive.search_driving_ms;
    tick(2000);
    assert(drive.active && drive.search_active && mission.leg.phase==LEG_WAITING_CAMERA && drive.search_driving_ms==search_ms);
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0); raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && !drive.camera_waiting && mission.leg.phase==LEG_SEARCHING_REFERENCE);
    assert(drive.search_driving_ms==search_ms && map.current_node==0);

    start_500_mm_leg(); camera_gap();
    drive.started_ms=now-ONBOARD_DRIVE_MAX_WALL_MS;
    tick(1);
    assert(!drive.active && !strcmp(drive.result,"LEG_TIME_LIMIT") && mission.state==MISSION_PAUSED && map.count==1);
}

static void test_camera_age_interrupts_reverse_without_restarting_the_pulse(void)
{
    start_500_mm_leg(); start_backup(); inputs.camera_image_age_ms=751; tick(20);
    assert(drive.camera_waiting && !mission_motion_enabled(&mission) && reverse_starts==1);
    assert(drive.camera_resume_phase==LEG_REACQUIRING);
    tick(2000); assert(drive.active && reverse_starts==1);
    reference_returns(); observe_reference(800,100,0); observe_reference(800,100,0);
    assert(drive.active && mission.leg.phase==LEG_REACQUIRING && reverse_starts==1);
    observe_reference(800,400,0); observe_reference(800,100,0);
    assert(drive.active && mission.leg.phase==LEG_APPROACHING && reverse_starts==1 && map.current_node==0);
}

static void start_mapped_recovery(uint8_t id,int16_t x,int16_t y,bool trusted,bool visited)
{
    setup();
    assert((trusted?rover_map_add(&map,id,x,y,visited):rover_map_observe(&map,id,x,y))==MAP_OK);
    drive.frame_valid=true;drive.north_heading_deg=0;
    assert(!strcmp(start(),"STARTED"));start_backup();
}
static void confirm_recovery_marker(uint8_t id,uint16_t range)
{
    raw_frame(100,VISION_STATUS_OK,1,id,range);
    assert(mission.leg.phase==LEG_REACQUIRING && !mission_motion_enabled(&mission));
    raw_frame(300,VISION_STATUS_OK,1,id,range);
    assert(drive.active && drive.mapped_recovery_frames==1);
    // Re-delivering the same camera sequence must not confirm a new identity.
    for(unsigned i=0;i<5;++i) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(drive.mapped_recovery_frames==1);
    raw_frame(100,VISION_STATUS_OK,1,id,range);
    assert(mission.leg.phase==LEG_REACQUIRING);
    raw_frame(100,VISION_STATUS_OK,1,id,range);
}
static void test_reverse_ignores_unknown_and_bad_markers_then_recovers_original_reference(void)
{
    start_500_mm_leg();start_backup();
    for(unsigned i=0;i<10;++i) raw_frame(100,VISION_STATUS_OK,1,45,900);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==1 && renews>=10);
    assert(!map.current_node && map.count==1 && !drive.recovery_marker_id);
    // The correct reference works even if it was never mapped.
    raw_frame(100,VISION_STATUS_OK,1,9,900);
    assert(mission.leg.phase==LEG_REACQUIRING && drive.recovery_marker_id==9);
    raw_frame(300,VISION_STATUS_OK,1,9,900);
    raw_frame(100,VISION_STATUS_OK,1,9,900);
    assert(mission.leg.phase==LEG_APPROACHING && !drive.reference_recovery && starts==3);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node);
}
static void test_alternate_mapped_reference_uses_grid_distance_and_confirms_only_target(void)
{
    for(unsigned trusted=0;trusted<2;++trusted) for(unsigned visited=0;visited<2;++visited) {
        start_mapped_recovery(14,0,3,trusted!=0,visited!=0);
        confirm_recovery_marker(14,1400);
        assert(mission.leg.phase==LEG_APPROACHING && drive.reference_arrival_mm==1200);
        assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==14);
        assert(drive.reference_id==14 && selection.reference_marker==14 && !map.current_node);
        assert(drive.recovery_marker_mapped && drive.recovery_marker_id==14);
        char json[1024];assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
        printf("MAPPED_RECOVERY_JSON:%s\n",json);
        raw_frame(100,VISION_STATUS_OK,1,14,1250);
        assert(mission.leg.phase==LEG_BRAKING && !map.current_node);
        raw_frame(300,VISION_STATUS_OK,1,14,1200);
        raw_frame(100,VISION_STATUS_OK,1,14,1200);
        assert(!map.current_node);
        raw_frame(100,VISION_STATUS_OK,1,14,1200);
        assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
        if(trusted) assert(rover_map_find_id(&map,14)->visited==(visited!=0));
        else assert(!rover_map_find_id(&map,14));
    }
}
static void test_mapped_off_route_marker_holds_without_claiming_location(void)
{
    start_mapped_recovery(14,1,1,false,false);
    confirm_recovery_marker(14,900);
    assert(drive.active && mission.leg.phase==LEG_REACQUIRING && !mission_motion_enabled(&mission));
    assert(!strcmp(drive.result,"MAPPED_MARKER_NEEDS_REFERENCE") && !map.current_node);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    // If that landmark disappears, continue reverse search instead of exhausting one retry.
    raw_frame(100,VISION_STATUS_OK,1,45,900);
    assert(mission.leg.phase==LEG_BACKING_UP && reverse_starts==2);
    raw_frame(100,VISION_STATUS_OK,1,9,850);
    raw_frame(300,VISION_STATUS_OK,1,9,850);raw_frame(100,VISION_STATUS_OK,1,9,850);
    assert(drive.active && mission.leg.phase==LEG_APPROACHING && !map.current_node);
}
static void test_mapped_destination_reacquisition_resumes_without_false_arrival(void)
{
    start_mapped_recovery(2,0,1,false,false);
    confirm_recovery_marker(2,500);
    assert(drive.active && mission.leg.phase==LEG_APPROACHING && !drive.reference_model_valid);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node);
    assert(!rover_map_find_id(&map,2));
}

static void start_alignment_reverse(bool without_reference)
{
    if(without_reference) {
        start_without_reference();selection.bearing_cdeg=1200;tick(20);
    } else enter_alignment(1200);
    mission.dfs_active=true;
    assert(drive.aligning && mission.leg.phase==LEG_ALIGNING);
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && drive.aligning && status==MOTOR_AUTO_STOPPED && !reverse_starts);
    raw_frame(ONBOARD_DRIVE_CAMERA_RECOVERY_MS,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && !drive.aligning && drive.reference_recovery && reverse_starts==1);
    assert(mission.state==MISSION_EXPLORING && mission.dfs_active && mission.leg.phase==LEG_BACKING_UP);
    assert(mission.leg.next_target_node==2 && !map.current_node && map.count==1);
}
static void test_alignment_target_loss_uses_visible_reference_without_reverse(void)
{
    // Replay the reported failure: the target vanishes while its reference
    // remains visible and the camera continues reporting fresh images.
    enter_alignment(1200);mission.dfs_active=true;
    uint32_t leg=mission.leg.leg_id,started=drive.started_ms,revision=map.revision;
    raw_bearing_frame(100,VISION_STATUS_OK,1,9,1092,789);
    assert(drive.aligning && !reverse_starts && !selection.available && selection.reference_available);
    assert(drive.b_alignment_reference);
    raw_bearing_frame(400,VISION_STATUS_OK,1,9,1092,789);
    assert(drive.active && mission.leg.phase==LEG_ALIGNING && !reverse_starts);
    assert(mission.leg.leg_id==leg && drive.started_ms==started && !drive.verification_frames);
    assert(drive.alignment_frames==1 && mission.dfs_active);
    raw_bearing_frame(100,VISION_STATUS_OK,1,9,1092,789);
    assert(drive.alignment_frames==2 && !map.current_node);
    raw_bearing_frame(100,VISION_STATUS_OK,1,9,1092,789);
    assert(!drive.reference_recovery && !drive.aligning && mission.leg.phase==LEG_APPROACHING && starts==2);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && map.revision==revision);
    raw_frame(100,VISION_STATUS_OK,1,9,650);
    raw_frame(300,VISION_STATUS_OK,1,9,600);raw_frame(100,VISION_STATUS_OK,1,9,600);
    assert(!map.current_node);
    raw_frame(100,VISION_STATUS_OK,1,9,600);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(mission.state==MISSION_EXPLORING && mission.dfs_active);
}
static void test_alignment_reverse_recovers_locked_target_without_a_map_entry(void)
{
    for(unsigned without_reference=0;without_reference<2;++without_reference) {
        start_alignment_reverse(without_reference!=0);
        uint32_t leg=mission.leg.leg_id,revision=map.revision;
        for(unsigned i=0;i<10;++i) raw_frame(100,VISION_STATUS_OK,1,45,900);
        assert(mission.leg.phase==LEG_BACKING_UP && reverse_starts>=1 && !drive.recovery_marker_id);
        unsigned before=reverse_starts,renewals=renews;
        for(unsigned i=0;i<10;++i) raw_frame(100,VISION_STATUS_OK,1,45,900);
        assert(reverse_starts==before && renews>=renewals+10);
        assert(mission.dfs_active && !rover_map_find_id(&map,2) && !map.observed_count);
        confirm_recovery_marker(2,500);
        assert(drive.active && !drive.reference_recovery && mission.leg.phase==LEG_APPROACHING);
        assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && !drive.reference_model_valid);
        assert(map.revision==revision && !map.current_node && map.count==1 && !drive.verification_frames);
        assert(mission.state==MISSION_EXPLORING && mission.dfs_active && last_speed>0);
    }
}
static void test_alignment_reverse_realigns_recovered_target_before_forward_motion(void)
{
    start_alignment_reverse(true);
    uint32_t revision=map.revision;
    for(unsigned i=0;i<10 && mission.leg.phase!=LEG_REACQUIRING;++i)
        raw_bearing_frame(100,VISION_STATUS_OK,1,2,500,1200);
    assert(mission.leg.phase==LEG_REACQUIRING && status==MOTOR_AUTO_STOPPED);
    unsigned moving_starts=starts;
    raw_bearing_frame(300,VISION_STATUS_OK,1,2,500,1200);
    raw_bearing_frame(100,VISION_STATUS_OK,1,2,500,1200);
    raw_bearing_frame(100,VISION_STATUS_OK,1,2,500,1200);
    assert(drive.active && drive.aligning && mission.leg.phase==LEG_ALIGNING);
    assert(starts==moving_starts && !drive.reference_recovery && status==MOTOR_AUTO_STOPPED);
    raw_bearing_frame(400,VISION_STATUS_OK,1,2,500,1200);
    assert(drive.alignment_rotating && alignment_turns==1 && starts==moving_starts);
    status=MOTOR_AUTO_HEADING_REACHED;
    raw_frame(10,VISION_STATUS_OK,1,2,500);
    raw_frame(300,VISION_STATUS_OK,1,2,500);
    raw_frame(100,VISION_STATUS_OK,1,2,500);raw_frame(100,VISION_STATUS_OK,1,2,500);
    assert(drive.active && !drive.aligning && mission.leg.phase==LEG_APPROACHING && starts==moving_starts+1);
    assert(map.revision==revision && !map.current_node && mission.leg.next_target_node==2);
}
static void test_alignment_reverse_adopts_compatible_mapped_marker(void)
{
    for(unsigned visited=0;visited<2;++visited) {
        setup();
        assert((visited?rover_map_add(&map,14,0,3,true):rover_map_observe(&map,14,0,3))==MAP_OK);
        drive.frame_valid=true;drive.north_heading_deg=0;
        assert(!strcmp(start(),"STARTED"));
        selection.bearing_cdeg=1200;tick(20);mission.dfs_active=true;
        uint32_t revision=map.revision,leg=mission.leg.leg_id;
        raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
        raw_frame(ONBOARD_DRIVE_CAMERA_RECOVERY_MS,VISION_STATUS_NO_MARKER,0,0,0);
        assert(mission.leg.phase==LEG_BACKING_UP);
        for(unsigned i=0;i<10 && mission.leg.phase!=LEG_REACQUIRING;++i)
            raw_frame(100,VISION_STATUS_OK,1,14,1400);
        assert(mission.leg.phase==LEG_REACQUIRING);
        raw_frame(300,VISION_STATUS_OK,1,14,1400);
        raw_frame(100,VISION_STATUS_OK,1,14,1400);raw_frame(100,VISION_STATUS_OK,1,14,1400);
        assert(drive.active && !drive.reference_recovery && mission.leg.phase==LEG_APPROACHING);
        assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==14 && mission.leg.leg_id==leg);
        assert(drive.reference_arrival_mm==1200 && drive.recovery_marker_mapped);
        assert(map.revision==revision && !map.current_node && !rover_map_find_id(&map,2));
        assert(!drive.verification_frames && mission.dfs_active && mission.state==MISSION_EXPLORING);
    }
}
static void test_alignment_reverse_retains_camera_wait_and_operator_stop(void)
{
    start_alignment_reverse(false);uint32_t leg=mission.leg.leg_id,revision=map.revision;
    inputs.camera_status=VISION_STATUS_STALLED;tick(20);tick(60000);
    assert(drive.active && drive.camera_waiting && mission.leg.phase==LEG_WAITING_CAMERA);
    assert(status==MOTOR_AUTO_STOPPED && reverse_starts==1 && mission.dfs_active);
    for(unsigned i=0;i<10 && mission.leg.phase!=LEG_BACKING_UP;++i)
        raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==2);
    assert(mission.leg.leg_id==leg && map.revision==revision && !map.current_node);
    mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
    raw_frame(100,VISION_STATUS_OK,1,9,900);
    assert(!drive.active && status==MOTOR_AUTO_STOPPED && starts==3);
    assert(mission.state==MISSION_PAUSED && !map.current_node);
    start_alignment_reverse(false);inputs.safety_latched=true;tick(20);
    assert(!drive.active && mission.reason==MISSION_REASON_ESTOP && status==MOTOR_AUTO_STOPPED);
}

static void test_reverse_uses_fresh_valid_markers_and_prioritises_locked_reference(void)
{
    start_mapped_recovery(14,0,3,false,false);
    // A known ID with bad quality or invalid range cannot stop reverse recovery.
    for(unsigned bad=0;bad<3;++bad) {
        now+=100;inputs.sensor_received_ms=now;inputs.camera_present=true;inputs.camera_status=VISION_STATUS_OK;inputs.camera_fresh=true;
        inputs.camera_packet_age_ms=inputs.camera_image_age_ms=0;inputs.camera_generation++;
        selection.sample_generation++;selection.sample_received_ms=now;selection.sample_age_ms=0;
        inputs.recovery_marker_count=1;
        inputs.recovery_markers[0]=(camera_marker_t){.id=14,.range_mm=bad==0?0:bad==1?UINT16_MAX:900,.quality=bad==2?1:0};
        onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
        assert(drive.active && mission.leg.phase==LEG_BACKING_UP);
    }
    // The original reference wins even when another mapped landmark is nearer.
    now+=100;inputs.sensor_received_ms=now;inputs.camera_generation++;inputs.camera_seq++;
    inputs.recovery_marker_count=3;
    inputs.recovery_markers[0]=(camera_marker_t){.id=45,.range_mm=400};
    inputs.recovery_markers[1]=(camera_marker_t){.id=14,.range_mm=800};
    inputs.recovery_markers[2]=(camera_marker_t){.id=9,.range_mm=1000};
    onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
    assert(mission.leg.phase==LEG_REACQUIRING && drive.recovery_marker_id==9);
    assert(mission.leg.arrival_reference_node==9 && !map.current_node);
}
static void test_reverse_relocalisation_respects_all_four_grid_directions(void)
{
    static const int16_t x[]={0,3,0,-3},y[]={3,0,-3,0};
    for(unsigned dir=0;dir<4;++dir) {
        setup();assert(rover_map_observe(&map,14,x[dir],y[dir])==MAP_OK);
        drive.frame_valid=true;drive.north_heading_deg=0;inputs.heading_deg=(float)(dir*90);
        assert(!strcmp(start(),"STARTED"));start_backup();confirm_recovery_marker(14,1400);
        assert(drive.active && mission.leg.phase==LEG_APPROACHING && drive.reference_arrival_mm==1200);
        assert(mission.leg.arrival_reference_node==14 && !map.current_node);
    }
}
static void test_unknown_camera_after_stale_wait_resumes_reverse_search(void)
{
    start_500_mm_leg();start_backup();inputs.camera_image_age_ms=751;tick(20);
    assert(mission.leg.phase==LEG_WAITING_CAMERA && !mission_motion_enabled(&mission));
    raw_frame(400,VISION_STATUS_OK,1,45,900);raw_frame(100,VISION_STATUS_OK,1,45,900);
    raw_frame(100,VISION_STATUS_OK,1,45,900); // First post-gap image re-synchronises selector timing.
    for(unsigned i=0;i<3;++i) raw_frame(100,VISION_STATUS_OK,1,45,900); // Complete the preserved stop/settle window.
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==2);
    assert(!map.current_node && mission.leg.next_target_node==2);
}

static void test_arrival_corrects_both_directions_and_reverifies_before_commit(void)
{
    for(unsigned overshoot=0;overshoot<2;++overshoot) {
        start_500_mm_leg(); brake_near_target();
        uint32_t leg=mission.leg.leg_id,revision=map.revision;
        observe_reference(overshoot?450:850,400,0);
        assert(drive.active && drive.correcting && mission.leg.phase==LEG_CORRECTING);
        assert(last_speed==(overshoot?-ONBOARD_DRIVE_CORRECTION_PCT:ONBOARD_DRIVE_CORRECTION_PCT));
        assert(!drive.verification_frames && map.revision==revision && !map.current_node);
        assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
        char json[1024];assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
        printf("CORRECTION_TRACKING_JSON:%s\n",json);
        observe_reference(600,100,0); // Renew a short correction lease, never commit while moving.
        assert(drive.correcting && !map.current_node);
        observe_reference(600,50,0);
        assert(!drive.correcting && mission.leg.phase==LEG_BRAKING && status==MOTOR_AUTO_STOPPED);
        observe_reference(600,400,0);observe_reference(600,100,0);
        assert(drive.verification_frames==2 && !map.current_node);
        observe_reference(600,100,0);
        assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    }
}
static void test_input_outages_wait_indefinitely_preserve_leg_and_resume_with_fresh_images(void)
{
    for(unsigned cause=0;cause<5;++cause) {
        start_500_mm_leg();mission.dfs_active=true;
        uint32_t leg=mission.leg.leg_id,revision=map.revision,started=drive.started_ms;
        if(cause==0) inputs.link_ok=false;
        if(cause==1) inputs.sensor_valid=false;
        if(cause==2) inputs.imu_ok=false;
        if(cause==3) inputs.heading_deg=NAN;
        if(cause==4) renew_ok=false;
        mission_tick(&mission,false,inputs.link_ok,now);
        tick(100);
        assert(drive.active && drive.input_waiting && mission.leg.phase==LEG_WAITING_INPUT);
        target_selector_tick(&selection,&mission,&map,NULL,false,now);
        assert(selection.state==TARGET_LOCKED && mission.leg.next_target_node==2);
        assert(mission.state==MISSION_EXPLORING && mission.dfs_active && !mission_motion_enabled(&mission));
        char json[1024];assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
        printf("INPUT_WAIT_TRACKING_JSON:%s\n",json);
        char leg_json[384];assert(travel_leg_format_json(&mission.leg,leg_json,sizeof(leg_json)));
        printf("INPUT_WAIT_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":false,\"revision\":%lu,\"leg\":%s}\n",(unsigned long)mission.revision,leg_json);
        tick(60000);
        assert(drive.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
        assert(!mission_motion_enabled(&mission) && !map.current_node && map.revision==revision);
        assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
        inputs.link_ok=inputs.sensor_valid=inputs.imu_ok=true;inputs.heading_deg=0;renew_ok=true;
        inputs.camera_packet_age_ms=301;
        tick(100);tick(100);
        assert(drive.active && drive.camera_waiting && starts==1 && !mission_motion_enabled(&mission));
        healthy_camera();
        for(unsigned i=0;i<8 && starts==1;++i) observe_reference(1000,100,0);
        assert(drive.active && starts==2 && mission.leg.phase==LEG_APPROACHING && mission.dfs_active);
        assert(drive.started_ms>started+59000 && !map.current_node && map.revision==revision);
    }
    start_500_mm_leg();inputs.link_ok=false;tick(100);
    mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);inputs.link_ok=true;tick(60000);
    assert(!drive.active && mission.state==MISSION_PAUSED && starts==1 && !map.current_node);
}
static void test_input_loss_and_stale_camera_interrupt_arrival_corrections(void)
{
    for(unsigned cause=0;cause<3;++cause) {
        start_500_mm_leg();brake_near_target();observe_reference(450,400,0);
        assert(drive.correcting);
        if(cause==0) inputs.link_ok=false;
        if(cause==1) inputs.camera_packet_age_ms=301;
        if(cause==2) inputs.safety_latched=true;
        tick(20);
        assert(status==MOTOR_AUTO_STOPPED && !drive.correcting && !map.current_node);
        if(cause==0) assert(drive.input_waiting && drive.input_resume_phase==LEG_BRAKING);
        if(cause==1) assert(drive.camera_waiting && drive.camera_resume_phase==LEG_BRAKING);
        if(cause==2) assert(!drive.active && mission.state==MISSION_PAUSED);
    }
}

static void test_input_recovery_resumes_alignment_reverse_and_arrival_verification(void)
{
    enter_alignment(1200);tick(400);inputs.imu_ok=false;tick(20);
    assert(drive.input_waiting && drive.input_resume_phase==LEG_ALIGNING && !drive.alignment_rotating);
    tick(60000);inputs.imu_ok=true;selection.bearing_cdeg=0;
    for(unsigned i=0;i<15 && starts==1;++i) tick(100);
    assert(drive.active && !drive.aligning && starts==2 && mission.leg.phase==LEG_APPROACHING);
    assert(!map.current_node);

    start_500_mm_leg();start_backup();inputs.link_ok=false;tick(20);
    assert(drive.input_waiting && drive.input_resume_phase==LEG_REACQUIRING && !mission_motion_enabled(&mission));
    tick(60000);inputs.link_ok=true;reference_returns();
    for(unsigned i=0;i<15 && starts==2;++i) observe_reference(900,100,0);
    assert(drive.active && starts==3 && !drive.reference_recovery && mission.leg.phase==LEG_APPROACHING);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node);

    start_500_mm_leg();brake_near_target();observe_reference(450,400,0);
    inputs.sensor_valid=false;tick(20);tick(60000);inputs.sensor_valid=true;
    assert(drive.input_waiting && drive.input_resume_phase==LEG_BRAKING);
    for(unsigned i=0;i<18 && drive.active;++i) observe_reference(600,100,0);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
}

static void test_camera_health_outage_keeps_leg_and_resumes_only_after_fresh_observations(void)
{
    for(unsigned cause=0;cause<3;++cause) {
        start_500_mm_leg();mission.dfs_active=true;
        uint32_t leg=mission.leg.leg_id,revision=map.revision,started=drive.started_ms;
        inputs.camera_status=cause==0?VISION_STATUS_STALLED:VISION_STATUS_CAMERA_LOST;
        if(cause==2) inputs.camera_present=false;
        tick(20);
        assert(drive.active && drive.camera_waiting && mission.leg.phase==LEG_WAITING_CAMERA);
        assert(!mission_motion_enabled(&mission) && status==MOTOR_AUTO_STOPPED && starts==1);
        uint32_t used=drive.driving_ms;
        tick(60000);
        assert(drive.active && drive.camera_waiting && drive.driving_ms==used && drive.camera_waited_ms>=60000);
        assert(mission.state==MISSION_EXPLORING && mission.dfs_active && mission.leg.leg_id==leg);
        assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node && map.revision==revision);
        inputs.camera_present=true;healthy_camera();observe_reference(900,400,0);
        assert(drive.camera_waiting && drive.camera_wait_frames==1 && starts==1);
        for(unsigned repeat=0;repeat<5;++repeat) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
        assert(drive.camera_wait_frames==1 && starts==1);
        observe_reference(900,100,0);
        assert(drive.active && !drive.camera_waiting && starts==2 && mission.leg.phase==LEG_APPROACHING);
        assert(!strcmp(drive.result,"RESUMED_AFTER_CAMERA_WAIT") && drive.started_ms==started);
        assert(mission.dfs_active && map.revision==revision && !map.current_node);
    }
}
static void test_stalled_health_packet_during_arrival_verification_does_not_cancel_dfs(void)
{
    start_500_mm_leg();mission.dfs_active=true;brake_near_target();
    observe_reference(505,400,200); // -95 mm, but image captured before the stopped verification window.
    assert(drive.remaining_mm==-95 && !drive.verification_frames && mission.leg.phase==LEG_VERIFYING);
    inputs.camera_packet_age_ms=477;inputs.camera_image_age_ms=717;tick(100);
    assert(drive.camera_waiting && !strcmp(drive.input_reason,"CAMERA_PACKET_STALE"));
    inputs.camera_status=VISION_STATUS_STALLED;inputs.camera_packet_age_ms=6;inputs.camera_image_age_ms=891;tick(100);
    assert(drive.active && drive.camera_waiting && !strcmp(drive.input_reason,"CAMERA_PROCESSING_STALLED"));
    char json[1024];assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
    printf("CAMERA_STALL_WAIT_JSON:%s\n",json);
    char leg_json[384];assert(travel_leg_format_json(&mission.leg,leg_json,sizeof(leg_json)));
    printf("CAMERA_STALL_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":false,\"revision\":%lu,\"leg\":%s}\n",(unsigned long)mission.revision,leg_json);
    tick(60000);assert(drive.active && !map.current_node && !drive.verification_frames);
    healthy_camera();observe_reference(600,100,0);
    assert(drive.camera_waiting && !map.current_node);
    observe_reference(600,100,0);
    assert(!drive.camera_waiting && drive.active && mission.leg.phase==LEG_VERIFYING && drive.verification_frames==1);
    observe_reference(600,100,0);assert(drive.active && !map.current_node);
    observe_reference(600,100,0);
    assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(mission.state==MISSION_EXPLORING && mission.dfs_active && starts==1);
}
static void test_camera_health_recovery_does_not_bypass_marker_identity_or_operator_stop(void)
{
    start_500_mm_leg();raw_frame(100,VISION_STATUS_STALLED,0,0,0);
    raw_frame(2000,VISION_STATUS_STALLED,0,0,0);
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && starts==1 && !map.current_node && !mission_motion_enabled(&mission));
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    for(unsigned cause=0;cause<2;++cause) {
        start_500_mm_leg();inputs.camera_status=VISION_STATUS_STALLED;tick(20);
        if(cause==0) mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
        else mission_tick(&mission,true,true,now);
        tick(20);healthy_camera();observe_reference(900,400,0);observe_reference(900,100,0);
        assert(!drive.active && starts==1 && !map.current_node);
        assert(mission.state==(cause==0?MISSION_PAUSED:MISSION_FAULT));
    }
}

static void test_live_unusable_reference_reverses_instead_of_camera_timeout(void)
{
    start_500_mm_leg();mission.dfs_active=true;observe_reference(999,100,0);
    uint32_t leg=mission.leg.leg_id,started=drive.started_ms,revision=map.revision;
    // The raw camera still reports the reference ID, but beyond the recovery
    // bearing window. This used to bypass the raw-absence-only reverse path.
    raw_bearing_frame(100,VISION_STATUS_OK,1,9,999,5000);
    assert(!inputs.reference_absent && !selection.available && !selection.reference_available);
    assert(drive.active && drive.reference_recovery && mission.leg.phase==LEG_HOLDING);
    for(unsigned i=0;i<15;++i) raw_bearing_frame(100,VISION_STATUS_OK,1,9,999,5000);
    assert(drive.active && mission.state==MISSION_EXPLORING && !reverse_starts);
    for(unsigned i=0;i<5;++i) raw_bearing_frame(100,VISION_STATUS_OK,1,9,999,5000);
    assert(mission.leg.phase==LEG_BACKING_UP && reverse_starts==1 && last_speed<0);
    assert(mission.dfs_active && mission.leg.leg_id==leg && drive.started_ms==started);
    assert(map.revision==revision && !map.current_node && !drive.verification_frames);
    for(unsigned i=0;i<10;++i) raw_frame(100,VISION_STATUS_OK,1,45,900);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==1);
    raw_bearing_frame(100,VISION_STATUS_OK,1,9,900,1200);
    assert(mission.leg.phase==LEG_REACQUIRING && status==MOTOR_AUTO_STOPPED);
    raw_bearing_frame(300,VISION_STATUS_OK,1,9,900,1200);
    assert(drive.aligning && drive.b_alignment_reference && !drive.reference_recovery);
    assert(mission.leg.phase==LEG_ALIGNING && starts==2 && !map.current_node);
    raw_bearing_frame(400,VISION_STATUS_OK,1,9,900,1200);
    assert(drive.alignment_rotating && alignment_heading==10 && starts==2);
    status=MOTOR_AUTO_HEADING_REACHED;raw_frame(10,VISION_STATUS_OK,1,9,900);
    raw_frame(300,VISION_STATUS_OK,1,9,900);raw_frame(100,VISION_STATUS_OK,1,9,900);
    assert(starts==2 && !map.current_node);
    raw_frame(100,VISION_STATUS_OK,1,9,900);
    assert(!drive.aligning && starts==3 && mission.leg.phase==LEG_APPROACHING);
    assert(!strcmp(drive.result,"REFERENCE_ALIGNED") && mission.dfs_active);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(drive.started_ms==started && map.revision==revision && !map.current_node);
}
static void test_visible_reference_aligns_on_fresh_stopped_frames_then_verifies_arrival(void)
{
    static const int16_t bearings[]={1001,-1200,3500,4500};
    for(unsigned index=0;index<sizeof(bearings)/sizeof(bearings[0]);++index) {
        start_500_mm_leg();mission.dfs_active=true;
        uint32_t leg=mission.leg.leg_id,revision=map.revision,started=drive.started_ms;
        raw_bearing_frame(100,VISION_STATUS_OK,1,9,999,bearings[index]);
        assert(selection.reference_available && !selection.available);
        assert(drive.active && drive.aligning && drive.b_alignment_reference && !reverse_starts);
        assert(mission.leg.phase==LEG_ALIGNING && starts==1 && status==MOTOR_AUTO_STOPPED);
        char json[1024];assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
        printf("REFERENCE_ALIGNMENT_TRACKING_JSON:%s\n",json);
        char leg_json[384];assert(travel_leg_format_json(&mission.leg,leg_json,sizeof(leg_json)));
        printf("REFERENCE_ALIGNMENT_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":true,\"revision\":%lu,\"leg\":%s}\n",(unsigned long)mission.revision,leg_json);
        raw_bearing_frame(400,VISION_STATUS_OK,1,9,999,bearings[index]);
        assert(drive.alignment_rotating && alignment_heading==(bearings[index]<0?350:10));
        status=MOTOR_AUTO_HEADING_REACHED;raw_frame(10,VISION_STATUS_OK,1,9,999);
        raw_frame(300,VISION_STATUS_OK,1,9,999);
        assert(drive.alignment_frames==1 && starts==1 && !map.current_node);
        for(unsigned repeat=0;repeat<5;++repeat) onboard_drive_tick(&drive,&mission,&map,&selection,&inputs,now);
        assert(drive.alignment_frames==1 && starts==1);
        raw_frame(100,VISION_STATUS_OK,1,9,999);raw_frame(100,VISION_STATUS_OK,1,9,999);
        assert(!drive.aligning && mission.leg.phase==LEG_APPROACHING && starts==2);
        assert(!strcmp(drive.result,"REFERENCE_ALIGNED") && drive.started_ms==started);
        assert(map.revision==revision && mission.leg.leg_id==leg && !drive.verification_frames);
        raw_frame(100,VISION_STATUS_OK,1,9,650);
        raw_frame(300,VISION_STATUS_OK,1,9,600);raw_frame(100,VISION_STATUS_OK,1,9,600);
        assert(!map.current_node);
        raw_frame(100,VISION_STATUS_OK,1,9,600);
        assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
        assert(mission.state==MISSION_EXPLORING && mission.dfs_active && !reverse_starts);
    }
}
static void test_legacy_camera_hold_enters_marker_recovery_and_adopts_only_mapped_reference(void)
{
    setup();assert(rover_map_observe(&map,14,0,3)==MAP_OK);
    drive.frame_valid=true;drive.north_heading_deg=0;
    assert(!strcmp(start(),"STARTED"));mission.dfs_active=true;
    uint32_t leg=mission.leg.leg_id,started=drive.started_ms,revision=map.revision;
    assert(mission_camera_hold(&mission,true,now));
    drive.hold_started_ms=now-ONBOARD_DRIVE_CAMERA_RECOVERY_MS;
    drive.reference_model_valid=false;drive.last_target_range_mm=0;
    raw_frame(100,VISION_STATUS_OK,1,45,900);
    assert(drive.active && drive.reference_recovery && mission.leg.phase==LEG_HOLDING);
    assert(!strcmp(drive.result,"WAITING_FOR_TRACKING_MARKERS"));
    for(unsigned i=0;i<20;++i) raw_frame(100,VISION_STATUS_OK,1,45,900);
    assert(drive.active && mission.leg.phase==LEG_BACKING_UP && reverse_starts==1);
    confirm_recovery_marker(14,1400);
    assert(!drive.reference_recovery && mission.leg.phase==LEG_APPROACHING);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==14);
    assert(mission.leg.leg_id==leg && drive.started_ms==started && drive.reference_arrival_mm==1200);
    assert(map.revision==revision && !map.current_node && !rover_map_find_id(&map,14));
    assert(!rover_map_find_id(&map,2) && !drive.verification_frames && mission.dfs_active);
}
static void test_reference_alignment_can_switch_to_target_or_reverse_when_both_lost(void)
{
    start_500_mm_leg();raw_bearing_frame(100,VISION_STATUS_OK,1,9,999,1200);
    raw_bearing_frame(400,VISION_STATUS_OK,1,9,999,1200);
    assert(drive.alignment_rotating && drive.b_alignment_reference);
    raw_bearing_frame(100,VISION_STATUS_OK,1,2,500,-1200);
    assert(drive.aligning && !drive.b_alignment_reference && !drive.alignment_rotating);
    assert(status==MOTOR_AUTO_STOPPED && !drive.alignment_frames && !reverse_starts);
    raw_bearing_frame(400,VISION_STATUS_OK,1,2,500,-1200);
    assert(drive.alignment_rotating && alignment_heading==350 && starts==1);
    start_500_mm_leg();raw_bearing_frame(100,VISION_STATUS_OK,1,9,999,1200);
    raw_frame(100,VISION_STATUS_NO_MARKER,0,0,0);
    raw_frame(ONBOARD_DRIVE_CAMERA_RECOVERY_MS,VISION_STATUS_NO_MARKER,0,0,0);
    assert(drive.active && !drive.aligning && mission.leg.phase==LEG_BACKING_UP);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node);
    inputs.safety_latched=true;tick(20);
    assert(!drive.active && status==MOTOR_AUTO_STOPPED && mission.reason==MISSION_REASON_ESTOP);
}
static void test_camera_wait_then_marker_loss_keeps_dfs_and_can_resume_from_target(void)
{
    start_500_mm_leg();mission.dfs_active=true;observe_reference(999,100,0);
    uint32_t leg=mission.leg.leg_id,revision=map.revision,started=drive.started_ms;
    camera_gap();
    raw_frame(400,VISION_STATUS_OK,1,45,900);raw_frame(100,VISION_STATUS_OK,1,45,900);
    for(unsigned i=0;i<5 && drive.camera_waiting;++i) raw_frame(100,VISION_STATUS_OK,1,45,900);
    assert(drive.active && !drive.camera_waiting && drive.reference_recovery);
    assert(mission.leg.phase==LEG_HOLDING && !mission_motion_enabled(&mission));
    assert(drive.reference_progress_mm==101 && !map.current_node);
    char json[1024],leg_json[384];
    assert(onboard_drive_format(&drive,&mission,json,sizeof(json)));
    assert(travel_leg_format_json(&mission.leg,leg_json,sizeof(leg_json)));
    printf("MARKER_RECOVERY_TRACKING_JSON:%s\n",json);
    printf("MARKER_RECOVERY_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"START_DFS\",\"test_mode\":false,\"motion_enabled\":false,\"revision\":%lu,\"leg\":%s}\n",(unsigned long)mission.revision,leg_json);
    // A recognised destination returning during the stopped grace period can
    // be confirmed without an unnecessary reverse or a false arrival.
    for(unsigned i=0;i<10 && drive.reference_recovery;++i) raw_frame(100,VISION_STATUS_OK,1,2,500);
    assert(drive.active && !drive.reference_recovery && mission.leg.phase==LEG_APPROACHING);
    assert(starts==2 && !reverse_starts && mission.dfs_active && drive.started_ms==started);
    assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(map.revision==revision && !map.current_node && !drive.verification_frames);
}
static void test_reference_outside_threshold_is_aligned_even_with_target_visible(void)
{
    start_500_mm_leg();
    camera.frame.markers[1]=(camera_marker_t){.id=9,.range_mm=999,.bearing_cdeg=1200};
    raw_bearing_frame(100,VISION_STATUS_OK,2,2,500,800);
    assert(selection.available && selection.reference_available && !inputs.reference_absent);
    assert(drive.aligning && drive.b_alignment_reference && mission.leg.phase==LEG_ALIGNING);
    raw_bearing_frame(400,VISION_STATUS_OK,2,2,500,800);
    assert(drive.alignment_rotating && alignment_heading==10 && starts==1);
    status=MOTOR_AUTO_HEADING_REACHED;
    camera.frame.markers[1].bearing_cdeg=200;
    raw_bearing_frame(10,VISION_STATUS_OK,2,2,500,-200);
    raw_bearing_frame(300,VISION_STATUS_OK,2,2,500,-200);
    raw_bearing_frame(100,VISION_STATUS_OK,2,2,500,-200);
    raw_bearing_frame(100,VISION_STATUS_OK,2,2,500,-200);
    assert(drive.active && !drive.aligning && starts==2 && !reverse_starts);
    assert(!strcmp(drive.result,"REFERENCE_ALIGNED") && !map.current_node);
}

int main(void)
{
    test_camera_health_outage_keeps_leg_and_resumes_only_after_fresh_observations();
    test_stalled_health_packet_during_arrival_verification_does_not_cancel_dfs();
    test_camera_health_recovery_does_not_bypass_marker_identity_or_operator_stop();
    test_input_recovery_resumes_alignment_reverse_and_arrival_verification();
    test_live_unusable_reference_reverses_instead_of_camera_timeout();
    test_visible_reference_aligns_on_fresh_stopped_frames_then_verifies_arrival();
    test_legacy_camera_hold_enters_marker_recovery_and_adopts_only_mapped_reference();
    test_reference_alignment_can_switch_to_target_or_reverse_when_both_lost();
    test_camera_wait_then_marker_loss_keeps_dfs_and_can_resume_from_target();
    test_reference_outside_threshold_is_aligned_even_with_target_visible();
    test_arrival_corrects_both_directions_and_reverifies_before_commit();
    test_input_outages_wait_indefinitely_preserve_leg_and_resume_with_fresh_images();
    test_input_loss_and_stale_camera_interrupt_arrival_corrections();
    test_reverse_uses_fresh_valid_markers_and_prioritises_locked_reference();
    test_reverse_relocalisation_respects_all_four_grid_directions();
    test_unknown_camera_after_stale_wait_resumes_reverse_search();
    test_reverse_ignores_unknown_and_bad_markers_then_recovers_original_reference();
    test_alternate_mapped_reference_uses_grid_distance_and_confirms_only_target();
    test_mapped_off_route_marker_holds_without_claiming_location();
    test_mapped_destination_reacquisition_resumes_without_false_arrival();
    test_alignment_target_loss_uses_visible_reference_without_reverse();
    test_alignment_reverse_recovers_locked_target_without_a_map_entry();
    test_alignment_reverse_realigns_recovered_target_before_forward_motion();
    test_alignment_reverse_adopts_compatible_mapped_marker();
    test_alignment_reverse_retains_camera_wait_and_operator_stop();
    test_stale_camera_waits_without_cancelling_dfs_and_resumes_fresh();
    test_camera_wait_freezes_outage_time_without_resetting_spent_journey_budget();
    test_camera_age_interrupts_reverse_without_restarting_the_pulse();
    test_target_alignment_turns_then_verifies_fresh_stopped_images();
    test_start_and_approach_share_ten_degree_selection_threshold();
    test_predeparture_alignment_confirms_images_before_starting_journey();
    test_predeparture_alignment_waits_for_camera_and_keeps_stop_authoritative();
    test_predeparture_target_loss_reverses_and_records_actual_departure();
    test_alignment_retries_past_old_limits_and_preserves_fault_checks();
    test_alignment_preserves_reference_search_budget();
    test_start_and_drive_preserve_locked_leg();
    test_refusals_have_no_map_or_motion_side_effects();
    test_stop_limits_and_lost_inputs();
    test_reference_can_be_added_during_approach();
    test_healthy_no_return_is_not_a_fault_or_an_obstacle();
    test_lease_and_clock_wrap();
    test_reference_progress_brake_verify_and_commit_once();
    test_reference_failures_do_not_commit_arrival();
    test_large_reference_range_changes_keep_the_locked_journey();
    test_verification_requires_post_settle_images_and_consistent_map();
    test_grid_heading_planning();
    test_reference_arrival_uses_600_mm_instead_of_last_target_range();
    test_camera_gap_brakes_retains_leg_and_resumes_on_distinct_images();
    test_camera_hold_timeout_stop_and_fault_cannot_resume();
    test_selector_keeps_locked_reference_while_holding();
    test_camera_gaps_during_verification_never_restart_wheels();
    test_recovery_ignores_delayed_pre_brake_images();
    test_verification_keeps_good_samples_across_cached_frame_expiry();
    test_recovery_keeps_good_image_across_cached_frame_expiry();
    test_missing_reference_waits_two_seconds_then_continuous_reverse();
    test_reference_return_before_two_seconds_never_reverses();
    test_reverse_without_rear_data_overall_deadline_and_retry();
    test_recovery_cannot_override_faults_cancel_or_camera_failure();
    test_backup_stops_early_on_new_reference_image();
    test_brief_reference_return_restarts_missing_timer();
    test_selector_keeps_reference_locked_during_reverse_and_reacquisition();
    test_reference_reverse_waits_for_fresh_absence_after_two_second_boundary();
    test_reference_frame_wait_retains_leg_and_respects_cancellation();
    test_slow_empty_camera_search_late_reference_600_mm_arrival();
    test_front_motor_stop_confirms_terminal_target_without_camera_reference();
    test_search_clearance_fault_cancel_and_budgets_do_not_fake_arrival();
    test_search_camera_gap_brakes_and_recovers_on_new_empty_images();
    test_unconfirmed_front_pulse_and_manual_estop_never_commit_arrival();
    test_no_front_return_still_has_time_sensor_and_obstacle_guards();
    puts("Onboard reference progress, brake/verify, single arrival commit, bounded drive and failure tests passed");
    return 0;
}
