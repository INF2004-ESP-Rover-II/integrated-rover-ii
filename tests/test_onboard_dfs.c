#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "onboard_dfs.h"
#include "motor_driver.h"

static uint32_t now;
static unsigned drive_starts,turns,stops,renewals;
static bool lease_ok;
static int turn_target;
static motor_auto_status_t motor_status;
motor_error_t motor_auto_drive_leased(int speed,uint16_t lease) {
    assert(speed && speed>=-100 && speed<=100 && lease==ONBOARD_DRIVE_LEASE_MS);drive_starts++;motor_status=MOTOR_AUTO_RUNNING;return MOTOR_OK;
}
motor_error_t motor_auto_rotate_leased(int heading,uint16_t lease) {
    assert(heading>=0 && heading<360 && lease==ONBOARD_DRIVE_LEASE_MS);turns++;turn_target=heading;motor_status=MOTOR_AUTO_RUNNING;return MOTOR_OK;
}
motor_error_t motor_emergency_stop(void) { stops++;motor_status=MOTOR_AUTO_STOPPED;return MOTOR_OK; }
motor_auto_status_t motor_auto_status(void) { return motor_status; }
bool motor_auto_renew_lease(void) { renewals++; return lease_ok && motor_status==MOTOR_AUTO_RUNNING; }

static rover_map_t map;
static mission_t mission;
static onboard_drive_t drive;
static target_selector_t selection;
static onboard_dfs_t dfs;
static onboard_dfs_inputs_t inputs;
static camera_observation_t camera;

static void setup(void)
{
    now=1000;drive_starts=turns=stops=renewals=0;lease_ok=true;motor_status=MOTOR_AUTO_IDLE;
    rover_map_init(&map,123);assert(rover_map_start(&map,1)==MAP_OK);
    mission_init(&mission,now);onboard_drive_init(&drive);target_selector_init(&selection);onboard_dfs_init(&dfs);
    inputs=(onboard_dfs_inputs_t){.drive={.stopped=true,.sensor_valid=true,.sensor_received_ms=now,.imu_ok=true,
        .link_ok=true,.camera_fresh=true,.camera_present=true,.camera_status=VISION_STATUS_OK,.front_mm=1023},
        .tof_mm={50,1023,50}};
    camera=(camera_observation_t){.generation=1,.received_ms=now,
        .frame={.seq=250,.status=VISION_STATUS_OK,.age_ms=100,.count=2,
            .markers={{.id=2,.range_mm=500,.bearing_cdeg=100},{.id=9,.range_mm=900,.bearing_cdeg=150}}}};
}
static void start(void)
{
    assert(!strcmp(onboard_dfs_start(&dfs,&mission,&map,&drive,&selection,&inputs,now),"STARTED"));
    assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
}
static void step(uint32_t dt)
{
    now+=dt;inputs.drive.sensor_received_ms=now;
    camera.received_ms=now;camera.generation++;camera.frame.seq++;
    inputs.drive.camera_generation=camera.generation;
    inputs.drive.camera_status=camera.frame.status;
    inputs.drive.camera_fresh=camera.frame.status==VISION_STATUS_OK;
    inputs.drive.camera_packet_age_ms=0; inputs.drive.camera_image_age_ms=camera.frame.age_ms;
    if (dfs.stage==DFS_TURNING && motor_status==MOTOR_AUTO_RUNNING) {
        inputs.drive.heading_deg=(float)turn_target;inputs.drive.stopped=true;motor_status=MOTOR_AUTO_HEADING_REACHED;
    }
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
    if (drive.active || onboard_dfs_selecting(&dfs,now))
        target_selector_tick(&selection,&mission,&map,&camera,inputs.drive.stopped,now);
    onboard_drive_tick(&drive,&mission,&map,&selection,&inputs.drive,now);
    inputs.drive.stopped=motor_status!=MOTOR_AUTO_RUNNING;
}

static void block_all(map_node_t *node) { for(unsigned i=0;i<4;++i) node->exits[i]=MAP_EXIT_BLOCKED; }
static void connect(uint8_t a,map_direction_t dir,uint8_t b)
{
    assert(rover_map_set_exit(&map,a,dir,MAP_EXIT_OPEN)==MAP_OK);
    assert(rover_map_set_exit(&map,b,(map_direction_t)(((unsigned)dir+2u)%4u),MAP_EXIT_OPEN)==MAP_OK);
}
static void simulated_leg(dfs_choice_t choice)
{
    uint8_t from=map.current_node;
    dfs.from_node=from;dfs.expected_id=choice.expected_id;dfs.backtracking=choice.kind==DFS_CHOOSE_BACKTRACK;
    const map_node_t *node=rover_map_find_id(&map,from);
    static const int8_t dx[]={0,1,0,-1},dy[]={1,0,-1,0};
    const map_node_t *next=rover_map_find_cell(&map,(int16_t)(node->x+dx[choice.direction]),(int16_t)(node->y+dy[choice.direction]));
    assert(next);uint8_t id=next->marker_id;
    rover_map_depart(&map);assert(rover_map_arrive(&map,id)==MAP_OK);
    assert(rover_map_set_exit(&map,from,choice.direction,MAP_EXIT_TRAVERSED)==MAP_OK);
    assert(rover_map_set_exit(&map,id,(map_direction_t)(((unsigned)choice.direction+2u)%4u),MAP_EXIT_TRAVERSED)==MAP_OK);
    assert(onboard_dfs_note_arrival(&dfs,&map));
    dfs.scanned[dfs.depth-1u]=15;
}

static void test_depth_first_branch_dead_end_backtrack_and_cycle(void)
{
    setup();
    assert(rover_map_add(&map,2,0,1,false)==MAP_OK);
    assert(rover_map_add(&map,3,1,1,false)==MAP_OK);
    assert(rover_map_add(&map,4,1,0,false)==MAP_OK);
    assert(rover_map_add(&map,5,0,2,false)==MAP_OK);
    for(size_t i=0;i<map.count;++i) block_all(&map.nodes[i]);
    connect(1,MAP_NORTH,2);connect(2,MAP_NORTH,5);connect(2,MAP_EAST,3);connect(3,MAP_SOUTH,4);connect(4,MAP_WEST,1);
    drive.frame_valid=true;start();dfs.scanned[0]=15;
    const uint8_t expected[]={2,5,2,3,4,3,2,1};unsigned count=0;
    map_direction_t heading=MAP_NORTH;
    for(unsigned safety=0;safety<30;++safety) {
        dfs_choice_t choice=onboard_dfs_choose(&dfs,&map,heading,false);
        if(choice.kind==DFS_CHOOSE_COMPLETE) {
            assert(count==sizeof(expected) && map.current_node==1 && dfs.depth==1);
            assert(mission.state==MISSION_EXPLORING); // Leg/graph arrivals never complete the mission.
            assert(mission_finish_dfs(&mission,now));
            assert(mission.state==MISSION_COMPLETE && mission.reason==MISSION_REASON_DFS_COMPLETE);
            char json[1024];assert(rover_map_format(&map,"",json,sizeof(json)));printf("COMPLETE_MAP_JSON:%s\n",json);
            return;
        }
        assert(choice.kind==DFS_CHOOSE_EXPLORE || choice.kind==DFS_CHOOSE_BACKTRACK);
        simulated_leg(choice);assert(count<sizeof(expected) && map.current_node==expected[count++]);heading=choice.direction;
    }
    assert(!"DFS did not terminate");
}

static void test_unknown_exits_and_open_frontier_never_mean_complete(void)
{
    setup();start();dfs.scanned[0]=15;
    assert(onboard_dfs_choose(&dfs,&map,MAP_NORTH,false).kind==DFS_CHOOSE_UNRESOLVED);
    for(unsigned i=0;i<4;++i) assert(rover_map_set_exit(&map,1,(map_direction_t)i,MAP_EXIT_BLOCKED)==MAP_OK);
    assert(onboard_dfs_choose(&dfs,&map,MAP_NORTH,false).kind==DFS_CHOOSE_COMPLETE);
    assert(rover_map_observe(&map,9,0,1)==MAP_OK);
    assert(onboard_dfs_choose(&dfs,&map,MAP_NORTH,false).kind==DFS_CHOOSE_UNRESOLVED);
    map.observed_count=0;
    assert(rover_map_set_exit(&map,1,MAP_EAST,MAP_EXIT_OPEN)==MAP_OK);
    dfs_choice_t choice=onboard_dfs_choose(&dfs,&map,MAP_NORTH,false);
    assert(choice.kind==DFS_CHOOSE_EXPLORE && choice.direction==MAP_EAST && choice.expected_id==0);
    assert(rover_map_set_exit(&map,1,MAP_EAST,MAP_EXIT_UNKNOWN)==MAP_OK);
    choice=onboard_dfs_choose(&dfs,&map,MAP_EAST,true);
    assert(choice.kind==DFS_CHOOSE_EXPLORE && choice.direction==MAP_EAST);
    assert(map.nodes[0].exits[MAP_EAST]==MAP_EXIT_UNKNOWN); // Camera hint does not fabricate an open/map node.
}

static void test_controller_reads_tof_and_completes_only_closed_resolved_map(void)
{
    setup();inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=50;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<100 && dfs.active;++i) step(100);
    assert(!dfs.active && dfs.stage==DFS_FINISHED && mission.state==MISSION_COMPLETE);
    assert(mission.reason==MISSION_REASON_DFS_COMPLETE && drive_starts==0 && turns==1);
    assert(turn_target==180); // Only the unseen rear needs a scan; side walls were already measured.
    assert(map.current_node==1 && map.count==1);
    for(unsigned dir=0;dir<4;++dir) assert(map.nodes[0].exits[dir]==MAP_EXIT_BLOCKED);
    setup();inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=0;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<100 && dfs.active;++i) step(100);
    assert(!dfs.active && dfs.stage==DFS_STOPPED && mission.state==MISSION_PAUSED);
    assert(!strcmp(dfs.detail,"DFS_EXITS_UNRESOLVED") && drive_starts==0);
    for(unsigned dir=0;dir<4;++dir) assert(map.nodes[0].exits[dir]==MAP_EXIT_UNKNOWN);
}

static void test_real_leg_arrival_returns_to_scanning_not_mission_complete(void)
{
    setup();start();
    for(unsigned i=0;i<50 && !drive.active;++i) step(100);
    assert(drive.active && dfs.stage==DFS_TRAVELLING && drive_starts==1);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
    assert(map.current_node==0 && dfs.depth==1);
    camera.frame.count=1;camera.frame.markers[0]=(camera_marker_t){.id=9,.range_mm=800,.bearing_cdeg=150};step(100);
    camera.frame.markers[0].range_mm=700;step(100);
    camera.frame.markers[0].range_mm=650;step(100);
    assert(mission.leg.phase==LEG_BRAKING && drive.active);
    camera.frame.markers[0].range_mm=600;step(400);step(100);step(100);
    assert(!drive.active && map.current_node==2 && mission.leg.phase==LEG_COMPLETE);
    assert(mission.state==MISSION_EXPLORING && mission.dfs_active && dfs.active);
    step(100);
    assert(dfs.stage==DFS_SAMPLING && dfs.depth==2 && mission.leg.phase==LEG_SCANNING);
    assert(mission.state==MISSION_EXPLORING && map.current_node==2 && map.count==2);
    char json[384];assert(onboard_dfs_format(&dfs,&map,json,sizeof(json)));printf("DFS_JSON:%s\n",json);
    char leg[384],snapshot[768];assert(travel_leg_format_json(&mission.leg,leg,sizeof(leg)));
    printf("NODE_MISSION_JSON:{\"state\":\"EXPLORING\",\"reason\":\"TARGET_ARRIVED\",\"test_mode\":false,\"motion_enabled\":false,\"revision\":%lu,\"leg\":%s}\n",
        (unsigned long)mission.revision,leg);
    assert(rover_map_format(&map,"",snapshot,sizeof(snapshot)));printf("NODE_MAP_JSON:%s\n",snapshot);
}

static void test_cancel_fault_and_no_target_cannot_fake_completion(void)
{
    setup();rover_map_init(&map,123);
    assert(!strcmp(onboard_dfs_start(&dfs,&mission,&map,&drive,&selection,&inputs,now),"START_NODE_NOT_SET"));
    assert(!dfs.active && drive_starts==0 && turns==0);
    setup();start();mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);step(100);
    assert(!dfs.active && mission.state==MISSION_PAUSED && drive_starts==0);
    setup();start();inputs.drive.sensor_valid=false;step(100);
    assert(dfs.active && dfs.input_waiting && mission.state==MISSION_EXPLORING && mission.leg.phase==LEG_WAITING_INPUT);
    setup();start();camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;
    for(unsigned i=0;i<100 && dfs.active;++i) step(100);
    assert(!dfs.active && mission.state==MISSION_PAUSED && !strcmp(dfs.detail,"DFS_TARGET_NOT_VISIBLE"));
    assert(drive_starts==0 && map.count==1);
}

static void test_turn_completion_waits_for_idle_snapshot(void)
{
    setup();inputs.tof_mm[1]=50;inputs.tof_mm[2]=1023;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<30 && dfs.stage!=DFS_TURNING;++i) step(100);
    assert(dfs.stage==DFS_TURNING && turn_target==90);
    motor_status=MOTOR_AUTO_HEADING_REACHED;inputs.drive.stopped=false;
    now+=10;inputs.drive.sensor_received_ms=now;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
    assert(dfs.active && dfs.stage==DFS_TURNING);
    inputs.drive.stopped=true;inputs.drive.heading_deg=90;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
    assert(dfs.active && dfs.stage==DFS_WAIT_TARGET && selection.route.direction==LEG_EAST);
}

static void test_terminal_obstacle_arrival_returns_to_dfs_scanning(void)
{
    setup();camera.frame.count=1;start();
    for(unsigned i=0;i<50 && !drive.active;++i) step(100);
    assert(drive.active && !mission.leg.arrival_reference_node && map.current_node==0);
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;step(100);
    assert(mission.leg.phase==LEG_SEARCHING_REFERENCE && drive.search_active);
    inputs.drive.front_mm=inputs.tof_mm[1]=100;inputs.drive.front_urgent=true;
    motor_emergency_stop();assert(onboard_drive_note_front_stop(&drive,&mission));
    step(100);assert(mission.leg.phase==LEG_OBSTACLE_VERIFYING && map.current_node==0);
    step(300);step(100);
    assert(!drive.active && map.current_node==2 && mission.state==MISSION_EXPLORING && mission.dfs_active);
    assert(rover_map_find_id(&map,2)->visited && rover_map_find_id(&map,2)->exits[MAP_NORTH]==MAP_EXIT_BLOCKED);
    step(100);
    assert(dfs.active && dfs.depth==2 && dfs.stage==DFS_SAMPLING && mission.leg.phase==LEG_SCANNING);
    assert(mission.state==MISSION_EXPLORING && map.current_node==2);
    char json[ROVER_MAP_JSON_CAPACITY]; assert(rover_map_format(&map,"",json,sizeof(json)));
    printf("TERMINAL_MAP_JSON:%s\n",json);
}

static void test_blocked_sides_do_not_get_redundant_scan_turns(void)
{
    setup();start();
    assert(rover_map_set_exit(&map,1,MAP_EAST,MAP_EXIT_BLOCKED)==MAP_OK);
    assert(rover_map_set_exit(&map,1,MAP_WEST,MAP_EXIT_BLOCKED)==MAP_OK);
    dfs.scanned[0]=(uint8_t)(1u<<MAP_NORTH);
    dfs_choice_t choice=onboard_dfs_choose(&dfs,&map,MAP_NORTH,false);
    assert(choice.kind==DFS_CHOOSE_SCAN && choice.direction==MAP_SOUTH); // Unknown rear, never the 200 mm side walls.
    choice=onboard_dfs_choose(&dfs,&map,MAP_NORTH,true);
    assert(choice.kind==DFS_CHOOSE_EXPLORE && choice.direction==MAP_NORTH); // Camera-supported forward route has priority.
    for(unsigned missing=0;missing<2;++missing) {
        setup();inputs.tof_mm[0]=inputs.tof_mm[2]=200;inputs.tof_mm[1]=missing?0:1023;
        inputs.drive.front_mm=inputs.tof_mm[1];start();
        for(unsigned i=0;i<50 && !drive.active;++i) step(100);
        assert(drive.active && turns==0 && mission.leg.direction==LEG_NORTH && mission.leg.next_target_node==2);
        assert(rover_map_find_id(&map,1)->exits[MAP_EAST]==MAP_EXIT_BLOCKED);
        assert(rover_map_find_id(&map,1)->exits[MAP_WEST]==MAP_EXIT_BLOCKED);
    }
    setup();inputs.tof_mm[0]=inputs.tof_mm[2]=200;inputs.tof_mm[1]=0;
    inputs.drive.front_mm=0;camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<30 && dfs.stage!=DFS_TURNING;++i) step(100);
    assert(dfs.stage==DFS_TURNING && turn_target==180 && turns==1 && drive_starts==0);
}

static void start_east_turn(void)
{
    setup();inputs.tof_mm[1]=50;inputs.tof_mm[2]=1023;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<30 && dfs.stage!=DFS_TURNING;++i) step(100);
    assert(dfs.stage==DFS_TURNING && dfs.direction==MAP_EAST);
}
static void tick_turn_at(uint32_t time_ms, bool stopped)
{
    now=time_ms;inputs.drive.sensor_received_ms=now;inputs.drive.stopped=stopped;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
}
static void test_dfs_turn_retries_after_three_seconds_with_five_second_limit(void)
{
    start_east_turn();
    uint32_t started=dfs.stage_started_ms,revision=map.revision;
    unsigned before=turns,before_stops=stops,before_renewals=renewals;
    tick_turn_at(started+2999,false);
    assert(dfs.active && motor_status==MOTOR_AUTO_RUNNING && turns==before);
    assert(renewals>before_renewals && stops==before_stops);
    tick_turn_at(started+3000,false);
    assert(dfs.active && dfs.b_turn_retry_waiting && !dfs.input_waiting);
    assert(motor_status==MOTOR_AUTO_STOPPED && stops==before_stops+1);
    assert(!mission_motion_enabled(&mission) && !strcmp(dfs.detail,"DFS_TURN_RETRY_WAIT"));
    tick_turn_at(started+3299,true);
    assert(turns==before && dfs.b_turn_retry_waiting);
    // Retry waits for a fresh stopped snapshot after the settling period.
    now=started+3300;inputs.drive.sensor_received_ms=started+3299;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
    assert(turns==before);
    tick_turn_at(started+3300,false);
    assert(turns==before);
    tick_turn_at(started+3300,true);
    assert(turns==before+1 && turn_target==90 && motor_status==MOTOR_AUTO_RUNNING);
    assert(dfs.stage_started_ms==started && dfs.turn_attempt_started_ms==now);
    assert(!dfs.b_turn_retry_waiting && mission_motion_enabled(&mission));
    assert(!strcmp(dfs.detail,"DFS_TURN_RETRYING"));
    tick_turn_at(started+4999,false);
    assert(dfs.active && mission.state==MISSION_EXPLORING);
    tick_turn_at(started+5000,false);
    assert(!dfs.active && dfs.stage==DFS_STOPPED && mission.state==MISSION_PAUSED);
    assert(!strcmp(dfs.detail,"DFS_TURN_TIMEOUT") && motor_status==MOTOR_AUTO_STOPPED);
    assert(map.current_node==1 && map.revision==revision && map.count==1);
    assert(dfs.depth==1 && dfs.stack[0]==1 && !drive_starts && turns==before+1);
}
static void test_dfs_turn_success_before_or_after_retry_and_at_deadline(void)
{
    for(unsigned retry=0;retry<3;++retry) {
        start_east_turn();uint32_t started=dfs.stage_started_ms;
        if(retry) {
            tick_turn_at(started+3000,false);
            tick_turn_at(started+3300,true);
        }
        inputs.drive.heading_deg=(float)turn_target;
        motor_status=MOTOR_AUTO_HEADING_REACHED;
        tick_turn_at(started+(retry==2?5000:4000),true);
        assert(dfs.active && dfs.stage==DFS_WAIT_TARGET && mission.state==MISSION_EXPLORING);
        assert(motor_status==MOTOR_AUTO_STOPPED && !drive_starts && map.current_node==1);
    }
    // A heading reached during the stop/settle period needs no second command.
    start_east_turn();uint32_t started=dfs.stage_started_ms;unsigned before=turns;
    tick_turn_at(started+3000,false);inputs.drive.heading_deg=90;
    tick_turn_at(started+3300,true);
    assert(dfs.active && dfs.stage==DFS_WAIT_TARGET && turns==before);
}
static void test_dfs_turn_retry_wait_is_bounded_and_timer_wraps(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        start_east_turn();
        uint32_t started=wrap?UINT32_MAX-2000:dfs.stage_started_ms;
        dfs.stage_started_ms=dfs.turn_attempt_started_ms=dfs.last_renew_ms=started;
        tick_turn_at(started+3000u,false);
        assert(dfs.b_turn_retry_waiting && dfs.active);
        // Even a motor that never confirms stopped cannot extend the limit.
        tick_turn_at(started+5000u,false);
        assert(!dfs.active && !strcmp(dfs.detail,"DFS_TURN_TIMEOUT"));
    }
}
static void test_dfs_turn_input_wait_preserves_used_turn_allowance(void)
{
    start_east_turn();uint32_t started=dfs.stage_started_ms;
    tick_turn_at(started+2000,false);
    inputs.drive.link_ok=false;tick_turn_at(started+2100,false);
    assert(dfs.input_waiting && dfs.active);
    tick_turn_at(started+62100,true);
    inputs.drive.link_ok=true;tick_turn_at(started+62200,true);
    assert(dfs.active && !dfs.input_waiting && dfs.stage==DFS_TURNING);
    assert(dfs.stage_started_ms==started+60100);
    tick_turn_at(dfs.stage_started_ms+4999,false);
    assert(dfs.active);
    tick_turn_at(dfs.stage_started_ms+5000,false);
    assert(!dfs.active && !strcmp(dfs.detail,"DFS_TURN_TIMEOUT"));
}
static void test_dfs_turn_still_waits_on_stopped_motor_or_failed_lease(void)
{
    for(unsigned expired=0;expired<2;++expired) {
        start_east_turn();
        if(expired) lease_ok=false;
        else motor_status=MOTOR_AUTO_STOPPED;
        now+=100;inputs.drive.sensor_received_ms=now;
        onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
        assert(dfs.active && dfs.input_waiting && mission.state==MISSION_EXPLORING);
        assert(!strcmp(dfs.detail,expired?"DFS_TURN_LEASE_EXPIRED":"DFS_TURN_MOTOR_WAIT"));
        assert(!mission_motion_enabled(&mission) && motor_status==MOTOR_AUTO_STOPPED && map.current_node==1);
    }
    start_east_turn();mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);step(100);
    assert(!dfs.active && dfs.stage==DFS_STOPPED && !drive_starts && motor_status==MOTOR_AUTO_STOPPED);
}

static void test_dfs_scan_and_turn_wait_for_inputs_without_losing_stack(void)
{
    for(unsigned cause=0;cause<3;++cause) {
        setup();start();step(100);
        assert(dfs.stage==DFS_SAMPLING);
        uint32_t revision=map.revision;
        if(cause==0) inputs.drive.link_ok=false;
        if(cause==1) inputs.drive.sensor_valid=false;
        if(cause==2) inputs.drive.imu_ok=false;
        step(100);step(60000);
        assert(dfs.active && dfs.input_waiting && mission.dfs_active && mission.state==MISSION_EXPLORING);
        assert(mission.leg.phase==LEG_WAITING_INPUT && map.current_node==1 && map.revision==revision);
        assert(dfs.depth==1 && dfs.stack[0]==1 && !drive_starts);
        char json[512];assert(onboard_dfs_format(&dfs,&map,json,sizeof(json)));
        printf("DFS_INPUT_WAIT_JSON:%s\n",json);
        inputs.drive.link_ok=inputs.drive.sensor_valid=inputs.drive.imu_ok=true;
        step(100);
        assert(dfs.active && !dfs.input_waiting && dfs.stage==DFS_SAMPLING && mission.leg.phase==LEG_SCANNING);
        assert(dfs.depth==1 && dfs.stack[0]==1 && map.revision==revision);
    }
    setup();inputs.tof_mm[1]=50;inputs.tof_mm[2]=1023;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<30 && dfs.stage!=DFS_TURNING;++i) step(100);
    assert(dfs.stage==DFS_TURNING);
    unsigned before=turns;motor_status=MOTOR_AUTO_STOPPED;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
    assert(dfs.input_waiting && dfs.active);
    step(60000);
    assert(dfs.active && !dfs.input_waiting && dfs.stage==DFS_TURNING && turns==before+1);
    assert(dfs.depth==1 && dfs.stack[0]==1 && map.current_node==1);
    step(100);
    assert(dfs.active && (dfs.stage==DFS_SAMPLING || dfs.stage==DFS_WAIT_TARGET) && !drive_starts);
    setup();start();inputs.drive.link_ok=false;step(100);
    assert(dfs.input_waiting);
    mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);inputs.drive.link_ok=true;step(60000);
    assert(!dfs.active && !dfs.input_waiting && dfs.stage==DFS_STOPPED && !drive_starts);
}

static void test_camera_health_outages_at_node_wait_and_then_select_the_same_exit(void)
{
    for(unsigned cause=0;cause<3;++cause) {
        setup();start();
        for(unsigned i=0;i<40 && dfs.stage!=DFS_WAIT_TARGET;++i) step(100);
        assert(dfs.stage==DFS_WAIT_TARGET && !drive.active);
        uint32_t revision=map.revision;
        if(cause<2) {
            camera.frame.status=cause==0?VISION_STATUS_STALLED:VISION_STATUS_CAMERA_LOST;
            camera.frame.count=0;
            step(60000);
        } else {
            now+=60000;inputs.drive.sensor_received_ms=now;
            onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,NULL,now);
        }
        assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
        assert(dfs.stage==DFS_WAIT_TARGET && !strcmp(dfs.detail,"WAITING_FOR_FRESH_CAMERA"));
        assert(dfs.depth==1 && dfs.stack[0]==1 && map.current_node==1 && map.revision==revision);
        assert(!drive.active && !drive_starts && motor_status!=MOTOR_AUTO_RUNNING);
        camera.frame.status=VISION_STATUS_OK;camera.frame.count=2;camera.frame.age_ms=100;
        camera.frame.markers[0]=(camera_marker_t){.id=2,.range_mm=500,.bearing_cdeg=100};
        camera.frame.markers[1]=(camera_marker_t){.id=9,.range_mm=900,.bearing_cdeg=150};
        for(unsigned i=0;i<12 && !drive.active;++i) step(200);
        assert(dfs.active && drive.active && dfs.stage==DFS_TRAVELLING);
        assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9 && !map.current_node);
    }
}

static void test_selected_target_drifts_outside_ten_degrees_and_aligns_before_departure(void)
{
    // Replay the reported 5.31-degree case through the actual DFS controller.
    setup();camera.frame.markers[0].bearing_cdeg=531;
    camera.frame.markers[1].bearing_cdeg=782;start();
    for(unsigned i=0;i<40 && !drive.active;++i) step(100);
    assert(dfs.active && drive.active && dfs.stage==DFS_TRAVELLING);
    assert(!drive.aligning && drive_starts==1 && mission.state==MISSION_EXPLORING);
    assert(mission.leg.next_target_node==2 && !map.current_node);

    setup();start();
    for(unsigned i=0;i<40 && dfs.stage!=DFS_WAIT_TARGET;++i) step(100);
    // Then simulate a locked target drifting outside the shared threshold.
    camera.frame.markers[0].bearing_cdeg=531;
    camera.frame.markers[1].bearing_cdeg=782;
    for(unsigned i=0;i<10 && selection.state!=TARGET_LOCKED;++i) step(100);
    assert(selection.state==TARGET_LOCKED && !drive.active && map.current_node==1);
    selection.bearing_cdeg=selection.reference_bearing_cdeg=1200;
    now+=20;inputs.drive.sensor_received_ms=now;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
    assert(dfs.active && dfs.stage==DFS_TRAVELLING && drive.active && drive.aligning);
    assert(mission.state==MISSION_EXPLORING && mission.leg.phase==LEG_ALIGNING);
    assert(drive.b_departure_pending && !drive_starts && map.current_node==1);
    for(unsigned i=0;i<20 && drive.aligning;++i) step(100);
    assert(dfs.active && drive.active && !drive.aligning && drive_starts==1);
    assert(!drive.b_departure_pending && mission.leg.phase==LEG_APPROACHING);
    assert(mission.leg.next_target_node==2 && !map.current_node && dfs.depth==1);
}

static void test_missing_tracking_markers_recover_without_cancelling_dfs_stack(void)
{
    setup();start();for(unsigned i=0;i<40 && !drive.active;++i) step(100);
    assert(drive.active && dfs.stage==DFS_TRAVELLING);
    uint32_t leg=mission.leg.leg_id,revision=map.revision;uint8_t depth=dfs.depth;
    camera.frame.count=1;camera.frame.markers[0]=(camera_marker_t){.id=45,.range_mm=900};
    // The raw-absence flag is false: unavailable tracking observations must
    // still enter recovery, including after the old 1.5-second deadline.
    inputs.drive.reference_absent=false;
    for(unsigned i=0;i<25;++i) step(100);
    assert(drive.active && drive.reference_recovery && mission.leg.phase==LEG_BACKING_UP);
    assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
    assert(dfs.depth==depth && dfs.stack[0]==1 && dfs.stage==DFS_TRAVELLING);
    assert(mission.leg.leg_id==leg && mission.leg.next_target_node==2 && !map.current_node);
    assert(map.revision==revision && map.count==1 && !drive.verification_frames);
    camera.frame.markers[0]=(camera_marker_t){.id=9,.range_mm=900};
    for(unsigned i=0;i<20 && drive.reference_recovery;++i) step(100);
    assert(drive.active && !drive.reference_recovery && mission.leg.phase==LEG_APPROACHING);
    assert(dfs.active && dfs.depth==depth && mission.leg.next_target_node==2);
    assert(map.revision==revision && !map.current_node && mission.state==MISSION_EXPLORING);
}

int main(void)
{
    test_camera_health_outages_at_node_wait_and_then_select_the_same_exit();
    test_dfs_scan_and_turn_wait_for_inputs_without_losing_stack();
    setup(); start();
    for (unsigned i=0;i<40 && dfs.stage!=DFS_WAIT_TARGET;++i) step(100);
    assert(dfs.stage==DFS_WAIT_TARGET && !drive.active);
    camera.frame.age_ms=751;
    for (unsigned i=0;i<70;++i) step(100);
    assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING && dfs.stage==DFS_WAIT_TARGET);
    assert(!strcmp(dfs.detail,"WAITING_FOR_FRESH_CAMERA") && map.current_node==1 && !drive_starts);
    camera.frame.age_ms=100;
    for (unsigned i=0;i<10 && !drive.active;++i) step(200);
    assert(dfs.active && drive.active && dfs.stage==DFS_TRAVELLING && mission.leg.next_target_node==2);
    test_depth_first_branch_dead_end_backtrack_and_cycle();
    test_unknown_exits_and_open_frontier_never_mean_complete();
    test_controller_reads_tof_and_completes_only_closed_resolved_map();
    test_real_leg_arrival_returns_to_scanning_not_mission_complete();
    test_cancel_fault_and_no_target_cannot_fake_completion();
    test_turn_completion_waits_for_idle_snapshot();
    test_terminal_obstacle_arrival_returns_to_dfs_scanning();
    test_blocked_sides_do_not_get_redundant_scan_turns();
    test_dfs_turn_retries_after_three_seconds_with_five_second_limit();
    test_dfs_turn_success_before_or_after_retry_and_at_deadline();
    test_dfs_turn_retry_wait_is_bounded_and_timer_wraps();
    test_dfs_turn_input_wait_preserves_used_turn_allowance();
    test_dfs_turn_still_waits_on_stopped_motor_or_failed_lease();
    test_selected_target_drifts_outside_ten_degrees_and_aligns_before_departure();
    test_missing_tracking_markers_recover_without_cancelling_dfs_stack();
    puts("Onboard DFS branch/cycle/backtracking, ToF scans, arrival continuation and exhaustion tests passed");
    return 0;
}
