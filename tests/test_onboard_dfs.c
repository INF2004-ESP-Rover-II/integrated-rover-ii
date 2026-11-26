#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "onboard_dfs.h"
#include "motor_driver.h"

static uint32_t now;
static unsigned drive_starts,turns,stops,renewals,scan_backups,scan_backup_ms;
static bool lease_ok;
static int turn_target;
static motor_auto_status_t motor_status;
static motor_error_t turn_result;
motor_error_t motor_auto_drive_leased(int speed,uint16_t lease) {
    assert(speed && speed>=-100 && speed<=100);
    if(speed==-DFS_TARGET_SCAN_BACKUP_PCT && lease<=DFS_TARGET_SCAN_BACKUP_MS) {
        assert(lease>0);scan_backups++;scan_backup_ms+=lease;
    } else { assert(lease==ONBOARD_DRIVE_LEASE_MS);drive_starts++; }
    motor_status=MOTOR_AUTO_RUNNING;return MOTOR_OK;
}
motor_error_t motor_auto_rotate_leased(int heading,uint16_t lease) {
    assert(heading>=0 && heading<360 && lease==ONBOARD_DRIVE_LEASE_MS);turns++;turn_target=heading;
    if(turn_result!=MOTOR_OK) return turn_result;
    motor_status=MOTOR_AUTO_RUNNING;return MOTOR_OK;
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
    now=1000;drive_starts=turns=stops=renewals=scan_backups=scan_backup_ms=0;lease_ok=true;motor_status=MOTOR_AUTO_IDLE;
    turn_result=MOTOR_OK;
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
    if ((dfs.stage==DFS_TURNING || dfs.b_target_scan_rotating) && motor_status==MOTOR_AUTO_RUNNING) {
        inputs.drive.heading_deg=fmodf((float)turn_target+inputs.heading_offset_deg+360.0f,360.0f);inputs.drive.stopped=true;motor_status=MOTOR_AUTO_HEADING_REACHED;
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
    assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
    assert(drive_starts==0 && map.current_node==1 && dfs.depth==1);
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
    assert(dfs.active && mission.state==MISSION_EXPLORING && dfs.b_target_scanning);
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
static void test_dfs_turn_retries_after_three_seconds_with_sixty_second_limit(void)
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
    for(uint32_t retry_at=6600;retry_at<60000;retry_at+=3300) {
        tick_turn_at(started+retry_at-300,false);
        assert(dfs.active && dfs.b_turn_retry_waiting);
        tick_turn_at(started+retry_at,true);
        assert(dfs.active && motor_status==MOTOR_AUTO_RUNNING && turn_target==90);
    }
    tick_turn_at(started+59999,false);
    assert(dfs.active && mission.state==MISSION_EXPLORING);
    tick_turn_at(started+60000,false);
    assert(!dfs.active && dfs.stage==DFS_STOPPED && mission.state==MISSION_PAUSED);
    assert(!strcmp(dfs.detail,"DFS_TURN_TIMEOUT") && motor_status==MOTOR_AUTO_STOPPED);
    assert(map.current_node==1 && map.revision==revision && map.count==1);
    assert(dfs.depth==1 && dfs.stack[0]==1 && !drive_starts && turns==before+18);
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
        tick_turn_at(started+(retry==2?60000:4000),true);
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
        tick_turn_at(started+60000u,false);
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
    tick_turn_at(dfs.stage_started_ms+59999,false);
    assert(dfs.active);
    tick_turn_at(dfs.stage_started_ms+60000,false);
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


static void start_target_scan_test(void)
{
    setup();camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<100 && !dfs.b_target_scan_rotating;++i) step(100);
    assert(dfs.active && dfs.stage==DFS_WAIT_TARGET && dfs.b_target_scan_rotating);
    assert(turn_target==350 && map.current_node==1 && !drive_starts);
}
static void scan_tick(uint32_t dt)
{
    now+=dt;inputs.drive.sensor_received_ms=now;camera.received_ms=now;
    camera.generation++;camera.frame.seq++;
    inputs.drive.stopped=motor_status!=MOTOR_AUTO_RUNNING;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
}
static void test_next_target_scan_repeats_without_losing_position_or_exits(void)
{
    start_target_scan_test();uint32_t revision=map.revision;
    for(unsigned i=0;i<700;++i) {
        step(100);
        assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
        assert(dfs.stage==DFS_WAIT_TARGET && map.current_node==1 && map.revision==revision);
        assert(dfs.depth==1 && dfs.stack[0]==1 && dfs.direction==MAP_NORTH && !drive_starts);
        assert(selection.route.enabled && selection.route.x==0 && selection.route.y==1 && selection.route.direction==LEG_NORTH);
        assert(turn_target==350 || turn_target==0 || turn_target==10);
    }
    assert(turns>20 && map.count==1 && map.nodes[0].visited && !drive.active);
    assert(scan_backups>0 && scan_backup_ms<=DFS_TARGET_SCAN_BACKUP_TOTAL_MS);
}
static void test_scan_camera_outages_stop_rotation_and_resume_fresh_selection(void)
{
    for(unsigned cause=0;cause<5;++cause) {
        start_target_scan_test();unsigned previous=turns;uint32_t revision=map.revision;
        if(cause==0) camera.frame.status=VISION_STATUS_STALLED;
        if(cause==1) camera.frame.status=VISION_STATUS_CAMERA_LOST;
        if(cause==2) camera.frame.age_ms=751;
        if(cause==3) camera.frame.count=VISION_MAX_MARKERS+1;
        if(cause==4) { now+=301;inputs.drive.sensor_received_ms=now;
            onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now); }
        else scan_tick(100);
        assert(!dfs.b_target_scan_rotating && motor_status==MOTOR_AUTO_STOPPED && dfs.active);
        assert(!strcmp(dfs.detail,"WAITING_FOR_FRESH_CAMERA"));
        if(cause<4) for(unsigned i=0;i<100;++i) step(100);
        assert(turns==previous && !drive_starts && dfs.depth==1 && map.revision==revision);
        camera.frame.status=VISION_STATUS_OK;camera.frame.age_ms=100;camera.frame.count=2;
        for(unsigned i=0;i<40 && !drive.active;++i) step(100);
        assert(drive.active && dfs.active && mission.leg.next_target_node==2);
    }
}
static void test_scan_requires_post_stop_images_then_departs_in_same_grid_direction(void)
{
    start_target_scan_test();step(100); // Stop at the first search pose.
    assert(!dfs.b_target_scan_rotating && inputs.drive.heading_deg==350);
    camera.frame.status=VISION_STATUS_OK;camera.frame.count=2;camera.frame.age_ms=700;
    for(unsigned i=0;i<6;++i) step(100); // All captures still predate selector arming.
    assert(selection.state!=TARGET_LOCKED && map.current_node==1 && !drive_starts);
    camera.frame.age_ms=100;
    for(unsigned i=0;i<12 && selection.state!=TARGET_LOCKED;++i) step(100);
    assert(selection.state==TARGET_LOCKED && map.current_node==1 && !drive_starts);
    step(100);
    assert(drive.active && dfs.active && dfs.stage==DFS_TRAVELLING && drive_starts==1);
    assert(mission.leg.direction==LEG_NORTH && mission.leg.target_x==0 && mission.leg.target_y==1);
    assert(mission.leg.next_target_node==2 && mission.leg.arrival_reference_node==9);
}
static void test_scan_turn_retry_and_limit_retain_dfs(void)
{
    start_target_scan_test();uint32_t started=now;unsigned previous=turns;
    scan_tick(3000);
    assert(dfs.active && dfs.b_turn_retry_waiting && motor_status==MOTOR_AUTO_STOPPED);
    scan_tick(299);assert(turns==previous);
    scan_tick(1);assert(turns==previous+1 && motor_status==MOTOR_AUTO_RUNNING);
    assert(dfs.stage_started_ms==started && turn_target==350);
    scan_tick(1700);
    assert(dfs.active && mission.state==MISSION_EXPLORING && !dfs.b_target_scan_rotating);
    assert(motor_status==MOTOR_AUTO_STOPPED && !strcmp(dfs.detail,"TARGET_SCAN_TURN_LIMIT"));
    for(unsigned i=0;i<30 && !dfs.b_target_scan_rotating;++i) step(100);
    // The centre pose already matches our simulated heading; keep scanning.
    for(unsigned i=0;i<30 && !dfs.b_target_scan_rotating;++i) step(100);
    assert(dfs.active && dfs.b_target_scan_rotating && !drive_starts && map.current_node==1);
}
static void test_scan_waits_for_inputs_and_honours_operator_stop_and_estop(void)
{
    for(unsigned cause=0;cause<4;++cause) {
        start_target_scan_test();uint32_t revision=map.revision;
        if(cause==0) inputs.drive.link_ok=false;
        if(cause==1) inputs.drive.sensor_valid=false;
        if(cause==2) inputs.drive.imu_ok=false;
        if(cause==3) lease_ok=false;
        scan_tick(100);
        assert(dfs.active && dfs.input_waiting && motor_status==MOTOR_AUTO_STOPPED);
        assert(mission.state==MISSION_EXPLORING && map.revision==revision);
        inputs.drive.link_ok=inputs.drive.sensor_valid=inputs.drive.imu_ok=lease_ok=true;
        scan_tick(60000);
        assert(dfs.active && !dfs.input_waiting && !dfs.b_target_scan_rotating);
        assert(map.current_node==1 && dfs.depth==1 && !drive_starts);
    }
    start_target_scan_test();mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);scan_tick(100);
    assert(!dfs.active && motor_status==MOTOR_AUTO_STOPPED && !drive_starts);
    start_target_scan_test();inputs.drive.safety_latched=true;scan_tick(100);
    assert(!dfs.active && motor_status==MOTOR_AUTO_STOPPED && !drive_starts);
}
static void test_scan_retains_backtrack_identity_and_heading_offset(void)
{
    start_target_scan_test();target_route_t route=selection.route;
    route.expected_id=2;target_selector_set_route(&selection,route);dfs.expected_id=2;dfs.backtracking=true;
    // Ignore a fresh, centred marker with the wrong expected identity.
    step(100);camera.frame.status=VISION_STATUS_OK;camera.frame.count=1;
    camera.frame.markers[0]=(camera_marker_t){.id=9,.range_mm=500};
    inputs.heading_offset_deg=90;
    for(unsigned i=0;i<100;++i) {
        step(100);assert(dfs.active && !drive.active && !drive_starts && map.current_node==1);
        assert(selection.route.expected_id==2 && dfs.expected_id==2 && dfs.backtracking);
        assert(turn_target==350 || turn_target==260 || turn_target==270 || turn_target==280);
    }
    camera.frame.markers[0].id=2;
    for(unsigned i=0;i<40 && !drive.active;++i) step(100);
    assert(drive.active && mission.leg.next_target_node==2 && dfs.active);
}


static void test_next_target_scan_uses_chosen_exit_and_original_grid_frame(void)
{
    setup();inputs.tof_mm[1]=50;inputs.tof_mm[2]=1023;
    inputs.drive.heading_deg=275;inputs.heading_offset_deg=90;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<100 && !dfs.b_target_scan_rotating;++i) step(100);
    assert(dfs.b_target_scan_rotating && dfs.direction==MAP_EAST && turn_target==265);
    for(unsigned i=0;i<200;++i) {
        step(100);assert(dfs.active && !drive.active && !drive_starts && map.current_node==1);
        assert(selection.route.direction==LEG_EAST && selection.route.x==1 && selection.route.y==0);
        assert(turn_target==265 || turn_target==275 || turn_target==285);
        assert(drive.north_heading_deg==275);
    }
}
static void test_confirmed_candidate_has_priority_over_scan_deadline(void)
{
    setup();start();for(unsigned i=0;i<40 && selection.state!=TARGET_LOCKED;++i) step(100);
    assert(selection.state==TARGET_LOCKED && map.current_node==1 && !drive.active);
    dfs.stage_started_ms=now-DFS_TARGET_SCAN_DELAY_MS-DFS_SETTLE_MS;
    step(100);
    assert(drive.active && dfs.stage==DFS_TRAVELLING && !dfs.b_target_scanning && turns==0);
}


static void start_unknown_exit_recovery(void)
{
    setup();inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=0;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<200 && strcmp(dfs.detail,"RESCANNING_UNRESOLVED_EXITS");++i) step(100);
    assert(dfs.active && !strcmp(dfs.detail,"RESCANNING_UNRESOLVED_EXITS"));
    assert(map.current_node==1 && dfs.depth==1 && !drive_starts);
}
static void test_unresolved_exits_repeat_all_retry_headings_without_fabricating_map(void)
{
    start_unknown_exit_recovery();uint32_t revision=map.revision;
    unsigned headings=0;
    for(unsigned i=0;i<700;++i) {
        step(100);headings|=1u<<(unsigned)dfs.exit_retry_direction;
        assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
        assert(map.revision==revision && map.current_node==1 && map.count==1);
        assert(dfs.depth==1 && dfs.stack[0]==1 && !drive_starts && !drive.active);
        for(unsigned dir=0;dir<4;++dir) assert(map.nodes[0].exits[dir]==MAP_EXIT_UNKNOWN);
    }
    assert(headings==15 && turns>4);
    char json[512];assert(onboard_dfs_format(&dfs,&map,json,sizeof(json)));
    assert(strstr(json,"\"active\":true") && !strstr(json,"STOPPED"));
    // Healthy repeated wall samples can finally close this single-node map.
    inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=50;
    for(unsigned i=0;i<150 && dfs.active;++i) step(100);
    assert(!dfs.active && dfs.stage==DFS_FINISHED && mission.state==MISSION_COMPLETE);
    assert(map.current_node==1 && map.count==1 && !drive_starts);
}
static void test_unknown_front_is_resolved_using_another_tof_sensor(void)
{
    setup();inputs.tof_mm[0]=inputs.tof_mm[2]=50;inputs.tof_mm[1]=0;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    bool recovery=false;
    for(unsigned i=0;i<150 && dfs.active;++i) {
        step(100);if(!strcmp(dfs.detail,"RESCANNING_UNRESOLVED_EXITS")) recovery=true;
    }
    assert(recovery && dfs.stage==DFS_FINISHED && mission.state==MISSION_COMPLETE);
    assert(turns>=2 && !drive_starts && map.current_node==1);
    for(unsigned dir=0;dir<4;++dir) assert(map.nodes[0].exits[dir]==MAP_EXIT_BLOCKED);
}
static void test_faulty_or_inconsistent_exit_samples_keep_retrying_then_allow_exploration(void)
{
    for(unsigned cause=0;cause<2;++cause) {
        start_unknown_exit_recovery();uint32_t revision=map.revision;
        for(unsigned i=0;i<200;++i) {
            inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=cause?(i%2?50:1023):1023;
            inputs.fault_mask=cause?0:7;
            step(100);
            assert(dfs.active && mission.state==MISSION_EXPLORING && map.revision==revision);
            assert(!drive_starts && map.current_node==1 && dfs.depth==1);
        }
        inputs.fault_mask=0;inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=1023;
        camera.frame.status=VISION_STATUS_OK;camera.frame.count=2;
        for(unsigned i=0;i<80 && !drive.active;++i) step(100);
        assert(dfs.active && drive.active && dfs.stage==DFS_TRAVELLING);
        assert(mission.leg.next_target_node==2 && !map.current_node && dfs.depth==1);
    }
}
static void test_unresolved_exit_recovery_preserves_backtrack_and_traversed_edges(void)
{
    setup();start();assert(rover_map_add(&map,2,0,1,false)==MAP_OK);
    rover_map_depart(&map);assert(rover_map_arrive(&map,2)==MAP_OK);
    dfs.from_node=1;assert(onboard_dfs_note_arrival(&dfs,&map));
    assert(rover_map_set_exit(&map,1,MAP_NORTH,MAP_EXIT_TRAVERSED)==MAP_OK);
    assert(rover_map_set_exit(&map,2,MAP_SOUTH,MAP_EXIT_TRAVERSED)==MAP_OK);
    dfs.stage=DFS_AT_NODE;inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=0;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;
    uint32_t revision=map.revision;
    for(unsigned i=0;i<400;++i) {
        step(100);assert(dfs.active && mission.state==MISSION_EXPLORING);
        assert(dfs.depth==2 && dfs.stack[0]==1 && dfs.stack[1]==2);
        assert(map.current_node==2 && map.revision==revision && map.nodes[1].exits[MAP_SOUTH]==MAP_EXIT_TRAVERSED);
    }
    inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=50;
    for(unsigned i=0;i<100 && dfs.stage!=DFS_WAIT_TARGET;++i) step(100);
    assert(dfs.active && dfs.stage==DFS_WAIT_TARGET && dfs.backtracking && dfs.expected_id==1);
    assert(selection.route.expected_id==1 && selection.route.x==0 && selection.route.y==0);
    assert(map.current_node==2 && dfs.depth==2 && map.nodes[1].exits[MAP_SOUTH]==MAP_EXIT_TRAVERSED);
}
static void test_exit_recovery_waits_for_inputs_and_honours_stop_and_estop(void)
{
    start_unknown_exit_recovery();uint32_t revision=map.revision;
    inputs.drive.sensor_valid=false;step(100);step(60000);
    assert(dfs.active && dfs.input_waiting && mission.state==MISSION_EXPLORING);
    assert(motor_status==MOTOR_AUTO_STOPPED && map.revision==revision && map.current_node==1);
    inputs.drive.sensor_valid=true;step(100);
    assert(dfs.active && !dfs.input_waiting && map.revision==revision);
    mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);step(100);
    assert(!dfs.active && motor_status==MOTOR_AUTO_STOPPED && !drive_starts);
    start_unknown_exit_recovery();inputs.drive.safety_latched=true;step(100);
    assert(!dfs.active && mission.state==MISSION_PAUSED && motor_status==MOTOR_AUTO_STOPPED && !drive_starts);
}
static void test_sighted_unvisited_nodes_never_allow_false_dfs_completion_during_retries(void)
{
    setup();block_all(&map.nodes[0]);assert(rover_map_observe(&map,9,0,1)==MAP_OK);
    inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=50;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<300;++i) {
        step(100);assert(dfs.active && mission.state==MISSION_EXPLORING && map.current_node==1);
        assert(map.observed_count==1 && map.count==1 && !drive_starts);
    }
}


static void start_scan_backup_test(void)
{
    start_target_scan_test();
    for(unsigned i=0;i<200 && !dfs.b_target_scan_backing;++i) step(100);
    assert(dfs.active && dfs.b_target_scan_backing && scan_backups==1 && !drive_starts);
    assert(mission.leg.phase==LEG_BACKING_UP && mission_motion_enabled(&mission));
    assert(!onboard_dfs_selecting(&dfs,now) && inputs.drive.heading_deg==0);
}
static void test_scan_reverse_is_leased_bounded_and_requires_new_stopped_images(void)
{
    start_scan_backup_test();uint32_t revision=map.revision;
    uint32_t started=now;
    camera.frame.status=VISION_STATUS_OK;camera.frame.count=2;
    camera.frame.markers[0].quality=camera.frame.markers[1].quality=1;
    step(DFS_TARGET_SCAN_BACKUP_MS-1);
    assert(dfs.b_target_scan_backing && selection.state!=TARGET_LOCKED && map.current_node==1);
    assert(!onboard_dfs_selecting(&dfs,now) && !drive_starts);
    step(1);assert(!dfs.b_target_scan_backing && motor_status==MOTOR_AUTO_STOPPED);
    assert(now==started+DFS_TARGET_SCAN_BACKUP_MS && map.revision==revision);
    assert(!onboard_dfs_selecting(&dfs,now) && !drive_starts);
    camera.frame.age_ms=700;
    camera.frame.markers[0].quality=camera.frame.markers[1].quality=0;
    for(unsigned i=0;i<6;++i) step(100);
    assert(!drive.active && selection.state!=TARGET_LOCKED && map.current_node==1);
    camera.frame.age_ms=100;
    for(unsigned i=0;i<15 && !drive.active;++i) step(100);
    assert(drive.active && dfs.active && mission.leg.next_target_node==2 && !map.current_node);
}
static void test_scan_reverse_budget_does_not_reset_between_passes_or_outages(void)
{
    start_scan_backup_test();uint32_t revision=map.revision;
    step(100);camera.frame.status=VISION_STATUS_STALLED;step(10);
    assert(!dfs.b_target_scan_backing && motor_status==MOTOR_AUTO_STOPPED);
    assert(dfs.target_scan_backup_ms==DFS_TARGET_SCAN_BACKUP_MS && scan_backups==1);
    step(60000);assert(scan_backups==1 && dfs.active && map.revision==revision);
    camera.frame.status=VISION_STATUS_NO_MARKER;
    for(unsigned i=0;i<1500;++i) {
        step(100);assert(dfs.active && map.current_node==1 && map.revision==revision && !drive_starts);
    }
    assert(scan_backup_ms==DFS_TARGET_SCAN_BACKUP_TOTAL_MS);
    assert(dfs.target_scan_backup_ms==DFS_TARGET_SCAN_BACKUP_TOTAL_MS);
    unsigned before=scan_backups;
    for(unsigned i=0;i<300;++i) step(100);
    assert(dfs.active && scan_backups==before && !drive_starts && dfs.depth==1);
}
static void test_scan_reverse_interrupts_on_inputs_stop_estop_and_stale_camera(void)
{
    for(unsigned cause=0;cause<6;++cause) {
        start_scan_backup_test();unsigned before=scan_backups;
        if(cause==0) inputs.drive.link_ok=false;
        if(cause==1) inputs.drive.sensor_valid=false;
        if(cause==2) inputs.drive.imu_ok=false;
        if(cause==3) inputs.drive.safety_latched=true;
        if(cause==4) mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
        if(cause==5) camera.frame.age_ms=751;
        step(1);assert(motor_status==MOTOR_AUTO_STOPPED && scan_backups==before && !drive_starts);
        assert(map.current_node==1 && dfs.depth==1);
        if(cause<3) {
            assert(dfs.active && dfs.input_waiting);
            inputs.drive.link_ok=inputs.drive.sensor_valid=inputs.drive.imu_ok=true;
            step(500);assert(dfs.active && !dfs.b_target_scan_backing && !dfs.input_waiting);
            assert(dfs.target_scan_backup_ms==DFS_TARGET_SCAN_BACKUP_MS);
        } else if(cause==5) {
            assert(dfs.active && !dfs.b_target_scan_backing);
            step(60000);assert(scan_backups==before && !drive_starts);
        } else assert(!dfs.active);
    }
}


static void sight_marker_during_scan(uint8_t id, int16_t bearing)
{
    camera.frame.status=VISION_STATUS_OK;camera.frame.count=1;
    camera.frame.markers[0]=(camera_marker_t){.id=id,.range_mm=650,.bearing_cdeg=bearing};
    scan_tick(100);
    assert(dfs.active && dfs.stage==DFS_WAIT_TARGET && dfs.b_target_scan_marker_hold);
    assert(!dfs.b_target_scan_rotating && !dfs.b_target_scan_backing && motor_status==MOTOR_AUTO_STOPPED);
    assert(selection.b_acquiring && !drive.active && !drive_starts && map.current_node==1);
}
static void test_marker_sighting_stops_scan_then_aligns_left_before_departure(void)
{
    for(unsigned angle=0;angle<2;++angle) {
    start_target_scan_test();sight_marker_during_scan(4,angle?-3500:-1800);
    unsigned before=turns;
    for(unsigned i=0;i<25;++i) scan_tick(100); // No selector confirmation yet.
    assert(dfs.active && turns==before && !drive_starts && dfs.b_target_scan_marker_hold);
    assert(!strcmp(dfs.detail,"CONFIRMING_VISIBLE_MARKER"));
    for(unsigned i=0;i<20 && !drive.active;++i) step(100);
    assert(drive.active && drive.aligning && drive.b_departure_pending);
    assert(mission.leg.next_target_node==4 && map.current_node==1 && !drive_starts);
    for(unsigned i=0;i<20 && !drive.alignment_rotating;++i) step(100);
    assert(drive.alignment_rotating && turn_target==355); // Negative bearing: left.
    assert(map.current_node==1 && !drive_starts && mission.leg.direction==LEG_NORTH);
    camera.frame.markers[0].bearing_cdeg=-800;
    motor_status=MOTOR_AUTO_HEADING_REACHED;inputs.drive.heading_deg=355;inputs.drive.stopped=true;
    for(unsigned i=0;i<30 && !drive_starts;++i) step(100);
    assert(drive.active && !drive.aligning && drive_starts==1 && !map.current_node);
    assert(mission.leg.next_target_node==4 && map.count==1 && !rover_map_find_id(&map,4));
    }
}
static void test_any_marker_stops_once_but_incompatible_sightings_do_not_retarget(void)
{
    for(unsigned wrong=0;wrong<3;++wrong) {
        start_target_scan_test();
        if(wrong==0) assert(rover_map_add(&map,4,1,0,false)==MAP_OK); // Wrong exit.
        sight_marker_during_scan(wrong==1?1:4,wrong==2?5000:-1800);
        assert(!target_selector_has_acquisition_candidate(&selection,&map,&camera.frame));
        for(unsigned i=0;i<70;++i) step(100);
        assert(dfs.active && !drive.active && !drive_starts && map.current_node==1);
        assert(!mission.leg.next_target_node && turns>1); // Scan resumes, no invented destination.
    }
}
static void test_sighting_during_reverse_stops_and_keeps_reverse_budget(void)
{
    start_scan_backup_test();uint16_t used=dfs.target_scan_backup_ms;
    sight_marker_during_scan(4,1600);
    assert(dfs.target_scan_backup_ms==used && scan_backups==1);
    for(unsigned i=0;i<20 && !drive.active;++i) step(100);
    assert(drive.active && drive.aligning && map.current_node==1 && !drive_starts);
    for(unsigned i=0;i<20 && !drive.alignment_rotating;++i) step(100);
    assert(drive.alignment_rotating && turn_target==5);
    assert(mission.leg.next_target_node==4 && scan_backups==1);
}
static void test_scan_sighting_ignores_bad_or_stale_data_and_checks_stop_controls(void)
{
    start_target_scan_test();camera.frame.status=VISION_STATUS_OK;camera.frame.count=1;
    camera.frame.markers[0]=(camera_marker_t){.id=4,.range_mm=650,.bearing_cdeg=-1800,.quality=1};
    scan_tick(100);assert(dfs.b_target_scan_rotating && !dfs.b_target_scan_marker_hold);
    camera.frame.markers[0].quality=0;camera.frame.age_ms=751;
    scan_tick(100);assert(!dfs.b_target_scan_marker_hold && motor_status==MOTOR_AUTO_STOPPED);
    for(unsigned stop=0;stop<3;++stop) {
        start_target_scan_test();sight_marker_during_scan(4,-1800);
        if(stop==0) mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
        if(stop==1) inputs.drive.safety_latched=true;
        if(stop==2) inputs.drive.link_ok=false;
        step(100);assert(motor_status==MOTOR_AUTO_STOPPED && !drive_starts && map.current_node==1);
        if(stop<2) assert(!dfs.active);else assert(dfs.active && dfs.input_waiting);
    }
}


static void recheck_camera_outage(unsigned cause,uint32_t dt)
{
    now+=dt;inputs.drive.sensor_received_ms=now;
    camera.received_ms=now-(cause==4?301u:0u);
    camera.frame.status=cause==0?VISION_STATUS_STALLED:cause==1?VISION_STATUS_CAMERA_LOST:VISION_STATUS_OK;
    camera.frame.age_ms=cause==2?751:100;
    camera.frame.count=cause==3?VISION_MAX_MARKERS+1:1;
    onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,cause==5?NULL:&camera,now);
}

static void test_marker_recheck_progress_survives_camera_outages_and_reaches_reverse(void)
{
    for(unsigned cause=0;cause<6;++cause) {
        start_target_scan_test();step(100);
        sight_marker_during_scan(1,0); // Current node is not a next target.
        if(cause==5) { // Repeat across the millisecond clock rollover.
            uint32_t shift=UINT32_MAX-1000u-now;
            now+=shift;dfs.stage_started_ms+=shift;dfs.ready_ms+=shift;
        }
        uint32_t revision=map.revision;unsigned before=turns;
        for(unsigned gap=0;gap<3;++gap) {
            step(500);
            assert(dfs.b_target_scan_marker_hold && !drive_starts && turns==before);
            recheck_camera_outage(cause,10);uint32_t used=now-dfs.stage_started_ms;
            assert(!strcmp(dfs.detail,"WAITING_FOR_FRESH_CAMERA"));
            recheck_camera_outage(cause,60000);
            assert(dfs.active && dfs.b_target_scan_marker_hold && !scan_backups && turns==before);
            assert(motor_status==MOTOR_AUTO_STOPPED && map.revision==revision && map.current_node==1);
            camera.frame.status=VISION_STATUS_OK;camera.frame.age_ms=100;camera.frame.count=1;
            step(10);
            assert(dfs.b_target_scan_marker_hold && now-dfs.stage_started_ms==used);
            assert(!strcmp(dfs.detail,"CHECKING_VISIBLE_MARKER"));
        }
        for(unsigned i=0;i<15 && dfs.b_target_scan_marker_hold;++i) step(100);
        assert(!dfs.b_target_scan_marker_hold && dfs.b_target_scan_rotating && turns==before+1);
        for(unsigned i=0;i<200 && !dfs.b_target_scan_backing;++i) step(100);
        assert(dfs.active && dfs.b_target_scan_backing && scan_backups==1);
        assert(scan_backup_ms==DFS_TARGET_SCAN_BACKUP_MS && !drive_starts && !drive.active);
        assert(map.current_node==1 && map.revision==revision && dfs.depth==1 && dfs.stack[0]==1);
        assert(selection.route.direction==LEG_NORTH && selection.route.x==0 && selection.route.y==1);
    }
}

static void test_marker_recheck_camera_and_input_waits_do_not_double_count(void)
{
    start_target_scan_test();step(100);sight_marker_during_scan(1,0);step(500);
    camera.frame.status=VISION_STATUS_STALLED;step(10);
    uint32_t used=now-dfs.stage_started_ms;
    inputs.drive.link_ok=false;step(100);assert(dfs.input_waiting);
    step(60000);inputs.drive.link_ok=true;step(400);
    assert(!dfs.input_waiting && dfs.b_target_scan_marker_hold && motor_status==MOTOR_AUTO_STOPPED);
    camera.frame.status=VISION_STATUS_OK;step(100);
    assert(dfs.b_target_scan_marker_hold && now-dfs.stage_started_ms==used);
    for(unsigned i=0;i<25 && dfs.b_target_scan_marker_hold;++i) step(100);
    assert(dfs.active && !dfs.b_target_scan_marker_hold && dfs.b_target_scan_rotating);
    assert(!drive_starts && map.current_node==1);
}

static void test_dfs_node_limit_settings_are_validated_atomically(void)
{
    setup();assert(dfs.node_limit==0 && dfs.open_distance_mm==700);
    assert(!onboard_dfs_configure(&dfs,":700:4") && dfs.node_limit==4);
    assert(!onboard_dfs_configure(&dfs,":800") && dfs.node_limit==4 && dfs.open_distance_mm==800);
    assert(!onboard_dfs_configure(&dfs,"") && dfs.node_limit==4);
    assert(!onboard_dfs_configure(&dfs,":401:64") && dfs.node_limit==64);
    assert(!onboard_dfs_configure(&dfs,":1000:0") && !dfs.node_limit);
    const char *invalid[]={":400:4",":1001:4",":700x:4",":700:",":700:-1",":700:65",":700:1:2",":700:999999999999999999999"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) {
        assert(onboard_dfs_configure(&dfs,invalid[i]));
        assert(dfs.open_distance_mm==1000 && !dfs.node_limit);
    }
    start();
    assert(!strcmp(onboard_dfs_configure(&dfs,":700:2"),"STOP_DFS_BEFORE_CONFIG_CHANGE"));
    assert(dfs.open_distance_mm==1000 && !dfs.node_limit);
    assert(!onboard_dfs_configure(&dfs,":1000:0"));
}

static void test_one_node_limit_scans_before_finishing_and_keeps_blue_nodes(void)
{
    setup();assert(rover_map_observe(&map,9,0,1)==MAP_OK);
    dfs.node_limit=1;camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    assert(dfs.active && dfs.node_limit==1 && mission.state==MISSION_EXPLORING);
    step(100);step(DFS_SETTLE_MS);
    assert(dfs.active && !drive_starts); // Do not finish before settled exit sampling.
    for(unsigned i=0;i<100 && dfs.active;++i) step(100);
    assert(!dfs.active && dfs.stage==DFS_FINISHED && mission.state==MISSION_COMPLETE);
    assert(mission.reason==MISSION_REASON_NODE_LIMIT_REACHED && !strcmp(dfs.detail,"NODE_LIMIT_REACHED"));
    assert(!mission.dfs_active && !mission_motion_enabled(&mission) && !drive_starts);
    assert(map.current_node==1 && map.count==1 && map.observed_count==1 && map.observed[0].marker_id==9);
    for(unsigned dir=0;dir<4;++dir) assert(map.nodes[0].exits[dir]!=MAP_EXIT_UNKNOWN);
    char json[512];assert(onboard_dfs_format(&dfs,&map,json,sizeof(json)));
    assert(strstr(json,"\"confirmed_nodes\":1") && strstr(json,"\"node_limit\":1"));
    printf("NODE_LIMIT_STATE_JSON:%s\n",json);
}

static void test_node_limit_counts_arrival_not_sightings_and_stops_before_next_leg(void)
{
    setup();dfs.node_limit=2;camera.frame.count=1;
    assert(rover_map_observe(&map,9,0,2)==MAP_OK);start();
    for(unsigned i=0;i<50 && !drive.active;++i) step(100);
    assert(drive.active && dfs.active && mission.leg.next_target_node==2 && map.count==1 && map.observed_count==1);
    assert(mission.state==MISSION_EXPLORING && dfs.node_limit==2);
    inputs.drive.front_mm=inputs.tof_mm[1]=350;inputs.drive.front_urgent=true;
    inputs.tof_mm[2]=1023;motor_emergency_stop();
    assert(onboard_drive_note_front_stop(&drive,&mission));
    step(100);step(300);
    assert(drive.active && drive.verification_frames==1 && !map.current_node && mission.state==MISSION_EXPLORING);
    step(100);assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(dfs.active && mission.state==MISSION_EXPLORING); // Final scan still required.
    unsigned before=drive_starts;
    for(unsigned i=0;i<100 && dfs.active;++i) step(100);
    assert(!dfs.active && mission.reason==MISSION_REASON_NODE_LIMIT_REACHED && mission.state==MISSION_COMPLETE);
    assert(map.current_node==2 && map.count==2 && map.observed_count==1 && drive_starts==before);
    assert(rover_map_find_id(&map,2)->exits[MAP_EAST]==MAP_EXIT_OPEN); // Remains unexplored.
    char json[512];assert(onboard_dfs_format(&dfs,&map,json,sizeof(json)));
    assert(strstr(json,"\"confirmed_nodes\":2") && strstr(json,"\"node_limit\":2"));
    assert(rover_map_arrive(&map,2)==MAP_OK && rover_map_arrive(&map,2)==MAP_OK);
    assert(onboard_dfs_format(&dfs,&map,json,sizeof(json)) && strstr(json,"\"confirmed_nodes\":2"));
}

static void test_limited_final_scan_preserves_unknowns_and_normal_exhaustion_reason(void)
{
    setup();dfs.node_limit=1;inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=0;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<100 && dfs.active;++i) step(100);
    assert(!dfs.active && mission.reason==MISSION_REASON_NODE_LIMIT_REACHED && !drive_starts);
    for(unsigned dir=0;dir<4;++dir) assert(map.nodes[0].exits[dir]==MAP_EXIT_UNKNOWN);
    setup();dfs.node_limit=5;inputs.tof_mm[0]=inputs.tof_mm[1]=inputs.tof_mm[2]=50;
    camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
    for(unsigned i=0;i<100 && dfs.active;++i) step(100);
    assert(!dfs.active && mission.reason==MISSION_REASON_DFS_COMPLETE && !strcmp(dfs.detail,"DFS_EXHAUSTED"));
}

static void test_final_node_limit_scan_waits_for_inputs_and_honours_stop(void)
{
    for(unsigned cause=0;cause<5;++cause) {
        setup();dfs.node_limit=1;camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();step(100);
        if(cause==0) mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
        if(cause==1) inputs.drive.safety_latched=true;
        if(cause==2) inputs.drive.link_ok=false;
        if(cause==3) inputs.drive.sensor_valid=false;
        if(cause==4) inputs.drive.imu_ok=false;
        step(100);assert(mission.state!=MISSION_COMPLETE && !drive_starts && map.current_node==1);
        if(cause<2) { assert(!dfs.active);continue; }
        step(60000);assert(dfs.active && dfs.input_waiting && mission.state==MISSION_EXPLORING);
        inputs.drive.link_ok=inputs.drive.sensor_valid=inputs.drive.imu_ok=true;
        for(unsigned i=0;i<100 && dfs.active;++i) step(100);
        assert(!dfs.active && mission.reason==MISSION_REASON_NODE_LIMIT_REACHED && !drive_starts);
    }
}

static void prepare_grid_misaligned_departure(map_direction_t direction,bool backtrack)
{
    setup();inputs.drive.heading_deg=35;inputs.heading_offset_deg=90;
    drive.frame_valid=true;drive.north_heading_deg=35;start();
    if(backtrack) {
        assert(rover_map_add(&map,2,0,1,true)==MAP_OK);
        assert(rover_map_arrive(&map,2)==MAP_OK);
        dfs.depth=2;dfs.stack[1]=2;dfs.backtracking=true;dfs.from_node=2;dfs.expected_id=1;
    }
    dfs.stage=DFS_WAIT_TARGET;dfs.direction=direction;dfs.ready_ms=dfs.stage_started_ms=now;
    mission_clear_leg(&mission,now);
    dfs.target_scan_backup_ms=450;dfs.b_target_scanning=true;
    static const int8_t dx[]={0,1,0,-1},dy[]={1,0,-1,0};
    const map_node_t *from=rover_map_find_id(&map,map.current_node);
    target_selector_set_route(&selection,(target_route_t){.enabled=true,.expected_id=backtrack?1:0,
        .x=(int16_t)(from->x+dx[direction]),.y=(int16_t)(from->y+dy[direction]),.direction=(leg_direction_t)direction});
    camera.frame.count=1;camera.frame.markers[0]=(camera_marker_t){.id=backtrack?1:4,.range_mm=650};
    for(unsigned i=0;i<6;++i) {
        now+=200;camera.received_ms=now;camera.generation++;camera.frame.seq++;
        target_selector_tick(&selection,&mission,&map,&camera,true,now);
    }
    inputs.drive.sensor_received_ms=now;inputs.drive.heading_deg=fmodf(35.0f+90.0f*direction+25.0f,360.0f);
    assert(selection.state==TARGET_LOCKED && mission.leg.configured && !drive.active);
}

static void test_grid_alignment_recovers_same_exit_and_reconfirms_locked_destination(void)
{
    for(unsigned which=0;which<5;++which) {
        bool backtrack=which==4;map_direction_t direction=backtrack?MAP_SOUTH:(map_direction_t)which;
        prepare_grid_misaligned_departure(direction,backtrack);
        rover_map_t before=map;uint8_t target=mission.leg.next_target_node,depth=dfs.depth;
        target_route_t route=selection.route;
        onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
        assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING && dfs.stage==DFS_TURNING);
        assert(!strcmp(dfs.detail,"RECOVERING_GRID_HEADING") && !drive_starts && !drive.active);
        assert(turn_target==((35+90*(int)direction-90+360)%360));
        assert(selection.route.expected_id==target && selection.route.direction==route.direction);
        assert(selection.route.x==route.x && selection.route.y==route.y && dfs.depth==depth && dfs.backtracking==backtrack);
        assert(dfs.target_scan_backup_ms==450 && !memcmp(&before,&map,sizeof(map)));
        step(100);assert(dfs.stage==DFS_WAIT_TARGET && !drive.active);
        camera.frame.age_ms=700;
        for(unsigned i=0;i<6;++i) step(100); // Cached/pre-stop captures cannot restart travel.
        assert(!drive.active && !drive_starts && selection.state!=TARGET_LOCKED && map.current_node==before.current_node);
        camera.frame.age_ms=100;camera.frame.markers[0].id=33;
        for(unsigned i=0;i<4;++i) step(100); // A closer/wrong identity cannot replace the locked destination.
        assert(!drive.active && !drive_starts && selection.route.expected_id==target);
        camera.frame.markers[0].id=target;
        for(unsigned i=0;i<20 && !drive.active;++i) step(100);
        assert(dfs.active && drive.active && dfs.stage==DFS_TRAVELLING && mission.leg.next_target_node==target);
        assert(mission.leg.direction==(leg_direction_t)direction && mission.leg.target_x==route.x && mission.leg.target_y==route.y);
        assert(dfs.depth==depth && dfs.backtracking==backtrack && dfs.target_scan_backup_ms==450);
    }
}

static void test_grid_heading_recovery_retains_input_waits_and_honours_stop(void)
{
    for(unsigned cause=0;cause<7;++cause) {
        prepare_grid_misaligned_departure(MAP_WEST,false);rover_map_t before=map;
        if(cause==6) turn_result=MOTOR_ERR_NO_HEADING;
        onboard_dfs_tick(&dfs,&mission,&map,&drive,&selection,&inputs,&camera,now);
        assert(dfs.active && dfs.stage==DFS_TURNING && !drive_starts);
        if(cause==0) mission_pause(&mission,MISSION_REASON_OPERATOR_STOP,now);
        if(cause==1) inputs.drive.safety_latched=true;
        if(cause==2) inputs.drive.link_ok=false;
        if(cause==3) inputs.drive.sensor_valid=false;
        if(cause==4) inputs.drive.imu_ok=false;
        if(cause==5) motor_status=MOTOR_AUTO_STOPPED;
        scan_tick(100);assert(motor_status==MOTOR_AUTO_STOPPED && !drive_starts && !memcmp(&before,&map,sizeof(map)));
        if(cause<2) { assert(!dfs.active);continue; }
        assert(dfs.active && dfs.input_waiting && mission.dfs_active && mission.state==MISSION_EXPLORING);
        inputs.drive.link_ok=inputs.drive.sensor_valid=inputs.drive.imu_ok=true;turn_result=MOTOR_OK;
        scan_tick(400);
        assert(dfs.active && !dfs.input_waiting && dfs.stage==DFS_TURNING && motor_status==MOTOR_AUTO_RUNNING);
        assert(turn_target==215 && selection.route.expected_id==4 && dfs.target_scan_backup_ms==450);
        assert(!drive_starts && !memcmp(&before,&map,sizeof(map)));
    }
}

static void test_default_700_mm_exit_threshold_avoids_obstructed_forward_exit(void)
{
    const uint16_t ranges[]={521,699,700,701};
    for(unsigned i=0;i<sizeof(ranges)/sizeof(ranges[0]);++i) {
        setup();inputs.tof_mm[1]=ranges[i];inputs.tof_mm[2]=1023;
        camera.frame.status=VISION_STATUS_NO_MARKER;camera.frame.count=0;start();
        assert(dfs.open_distance_mm==700);
        for(unsigned tick=0;tick<30 && dfs.stage!=DFS_TURNING && dfs.stage!=DFS_WAIT_TARGET;++tick) step(100);
        bool blocked=ranges[i]<700;
        assert(rover_map_find_id(&map,1)->exits[MAP_NORTH]==(blocked?MAP_EXIT_BLOCKED:MAP_EXIT_OPEN));
        assert(dfs.direction==(blocked?MAP_EAST:MAP_NORTH));
        assert(dfs.active && mission.dfs_active && !drive_starts && map.current_node==1 && map.count==1);
    }
}

static void test_front_stop_during_camera_wait_arrives_then_turns_to_next_exit(void)
{
    setup();camera.frame.count=1;start();
    for(unsigned i=0;i<50 && !drive.active;++i) step(100);
    assert(drive.active && mission.leg.next_target_node==2 && !mission.leg.arrival_reference_node);
    camera.frame.status=VISION_STATUS_STALLED;camera.frame.count=0;step(100);
    assert(drive.camera_waiting && mission.leg.phase==LEG_WAITING_CAMERA);
    inputs.drive.front_mm=inputs.tof_mm[1]=250;inputs.drive.front_urgent=true;
    inputs.tof_mm[2]=1023; // The next node has an open right exit.
    motor_emergency_stop();assert(onboard_drive_note_front_stop(&drive,&mission));
    step(100);assert(mission.leg.phase==LEG_OBSTACLE_VERIFYING && !drive.camera_waiting);
    assert(dfs.active && !map.current_node && !mission_motion_enabled(&mission));
    step(300);assert(drive.verification_frames==1 && !map.current_node);
    step(100);assert(!drive.active && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(dfs.active && mission.dfs_active && mission.state==MISSION_EXPLORING);
    step(100);assert(dfs.depth==2 && dfs.stage==DFS_SAMPLING);
    for(unsigned i=0;i<30 && dfs.stage!=DFS_TURNING;++i) step(100);
    assert(dfs.active && dfs.stage==DFS_TURNING && dfs.direction==MAP_EAST && turn_target==90);
    assert(map.current_node==2 && dfs.depth==2 && drive_starts==1);
}

int main(void)
{
    test_dfs_node_limit_settings_are_validated_atomically();
    test_one_node_limit_scans_before_finishing_and_keeps_blue_nodes();
    test_node_limit_counts_arrival_not_sightings_and_stops_before_next_leg();
    test_limited_final_scan_preserves_unknowns_and_normal_exhaustion_reason();
    test_final_node_limit_scan_waits_for_inputs_and_honours_stop();
    test_grid_alignment_recovers_same_exit_and_reconfirms_locked_destination();
    test_grid_heading_recovery_retains_input_waits_and_honours_stop();
    test_default_700_mm_exit_threshold_avoids_obstructed_forward_exit();
    test_marker_recheck_progress_survives_camera_outages_and_reaches_reverse();
    test_marker_recheck_camera_and_input_waits_do_not_double_count();
    test_front_stop_during_camera_wait_arrives_then_turns_to_next_exit();
    test_marker_sighting_stops_scan_then_aligns_left_before_departure();
    test_any_marker_stops_once_but_incompatible_sightings_do_not_retarget();
    test_sighting_during_reverse_stops_and_keeps_reverse_budget();
    test_scan_sighting_ignores_bad_or_stale_data_and_checks_stop_controls();
    test_scan_reverse_is_leased_bounded_and_requires_new_stopped_images();
    test_scan_reverse_budget_does_not_reset_between_passes_or_outages();
    test_scan_reverse_interrupts_on_inputs_stop_estop_and_stale_camera();
    test_unresolved_exits_repeat_all_retry_headings_without_fabricating_map();
    test_unknown_front_is_resolved_using_another_tof_sensor();
    test_faulty_or_inconsistent_exit_samples_keep_retrying_then_allow_exploration();
    test_unresolved_exit_recovery_preserves_backtrack_and_traversed_edges();
    test_exit_recovery_waits_for_inputs_and_honours_stop_and_estop();
    test_sighted_unvisited_nodes_never_allow_false_dfs_completion_during_retries();
    test_next_target_scan_uses_chosen_exit_and_original_grid_frame();
    test_confirmed_candidate_has_priority_over_scan_deadline();
    test_next_target_scan_repeats_without_losing_position_or_exits();
    test_scan_camera_outages_stop_rotation_and_resume_fresh_selection();
    test_scan_requires_post_stop_images_then_departs_in_same_grid_direction();
    test_scan_turn_retry_and_limit_retain_dfs();
    test_scan_waits_for_inputs_and_honours_operator_stop_and_estop();
    test_scan_retains_backtrack_identity_and_heading_offset();
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
    test_dfs_turn_retries_after_three_seconds_with_sixty_second_limit();
    test_dfs_turn_success_before_or_after_retry_and_at_deadline();
    test_dfs_turn_retry_wait_is_bounded_and_timer_wraps();
    test_dfs_turn_input_wait_preserves_used_turn_allowance();
    test_dfs_turn_still_waits_on_stopped_motor_or_failed_lease();
    test_selected_target_drifts_outside_ten_degrees_and_aligns_before_departure();
    test_missing_tracking_markers_recover_without_cancelling_dfs_stack();
    puts("Onboard DFS branch/cycle/backtracking, ToF scans, arrival continuation and exhaustion tests passed");
    return 0;
}
