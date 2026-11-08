#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "onboard_dfs.h"
#include "motor_driver.h"

static uint32_t now;
static unsigned drive_starts,turns,stops;
static int turn_target;
static motor_auto_status_t motor_status;
motor_error_t motor_auto_drive_leased(int speed,uint16_t lease) {
    assert(speed>0 && lease==ONBOARD_DRIVE_LEASE_MS);drive_starts++;motor_status=MOTOR_AUTO_RUNNING;return MOTOR_OK;
}
motor_error_t motor_auto_rotate_leased(int heading,uint16_t lease) {
    assert(heading>=0 && heading<360 && lease==ONBOARD_DRIVE_LEASE_MS);turns++;turn_target=heading;motor_status=MOTOR_AUTO_RUNNING;return MOTOR_OK;
}
motor_error_t motor_emergency_stop(void) { stops++;motor_status=MOTOR_AUTO_STOPPED;return MOTOR_OK; }
motor_auto_status_t motor_auto_status(void) { return motor_status; }
bool motor_auto_renew_lease(void) { return motor_status==MOTOR_AUTO_RUNNING; }

static rover_map_t map;
static mission_t mission;
static onboard_drive_t drive;
static target_selector_t selection;
static onboard_dfs_t dfs;
static onboard_dfs_inputs_t inputs;
static camera_observation_t camera;

static void setup(void)
{
    now=1000;drive_starts=turns=stops=0;motor_status=MOTOR_AUTO_IDLE;
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
    assert(mission.reason==MISSION_REASON_DFS_COMPLETE && drive_starts==0 && turns>=3);
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
    camera.frame.markers[0].range_mm=600;step(100);
    camera.frame.markers[0].range_mm=450;step(100);
    assert(mission.leg.phase==LEG_BRAKING && drive.active);
    camera.frame.markers[0].range_mm=400;step(400);step(100);step(100);
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
    assert(!dfs.active && mission.state==MISSION_PAUSED);
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

int main(void)
{
    test_depth_first_branch_dead_end_backtrack_and_cycle();
    test_unknown_exits_and_open_frontier_never_mean_complete();
    test_controller_reads_tof_and_completes_only_closed_resolved_map();
    test_real_leg_arrival_returns_to_scanning_not_mission_complete();
    test_cancel_fault_and_no_target_cannot_fake_completion();
    test_turn_completion_waits_for_idle_snapshot();
    puts("Onboard DFS branch/cycle/backtracking, ToF scans, arrival continuation and exhaustion tests passed");
    return 0;
}
