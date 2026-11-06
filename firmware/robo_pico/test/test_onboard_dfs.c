#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "onboard_dfs.h"

#define TEST_ARENA_SIZE 64
#define RAD 0.017453292519943295f
typedef struct {
    onboard_dfs_t nav;
    bool passage[TEST_ARENA_SIZE][TEST_ARENA_SIZE][4], drive, turning, frozen_turn;
    int width, height;
    unsigned ids[TEST_ARENA_SIZE][TEST_ARENA_SIZE], drive_calls, stops, arrivals, backtracks;
    float x,y,heading;
    uint32_t now,last_camera;
    uint8_t sequence;
    bool camera_disabled,empty_camera,invalid_front;
    vision_parser_t parser;
} simulation_t;
static bool drive(void *ctx,int speed) {simulation_t *s=ctx;(void)speed;s->drive=true;++s->drive_calls;return true;}
static bool turn(void *ctx,int heading) {
    simulation_t *s=ctx;s->drive=false;s->turning=true;
    if(!s->frozen_turn)s->heading=(float)heading;
    return true;
}
static bool turn_done(void *ctx) {simulation_t *s=ctx;return s->turning&&!s->frozen_turn;}
static void stop(void *ctx) {simulation_t *s=ctx;s->drive=s->turning=false;++s->stops;}
static void link_cells(simulation_t *s,int x,int y,unsigned d) {
    int nx=x+rover_dx[d],ny=y+rover_dy[d];
    assert(nx>=0&&nx<s->width&&ny>=0&&ny<s->height);
    s->passage[y][x][d]=s->passage[ny][nx][(d+2)%4]=true;
}
static void initialise(simulation_t *s) {
    memset(s,0,sizeof(*s));s->now=1000;s->width=s->height=6;
    onboard_motor_io_t io={s,drive,turn,turn_done,stop};
    onboard_dfs_init(&s->nav,&io);
    assert(onboard_dfs_set_geometry(&s->nav,325,120,0,s->now));
    vision_parser_init(&s->parser);
}
static float wall_distance(const simulation_t *s,unsigned d) {
    int x=(int)lroundf(s->x/600),y=(int)lroundf(s->y/600);
    if(x<0)x=0;
    if(x>=s->width)x=s->width-1;
    if(y<0)y=0;
    if(y>=s->height)y=s->height-1;
    for(unsigned step=0;step<TEST_ARENA_SIZE;++step) {
        if(!s->passage[y][x][d]) {
            float plane=(float)(x*rover_dx[d]+y*rover_dy[d])*600+300;
            return plane-s->x*rover_dx[d]-s->y*rover_dy[d];
        }
        x+=rover_dx[d];y+=rover_dy[d];
    }
    return 4000;
}
static void sensors(simulation_t *s) {
    unsigned d=(unsigned)lroundf(s->heading/90)%4;
    onboard_sensor_t sensor={0};sensor.heading_deg=s->heading;sensor.imu_valid=true;sensor.received_ms=s->now;
    unsigned directions[]={(d+3)%4,d,(d+1)%4};
    for(unsigned i=0;i<3;++i) {
        float r=wall_distance(s,directions[i]);sensor.tof_mm[i]=r>1000?1023:(uint16_t)fmaxf(1,roundf(r));
    }
    sensor.rear_valid=true;sensor.rear_mm=(uint16_t)fmaxf(1,fminf(4000,wall_distance(s,(d+2)%4)));
    if(s->invalid_front) {sensor.tof_mm[1]=0;sensor.tof_faults=2;}
    onboard_dfs_sensor(&s->nav,&sensor);
}
static void camera(simulation_t *s) {
    if(s->camera_disabled)return;
    vision_marker_t markers[ROVER_MAX_NODES];unsigned count=0;
    if(!s->empty_camera)for(int y=0;y<s->height;++y)for(int x=0;x<s->width;++x) {
        if(!s->ids[y][x])continue;
        float dx=(float)x*600-s->x,dy=(float)y*600-s->y,angle=s->heading*RAD;
        float forward=dx*sinf(angle)+dy*cosf(angle),right=dx*cosf(angle)-dy*sinf(angle);
        float lens_forward=forward-120;
        float r=sqrtf(lens_forward*lens_forward+right*right+325*325);
        unsigned d=(unsigned)lroundf(s->heading/90)%4;
        if(lens_forward<=0||fabsf(right)>80||r<500||r>2500||forward>wall_distance(s,d))continue;
        markers[count++]=(vision_marker_t){(uint8_t)s->ids[y][x],(uint16_t)lroundf(r),(int16_t)lroundf(atan2f(right,lens_forward)/RAD*100),0};
    }
    for(unsigned i=0;i<count;++i)for(unsigned j=i+1;j<count;++j)if(markers[j].range_mm<markers[i].range_mm) {
        vision_marker_t tmp=markers[i];markers[i]=markers[j];markers[j]=tmp;
    }
    if(count>4)count=4;
    uint8_t wire[31]={VISION_SYNC,s->sequence++,count?VISION_OK:VISION_NO_MARKER,0,0,(uint8_t)count};
    for(unsigned i=0;i<count;++i) {
        unsigned offset=6+6*i;const vision_marker_t *m=&markers[i];
        wire[offset]=m->id;wire[offset+1]=(uint8_t)m->range_mm;wire[offset+2]=(uint8_t)(m->range_mm>>8);
        wire[offset+3]=(uint8_t)m->bearing_cdeg;wire[offset+4]=(uint8_t)((uint16_t)m->bearing_cdeg>>8);wire[offset+5]=m->quality;
    }
    unsigned length=7+6*count;wire[length-1]=vision_crc8(wire+1,length-2);
    vision_frame_t frame;
    for(unsigned i=0;i<length;++i)if(vision_parser_feed(&s->parser,wire[i],s->now,&frame))
        onboard_dfs_camera(&s->nav,&frame,s->parser.frame_received_ms,s->now);
    s->last_camera=s->now;
}
static void step(simulation_t *s) {
    if(s->drive) {s->x+=4*sinf(s->heading*RAD);s->y+=4*cosf(s->heading*RAD);}
    s->now+=20;sensors(s);
    if(s->now-s->last_camera>=180)camera(s);
    uint8_t before=s->nav.map.current;
    onboard_dfs_tick(&s->nav,s->now);
    if(before!=s->nav.map.current) {
        int x,y;assert(rover_map_find(&s->nav.map,s->nav.map.current,&x,&y));
        float zero=s->nav.zero_heading*RAD;
        float expected_x=((float)x*cosf(zero)+(float)y*sinf(zero))*600;
        float expected_y=(-(float)x*sinf(zero)+(float)y*cosf(zero))*600;
        assert(hypotf(s->x-expected_x,s->y-expected_y)<=s->nav.config.position_tolerance_mm);
        ++s->arrivals;if(s->nav.map.current<before)++s->backtracks;
    }
}
static void anchor(simulation_t *s,unsigned wanted) {
    sensors(s);assert(onboard_dfs_set_start(&s->nav,(uint8_t)s->ids[0][0],s->now));
    for(unsigned i=0;i<50;++i)step(s);
    assert(onboard_dfs_start(&s->nav,(uint8_t)wanted,500,s->now));
}
static bool run(simulation_t *s,unsigned max_steps,bool demo) {
    uint32_t revision=UINT32_MAX,state_revision=UINT32_MAX;
    for(unsigned i=0;i<max_steps;++i) {
        step(s);
        if(demo&&(revision!=s->nav.map.revision||state_revision!=s->nav.state_revision)) {
            char json[ONBOARD_SNAPSHOT_BYTES];assert(onboard_dfs_snapshot(&s->nav,json,sizeof(json)));puts(json);
            revision=s->nav.map.revision;state_revision=s->nav.state_revision;
        }
        if(s->nav.phase==DFS_COMPLETE)return true;
        if(s->nav.phase==DFS_FAULT) {
            fprintf(stderr,"Simulation fault at %lu ms: %s node=%u phase=%s physical=(%.1f,%.1f) target=(%d,%d) visited=%u\n",
                (unsigned long)s->now,s->nav.reason,s->nav.map.current,onboard_dfs_phase_name(s->nav.phase),
                (double)s->x,(double)s->y,s->nav.target_x,s->nav.target_y,s->nav.map.visited_count);
            return false;
        }
    }
    fprintf(stderr,"Simulation did not finish: phase=%s reason=%s\n",onboard_dfs_phase_name(s->nav.phase),s->nav.reason);return false;
}
static void snake(simulation_t *s) {
    unsigned id=1;
    for(int x=0;x<s->width;++x)for(int i=0;i<s->height;++i) {
        int y=(x%2)?s->height-1-i:i;s->ids[y][x]=id++;
        if(i<s->height-1)link_cells(s,x,y,x%2?MAP_SOUTH:MAP_NORTH);
        else if(x<s->width-1)link_cells(s,x,y,MAP_EAST);
    }
}
static void test_map(void) {
    rover_map_t map;rover_map_init(&map,0);
    assert(rover_map_rows(&map)==0&&rover_map_columns(&map)==0);
    rover_map_init(&map,255);
    assert(rover_map_rows(&map)==1&&rover_map_columns(&map)==1);
    assert(rover_map_place(&map,40,0,1));
    assert(rover_map_connect(&map,255,40,MAP_NORTH));
    assert(rover_map_find(&map,40,NULL,NULL)->exits[MAP_SOUTH]==MAP_EXPLORED);
    assert(rover_map_place(&map,2,-10,0)); /* Wider than six, west of start. */
    assert(rover_map_place(&map,3,0,-8));  /* Grow south without moving nodes. */
    assert(rover_map_place(&map,4,9,7));
    assert(rover_map_columns(&map)==20&&rover_map_rows(&map)==16);
    assert(rover_map_find(&map,255,NULL,NULL)==rover_map_at(&map,0,0));
    assert(rover_map_visit(&map,40)&&rover_map_visit(&map,40)&&map.visited_count==2);
    rover_map_t before=map;
    assert(!rover_map_place(&map,40,1,1)); /* Same ID at a different location. */
    assert(!rover_map_place(&map,5,0,1));  /* Different ID at an occupied node. */
    assert(!rover_map_place(&map,0,1,1));
    assert(!rover_map_place(&map,5,ROVER_COORD_LIMIT+1,0));
    assert(memcmp(&before,&map,sizeof(map))==0);
    uint32_t rev=map.revision;
    assert(rover_map_place(&map,40,0,1)&&map.revision==rev);
    rover_map_init(&map,1);
    assert(rover_map_exit(&map,1,MAP_WEST,MAP_OPEN));
    assert(rover_map_place(&map,2,-1,0));
    assert(rover_map_find(&map,2,NULL,NULL)->exits[MAP_EAST]==MAP_OPEN);
    assert(rover_map_connect(&map,1,2,MAP_WEST));
    assert(rover_map_exit(&map,2,MAP_EAST,MAP_OPEN));
    assert(rover_map_find(&map,1,NULL,NULL)->exits[MAP_WEST]==MAP_EXPLORED);
    rover_map_init(&map,1);
    for(unsigned i=1;i<ROVER_MAX_NODES;++i)assert(rover_map_place(&map,(uint8_t)(i+1),(int)i,0));
    assert(rover_map_columns(&map)==64&&map.node_count==ROVER_MAX_NODES);
    before=map;
    assert(!rover_map_place(&map,255,0,1));
    assert(memcmp(&before,&map,sizeof(map))==0); /* Capacity failure is atomic. */
    assert(rover_map_place(&map,64,63,0));      /* Idempotent even when full. */
}
static void test_snapshot_capacity(void) {
    simulation_t s;initialise(&s);s.nav.phase=DFS_COMPLETE;s.nav.reason="TEST";s.nav.goal=64;
    for(unsigned y=0;y<8;++y)for(unsigned x=0;x<8;++x) {
        uint8_t id=(uint8_t)(255-y*8-x);
        assert(rover_map_place(&s.nav.map,id,(int)x-4,(int)y-4));
        assert(rover_map_visit(&s.nav.map,id));
    }
    for(unsigned i=0;i<s.nav.map.node_count;++i)for(unsigned d=0;d<2;++d)
        assert(rover_map_exit(&s.nav.map,s.nav.map.nodes[i].marker_id,d,MAP_EXPLORED));
    char json[ONBOARD_SNAPSHOT_BYTES];size_t bytes=onboard_dfs_snapshot(&s.nav,json,sizeof(json));
    assert(bytes>0&&bytes<sizeof(json)&&strstr(json,"\"rows\":8,\"columns\":8"));
    char small[16];assert(onboard_dfs_snapshot(&s.nav,small,sizeof(small))==0);
    FILE *file=fopen("build/onboard-capacity-snapshot.json","w");
    if(file){fputs(json,file);fclose(file);}
}
static void test_camera_capacity(void) {
    simulation_t s;initialise(&s);sensors(&s);
    assert(onboard_dfs_set_start(&s.nav,1,s.now));
    for(unsigned i=1;i<ROVER_MAX_NODES;++i)
        assert(rover_map_place(&s.nav.map,(uint8_t)(i+1),(int)i,0));
    rover_map_t before=s.nav.map;
    s.nav.phase=DFS_SCAN;s.now+=100;sensors(&s);
    vision_frame_t frame={0};frame.status=VISION_OK;frame.count=1;
    frame.markers[0]=(vision_marker_t){255,580,0,0};
    for(unsigned i=0;i<3;++i) {
        frame.sequence=(uint8_t)i;onboard_dfs_camera(&s.nav,&frame,s.now,s.now);
    }
    assert(s.nav.phase==DFS_FAULT&&!strcmp(s.nav.reason,"MAP_CAPACITY_REACHED"));
    assert(s.drive_calls==0&&memcmp(&before,&s.nav.map,sizeof(before))==0);
    initialise(&s);s.ids[0][0]=40;s.ids[1][0]=255;link_cells(&s,0,0,MAP_NORTH);
    anchor(&s,2);assert(run(&s,20000,false));
    assert(s.nav.map.current==255&&s.nav.map.visited_count==2);
}
static void test_geometry_and_quality(void) {
    simulation_t s;initialise(&s);
    onboard_motor_io_t io={&s,drive,turn,turn_done,stop};onboard_dfs_init(&s.nav,&io);
    sensors(&s);assert(!s.nav.config.camera_calibrated);
    vision_frame_t f={0};f.status=VISION_NO_MARKER;f.sequence=1;
    onboard_dfs_camera(&s.nav,&f,s.now,s.now);
    assert(onboard_dfs_set_start(&s.nav,1,s.now));
    assert(!onboard_dfs_start(&s.nav,2,500,s.now)&&s.drive_calls==0);
    assert(!onboard_dfs_set_geometry(&s.nav,325,120,0,s.now)); /* Map anchored. */
    onboard_dfs_init(&s.nav,&io);
    assert(!onboard_dfs_set_geometry(&s.nav,0,120,0,s.now));
    assert(!onboard_dfs_set_geometry(&s.nav,NAN,120,0,s.now));
    assert(!onboard_dfs_set_geometry(&s.nav,325,120,76,s.now));
    assert(onboard_dfs_set_geometry(&s.nav,325,120,0,s.now));
    sensors(&s);assert(onboard_dfs_set_start(&s.nav,1,s.now));
    f.status=VISION_OK;f.count=1;f.markers[0]=(vision_marker_t){2,0,0,VISION_POSE_FAILED};
    for(unsigned i=0;i<3;++i) {f.sequence=(uint8_t)(i+1);onboard_dfs_camera(&s.nav,&f,s.now,s.now);}
    assert(s.nav.image_seen&&s.nav.map.node_count==1&&s.nav.last_camera.markers[0].quality==VISION_POSE_FAILED);
    for(unsigned q=2;q<=4;++q) {
        f.markers[0]=(vision_marker_t){2,580,0,(uint8_t)q};
        for(unsigned i=0;i<3;++i){++f.sequence;onboard_dfs_camera(&s.nav,&f,s.now,s.now);}
        assert(s.nav.map.node_count==1);
    }
    f.markers[0].quality=VISION_QUALITY_OK;
    for(unsigned i=0;i<3;++i){++f.sequence;onboard_dfs_camera(&s.nav,&f,s.now,s.now);}
    assert(s.nav.map.node_count==2);
    ++f.sequence;onboard_dfs_camera(&s.nav,&f,s.now,s.now);
    uint32_t fixed=s.nav.last_pose_ms;
    s.now+=100;sensors(&s);f.markers[0].quality=VISION_TOO_SMALL;++f.sequence;
    onboard_dfs_camera(&s.nav,&f,s.now,s.now);
    assert(s.nav.last_pose_ms==fixed&&s.nav.arrival_count==0);
    char json[ONBOARD_SNAPSHOT_BYTES];assert(onboard_dfs_snapshot(&s.nav,json,sizeof(json)));
    assert(strstr(json,"\"geometry_ready\":true")&&strstr(json,"\"quality\":3"));
}
static void test_dynamic_arenas(void) {
    const int sizes[][2]={{1,1},{8,3},{2,20},{8,8},{64,1},{3,8}};
    for(unsigned i=0;i<sizeof(sizes)/sizeof(sizes[0]);++i) {
        simulation_t s;initialise(&s);s.width=sizes[i][0];s.height=sizes[i][1];
        snake(&s);
        /* Alternate map-north heading so real discoveries also grow west. */
        if(i%2)s.heading=90;
        anchor(&s,(unsigned)(s.width*s.height));
        assert(run(&s,120000,false));
        assert(s.nav.map.visited_count==s.width*s.height&&!s.drive);
        assert(s.nav.map.node_count==s.nav.map.visited_count);
        assert((rover_map_columns(&s.nav.map)==(unsigned)s.width&&rover_map_rows(&s.nav.map)==(unsigned)s.height)||
               (rover_map_columns(&s.nav.map)==(unsigned)s.height&&rover_map_rows(&s.nav.map)==(unsigned)s.width));
        if(i%2&&s.height>1)assert(s.nav.map.min_x<0);
    }
}
static void test_branch_and_backtrack(void) {
    simulation_t s;initialise(&s);s.ids[0][0]=1;s.ids[1][0]=2;s.ids[0][1]=3;s.ids[1][1]=4;
    link_cells(&s,0,0,MAP_NORTH);link_cells(&s,0,0,MAP_EAST);link_cells(&s,1,0,MAP_NORTH);
    anchor(&s,4);assert(run(&s,20000,false));
    assert(s.backtracks>=1&&s.nav.map.visited_count==4&&s.nav.map.current==4&&!s.drive);
    assert(s.nav.map.visited_count!=s.arrivals+1); /* Revisit A did not increase count. */
    const rover_map_t *view=onboard_dfs_map(&s.nav);assert(view==&s.nav.map);
    assert(!onboard_dfs_begin_hop(&s.nav,1,MAP_WEST,1,s.now)); /* Nonadjacent. */
    assert(onboard_dfs_begin_hop(&s.nav,3,MAP_SOUTH,1,s.now));
    assert(run(&s,5000,false));
    onboard_hop_result_t result;assert(onboard_dfs_take_result(&s.nav,&result));
    assert(result.arrived&&result.target==3&&result.sequence==1);
    assert(!onboard_dfs_take_result(&s.nav,&result));
    assert(!onboard_dfs_begin_hop(&s.nav,4,MAP_NORTH,1,s.now)); /* Replayed hop. */
}
static void test_faults(void) {
    simulation_t s;initialise(&s);snake(&s);anchor(&s,36);
    while(s.nav.phase!=DFS_DRIVE){step(&s);assert(s.nav.phase!=DFS_FAULT);}
    unsigned visited=s.nav.map.visited_count;uint8_t current=s.nav.map.current;
    s.camera_disabled=true;
    for(unsigned i=0;i<35;++i)step(&s);
    assert(s.nav.phase==DFS_HOLD&&!s.drive);
    assert(s.nav.map.current==current&&s.nav.map.visited_count==visited&&!s.nav.current_confirmed);
    for(unsigned i=0;i<250;++i)step(&s);
    assert(s.nav.phase==DFS_FAULT&&!strcmp(s.nav.reason,"CAMERA_TIMEOUT"));
    s.camera_disabled=false;for(unsigned i=0;i<20;++i)step(&s);
    assert(s.nav.phase==DFS_FAULT&&!s.drive); /* No spontaneous restart. */

    initialise(&s);snake(&s);anchor(&s,36);
    while(s.nav.phase!=DFS_DRIVE){step(&s);assert(s.nav.phase!=DFS_FAULT);}
    s.camera_disabled=true;
    vision_frame_t frozen={0};frozen.sequence=s.nav.last_sequence;frozen.status=VISION_NO_MARKER;
    for(unsigned i=0;i<300;++i) {
        step(&s);onboard_dfs_camera(&s.nav,&frozen,s.now,s.now);
    }
    assert(s.nav.phase==DFS_FAULT&&!strcmp(s.nav.reason,"CAMERA_TIMEOUT"));

    initialise(&s);snake(&s);anchor(&s,36);
    while(s.nav.phase!=DFS_DRIVE){step(&s);assert(s.nav.phase!=DFS_FAULT);}
    s.invalid_front=true;step(&s);assert(s.nav.phase==DFS_FAULT&&!s.drive);

    initialise(&s);snake(&s);anchor(&s,36);
    while(s.nav.phase!=DFS_DRIVE){step(&s);assert(s.nav.phase!=DFS_FAULT);}
    s.empty_camera=true;for(unsigned i=0;i<1000&&s.nav.phase!=DFS_FAULT;++i)step(&s);
    assert(s.nav.phase==DFS_FAULT&&!s.drive); /* Healthy empty frames cannot renew pose forever. */
    assert(strcmp(s.nav.reason,"CAMERA_TIMEOUT")!=0&&s.nav.recovery_angle>0);

    initialise(&s);s.ids[0][0]=1;anchor(&s,2);
    for(unsigned i=0;i<2000&&s.nav.phase!=DFS_FAULT;++i)step(&s);
    assert(s.nav.phase==DFS_FAULT&&!strcmp(s.nav.reason,"EXPLORATION_EXHAUSTED"));

    initialise(&s);snake(&s);anchor(&s,36);
    while(s.nav.phase!=DFS_DRIVE){step(&s);assert(s.nav.phase!=DFS_FAULT);}
    onboard_dfs_stop(&s.nav,"OPERATOR_STOP",s.now);
    for(unsigned i=0;i<50;++i)step(&s);
    assert(s.nav.phase==DFS_PAUSED&&!s.drive&&s.nav.map.visited_count==1);
}
static void test_loops_arbitrary_ids_and_recovery(void) {
    simulation_t s;initialise(&s);
    s.ids[0][0]=7;s.ids[1][0]=12;s.ids[0][1]=3;s.ids[1][1]=35;
    link_cells(&s,0,0,MAP_NORTH);link_cells(&s,0,0,MAP_EAST);link_cells(&s,1,0,MAP_NORTH);
    anchor(&s,4);assert(run(&s,20000,false));assert(s.nav.map.current==35&&s.nav.map.start==7);
    initialise(&s);s.ids[0][0]=1;s.ids[1][0]=2;s.ids[0][1]=3;s.ids[1][1]=4;
    link_cells(&s,0,0,MAP_NORTH);link_cells(&s,0,0,MAP_EAST);link_cells(&s,1,0,MAP_NORTH);link_cells(&s,0,1,MAP_EAST);
    anchor(&s,5);
    for(unsigned i=0;i<20000&&s.nav.phase!=DFS_FAULT;++i)step(&s);
    assert(s.nav.phase==DFS_FAULT&&!strcmp(s.nav.reason,"EXPLORATION_EXHAUSTED")&&s.nav.map.visited_count==4);
    initialise(&s);snake(&s);anchor(&s,36);
    while(s.nav.phase!=DFS_DRIVE){step(&s);assert(s.nav.phase!=DFS_FAULT);}
    s.empty_camera=true;
    for(unsigned i=0;i<35;++i)step(&s);
    assert(s.nav.phase==DFS_HOLD&&!s.drive);
    s.empty_camera=false;
    assert(run(&s,60000,false));
    initialise(&s);snake(&s);s.empty_camera=true;anchor(&s,36);
    for(unsigned i=0;i<500&&s.nav.phase!=DFS_HOLD;++i)step(&s);
    assert(s.nav.phase==DFS_HOLD&&s.nav.current_confirmed&&s.drive_calls==0);
    s.empty_camera=false;assert(run(&s,60000,false));
    initialise(&s);snake(&s);anchor(&s,36);
    while(s.nav.phase!=DFS_CONFIRM){step(&s);assert(s.nav.phase!=DFS_FAULT);}
    s.now+=500;sensors(&s);
    vision_frame_t fix={0};fix.status=VISION_OK;fix.count=1;fix.sequence=(uint8_t)(s.nav.last_sequence+1);
    fix.markers[0]=(vision_marker_t){3,580,0,0};
    onboard_dfs_camera(&s.nav,&fix,s.now,s.now);assert(s.nav.arrival_count==1);
    for(unsigned i=0;i<5;++i)onboard_dfs_camera(&s.nav,&fix,s.now,s.now);
    assert(s.nav.arrival_count==1&&s.nav.map.visited_count==1);
    ++fix.sequence;onboard_dfs_camera(&s.nav,&fix,s.now,s.now);
    ++fix.sequence;onboard_dfs_camera(&s.nav,&fix,s.now,s.now);
    onboard_dfs_tick(&s.nav,s.now);assert(s.nav.map.visited_count==2);
}
int main(int argc,char **argv) {
    bool demo=argc>1&&!strcmp(argv[1],"--demo");
    if(!demo) {test_map();test_geometry_and_quality();test_snapshot_capacity();test_camera_capacity();test_dynamic_arenas();test_branch_and_backtrack();test_faults();test_loops_arbitrary_ids_and_recovery();}
    simulation_t s;initialise(&s);
    if(demo) {
        s.width=argc>2?atoi(argv[2]):8;s.height=argc>3?atoi(argv[3]):8;
        if(s.width<1||s.height<1||s.width>TEST_ARENA_SIZE||s.height>TEST_ARENA_SIZE||s.width*s.height>(int)ROVER_MAX_NODES) {
            fprintf(stderr,"Require positive columns/rows with at most %u nodes\n",ROVER_MAX_NODES);return 2;
        }
    }
    snake(&s);anchor(&s,(unsigned)(s.width*s.height));
    assert(run(&s,120000,demo));
    assert(s.nav.map.visited_count==s.width*s.height&&!s.drive&&s.nav.phase==DFS_COMPLETE);
    if(!demo)puts("PASS: dynamic map expansion, capacity, 8x8/2x20/64x1 DFS, backtracking, recovery, B2 hops and 36-node regression");
    return 0;
}
