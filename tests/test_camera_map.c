#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "camera_map.h"

static rover_map_t map;
static camera_map_t mapper;
static camera_observation_t ob;
static uint32_t now;
static void setup(float heading,float north)
{
    rover_map_init(&map,123); assert(rover_map_start(&map,1)==MAP_OK);
    camera_map_init(&mapper); now=100;
    ob=(camera_observation_t){.frame={.status=VISION_STATUS_OK,.age_ms=100,.count=2,
        .markers={{.id=9,.range_mm=870,.bearing_cdeg=200},{.id=2,.range_mm=470,.bearing_cdeg=-100}}}};
    camera_map_tick(&mapper,&map,NULL,true,true,heading,north,now);
}
static void tick(float heading,float north)
{
    now+=500; ob.generation++; ob.frame.seq++; ob.received_ms=now;
    camera_map_tick(&mapper,&map,&ob,true,true,heading,north,now);
}
static void frames(float heading,float north)
{
    for(unsigned i=0;i<3;++i) tick(heading,north);
}
static void expected(uint8_t id,int x,int y)
{
    for(size_t i=0;i<map.observed_count;++i)
        if(map.observed[i].marker_id==id) { assert(map.observed[i].x==x && map.observed[i].y==y); return; }
    assert(!"Missing observed marker");
}
static void fixture(const char *prefix)
{
    char json[ROVER_MAP_JSON_CAPACITY]; assert(rover_map_format(&map,"",json,sizeof(json)));
    printf("%s%s\n",prefix,json);
}
static void test_order_stability_and_arrival_colour_evidence(void)
{
    setup(0,0); tick(0,0);
    assert(!map.observed_count);
    // Repeated packets/reads and a cached frame aging out cannot add confirmations.
    camera_map_tick(&mapper,&map,&ob,true,true,0,0,now);
    assert(mapper.confirmations==1);
    ob.generation++; camera_map_tick(&mapper,&map,&ob,true,true,0,0,now);
    assert(mapper.confirmations==1);
    camera_map_tick(&mapper,&map,&ob,true,true,0,0,now+350);
    assert(mapper.confirmations==1);
    tick(0,0); assert(!map.observed_count); tick(0,0);
    assert(map.count==1 && map.observed_count==2 && map.current_node==1 && map.nodes[0].visited);
    expected(2,0,1); expected(9,0,2);
    for(unsigned i=0;i<4;++i) assert(map.nodes[0].exits[i]==MAP_EXIT_UNKNOWN);
    assert(!rover_map_find_id(&map,2)); // Estimates cannot become DFS coordinates.
    fixture("SEEN_MAP_JSON:");
    uint32_t rev=map.revision;
    ob.frame.count=1; ob.frame.markers[0]=(camera_marker_t){.id=9,.range_mm=700}; frames(0,0);
    assert(map.revision==rev); expected(9,0,2); // Does not collapse to slot 1.
    ob.frame.count=2; ob.frame.markers[1]=(camera_marker_t){.id=10,.range_mm=1000}; frames(0,0);
    expected(10,0,3);
    // Reliable coordinate assignment alone is still blue; only arrival marks green.
    assert(rover_map_add(&map,2,0,1,false)==MAP_OK);
    assert(!rover_map_find_id(&map,2)->visited && map.current_node==1);
    assert(rover_map_arrive(&map,2)==MAP_OK && map.current_node==2 && rover_map_find_id(&map,2)->visited);
    assert(map.observed_count==2); expected(9,0,2);
    fixture("ARRIVAL_MAP_JSON:");
    assert(rover_map_start(&map,1)==MAP_OK && !map.observed_count && map.count==1);
}
static void test_heading_bearing_and_new_anchor(void)
{
    static const int x[]={0,1,0,-1},y[]={1,0,-1,0};
    for(unsigned d=0;d<4;++d) {
        float heading=fmodf(350.0f+90.0f*(float)d,360.0f);
        setup(heading,350); frames(heading,350);
        expected(2,x[d],y[d]); expected(9,2*x[d],2*y[d]);
    }
    setup(0,0); frames(0,0);
    now+=10; camera_map_tick(&mapper,&map,&ob,false,true,90,0,now);
    now+=10; camera_map_tick(&mapper,&map,&ob,true,true,90,0,now);
    ob.frame.markers[0].id=3; ob.frame.markers[1].id=4;
    // Reject a delayed pre-turn image even though it arrived after the turn.
    now+=100; ob.generation++;ob.frame.seq++;ob.received_ms=now;ob.frame.age_ms=200;
    camera_map_tick(&mapper,&map,&ob,true,true,90,0,now);
    assert(map.observed_count==2);
    ob.frame.age_ms=100; frames(90,0); expected(4,1,0); expected(3,2,0);
    fixture("ROTATED_MAP_JSON:");
    assert(rover_map_add(&map,2,0,1,false)==MAP_OK && rover_map_arrive(&map,2)==MAP_OK);
    ob.frame.count=1;ob.frame.markers[0]=(camera_marker_t){.id=5,.range_mm=500};
    now+=10;camera_map_tick(&mapper,&map,&ob,true,true,90,0,now);
    frames(90,0); expected(5,1,1); // Relative to node 2, not start node 1.
    setup(75,0);ob.frame.count=1;ob.frame.markers[0]=(camera_marker_t){.id=8,.range_mm=500,.bearing_cdeg=1500};
    frames(75,0); expected(8,1,0); // Bearing combines with rover heading.
}
static void test_unreliable_unknown_pose_and_capacity(void)
{
    setup(0,0);ob.frame.markers[0].quality=1;ob.frame.markers[1].bearing_cdeg=4000;frames(0,0);
    assert(!map.observed_count);
    ob.frame.markers[0].quality=0;ob.frame.markers[1].bearing_cdeg=0;ob.frame.age_ms=2000;frames(0,0);
    assert(!map.observed_count);
    ob.frame.age_ms=100;ob.frame.status=VISION_STATUS_NO_MARKER;frames(0,0);assert(!map.observed_count);
    ob.frame.status=VISION_STATUS_OK;
    for(unsigned i=0;i<3;++i) { now+=500;ob.received_ms=now;ob.generation++;ob.frame.seq++;
        camera_map_tick(&mapper,&map,&ob,true,false,0,0,now); }
    assert(!map.observed_count);
    camera_map_tick(&mapper,&map,&ob,true,true,NAN,0,now);assert(!map.observed_count);
    rover_map_depart(&map);frames(0,0);assert(!map.observed_count && !map.current_node);
    assert(rover_map_start(&map,1)==MAP_OK);
    for(unsigned id=2;id<=ROVER_MAP_CAPACITY;++id)
        assert(rover_map_observe(&map,(uint8_t)id,0,(int16_t)(id-1))==MAP_OK);
    assert(rover_map_observe(&map,255,1,0)==MAP_FULL);
    char json[ROVER_MAP_JSON_CAPACITY];
    assert(rover_map_format(&map,"",json,sizeof(json)) && strlen(json)<8192u-32u);
    assert(rover_map_add(&map,255,1,0,true)==MAP_OK); // A real arrival takes priority at capacity.
    assert(map.count+map.observed_count==ROVER_MAP_CAPACITY);
    assert(rover_map_add(&map,254,0,1,false)==MAP_OK); // Discards a conflicting display estimate.
    for(size_t i=0;i<map.observed_count;++i) assert(map.observed[i].marker_id!=2);
}
int main(void)
{
    test_order_stability_and_arrival_colour_evidence();
    test_heading_bearing_and_new_anchor();
    test_unreliable_unknown_pose_and_capacity();
    puts("Camera map tests passed: ordered sightings, rotations, confirmed-node anchoring, arrival promotion, freshness and capacity");
    return 0;
}
