#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "planner_fixture.h"

static void check_reciprocal(const rover_map_t *map) {
    for (unsigned i=0;i<map->node_count;++i) {
        const rover_node_t *a=&map->nodes[i];
        for (unsigned d=0;d<4;++d) {
            const rover_node_t *b=rover_map_at(map,a->x+rover_dx[d],a->y+rover_dy[d]);
            if (b) assert(a->exits[d]==b->exits[(d+2u)%4u]);
        }
    }
}
int main(void) {
    rover_map_t map;
    assert(planner_fixture_build(&map));
    assert(map.start==7 && map.current==7 && map.node_count==8 && map.visited_count==6);
    assert(rover_map_columns(&map)==5 && rover_map_rows(&map)==2);
    assert(map.min_x==-1 && map.max_x==3 && map.min_y==0 && map.max_y==1);
    assert(!rover_map_at(&map,-1,1));
    assert(rover_map_find(&map,40,0,0)->visited==false);
    assert(rover_map_find(&map,255,0,0)->marker_id==255);
    check_reciprocal(&map);
    rover_map_t before=map;
    assert(!planner_fixture_arrive(&map,12,MAP_NORTH));
    assert(!planner_fixture_arrive(&map,40,MAP_WEST)); /* Open, untraversed. */
    assert(!planner_fixture_arrive(&map,255,MAP_EAST));
    assert(!planner_fixture_arrive(&map,0,MAP_EAST));
    assert(memcmp(&map,&before,sizeof(map))==0);
    assert(planner_fixture_arrive(&map,3,MAP_NORTH));
    assert(planner_fixture_arrive(&map,9,MAP_EAST));
    assert(planner_fixture_arrive(&map,36,MAP_EAST));
    assert(map.current==36 && map.visited_count==6);
    assert(planner_fixture_build(&map));
    assert(rover_map_exit(&map,7,MAP_NORTH,MAP_BLOCKED));
    assert(!planner_fixture_arrive(&map,3,MAP_NORTH));
    assert(planner_fixture_arrive(&map,12,MAP_EAST));
    assert(!planner_fixture_arrive(&map,9,MAP_NORTH)); /* Adjacent but blocked. */
    assert(planner_fixture_arrive(&map,20,MAP_EAST));
    assert(planner_fixture_arrive(&map,36,MAP_NORTH));
    assert(planner_fixture_build(&map));
    assert(rover_map_exit(&map,7,MAP_NORTH,MAP_BLOCKED));
    assert(rover_map_exit(&map,7,MAP_EAST,MAP_BLOCKED));
    const rover_node_t *start=rover_map_find(&map,7,0,0);
    for (unsigned d=0;d<4;++d) assert(start->exits[d]!=MAP_EXPLORED);
    check_reciprocal(&map);
    assert(planner_fixture_build(&map));
    puts("Offline planner fixture: 8 known nodes, 6 visited, start/current 7; bounds 5 columns x 2 rows.");
    for (unsigned i=0;i<map.node_count;++i) {
        const rover_node_t *n=&map.nodes[i];
        printf("ID=%u position=(%d,%d) visited=%s exits[N,E,S,W]=[%u,%u,%u,%u]\n",
            n->marker_id,n->x,n->y,n->visited?"true":"false",n->exits[0],n->exits[1],n->exits[2],n->exits[3]);
    }
    puts("PASS: fixture integrity, both three-hop routes, blocked/open rejection, simulated arrivals and no-route setup.");
    return 0;
}
