#include "planner_fixture.h"

bool planner_fixture_build(rover_map_t *map) {
    if (!map) return false;
    const struct { uint8_t id; int x, y; bool visited; } nodes[] = {
        {7,0,0,true}, {12,1,0,true}, {20,2,0,true},
        {3,0,1,true}, {9,1,1,true}, {36,2,1,true},
        {40,-1,0,false}, {255,3,0,false}
    };
    const struct { uint8_t from, to; unsigned direction; } edges[] = {
        {7,12,MAP_EAST}, {12,20,MAP_EAST}, {7,3,MAP_NORTH},
        {3,9,MAP_EAST}, {9,36,MAP_EAST}, {20,36,MAP_NORTH}
    };
    rover_map_init(map,7);
    for (unsigned i=0;i<sizeof(nodes)/sizeof(nodes[0]);++i)
        if (!rover_map_place(map,nodes[i].id,nodes[i].x,nodes[i].y)) return false;
    for (unsigned i=0;i<map->node_count;++i)
        for (unsigned d=0;d<4;++d)
            if (!rover_map_exit(map,map->nodes[i].marker_id,d,MAP_BLOCKED)) return false;
    for (unsigned i=0;i<sizeof(edges)/sizeof(edges[0]);++i)
        if (!rover_map_connect(map,edges[i].from,edges[i].to,edges[i].direction)) return false;
    if (!rover_map_exit(map,7,MAP_WEST,MAP_OPEN)) return false;
    for (unsigned i=0;i<sizeof(nodes)/sizeof(nodes[0]);++i)
        if (nodes[i].visited && !rover_map_visit(map,nodes[i].id)) return false;
    return rover_map_visit(map,7); /* Start and current are both 7. */
}

bool planner_fixture_arrive(rover_map_t *map,uint8_t target,unsigned heading) {
    if (!map || heading>3) return false;
    const rover_node_t *from=rover_map_find(map,map->current,0,0);
    const rover_node_t *to=rover_map_find(map,target,0,0);
    if (!from || !to || to->x!=from->x+rover_dx[heading] ||
        to->y!=from->y+rover_dy[heading] || from->exits[heading]!=MAP_EXPLORED ||
        to->exits[(heading+2u)%4u]!=MAP_EXPLORED) return false;
    return rover_map_visit(map,target);
}
