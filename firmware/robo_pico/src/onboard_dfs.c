#include "onboard_dfs.h"
#include "motor_driver.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const int8_t dx[4] = {0,1,0,-1}, dy[4] = {1,0,-1,0};
static bool seen(const onboard_dfs_t *d, uint8_t id) { return (d->seen[id/8u] & (1u << (id%8u))) != 0; }
static void mark_seen(onboard_dfs_t *d, uint8_t id) { d->seen[id/8u] |= (uint8_t)(1u << (id%8u)); }
static bool cell(const map_node_t *from, unsigned dir, int16_t *x, int16_t *y)
{
    int32_t tx = (int32_t)from->x + dx[dir], ty = (int32_t)from->y + dy[dir];
    if (tx < INT16_MIN || tx > INT16_MAX || ty < INT16_MIN || ty > INT16_MAX) return false;
    *x = (int16_t)tx; *y = (int16_t)ty; return true;
}
static const map_node_t *neighbour(const rover_map_t *map, const map_node_t *node, unsigned dir)
{
    int16_t x,y;
    return cell(node, dir, &x, &y) ? rover_map_find_cell(map,x,y) : NULL;
}
static bool whole_map_resolved(const onboard_dfs_t *d, const rover_map_t *map)
{
    // Sighted unvisited markers remain unresolved, even though their estimated
    // display positions must not be used as verified planner connectivity.
    if (map->observed_count) return false;
    for (size_t i=0; i<map->count; ++i) {
        const map_node_t *node = &map->nodes[i];
        if (!node->visited || !seen(d,node->marker_id)) return false;
        for (unsigned dir=0; dir<4; ++dir) {
            if (node->exits[dir] == MAP_EXIT_UNKNOWN) return false;
            if (node->exits[dir] == MAP_EXIT_OPEN || node->exits[dir] == MAP_EXIT_TRAVERSED) {
                const map_node_t *next = neighbour(map,node,dir);
                if (!next || !next->visited || !seen(d,next->marker_id) || next->exits[(dir+2u)%4u] == MAP_EXIT_BLOCKED)
                    return false;
            }
        }
    }
    return map->count != 0;
}

dfs_choice_t onboard_dfs_choose(const onboard_dfs_t *d, const rover_map_t *map,
    map_direction_t facing, bool camera_hint)
{
    const map_node_t *node = rover_map_find_id(map,map->current_node);
    if (!d->depth || !node || d->stack[d->depth-1u] != node->marker_id)
        return (dfs_choice_t){.kind=DFS_CHOOSE_ERROR};
    unsigned order[4] = {(unsigned)facing, ((unsigned)facing+1u)%4u, ((unsigned)facing+3u)%4u, ((unsigned)facing+2u)%4u};
    for (unsigned i=0; i<4; ++i) {
        unsigned dir=order[i]; const map_node_t *next=neighbour(map,node,dir);
        bool open=node->exits[dir]==MAP_EXIT_OPEN || node->exits[dir]==MAP_EXIT_TRAVERSED;
        bool provisional=i==0 && camera_hint && node->exits[dir]!=MAP_EXIT_BLOCKED;
        if ((open || provisional) && (!next || !seen(d,next->marker_id)))
            return (dfs_choice_t){.kind=DFS_CHOOSE_EXPLORE,.direction=(map_direction_t)dir,.expected_id=next?next->marker_id:0};
    }
    bool unknown=false;
    for (unsigned i=0; i<4; ++i) {
        unsigned dir=order[i];
        if (node->exits[dir] == MAP_EXIT_UNKNOWN) unknown=true;
        if (!(d->scanned[d->depth-1u] & (1u<<dir)))
            return (dfs_choice_t){.kind=DFS_CHOOSE_SCAN,.direction=(map_direction_t)dir};
    }
    if (unknown) return (dfs_choice_t){.kind=DFS_CHOOSE_UNRESOLVED};
    if (d->depth>1u) {
        const map_node_t *parent=rover_map_find_id(map,d->stack[d->depth-2u]);
        for (unsigned dir=0; parent && dir<4; ++dir) {
            const map_node_t *next=neighbour(map,node,dir);
            if (next && next->marker_id==parent->marker_id && node->exits[dir]!=MAP_EXIT_BLOCKED)
                return (dfs_choice_t){.kind=DFS_CHOOSE_BACKTRACK,.direction=(map_direction_t)dir,.expected_id=parent->marker_id};
        }
        return (dfs_choice_t){.kind=DFS_CHOOSE_ERROR};
    }
    return (dfs_choice_t){.kind=whole_map_resolved(d,map)?DFS_CHOOSE_COMPLETE:DFS_CHOOSE_UNRESOLVED};
}

bool onboard_dfs_note_arrival(onboard_dfs_t *d, const rover_map_t *map)
{
    if (!d->depth || !map->current_node || (d->expected_id && d->expected_id!=map->current_node)) return false;
    if (d->from_node && d->stack[d->depth-1u]!=d->from_node) return false;
    if (d->backtracking) {
        if (d->depth<2u || d->stack[d->depth-2u]!=map->current_node) return false;
        d->depth--;
    } else {
        if (seen(d,map->current_node) || d->depth>=ROVER_MAP_CAPACITY) return false;
        d->stack[d->depth]=map->current_node; d->scanned[d->depth]=0; d->depth++;
        mark_seen(d,map->current_node);
    }
    return true;
}

void onboard_dfs_init(onboard_dfs_t *d) { *d=(onboard_dfs_t){.stage=DFS_IDLE,.detail="NOT_STARTED",.open_distance_mm=DFS_OPEN_DISTANCE_MM}; }
static void phase(mission_t *m, leg_phase_t value, uint32_t now)
{
    m->leg.phase=value; m->revision++; m->changed_ms=now;
}
void onboard_dfs_stop(onboard_dfs_t *d, mission_t *m, const char *detail, uint32_t now)
{
    motor_emergency_stop(); d->active=false; d->stage=DFS_STOPPED; d->detail=detail;
    mission_pause(m,MISSION_REASON_INPUT_LOST,now);
}
static float wrap(float h) { h=fmodf(h,360.0f); return h<0?h+360.0f:h; }
static map_direction_t facing(const onboard_drive_t *drive, float raw)
{
    return (map_direction_t)((unsigned)((wrap(raw-drive->north_heading_deg)+45.0f)/90.0f)%4u);
}
static float error(float a, float b) { float e=fabsf(wrap(a)-wrap(b)); return e>180?360-e:e; }
static void begin_node(onboard_dfs_t *d, mission_t *m, onboard_drive_t *drive, target_selector_t *s, uint32_t now)
{
    bool frame=drive->frame_valid; float north=drive->north_heading_deg;
    onboard_drive_init(drive); drive->frame_valid=frame; drive->north_heading_deg=north; drive->result="SCANNING_NODE";
    mission_clear_leg(m,now); target_selector_init(s); phase(m,LEG_SCANNING,now);
    d->stage=DFS_AT_NODE; d->detail="READING_NODE_EXITS";
}
const char *onboard_dfs_start(onboard_dfs_t *d, mission_t *m, const rover_map_t *map,
    onboard_drive_t *drive, target_selector_t *s, const onboard_dfs_inputs_t *in, uint32_t now)
{
    if (!map->count) return "START_NODE_NOT_SET";
    if (d->active && m->dfs_active) return "ALREADY_RUNNING";
    const map_node_t *node=rover_map_find_id(map,map->current_node);
    if (!node || !node->visited) return "CURRENT_NODE_UNKNOWN";
    if (in->drive.safety_latched) return "SAFETY_LOCKED";
    if (!in->drive.link_ok) return "LINK_LOST";
    if (!in->drive.sensor_valid || (uint32_t)(now-in->drive.sensor_received_ms)>ONBOARD_DRIVE_SENSOR_MAX_AGE_MS) return "SENSOR_STALE";
    if (!in->drive.imu_ok || !isfinite(in->drive.heading_deg) || in->drive.heading_deg<0 || in->drive.heading_deg>=360) return "HEADING_UNAVAILABLE";
    if (!in->drive.stopped) return "MOTOR_BUSY";
    if (!drive->frame_valid && map->count!=1u) return "GRID_HEADING_UNAVAILABLE";
    if (!mission_start_dfs(m,now)) return "MISSION_NOT_IDLE";
    if (!drive->frame_valid) { drive->north_heading_deg=in->drive.heading_deg; drive->frame_valid=true; }
    uint16_t opening=d->open_distance_mm?d->open_distance_mm:DFS_OPEN_DISTANCE_MM;
    *d=(onboard_dfs_t){.active=true,.stage=DFS_AT_NODE,.depth=1,.detail="READING_NODE_EXITS",.open_distance_mm=opening};
    d->stack[0]=map->current_node; mark_seen(d,map->current_node);
    begin_node(d,m,drive,s,now);
    return "STARTED";
}

bool onboard_dfs_selecting(const onboard_dfs_t *d, uint32_t now)
{
    return d->active && d->stage==DFS_WAIT_TARGET && (int32_t)(now-d->ready_ms)>=0;
}
static void sampling(onboard_dfs_t *d, mission_t *m, uint32_t now)
{
    d->stage=DFS_SAMPLING; d->stage_started_ms=now; d->ready_ms=now+DFS_SETTLE_MS;
    d->last_sensor_ms=0; memset(d->sample_code,0,sizeof(d->sample_code)); memset(d->sample_count,0,sizeof(d->sample_count));
    phase(m,LEG_SCANNING,now); d->detail="SAMPLING_TOF_EXITS";
}
static bool aim(onboard_dfs_t *d, mission_t *m, const onboard_drive_t *drive,
                const onboard_dfs_inputs_t *in, dfs_stage_t after, uint32_t now)
{
    d->after_turn=after;
    float raw_target=wrap(drive->north_heading_deg+90.0f*(float)d->direction);
    if (error(raw_target,in->drive.heading_deg)<=3.0f) {
        if (after==DFS_SAMPLING) sampling(d,m,now);
        else { d->stage=DFS_WAIT_TARGET; d->stage_started_ms=now; d->ready_ms=now+DFS_SETTLE_MS; }
        return true;
    }
    int target=(int)(wrap(raw_target-in->heading_offset_deg)+0.5f)%360;
    if (motor_auto_rotate_leased(target,ONBOARD_DRIVE_LEASE_MS)!=MOTOR_OK) return false;
    phase(m,LEG_TURNING,now); d->stage=DFS_TURNING; d->stage_started_ms=d->last_renew_ms=now;
    d->detail=after==DFS_SAMPLING?"TURNING_TO_SCAN":"TURNING_TO_NEXT_EXIT";
    return true;
}
static bool camera_hint(const camera_observation_t *ob, const rover_map_t *map,
    map_direction_t dir, uint32_t settled, uint32_t now)
{
    if (!ob || ob->frame.status!=VISION_STATUS_OK || ob->frame.count>VISION_MAX_MARKERS ||
        (uint32_t)(now-ob->received_ms)>TARGET_UART_MAX_AGE_MS ||
        (uint32_t)(now-ob->received_ms)+ob->frame.age_ms>ONBOARD_DRIVE_IMAGE_MAX_AGE_MS ||
        (int32_t)(ob->received_ms-ob->frame.age_ms-settled)<0) return false;
    const map_node_t *from=rover_map_find_id(map,map->current_node); int16_t x,y;
    if (!from || !cell(from,(unsigned)dir,&x,&y)) return false;
    const map_node_t *occupant=rover_map_find_cell(map,x,y);
    for (uint8_t i=0;i<ob->frame.count;++i) {
        const camera_marker_t *mk=&ob->frame.markers[i];
        if (!mk->id || mk->id==from->marker_id || mk->quality || mk->range_mm<=ONBOARD_DRIVE_NEAR_RANGE_MM ||
            mk->range_mm==UINT16_MAX || mk->bearing_cdeg < -ONBOARD_DRIVE_MAX_BEARING_CDEG ||
            mk->bearing_cdeg>ONBOARD_DRIVE_MAX_BEARING_CDEG || (occupant && occupant->marker_id!=mk->id)) continue;
        const map_node_t *known=rover_map_find_id(map,mk->id);
        if (!known || (known->x==x && known->y==y)) return true;
    }
    return false;
}

void onboard_dfs_tick(onboard_dfs_t *d, mission_t *m, rover_map_t *map, onboard_drive_t *drive,
    target_selector_t *s, const onboard_dfs_inputs_t *in, const camera_observation_t *camera, uint32_t now)
{
    if (!d->active) return;
    if (!m->dfs_active || m->state!=MISSION_EXPLORING || m->test_mode) {
        motor_emergency_stop(); d->active=false; d->stage=DFS_STOPPED; d->detail="CANCELLED"; return;
    }
    if (drive->active) return; // The leg executor owns all motion/input checks until arrival or cancellation.
    if (in->drive.safety_latched || !in->drive.link_ok || !in->drive.sensor_valid || !in->drive.imu_ok ||
        (uint32_t)(now-in->drive.sensor_received_ms)>ONBOARD_DRIVE_SENSOR_MAX_AGE_MS) {
        onboard_dfs_stop(d,m,"DFS_INPUT_UNAVAILABLE",now); return;
    }
    if (d->stage==DFS_TRAVELLING) {
        if (m->leg.phase!=LEG_COMPLETE || !onboard_dfs_note_arrival(d,map)) {
            onboard_dfs_stop(d,m,"DFS_ARRIVAL_MISMATCH",now); return;
        }
        begin_node(d,m,drive,s,now);
    }
    if (!map->current_node || d->stack[d->depth-1u]!=map->current_node) {
        onboard_dfs_stop(d,m,"DFS_POSITION_UNKNOWN",now); return;
    }
    if (d->stage==DFS_AT_NODE) {
        d->direction=facing(drive,in->drive.heading_deg);
        if (!aim(d,m,drive,in,DFS_SAMPLING,now)) onboard_dfs_stop(d,m,"DFS_TURN_REFUSED",now);
        return;
    }
    if (d->stage==DFS_TURNING) {
        motor_auto_status_t status=motor_auto_status();
        if ((uint32_t)(now-d->stage_started_ms)>=DFS_TURN_TIMEOUT_MS) {
            onboard_dfs_stop(d,m,"DFS_TURN_FAILED",now); return;
        }
        if (status==MOTOR_AUTO_HEADING_REACHED) {
            if (!in->drive.stopped) return; // Motor ISR may finish after this input snapshot was taken.
            motor_emergency_stop();
            if (d->after_turn==DFS_SAMPLING) sampling(d,m,now);
            else { mission_clear_leg(m,now); d->stage=DFS_WAIT_TARGET; d->stage_started_ms=now; d->ready_ms=now+DFS_SETTLE_MS; }
        } else if (status!=MOTOR_AUTO_RUNNING) {
            onboard_dfs_stop(d,m,"DFS_TURN_FAILED",now);
        } else if ((uint32_t)(now-d->last_renew_ms)>=ONBOARD_DRIVE_REFRESH_MS) {
            if (!motor_auto_renew_lease()) onboard_dfs_stop(d,m,"DFS_TURN_LEASE_EXPIRED",now);
            d->last_renew_ms=now;
        }
        return;
    }
    if (d->stage==DFS_WAIT_TARGET) {
        if ((uint32_t)(now-d->stage_started_ms)>=DFS_TARGET_TIMEOUT_MS) {
            onboard_dfs_stop(d,m,"DFS_TARGET_NOT_VISIBLE",now); return;
        }
        if ((int32_t)(now-d->ready_ms)<0 || s->state!=TARGET_LOCKED || m->leg.phase!=LEG_CONFIGURED) return;
        const char *result=onboard_drive_start(drive,m,map,s,&in->drive,now);
        if (strcmp(result,"STARTED")==0) { d->stage=DFS_TRAVELLING; d->detail=d->backtracking?"BACKTRACKING":"EXPLORING_EXIT"; }
        else if (strncmp(result,"CAMERA_",7)==0 || strcmp(result,"TARGET_NOT_VISIBLE_AT_START")==0) return;
        else onboard_dfs_stop(d,m,result,now);
        return;
    }
    if (d->stage!=DFS_SAMPLING || (int32_t)(now-d->ready_ms)<0) return;
    const map_node_t *node=rover_map_find_id(map,map->current_node);
    if (in->drive.sensor_received_ms!=d->last_sensor_ms && (int32_t)(in->drive.sensor_received_ms-d->ready_ms)>=0) {
        d->last_sensor_ms=in->drive.sensor_received_ms;
        for (unsigned i=0;i<3;++i) {
            unsigned dir=((unsigned)d->direction+(i==0?3u:i==1?0u:1u))%4u;
            uint16_t r=in->tof_mm[i];
            uint8_t code=(in->fault_mask&(1u<<i)) || !r ? MAP_EXIT_UNKNOWN :
                r==1023u || r>=d->open_distance_mm ? MAP_EXIT_OPEN : MAP_EXIT_BLOCKED;
            if (code==MAP_EXIT_UNKNOWN) { d->sample_count[dir]=0; continue; }
            d->sample_count[dir]=d->sample_code[dir]==code ? (uint8_t)(d->sample_count[dir]+1u) : 1u;
            d->sample_code[dir]=code;
            if (d->sample_count[dir]>=2u && node->exits[dir]!=MAP_EXIT_TRAVERSED)
                rover_map_set_exit(map,node->marker_id,(map_direction_t)dir,(map_exit_t)code);
        }
    }
    // Wait a bounded interval even for unknown returns, then inspect another heading.
    if ((uint32_t)(now-d->ready_ms)<DFS_SAMPLE_MS) return;
    d->scanned[d->depth-1u] |= (uint8_t)(1u<<(unsigned)d->direction);
    dfs_choice_t choice=onboard_dfs_choose(d,map,d->direction,camera_hint(camera,map,d->direction,d->ready_ms,now));
    if (choice.kind==DFS_CHOOSE_COMPLETE) {
        motor_emergency_stop(); mission_finish_dfs(m,now); d->active=false; d->depth=0; d->stage=DFS_FINISHED; d->detail="DFS_EXHAUSTED"; return;
    }
    if (choice.kind==DFS_CHOOSE_ERROR || choice.kind==DFS_CHOOSE_UNRESOLVED) {
        onboard_dfs_stop(d,m,choice.kind==DFS_CHOOSE_ERROR?"DFS_STACK_MISMATCH":"DFS_EXITS_UNRESOLVED",now); return;
    }
    d->direction=choice.direction;
    if (choice.kind==DFS_CHOOSE_SCAN) {
        if (!aim(d,m,drive,in,DFS_SAMPLING,now)) onboard_dfs_stop(d,m,"DFS_TURN_REFUSED",now);
        return;
    }
    int16_t x,y;
    if (!cell(node,(unsigned)choice.direction,&x,&y)) { onboard_dfs_stop(d,m,"TARGET_CELL_OVERFLOW",now); return; }
    d->backtracking=choice.kind==DFS_CHOOSE_BACKTRACK; d->expected_id=choice.expected_id; d->from_node=node->marker_id;
    mission_clear_leg(m,now);
    target_selector_set_route(s,(target_route_t){.enabled=true,.expected_id=choice.expected_id,.x=x,.y=y,.direction=(leg_direction_t)choice.direction});
    if (!aim(d,m,drive,in,DFS_WAIT_TARGET,now)) onboard_dfs_stop(d,m,"DFS_TURN_REFUSED",now);
    else d->detail=d->backtracking?"SELECTING_PARENT_TARGET":"SELECTING_NEXT_TARGET";
}

size_t onboard_dfs_format(const onboard_dfs_t *d, const rover_map_t *map, char *out, size_t cap)
{
    if (!out || !cap) return 0;
    static const char *names[]={"IDLE","AT_NODE","TURNING","SCANNING","WAIT_TARGET","TRAVELLING","COMPLETE","STOPPED"};
    int n=snprintf(out,cap,"{\"active\":%s,\"stage\":\"%s\",\"current_node\":%u,\"depth\":%u,\"direction\":\"%s\",\"expected_node\":%u,\"detail\":\"%s\"}",
        d->active?"true":"false",names[d->stage],(unsigned)map->current_node,(unsigned)d->depth,
        travel_leg_direction_name((leg_direction_t)d->direction),(unsigned)d->expected_id,d->detail);
    if (n<0 || (size_t)n>=cap) { out[0]='\0'; return 0; }
    return (size_t)n;
}
