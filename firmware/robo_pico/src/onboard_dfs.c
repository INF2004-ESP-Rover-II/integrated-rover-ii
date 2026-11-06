#include "onboard_dfs.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define RAD 0.017453292519943295f
enum { LEG_EXPLORE, LEG_BACKTRACK, LEG_LOOP_RETURN, LEG_ROUTE };
static float wrap(float v) { v = fmodf(v, 360); return v < 0 ? v + 360 : v; }
static float error(float a, float b) { return fmodf(a - b + 540, 360) - 180; }
static uint32_t age(uint32_t now, uint32_t then) { return now - then; }
static void phase(onboard_dfs_t *n, onboard_phase_t p, const char *why, uint32_t now) {
    n->clock_ms=now; n->phase = p; n->reason = why; n->state_since = now; ++n->state_revision;
}
static void stop_motor(onboard_dfs_t *n) { if (n->motor.stop) n->motor.stop(n->motor.context); }
bool onboard_dfs_busy(const onboard_dfs_t *n) { return n->phase >= DFS_SCAN && n->phase <= DFS_RECOVERY_SETTLE; }
static void terminal_result(onboard_dfs_t *n, bool arrived, const char *why) {
    if (n->route_mode && !n->result_pending) {
        n->result = (onboard_hop_result_t){n->hop_sequence, n->target, arrived, why};
        n->result_pending = true;
    }
    n->route_mode = false;
    n->leg_active = false;
}
static void fault(onboard_dfs_t *n, const char *why, uint32_t now) {
    stop_motor(n); terminal_result(n, false, why); phase(n, DFS_FAULT, why, now);
}
void onboard_dfs_fault(onboard_dfs_t *n, const char *why, uint32_t now) { fault(n,why,now); }
static bool sensor_fresh(const onboard_dfs_t *n, uint32_t now) {
    return n->sensor_seen && n->sensor.imu_valid && isfinite(n->sensor.heading_deg) &&
           age(now, n->sensor.received_ms) <= n->config.sensor_max_ms;
}
static bool image_fresh(const onboard_dfs_t *n, uint32_t now) {
    return n->image_seen && age(now, n->last_image_ms) <= n->config.image_max_ms;
}
static float target_heading(const onboard_dfs_t *n) { return wrap(n->zero_heading + (float)n->direction * 90); }
static int range(const onboard_dfs_t *n, unsigned index) {
    if (n->sensor.tof_faults & (1u << index)) return -1;
    unsigned r = n->sensor.tof_mm[index];
    return r > 0 && (r <= 1000 || r == 1023) ? (int)r : -1;
}
static bool turn_clear(const onboard_dfs_t *n) {
    /* Sampled clearance only: physical swept-area coverage must be validated. */
    for (unsigned i = 0; i < 3; ++i) if (range(n, i) < n->config.turn_clearance_mm) return false;
    return n->sensor.rear_valid && n->sensor.rear_mm >= n->config.turn_clearance_mm;
}
onboard_config_t onboard_dfs_default_config(void) {
    return (onboard_config_t){600, 0, 0, 0, 100, 20, 500, 200,
                              30, 20, 600, 500, 400, 30000, false};
}
void onboard_dfs_init(onboard_dfs_t *n, const onboard_motor_io_t *io) {
    memset(n, 0, sizeof(*n));
    n->config = onboard_dfs_default_config(); n->motor = *io;
    n->phase = DFS_IDLE; n->reason = "SET_START_REQUIRED";
    rover_map_init(&n->map, 0);
}
void onboard_dfs_sensor(onboard_dfs_t *n, const onboard_sensor_t *s) {
    n->sensor = *s; n->sensor_seen = true;
}
bool onboard_dfs_set_geometry(onboard_dfs_t *n, float height, float forward, float down, uint32_t now) {
    if (onboard_dfs_busy(n) || n->map.start || !isfinite(height) || !isfinite(forward) || !isfinite(down) ||
        height <= 0 || height > 2000 || forward < -1000 || forward > 1000 || down < -45 || down > 75) return false;
    n->config.camera_height_mm=height; n->config.camera_forward_mm=forward; n->config.camera_down_deg=down;
    n->config.camera_calibrated=true;
    phase(n,n->phase,"CAMERA_GEOMETRY_SET",now); return true;
}
bool onboard_dfs_set_start(onboard_dfs_t *n, uint8_t id, uint32_t now) {
    if (onboard_dfs_busy(n) || !id || !sensor_fresh(n, now)) return false;
    stop_motor(n); rover_map_init(&n->map, id);
    n->zero_heading = wrap(n->sensor.heading_deg);
    n->pose_x = n->pose_y = 0; n->pose_valid = n->current_confirmed = true;
    n->last_pose_ms = now; n->depth = 1; n->stack[0] = id; n->goal = ROVER_DEFAULT_GOAL;
    n->loop_return = 0; n->route_mode = n->result_pending = n->leg_active = n->wall_valid = false;
    memset(n->candidate_count, 0, sizeof(n->candidate_count));
    phase(n, DFS_READY, "START_ANCHORED", now); return true;
}
static void scan_start(onboard_dfs_t *n, uint32_t now) {
    memset(n->scan_count, 0, sizeof(n->scan_count));
    n->last_scan_sensor_ms = n->sensor.received_ms;
    phase(n, DFS_SCAN, "SCANNING_EXITS", now);
}
static void hold(onboard_dfs_t *,const char *,uint32_t);
static bool turn_to(onboard_dfs_t *,float,uint32_t,bool);
bool onboard_dfs_start(onboard_dfs_t *n, uint8_t goal, uint16_t opening, uint32_t now) {
    if (!n->config.camera_calibrated || n->phase != DFS_READY || !n->current_confirmed || !sensor_fresh(n, now) ||
        !image_fresh(n, now) || goal < n->map.visited_count || goal > ROVER_MAX_NODES || opening < 200 || opening > 1000) return false;
    n->goal = goal; n->config.opening_mm = opening;
    n->direction = (unsigned)lroundf(wrap(n->sensor.heading_deg - n->zero_heading) / 90) % 4u;
    if (fabsf(error(target_heading(n), n->sensor.heading_deg)) > 5) return false;
    n->last_pose_ms = now; n->recovery_used = false;
    if (n->map.visited_count == goal) phase(n, DFS_COMPLETE, "DFS_COMPLETE", now);
    else scan_start(n, now);
    return true;
}
void onboard_dfs_stop(onboard_dfs_t *n, const char *why, uint32_t now) {
    stop_motor(n); terminal_result(n, false, why);
    if (n->map.start) phase(n, DFS_PAUSED, why, now);
}
bool onboard_dfs_resume(onboard_dfs_t *n, uint32_t now) {
    if (!n->config.camera_calibrated || n->phase != DFS_PAUSED || !sensor_fresh(n, now) || !image_fresh(n, now)) return false;
    if(n->current_confirmed) {
        if(n->map.visited_count>=n->goal)phase(n,DFS_COMPLETE,"DFS_COMPLETE",now);
        else {
            n->direction=(unsigned)lroundf(wrap(n->sensor.heading_deg-n->zero_heading)/90)%4;
            if(fabsf(error(target_heading(n),n->sensor.heading_deg))>5) {
                n->inspection=true;turn_to(n,target_heading(n),now,false);
            } else scan_start(n,now);
        }
    } else {
        if(!n->source||!n->target)return false;
        n->leg_active=true;n->leg_since=now;n->recovery_used=false;
        hold(n,"RESUMING_POSITION_CHECK",now);
    }
    return true;
}

/* Floor-plane solution from lens range, optical x/z bearing and mounting pitch.
 * Camera roll/height changes are outside this initial calibrated flat-floor model. */
static bool displacement(const onboard_dfs_t *n, const vision_marker_t *m, float heading,
                          float *dx, float *dy) {
    if (m->quality != VISION_QUALITY_OK) return false;
    float beta = (float)m->bearing_cdeg * 0.01f;
    if (fabsf(beta) > n->config.bearing_limit_deg || m->range_mm <= n->config.camera_height_mm) return false;
    float alpha = n->config.camera_down_deg * RAD;
    float c = cosf(alpha), s = sinf(alpha), t = tanf(beta * RAD);
    if (c < 0.2f) return false;
    float h = n->config.camera_height_mm, r = m->range_mm;
    float a = 1 + t*t + s*s/(c*c), b = -2*h*s/(c*c), d = h*h/(c*c)-r*r;
    float discriminant = b*b-4*a*d;
    if (discriminant < 0) return false;
    float z = (-b + sqrtf(discriminant))/(2*a), y = (h-z*s)/c;
    float forward = z*c-y*s+n->config.camera_forward_mm, right = z*t;
    if (z <= 0 || forward <= 0) return false;
    float angle = heading*RAD;
    *dx = right*cosf(angle) + forward*sinf(angle);
    *dy = forward*cosf(angle) - right*sinf(angle);
    return isfinite(*dx) && isfinite(*dy);
}

void onboard_dfs_camera(onboard_dfs_t *n, const vision_frame_t *frame, uint32_t rx, uint32_t now) {
    if(frame->count>VISION_MAX_MARKERS)return;
    for(unsigned i=0;i<frame->count;++i)if(frame->markers[i].id<1)return;
    n->clock_ms=now; n->last_camera_rx_ms=rx; n->last_camera=*frame; n->camera_received=true;
    uint32_t effective = vision_effective_age_ms(frame, rx, now);
    if (!vision_frame_is_image(frame)) { n->image_seen = false; return; }
    if (effective > n->config.image_max_ms) return;
    uint8_t delta = (uint8_t)(frame->sequence - n->last_sequence);
    if (n->sequence_seen && (delta == 0 || (age(now,n->last_image_ms)<1000u && delta>=128u))) return;
    n->sequence_seen = n->image_seen = true; n->last_sequence = frame->sequence;
    n->last_image_ms = now-effective;
    if (!n->config.camera_calibrated || !sensor_fresh(n, now) || !n->map.start || n->phase == DFS_TURN || n->phase == DFS_RECOVERY_TURN ||
        n->phase == DFS_SETTLE || n->phase == DFS_RECOVERY_SETTLE) return;
    if ((n->phase == DFS_ACQUIRE || n->phase == DFS_SCAN) && (int32_t)(n->last_image_ms-n->state_since)<0) return;
    if ((n->phase == DFS_CONFIRM || n->phase == DFS_HOLD) &&
        (age(now, n->state_since) < n->config.settle_ms || (int32_t)(n->last_image_ms-n->state_since) < 0)) return;
    float heading = wrap(n->sensor.heading_deg-n->zero_heading);
    float fixx = 0, fixy = 0; unsigned fixes = 0;
    for (unsigned i = 0; i < frame->count; ++i) {
        int x, y; float dx, dy;
        if (!rover_map_find(&n->map, frame->markers[i].id, &x, &y) ||
            !displacement(n, &frame->markers[i], heading, &dx, &dy)) continue;
        float px = (float)x*n->config.pitch_mm-dx, py = (float)y*n->config.pitch_mm-dy;
        if (fixes && hypotf(px-fixx/(float)fixes, py-fixy/(float)fixes) > 150) {
            if (onboard_dfs_busy(n)) fault(n, "LANDMARKS_DISAGREE", now);
            return;
        }
        fixx += px; fixy += py; ++fixes;
    }
    if (fixes) {
        fixx /= (float)fixes; fixy /= (float)fixes;
        if (n->current_confirmed) {
            int x, y; rover_map_find(&n->map, n->map.current, &x, &y);
            if (hypotf(fixx-(float)x*n->config.pitch_mm, fixy-(float)y*n->config.pitch_mm) > 150) {
                if (onboard_dfs_busy(n)) fault(n, "START_OR_NODE_POSE_CONFLICT", now);
                return;
            }
        }
        n->pose_x = fixx; n->pose_y = fixy; n->pose_valid = true; n->last_pose_ms = n->last_image_ms;
    }
    /* Pin unknown landmarks only from a known stationary node or a fresh fix. */
    if (n->current_confirmed || (fixes && n->phase != DFS_HOLD)) {
        bool seen[ROVER_MAX_MARKER_ID + 1u] = {false};
        for (unsigned i = 0; i < frame->count; ++i) {
            const vision_marker_t *m = &frame->markers[i]; float dx, dy;
            if (rover_map_find(&n->map, m->id, NULL, NULL) || !displacement(n, m, heading, &dx, &dy)) continue;
            float wx = (n->pose_x+dx)/n->config.pitch_mm, wy = (n->pose_y+dy)/n->config.pitch_mm;
            int x = (int)lroundf(wx), y = (int)lroundf(wy);
            if (fabsf(wx-(float)x)*n->config.pitch_mm > 120 || fabsf(wy-(float)y)*n->config.pitch_mm > 120) continue;
            if (x < -ROVER_COORD_LIMIT || x > ROVER_COORD_LIMIT || y < -ROVER_COORD_LIMIT || y > ROVER_COORD_LIMIT) {
                if (onboard_dfs_busy(n)) fault(n, "MAP_COORDINATE_LIMIT", now);
                return;
            }
            seen[m->id] = true;
            if (n->candidate_x[m->id] == x && n->candidate_y[m->id] == y) {
                if (n->candidate_count[m->id] < 3) ++n->candidate_count[m->id];
            } else { n->candidate_x[m->id]=(int16_t)x; n->candidate_y[m->id]=(int16_t)y; n->candidate_count[m->id]=1; }
            if (n->candidate_count[m->id] >= 3 && !rover_map_place(&n->map, m->id, x, y)) {
                if (onboard_dfs_busy(n)) fault(n, n->map.node_count >= ROVER_MAX_NODES ? "MAP_CAPACITY_REACHED" : "MAP_CONFLICT", now);
                return;
            }
        }
        for (unsigned id=1; id<=ROVER_MAX_MARKER_ID; ++id) if (!seen[id]) n->candidate_count[id]=0;
    }
    if (!fixes) { n->arrival_count = 0; n->recovery_confirmations = 0; return; }
    if (n->phase == DFS_CONFIRM) {
        if (hypotf(n->pose_x-(float)n->target_x*n->config.pitch_mm,
                   n->pose_y-(float)n->target_y*n->config.pitch_mm) <= n->config.position_tolerance_mm &&
            fabsf(error(target_heading(n), n->sensor.heading_deg)) <= 5) {
            if (n->arrival_count < 3) ++n->arrival_count;
        } else n->arrival_count = 0;
    }
    if (n->phase == DFS_HOLD && n->recovery_confirmations < 3) ++n->recovery_confirmations;
}

static void hold(onboard_dfs_t *n, const char *why, uint32_t now) {
    stop_motor(n);
    if(!n->current_confirmed)n->pose_valid=false;
    if (n->phase != DFS_HOLD && n->phase != DFS_RECOVERY_TURN && n->phase != DFS_RECOVERY_SETTLE && !n->recovery_used) {
        n->hold_since = now; n->recovery_heading = n->sensor.heading_deg; n->recovery_angle = 0;
    }
    n->recovery_confirmations = 0;
    phase(n, DFS_HOLD, why, now);
}
static bool turn_to(onboard_dfs_t *n, float heading, uint32_t now, bool recovering) {
    if (fabsf(error(heading, n->sensor.heading_deg)) <= 5) {
        stop_motor(n); phase(n, recovering ? DFS_RECOVERY_SETTLE : DFS_SETTLE, "SETTLING", now); return true;
    }
    if (!turn_clear(n)) { fault(n, "TURN_CLEARANCE_UNAVAILABLE", now); return false; }
    if (!n->motor.turn || !n->motor.turn(n->motor.context, (int)lroundf(wrap(heading))%360)) {
        fault(n, "MOTOR_TURN_REFUSED", now); return false;
    }
    phase(n, recovering ? DFS_RECOVERY_TURN : DFS_TURN, recovering ? "RECOVERY_SCAN" : "TURNING", now);
    return true;
}
static void begin_leg(onboard_dfs_t *n, unsigned direction, uint8_t kind, uint32_t now) {
    int x, y; rover_map_find(&n->map, n->map.current, &x, &y);
    n->source = n->map.current; n->direction = direction;
    n->target_x = x + rover_dx[direction]; n->target_y = y + rover_dy[direction];
    const rover_cell_t *target = rover_map_at(&n->map, n->target_x, n->target_y);
    n->target = target ? target->marker_id : 0; n->leg_kind = kind;
    n->leg_since = now; n->inspection = false; n->recovery_used = false; n->arrival_count = 0;
    n->leg_active = true; n->wall_valid = false; n->wall_confirmations = 0;
    turn_to(n, target_heading(n), now, false);
}
static void commit(onboard_dfs_t *n, uint32_t now) {
    const rover_cell_t *target = rover_map_at(&n->map, n->target_x, n->target_y);
    if (!target || !target->marker_id || (n->target && n->target != target->marker_id)) { fault(n, "UNCONFIRMED_TARGET", now); return; }
    bool visited = target->visited; uint8_t id = target->marker_id;
    if (!rover_map_connect(&n->map, n->source, id, n->direction)) { fault(n, "ARRIVAL_TOPOLOGY_CONFLICT", now); return; }
    rover_map_visit(&n->map, id); n->current_confirmed = true; n->leg_active = false;
    n->pose_x = (float)n->target_x*n->config.pitch_mm; n->pose_y = (float)n->target_y*n->config.pitch_mm;
    if (n->route_mode) {
        terminal_result(n, true, "ARRIVED"); phase(n, DFS_COMPLETE, "HOP_ARRIVED", now); return;
    }
    if (!visited) {
        if (n->depth >= ROVER_MAX_NODES) { fault(n, "STACK_OVERFLOW", now); return; }
        n->stack[n->depth++] = id;
    } else if (n->leg_kind == LEG_BACKTRACK) {
        if (n->depth < 2 || n->stack[n->depth-2] != id) { fault(n, "BACKTRACK_CONFLICT", now); return; }
        --n->depth;
    } else if (n->leg_kind == LEG_LOOP_RETURN) n->loop_return = 0;
    else n->loop_return = n->source;
    if (n->map.visited_count >= n->goal) { stop_motor(n); phase(n, DFS_COMPLETE, "DFS_COMPLETE", now); }
    else scan_start(n, now);
}
static bool direction_to(const onboard_dfs_t *n, uint8_t id, unsigned *dir) {
    int x,y,tx,ty;
    if (!rover_map_find(&n->map,n->map.current,&x,&y) || !rover_map_find(&n->map,id,&tx,&ty)) return false;
    for (unsigned d=0; d<4; ++d) if (tx==x+rover_dx[d] && ty==y+rover_dy[d]) { *dir=d; return true; }
    return false;
}

void onboard_dfs_tick(onboard_dfs_t *n, uint32_t now) {
    n->clock_ms=now;
    if (!onboard_dfs_busy(n)) return;
    if (!sensor_fresh(n,now)) { fault(n,"SENSOR_OR_HEADING_STALE",now); return; }
    if (n->leg_active && age(now,n->leg_since)>n->config.leg_timeout_ms) {fault(n,"SEGMENT_TIMEOUT",now);return;}
    if (!image_fresh(n,now) && n->phase != DFS_HOLD) hold(n,"CAMERA_UNAVAILABLE",now);
    if (n->phase == DFS_HOLD) {
        if (!image_fresh(n,now)) {
            if (age(now,n->last_image_ms)>=5000) fault(n,"CAMERA_TIMEOUT",now);
            return;
        }
        if (n->recovery_confirmations>=3) {
            if (n->current_confirmed) {
                n->direction=(unsigned)lroundf(wrap(n->sensor.heading_deg-n->zero_heading)/90)%4;
                n->inspection=true;turn_to(n,target_heading(n),now,false);return;
            }
            if (hypotf(n->pose_x-(float)n->target_x*n->config.pitch_mm,n->pose_y-(float)n->target_y*n->config.pitch_mm) <= n->config.position_tolerance_mm) {
                n->arrival_count=0; phase(n,DFS_CONFIRM,"CONFIRMING_RECOVERED_POSITION",now); return;
            }
            turn_to(n,target_heading(n),now,false); return;
        }
        if (age(now,n->hold_since)>=15000 || n->recovery_angle>=180) { fault(n,"REFERENCE_RECOVERY_EXHAUSTED",now); return; }
        if (age(now,n->state_since)<1800 || !turn_clear(n)) return;
        n->recovery_angle += 30; n->recovery_used=true;
        turn_to(n, n->recovery_heading+(float)n->recovery_angle,now,true); return;
    }
    if (n->phase==DFS_RECOVERY_TURN || n->phase==DFS_TURN) {
        if (age(now,n->state_since)>5000) { fault(n,"TURN_TIMEOUT",now); return; }
        if (!turn_clear(n)) { fault(n,"TURN_CLEARANCE_LOST",now); return; }
        if (n->motor.turn_done && n->motor.turn_done(n->motor.context)) {
            bool recovery=n->phase==DFS_RECOVERY_TURN;
            stop_motor(n); phase(n,recovery?DFS_RECOVERY_SETTLE:DFS_SETTLE,"SETTLING",now);
        }
        return;
    }
    if (n->phase==DFS_RECOVERY_SETTLE) {
        if (age(now,n->state_since)>=n->config.settle_ms) phase(n,DFS_HOLD,"REOBSERVING_AFTER_SCAN",now);
        return;
    }
    if (n->phase==DFS_SETTLE) {
        if (age(now,n->state_since)<n->config.settle_ms || (int32_t)(n->sensor.received_ms-n->state_since)<0) return;
        if (fabsf(error(target_heading(n),n->sensor.heading_deg))>5) { fault(n,"TURN_HEADING_DRIFT",now); return; }
        if (n->inspection) { n->inspection=false; scan_start(n,now); }
        else phase(n,DFS_ACQUIRE,"ACQUIRING_REFERENCE",now);
        return;
    }
    if (n->phase==DFS_SCAN) {
        if (age(now,n->state_since)>10000) { fault(n,"EXIT_SCAN_TIMEOUT",now); return; }
        if (fabsf(error(target_heading(n),n->sensor.heading_deg))>5) { fault(n,"SCAN_HEADING_INVALID",now); return; }
        if (n->sensor.received_ms==n->last_scan_sensor_ms) return;
        n->last_scan_sensor_ms=n->sensor.received_ms;
        const unsigned directions[]={ (n->direction+3)%4,n->direction,(n->direction+1)%4 };
        for (unsigned i=0;i<3;++i) {
            int r=range(n,i); if(r<0) {n->scan_count[directions[i]]=0; continue;}
            unsigned d=directions[i]; uint8_t value=r>=n->config.opening_mm?MAP_OPEN:MAP_BLOCKED;
            if (n->scan_value[d]!=value) {n->scan_value[d]=value;n->scan_count[d]=0;}
            if(n->scan_count[d]<2) ++n->scan_count[d];
            if(n->scan_count[d]>=2) rover_map_exit(&n->map,n->map.current,d,value);
        }
        for(unsigned i=0;i<3;++i) if(n->scan_count[directions[i]]<2) return;
        unsigned d; const rover_cell_t *cell=rover_map_find(&n->map,n->map.current,NULL,NULL);
        if(n->loop_return) {
            if(!direction_to(n,n->loop_return,&d)||cell->exits[d]==MAP_BLOCKED) {fault(n,"LOOP_RETURN_BLOCKED",now);return;}
            begin_leg(n,d,LEG_LOOP_RETURN,now);return;
        }
        const unsigned order[]={n->direction,(n->direction+3)%4,(n->direction+1)%4,(n->direction+2)%4};
        for(unsigned i=0;i<4;++i) if(cell->exits[order[i]]==MAP_OPEN) {begin_leg(n,order[i],LEG_EXPLORE,now);return;}
        for(unsigned i=0;i<4;++i) if(cell->exits[order[i]]==MAP_UNCHECKED) {
            n->direction=order[i];n->inspection=true;turn_to(n,target_heading(n),now,false);return;
        }
        if(n->depth<2) {fault(n,"EXPLORATION_EXHAUSTED",now);return;}
        if(!direction_to(n,n->stack[n->depth-2],&d)||cell->exits[d]==MAP_BLOCKED) {fault(n,"BACKTRACK_BLOCKED",now);return;}
        begin_leg(n,d,LEG_BACKTRACK,now);return;
    }
    if (n->phase==DFS_ACQUIRE) {
        const rover_cell_t *target=rover_map_at(&n->map,n->target_x,n->target_y);
        if(!target||!target->marker_id||!n->pose_valid||age(now,n->last_pose_ms)>n->config.image_max_ms||
           (int32_t)(n->last_pose_ms-n->state_since)<0) {
            if(age(now,n->state_since)>1500u) hold(n,"REFERENCE_REQUIRED",now);
            return;
        }
        n->target=target->marker_id;
        if(range(n,1)<150) {fault(n,"FRONT_NOT_CLEAR",now);return;}
        /* Known stationary source + measured static wall beyond next cell.
         * Only this explicitly anchored wall can bridge loss of the floor tag. */
        if (n->current_confirmed && range(n,1)>=750 && range(n,1)<=1000) {
            n->wall_coordinate=n->pose_x*rover_dx[n->direction]+n->pose_y*rover_dy[n->direction]+(float)range(n,1);
            n->wall_valid=true;n->last_wall_sensor_ms=n->sensor.received_ms;
        }
        if(!n->motor.drive||!n->motor.drive(n->motor.context,n->config.cruise_percent)) {fault(n,"MOTOR_DRIVE_REFUSED",now);return;}
        n->current_confirmed=false;n->last_motor_ms=now; phase(n,DFS_DRIVE,"DRIVING",now);return;
    }
    if ((n->phase==DFS_DRIVE || n->phase==DFS_CONFIRM) && n->wall_valid &&
        n->sensor.received_ms!=n->last_wall_sensor_ms && range(n,1)>0 && range(n,1)<=1000) {
        n->last_wall_sensor_ms=n->sensor.received_ms;
        float along=n->wall_coordinate-(float)range(n,1);
        float projected=n->pose_x*rover_dx[n->direction]+n->pose_y*rover_dy[n->direction];
        if (fabsf(error(target_heading(n),n->sensor.heading_deg))>5 ||
            (age(now,n->last_pose_ms)<=200 && fabsf(along-projected)>150)) {
            fault(n,"WALL_REFERENCE_CONFLICT",now);return;
        }
        int sx,sy;rover_map_find(&n->map,n->source,&sx,&sy);
        float source_along=((float)sx*rover_dx[n->direction]+(float)sy*rover_dy[n->direction])*n->config.pitch_mm;
        if(along<source_along-100||along>source_along+n->config.pitch_mm+100) {fault(n,"WALL_POSE_OUTSIDE_LEG",now);return;}
        n->pose_x=(float)sx*n->config.pitch_mm+(along-source_along)*rover_dx[n->direction];
        n->pose_y=(float)sy*n->config.pitch_mm+(along-source_along)*rover_dy[n->direction];
        n->pose_valid=true;n->last_pose_ms=n->sensor.received_ms;
        if(n->phase==DFS_CONFIRM && age(now,n->state_since)>=n->config.settle_ms &&
           hypotf(n->pose_x-(float)n->target_x*n->config.pitch_mm,n->pose_y-(float)n->target_y*n->config.pitch_mm)<=n->config.position_tolerance_mm) {
            if(n->wall_confirmations<3)++n->wall_confirmations;
        } else n->wall_confirmations=0;
    }
    if(n->phase==DFS_DRIVE) {
        if(range(n,1)<0) {fault(n,"FRONT_SENSOR_INVALID",now);return;}
        if(!n->pose_valid||age(now,n->last_pose_ms)>n->config.image_max_ms) {hold(n,"POSITION_REFERENCE_LOST",now);return;}
        float dx=(float)n->target_x*n->config.pitch_mm-n->pose_x,dy=(float)n->target_y*n->config.pitch_mm-n->pose_y;
        float remaining=dx*rover_dx[n->direction]+dy*rover_dy[n->direction];
        float lateral=fabsf(dx*rover_dy[n->direction]-dy*rover_dx[n->direction]);
        if(lateral>150||remaining < -n->config.position_tolerance_mm) {fault(n,"POSITION_OFF_SEGMENT",now);return;}
        if(range(n,1)<150) {fault(n,"OBSTACLE_DURING_SEGMENT",now);return;}
        float radial_room=sqrtf(fmaxf(0,n->config.position_tolerance_mm*n->config.position_tolerance_mm-lateral*lateral));
        /* Aim near the node centre rather than chaining tolerance-edge turns.
         * The stopped confirmation still enforces the full arrival tolerance. */
        float stop_along=fminf(20,radial_room*0.8f);
        if(remaining<=stop_along) {
            stop_motor(n);n->arrival_count=0;n->wall_confirmations=0;phase(n,DFS_CONFIRM,"CONFIRMING_ARRIVAL",now);return;
        }
        if(age(now,n->last_motor_ms)>=100) {
            int speed=remaining<250?n->config.approach_percent:n->config.cruise_percent;
            if(!n->motor.drive(n->motor.context,speed)) {fault(n,"MOTOR_DRIVE_REFUSED",now);return;}
            n->last_motor_ms=now;
        }
    } else if(n->phase==DFS_CONFIRM) {
        if(n->arrival_count>=3 || n->wall_confirmations>=3) commit(n,now);
        else if(age(now,n->state_since)>5000) hold(n,"ARRIVAL_NOT_CONFIRMED",now);
    }
}

const rover_map_t *onboard_dfs_map(const onboard_dfs_t *n) {return &n->map;}
bool onboard_dfs_begin_hop(onboard_dfs_t *n,uint8_t target,unsigned d,uint32_t seq,uint32_t now) {
    if(!n->config.camera_calibrated||n->phase!=DFS_COMPLETE||n->result_pending||!seq||seq<=n->last_hop_sequence||!n->current_confirmed||
       !sensor_fresh(n,now)||!image_fresh(n,now)) return false;
    unsigned actual; const rover_cell_t *cell=rover_map_find(&n->map,n->map.current,NULL,NULL);
    if(d>3||!direction_to(n,target,&actual)||actual!=d||cell->exits[d]!=MAP_EXPLORED) return false;
    n->route_mode=true;n->hop_sequence=n->last_hop_sequence=seq;
    begin_leg(n,d,LEG_ROUTE,now);return n->phase!=DFS_FAULT;
}
bool onboard_dfs_take_result(onboard_dfs_t *n,onboard_hop_result_t *out) {
    if(!n->result_pending)return false;
    *out=n->result;n->result_pending=false;return true;
}
const char *onboard_dfs_phase_name(onboard_phase_t p) {
    static const char *names[]={"IDLE","READY","SCAN","TURN","SETTLE","ACQUIRE","DRIVE","CONFIRM","HOLD","RECOVERY_TURN","RECOVERY_SETTLE","COMPLETE","PAUSED","FAULT"};
    return (unsigned)p<sizeof(names)/sizeof(names[0])?names[p]:"INVALID";
}
size_t onboard_dfs_snapshot(const onboard_dfs_t *n,char *out,size_t size) {
    size_t used=0;
#define ADD(...) do {int written=snprintf(out+used,size>used?size-used:0,__VA_ARGS__);if(written<0||(size_t)written>=size-used)return 0;used+=(size_t)written;}while(0)
    if(!size)return 0;
    ADD("{\"schema\":1,\"revision\":%lu,\"state_revision\":%lu,\"state\":\"%s\",\"reason\":\"%s\",\"n\":%u,\"count\":%u,\"start\":%u,\"current\":%u,\"confirmed\":%s,\"target\":%u,\"route\":[],\"pose\":{\"x_mm\":%.0f,\"y_mm\":%.0f,\"valid\":%s},\"markers\":[",
        (unsigned long)n->map.revision,(unsigned long)n->state_revision,onboard_dfs_phase_name(n->phase),n->reason,
        n->goal,n->map.visited_count,n->map.start,n->map.current,n->current_confirmed?"true":"false",n->target,
        (double)n->pose_x,(double)n->pose_y,n->pose_valid?"true":"false");
    bool comma=false;
    for(unsigned i=0;i<n->map.node_count;++i){
        const rover_node_t *c=&n->map.nodes[i];
        ADD("%s{\"id\":%u,\"x\":%d,\"y\":%d,\"visited\":%s,\"junction\":true,\"exits\":[%u,%u,%u,%u]}",comma?",":"",c->marker_id,c->x,c->y,c->visited?"true":"false",c->exits[0],c->exits[1],c->exits[2],c->exits[3]);comma=true;
    }
    ADD("],\"edges\":[");comma=false;
    for(unsigned i=0;i<n->map.node_count;++i){
        const rover_node_t *c=&n->map.nodes[i];
        for(unsigned d=0;d<2;++d){const rover_cell_t *b=rover_map_at(&n->map,c->x+rover_dx[d],c->y+rover_dy[d]);
            if(!b||!b->marker_id||c->exits[d]!=MAP_EXPLORED)continue;
            ADD("%s[%u,%u,false]",comma?",":"",c->marker_id,b->marker_id);comma=true;
        }
    }
    ADD("],\"capacity\":%u,\"id_max\":%u,\"pitch_mm\":%.0f,\"bounds\":{\"min_x\":%d,\"max_x\":%d,\"min_y\":%d,\"max_y\":%d,\"rows\":%u,\"columns\":%u}",
        ROVER_MAX_NODES,(unsigned)ROVER_MAX_MARKER_ID,(double)n->config.pitch_mm,n->map.min_x,n->map.max_x,
        n->map.min_y,n->map.max_y,rover_map_rows(&n->map),rover_map_columns(&n->map));
    ADD(",\"geometry_ready\":%s,\"geometry\":{\"height_mm\":%.1f,\"forward_mm\":%.1f,\"down_deg\":%.1f},\"camera\":{\"received\":%s,\"sequence\":%u,\"status\":%u,\"reported_age_ms\":%u,\"received_age_ms\":%lu,\"effective_age_ms\":%lu,\"markers\":[",
        n->config.camera_calibrated?"true":"false",(double)n->config.camera_height_mm,
        (double)n->config.camera_forward_mm,(double)n->config.camera_down_deg,n->camera_received?"true":"false",
        n->last_camera.sequence,(unsigned)n->last_camera.status,n->last_camera.age_ms,
        (unsigned long)(n->camera_received?age(n->clock_ms,n->last_camera_rx_ms):0),
        (unsigned long)(n->camera_received?vision_effective_age_ms(&n->last_camera,n->last_camera_rx_ms,n->clock_ms):0));
    for(unsigned i=0;i<n->last_camera.count;++i) {
        const vision_marker_t *m=&n->last_camera.markers[i];
        ADD("%s{\"id\":%u,\"range_mm\":%u,\"bearing_cdeg\":%d,\"quality\":%u}",i?",":"",m->id,m->range_mm,m->bearing_cdeg,m->quality);
    }
    ADD("]}}");return used;
#undef ADD
}
