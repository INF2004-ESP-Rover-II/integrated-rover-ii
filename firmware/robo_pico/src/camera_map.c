#include "camera_map.h"
#include <limits.h>
#include <math.h>
#include <string.h>

static const int8_t dx[4]={0,1,0,-1}, dy[4]={1,0,-1,0};
static float wrap(float a) { a=fmodf(a,360.0f); return a<0?a+360.0f:a; }
static float angle_error(float a,float b) { float e=fabsf(wrap(a)-wrap(b)); return e>180?360-e:e; }
void camera_map_init(camera_map_t *m) { memset(m,0,sizeof(*m)); }

static bool position(const rover_map_t *map,uint8_t id,int16_t *x,int16_t *y)
{
    const map_node_t *known=rover_map_find_id(map,id);
    if (known) { *x=known->x; *y=known->y; return true; }
    for (size_t i=0;i<map->observed_count;++i)
        if (map->observed[i].marker_id==id) { *x=map->observed[i].x; *y=map->observed[i].y; return true; }
    return false;
}

void camera_map_tick(camera_map_t *m,rover_map_t *map,const camera_observation_t *ob,
    bool stopped,bool heading_valid,float heading,float north,uint32_t now)
{
    const map_node_t *anchor=rover_map_find_id(map,map->current_node);
    if (!anchor || !anchor->visited || !stopped || !heading_valid || !isfinite(heading) ||
        !isfinite(north) || heading<0 || heading>=360) { camera_map_init(m); return; }
    if (!m->pose_valid || m->anchor_id!=anchor->marker_id || angle_error(heading,m->heading_deg)>3.0f) {
        camera_map_init(m); m->pose_valid=true; m->anchor_id=anchor->marker_id;
        m->heading_deg=heading; m->pose_ms=now; return;
    }
    if (!ob || ob->frame.status!=VISION_STATUS_OK || ob->frame.count>VISION_MAX_MARKERS ||
        (uint32_t)(now-ob->received_ms)>300u ||
        (uint32_t)(now-ob->received_ms)+ob->frame.age_ms>CAMERA_MAP_IMAGE_MAX_AGE_MS) {
        return; // A cached image aging out is not a contradictory new frame.
    }
    if (m->generation_valid && m->generation==ob->generation) return;
    if (m->generation_valid && m->seq==ob->frame.seq) { m->generation=ob->generation; return; }
    if (m->generation_valid && (uint8_t)(ob->frame.seq-m->seq)>=128u) m->confirmations=0;
    m->generation_valid=true; m->generation=ob->generation;
    m->seq=ob->frame.seq;
    // Reject images captured before the node/heading settled, including delayed
    // packets from before a turn or arrival. Only distinct images count.
    if ((int32_t)(ob->received_ms-ob->frame.age_ms-m->pose_ms)<(int32_t)CAMERA_MAP_SETTLE_MS) return;
    camera_marker_t sorted[VISION_MAX_MARKERS]; uint8_t n=0;
    for (uint8_t i=0;i<ob->frame.count;++i) {
        const camera_marker_t *mk=&ob->frame.markers[i];
        if (!mk->id || mk->id==anchor->marker_id || mk->quality || !mk->range_mm ||
            mk->range_mm==UINT16_MAX || mk->bearing_cdeg < -4500 || mk->bearing_cdeg>4500) continue;
        bool duplicate=false;
        for (uint8_t j=0;j<n;++j) if (sorted[j].id==mk->id) duplicate=true;
        if (!duplicate) sorted[n++]=*mk;
    }
    for (uint8_t i=1;i<n;++i) {
        camera_marker_t value=sorted[i]; uint8_t j=i;
        while (j && sorted[j-1u].range_mm>value.range_mm) { sorted[j]=sorted[j-1u]; --j; }
        sorted[j]=value;
    }
    uint32_t slots[4]={0}; map_observed_node_t candidates[VISION_MAX_MARKERS]; uint8_t count=0;
    for (uint8_t i=0;i<n;++i) {
        float world=wrap(heading-north+(float)sorted[i].bearing_cdeg/100.0f);
        unsigned dir=(unsigned)((world+45.0f)/90.0f)%4u;
        // A diagonal sighting cannot reliably identify a cardinal grid cell.
        if (angle_error(world,90.0f*(float)dir)>15.0f) continue;
        int16_t x,y;
        if (position(map,sorted[i].id,&x,&y)) {
            int32_t along=(int32_t)(x-anchor->x)*dx[dir]+(int32_t)(y-anchor->y)*dy[dir];
            int32_t across=(int32_t)(x-anchor->x)*dy[dir]-(int32_t)(y-anchor->y)*dx[dir];
            if (!across && along>0 && (uint32_t)along>slots[dir]) slots[dir]=(uint32_t)along;
            continue;
        }
        ++slots[dir];
        int32_t tx=(int32_t)anchor->x+(int32_t)slots[dir]*dx[dir];
        int32_t ty=(int32_t)anchor->y+(int32_t)slots[dir]*dy[dir];
        if (tx<INT16_MIN || tx>INT16_MAX || ty<INT16_MIN || ty>INT16_MAX) continue;
        candidates[count++]=(map_observed_node_t){.marker_id=sorted[i].id,.x=(int16_t)tx,.y=(int16_t)ty};
    }
    bool same=count==m->count;
    for (uint8_t i=0;same && i<count;++i)
        same=candidates[i].marker_id==m->candidates[i].marker_id && candidates[i].x==m->candidates[i].x && candidates[i].y==m->candidates[i].y;
    if (!same) {
        memcpy(m->candidates,candidates,(size_t)count*sizeof(candidates[0])); m->count=count; m->confirmations=1;
    } else if (m->confirmations<CAMERA_MAP_CONFIRM_FRAMES) ++m->confirmations;
    if (m->confirmations>=CAMERA_MAP_CONFIRM_FRAMES)
        for (uint8_t i=0;i<count;++i)
            rover_map_observe(map,candidates[i].marker_id,candidates[i].x,candidates[i].y);
}
