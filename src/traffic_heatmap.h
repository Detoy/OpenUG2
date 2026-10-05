/* Read-only traffic diagnostics. No routing, random numbers or physics inputs. */
#ifndef OPENUG2_TRAFFIC_HEATMAP_H
#define OPENUG2_TRAFFIC_HEATMAP_H
#include <math.h>
#include <string.h>

enum { HEAT_ACTORS=32, HEAT_CONTACTS=256, HEAT_WORLD=0, HEAT_VEHICLE=1 };
typedef struct {
    int from,to,layer,racer,count,stationary;
    float a[2],b[2],z,speed;
} HeatRoad;
typedef struct {
    int used,x,y,z,kind;
    unsigned corrections;
    float pos[3],heat,peak;
    double last;
} HeatContact;
typedef struct {
    int valid,seen;
    unsigned generation;
    float pos[3],stationary;
} HeatActor;
typedef struct {
    HeatRoad roads[HEAT_ACTORS]; int nroads;
    HeatContact contacts[HEAT_CONTACTS];
    HeatActor actors[HEAT_ACTORS];
    double seconds;
    unsigned evictions;
} TrafficHeatmap;

static inline void heatmap_clear(TrafficHeatmap *h) { memset(h,0,sizeof *h); }
static inline int heatmap_finite(const float p[3]) {
    return isfinite(p[0]) && isfinite(p[1]) && isfinite(p[2]) &&
           fabsf(p[0])<1e7f && fabsf(p[1])<1e7f && fabsf(p[2])<1e7f;
}
/* Call at 4 Hz simulation time. Snapshot occupancy; decay correction heat
 * with a 10 s time constant. Pausing collection also pauses this clock. */
static inline void heatmap_begin(TrafficHeatmap *h,float dt) {
    if(!isfinite(dt) || dt<=0)return;
    h->seconds+=dt;h->nroads=0;
    float decay=expf(-dt/10.0f);
    for(int i=0;i<HEAT_ACTORS;i++)h->actors[i].seen=0;
    for(int i=0;i<HEAT_CONTACTS;i++)if(h->contacts[i].used) {
        h->contacts[i].heat*=decay;
        if(h->seconds-h->contacts[i].last>60)h->contacts[i].used=0;
    }
}
/* Directed segment IDs are not merged with their reverse. Height bands keep
 * stacked paths separate; this is a diagnostic filter, not a road-layer parser. */
static inline void heatmap_car(TrafficHeatmap *h,int slot,unsigned generation,
        int from,int to,const float a[2],const float b[2],const float pos[3],
        float speed,int racer,float dt) {
    if(slot<0 || slot>=HEAT_ACTORS || !heatmap_finite(pos) ||
       !isfinite(speed) || !isfinite(dt) || dt<=0)return;
    HeatActor *actor=&h->actors[slot];
    if(actor->valid && actor->generation==generation &&
       hypotf(pos[0]-actor->pos[0],pos[1]-actor->pos[1])<.5f*dt &&
       fabsf(pos[2]-actor->pos[2])<.5f && fabsf(speed)<.5f)
        actor->stationary+=dt;
    else actor->stationary=0;
    memcpy(actor->pos,pos,sizeof actor->pos);
    actor->valid=actor->seen=1;actor->generation=generation;
    int layer=(int)floorf(pos[2]/4.0f),i;
    /* Stopped routes without an edge remain visible as individual points. */
    if(from<0 || to<0){from=-1-slot;to=-1-slot;a=pos;b=pos;}
    if(!isfinite(a[0]) || !isfinite(a[1]) || !isfinite(b[0]) || !isfinite(b[1]))return;
    for(i=0;i<h->nroads;i++) {
        HeatRoad *r=&h->roads[i];
        if(r->from==from && r->to==to && r->layer==layer && r->racer==racer)break;
    }
    if(i==h->nroads) {
        if(h->nroads==HEAT_ACTORS)return;
        HeatRoad *r=&h->roads[h->nroads++];memset(r,0,sizeof *r);
        r->from=from;r->to=to;r->layer=layer;r->racer=racer;r->z=pos[2];
        memcpy(r->a,a,sizeof r->a);memcpy(r->b,b,sizeof r->b);
    }
    HeatRoad *r=&h->roads[i];r->count++;
    r->speed+=(fabsf(speed)-r->speed)/r->count;
    r->stationary+=actor->stationary>=3.0f;
}
static inline void heatmap_end(TrafficHeatmap *h) {
    for(int i=0;i<HEAT_ACTORS;i++)if(!h->actors[i].seen)h->actors[i].valid=0;
}
/* Vehicle-position bins, NOT exact surface contact points. Count solver
 * corrections, not crashes: one impact can generate many solver iterations.
 * ponytail: bounded linear lookup; spatial hashing if profiling warrants it. */
static inline void heatmap_contact(TrafficHeatmap *h,const float before[3],
                                    const float after[3],int kind) {
    if(!heatmap_finite(before) || !heatmap_finite(after) || kind<0 || kind>1)return;
    float distance=hypotf(after[0]-before[0],after[1]-before[1]);
    if(distance<1e-6f)return;
    int x=(int)floorf(before[0]/8),y=(int)floorf(before[1]/8),z=(int)floorf(before[2]/4);
    int slot=-1,oldest=0;
    for(int i=0;i<HEAT_CONTACTS;i++) {
        HeatContact *c=&h->contacts[i];
        if(c->used && c->x==x && c->y==y && c->z==z && c->kind==kind){slot=i;break;}
        if(!c->used && slot<0)slot=i;
        if(c->last<h->contacts[oldest].last)oldest=i;
    }
    if(slot<0){slot=oldest;h->evictions++;h->contacts[slot].used=0;}
    HeatContact *c=&h->contacts[slot];
    if(!c->used) {
        memset(c,0,sizeof *c);c->used=1;c->x=x;c->y=y;c->z=z;c->kind=kind;
        memcpy(c->pos,before,sizeof c->pos);
    }
    c->heat+=distance;c->peak=fmaxf(c->peak,distance);c->corrections++;c->last=h->seconds;
}
#endif
