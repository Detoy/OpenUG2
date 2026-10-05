#include "traffic_heatmap.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    TrafficHeatmap h={0};float a[]={0,0},b[]={20,0},p[]={5,0,0},q[]={5.1f,0,0};
    heatmap_begin(&h,.25f);
    heatmap_car(&h,0,0,0,1,a,b,p,4,0,.25f);
    heatmap_car(&h,1,0,0,1,a,b,p,8,0,.25f);
    heatmap_car(&h,2,0,1,0,b,a,p,4,0,.25f);
    float bridge[]={5,0,8};heatmap_car(&h,3,0,0,1,a,b,bridge,4,0,.25f);
    heatmap_car(&h,4,0,0,1,a,b,p,10,1,.25f);
    assert(h.nroads==4 && h.roads[0].count==2 && h.roads[0].speed==6);
    assert(h.roads[1].from==1 && h.roads[2].layer==2 && h.roads[3].racer);
    for(int i=0;i<14;i++) {
        heatmap_begin(&h,.25f);heatmap_car(&h,0,0,0,1,a,b,p,0,0,.25f);heatmap_end(&h);
    }
    assert(h.roads[0].stationary==1 && !h.actors[1].valid);
    heatmap_begin(&h,.25f);heatmap_car(&h,0,1,0,1,a,b,p,0,0,.25f);heatmap_end(&h);
    assert(!h.roads[0].stationary); /* reused slot is not a stuck vehicle */
    heatmap_begin(&h,.25f);heatmap_end(&h);
    assert(!h.nroads && !h.actors[0].valid); /* despawn clears snapshot */
    heatmap_begin(&h,.25f);heatmap_car(&h,0,1,-1,-1,a,b,p,0,0,.25f);heatmap_end(&h);
    assert(h.nroads==1 && h.roads[0].a[0]==p[0] && h.roads[0].b[0]==p[0]);
    heatmap_contact(&h,p,q,HEAT_WORLD);heatmap_contact(&h,p,q,HEAT_WORLD);
    heatmap_contact(&h,p,q,HEAT_VEHICLE);
    float up[]={5.1f,0,8};heatmap_contact(&h,bridge,up,HEAT_WORLD);
    assert(h.contacts[0].corrections==2 && h.contacts[1].kind==HEAT_VEHICLE && h.contacts[2].z==2);
    float heat=h.contacts[0].heat;heatmap_begin(&h,10);
    assert(fabsf(h.contacts[0].heat-heat/expf(1))<1e-6f);
    TrafficHeatmap unchanged=h;float bad[]={NAN,0,0};
    heatmap_contact(&h,bad,q,HEAT_WORLD);heatmap_contact(&h,p,p,HEAT_WORLD);
    heatmap_car(&h,0,0,0,1,a,b,bad,0,0,.25f);heatmap_begin(&h,NAN);
    assert(!memcmp(&h,&unchanged,sizeof h));
    heatmap_begin(&h,51);assert(!h.contacts[0].used);
    heatmap_clear(&h);
    for(int i=0;i<HEAT_CONTACTS+1;i++) {
        float x[]={i*9.0f,0,0},y[]={i*9.0f+.1f,0,0};
        heatmap_contact(&h,x,y,HEAT_WORLD);h.seconds+=.25;
    }
    assert(h.evictions==1 && h.contacts[0].x==HEAT_CONTACTS*9/8);
    heatmap_clear(&h);TrafficHeatmap empty={0};assert(!memcmp(&h,&empty,sizeof h));
    puts("traffic_heatmap_test: PASS (direction, height, racers, occupancy, stationary, respawn, decay, bounds, reset)");
}
