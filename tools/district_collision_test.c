#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "physics.h"

static int body_contact(float *p,float *v,float heading,const float bb[6],
        const float obst[][4],const float obz[][2],const N2Scene *s,const int *src) {
#ifdef M156_POINT_BASELINE
    (void)heading;(void)bb;
    return collide_walls(p,v,obst,obz,1,1.3f,.28f,2.1f,s,src,NULL,0);
#else
    return collide_body_walls(p,v,heading,bb,obst,obz,1,.28f,2.1f,s,src,NULL,0);
#endif
}

static void make_vertical_panel(N2Scene *scene, N2Mesh *mesh,
                                float verts[20], uint16_t idx[6],
                                int scen, float height) {
    memset(scene, 0, sizeof *scene);
    memset(mesh, 0, sizeof *mesh);
    const float points[4][3] = {
        {0.0f, -2.0f, 0.0f}, {0.0f, 2.0f, 0.0f},
        {0.0f,  2.0f, height}, {0.0f, -2.0f, height}
    };
    for (int i = 0; i < 4; i++) {
        verts[i * 5 + 0] = points[i][0];
        verts[i * 5 + 1] = points[i][1];
        verts[i * 5 + 2] = points[i][2];
    }
    const uint16_t triangles[6] = {0, 1, 2, 0, 2, 3};
    memcpy(idx, triangles, sizeof triangles);
    mesh->verts = verts;
    mesh->nverts = 4;
    mesh->idx = idx;
    mesh->nidx = 6;
    mesh->cat = N2_OTHER;
    mesh->scen = (unsigned char)scen;
    scene->meshes = mesh;
    scene->count = scene->cap = 1;
}

static int collect_one(N2Scene *scene, float obst[1][4], int src[1],
                       float obz[1][2]) {
    return phys_collect_walls(scene, obst, src, obz, 1);
}

static void test_body_ends_stay_on_wall_side(void) {
    const float bb[6]={-2.4f,-1.2f,-.15f,2.4f,1.2f,1.7f};
    for(int turn=0;turn<3;turn++)for(int rev=0;rev<2;rev++) {
        float angle=turn*.71f,c=cosf(angle),s=sinf(angle);
        N2Scene sc;N2Mesh m;float verts[20]={0},obst[1][4],obz[1][2];
        uint16_t ix[6];int src[1];
        make_vertical_panel(&sc,&m,verts,ix,N2_SC_WALL,3);
        for(int i=0;i<4;i++){float y=verts[i*5+1];verts[i*5]=20-s*y;verts[i*5+1]=-30+c*y;}
        if(rev)for(int i=0;i<6;i+=3){uint16_t t=ix[i];ix[i]=ix[i+2];ix[i+2]=t;}
        assert(collect_one(&sc,obst,src,obz)==1);
        /* At center distance2.25 the rear bumper is already15cm through the
         * wall. A 1.3m center circle misses it. Test front-first and reverse. */
        for(int dir=0;dir<2;dir++) {
            float p[3]={20+2.25f*c,-30+2.25f*s,0},v[2]={-c-.25f*s,-s+.25f*c};
            int n=body_contact(p,v,angle+dir*3.14159265f,bb,obst,obz,&sc,src);
            float distance=(p[0]-20)*c+(p[1]+30)*s;
            printf("body-wall: hits%d distance%.6f (required2.4)\n",n,distance);fflush(stdout);
            assert(n>0 && distance>=2.3999f && distance<2.401f);
            assert(fabsf(v[0]*c+v[1]*s)<1e-5f);
            assert(fabsf(-v[0]*s+v[1]*c-.25f)<1e-5f);
            assert(p[2]==0);
        }
        /* A side-on car uses width, not an oversized length-radius circle. */
        float p[3]={20+1.21f*c,-30+1.21f*s,0},v[2]={c,s};
        float before[3];memcpy(before,p,sizeof p);
        assert(body_contact(p,v,angle+1.57079633f,bb,obst,obz,&sc,src)==0);
        assert(memcmp(before,p,sizeof p)==0);
        p[0]=20+1.1f*c;p[1]=-30+1.1f*s;v[0]=-c;v[1]=-s;
        assert(body_contact(p,v,angle+1.57079633f,bb,obst,obz,&sc,src)>0);
        assert(fabsf((p[0]-20)*c+(p[1]+30)*s-1.2f)<1e-4f);
    }
}

static void test_body_feature_edges_and_fallback(void) {
    const float bb[6]={-2.4f,-1.2f,-.15f,2.4f,1.2f,1.7f};
    N2Scene sc;N2Mesh m;float verts[20]={0},obst[1][4],obz[1][2];
    uint16_t ix[6];int src[1];
    /* A short wall crosses the middle of the capsule axis. Neither capsule
     * endpoint is inside the wall strip: segment intersection must find it. */
    for(int rev=0;rev<2;rev++) {
        make_vertical_panel(&sc,&m,verts,ix,N2_SC_WALL,3);
        for(int i=0;i<4;i++)verts[i*5+1]*=.025f;
        if(rev)for(int i=0;i<6;i+=3){uint16_t t=ix[i];ix[i]=ix[i+2];ix[i+2]=t;}
        assert(collect_one(&sc,obst,src,obz)==1);
        float p[3]={.1f,0,0},v[2]={-1,.3f};
        assert(body_contact(p,v,0,bb,obst,obz,&sc,src)>0);
        assert(fabsf(p[0]-2.4f)<1e-5f && fabsf(v[0])<1e-5f && v[1]==.3f);
        /* The finite wall must not turn into an infinite plane. */
        p[0]=.1f;p[1]=2;v[0]=-1;
        assert(body_contact(p,v,0,bb,obst,obz,&sc,src)==0);
        assert(p[0]==.1f && p[1]==2 && v[0]==-1);
    }
    /* Neither a seam nor a wall above the car becomes a collision because
     * the footprint is longer. Existing height clipping remains mandatory. */
    make_vertical_panel(&sc,&m,verts,ix,N2_SC_WALL,.10f);
    for(int i=0;i<4;i++)verts[i*5+2]+=.5f; /* inside the car's height window */
    assert(collect_one(&sc,obst,src,obz)==1);
    float p[3]={2.25f,0,0},v[2]={-1,0};
    assert(body_contact(p,v,0,bb,obst,obz,&sc,src)==0);
    make_vertical_panel(&sc,&m,verts,ix,N2_SC_WALL,3);
    for(int i=0;i<4;i++)verts[i*5+2]+=10;
    assert(collect_one(&sc,obst,src,obz)==1);
    assert(body_contact(p,v,0,bb,obst,obz,&sc,src)==0);
    make_vertical_panel(&sc,&m,verts,ix,N2_SC_WALL,3);
    assert(collect_one(&sc,obst,src,obz)==1);
    const float asym[6]={-2.6f,-.9f,0,2.2f,1.1f,2};
    p[0]=2.45f;p[1]=0;v[0]=-1;
    assert(body_contact(p,v,0,asym,obst,obz,&sc,src)>0);
    assert(fabsf(p[0]-2.6f)<1e-5f);
    /* Missing model bounds keep the existing circle contract. */
    p[0]=1.1f;v[0]=-1;
    assert(body_contact(p,v,0,NULL,obst,obz,&sc,src)>0);
    assert(fabsf(p[0]-1.3f)<1e-5f);
}

static void test_rail_uses_body_footprint(void) {
    /* First false L4RD/4401 rail response: the old fixed 1.3 m centre circle
     * reaches this authored face while the MIATA-sized body capsule does not. */
    const float p0[3]={-332.915100f,638.782104f,30.112734f};
    const float p1[3]={-338.979919f,646.503601f,29.497175f};
    const float p2[3]={-338.979919f,646.503601f,31.497177f};
    float verts[15]={0};
    for(int c=0;c<3;c++){verts[c]=p0[c];verts[5+c]=p1[c];verts[10+c]=p2[c];}
    uint16_t idx[3]={0,1,2};
    N2Mesh mesh={0};mesh.verts=verts;mesh.nverts=3;mesh.idx=idx;mesh.nidx=3;mesh.cat=N2_ROAD;
    N2Scene scene={&mesh,1,1};
    const float bb[6]={-1.973f,-.9415f,0,1.973f,.9415f,1.221f};
    float pos[3]={-337.657928f,646.878296f,29.322237f};
    float vel[2]={-.786421f,-.617691f},before[3];memcpy(before,pos,sizeof pos);
    PhysWallContact hit={0};
#ifdef M157_RAIL_CIRCLE_BASELINE
    int n=cw_mesh_feature(&scene,0,pos[0],pos[1],1.3f,pos[2]-.5f,pos[2]+.5f,&hit);
#else
    int n=collide_body_mesh_wall(pos,vel,-1.1702f,bb,29.3f,30.6f,
                                 &scene,0,.75f,2.5f,&hit);
#endif
    assert(n==0 && memcmp(pos,before,sizeof pos)==0);

#ifndef M157_RAIL_CIRCLE_BASELINE
    /* Ten centimetres closer is a real contact. Remove only inward velocity;
     * the component along the barrier must survive. */
    pos[0]=before[0]-.0786421f;pos[1]=before[1]-.0617691f;
    vel[0]=-.786421f-.25f*.617691f;vel[1]=-.617691f+.25f*.786421f;
    assert(collide_body_mesh_wall(pos,vel,-1.1702f,bb,29.3f,30.6f,
                                  &scene,0,.75f,2.5f,&hit)==1);
    assert(hit.pen>0 && hit.pen<.05f);
    assert(fabsf(vel[0]*hit.nx+vel[1]*hit.ny)<1e-5f);
    assert(fabsf(-vel[0]*hit.ny+vel[1]*hit.nx-.25f)<5e-5f);

    /* Keep the measured empty height band: road seams are not rails and tall
     * terrain faces stay owned by terrain/building collision. */
    pos[0]=before[0]-.0786421f;pos[1]=before[1]-.0617691f;
    vel[0]=-.786421f;vel[1]=-.617691f;verts[12]=29.9f;
    assert(!collide_body_mesh_wall(pos,vel,-1.1702f,bb,29.3f,30.6f,
                                   &scene,0,.75f,2.5f,&hit));
    verts[12]=33.0f;
    assert(!collide_body_mesh_wall(pos,vel,-1.1702f,bb,29.3f,30.6f,
                                   &scene,0,.75f,2.5f,&hit));
#endif
}

static void test_sloping_curb_is_not_a_rail(void) {
    /* A ten-centimetre curb rising a metre over twenty metres has >.75 m
       total Z span, but remains a drivable curb. A tall rail on the same
       grade must still block. Rotate and reverse winding, without names. */
    const float bb[]={-1.97f,-.94f,0,1.97f,.94f,1.37f};
    for(int turn=0;turn<3;turn++)for(int winding=0;winding<2;winding++)
    for(int tall=0;tall<2;tall++) {
        float height=tall?1.2f:.1f,c=cosf(turn*.71f),sn=sinf(turn*.71f);
        float v[20]={0};uint16_t idx[]={0,1,2,0,2,3};
        float local[][3]={{0,0,0},{20,0,1},{20,0,1+height},{0,0,height}};
        for(int k=0;k<4;k++) {
            v[k*5]=c*local[k][0];v[k*5+1]=sn*local[k][0];v[k*5+2]=local[k][2];
        }
        if(winding)for(int t=0;t<6;t+=3){uint16_t tmp=idx[t];idx[t]=idx[t+2];idx[t+2]=tmp;}
        N2Mesh mesh={.verts=v,.nverts=4,.idx=idx,.nidx=6,.cat=N2_ROAD};
        N2Scene scene={&mesh,1,1};
        for(int t=0;t<6;t+=3)
            assert(fabsf(phys_wall_face_height(v+idx[t]*5,v+idx[t+1]*5,v+idx[t+2]*5)-height)<.001f);
        float p[]={10*c-.5f*sn,10*sn+.5f*c,.5f},vel[]={sn,-c};
        int hit=collide_body_mesh_wall(p,vel,turn*.71f,bb,.55f,1.9f,&scene,0,.75f,2.5f,NULL);
        assert(hit==tall);
    }
}

static void test_wall_contact_uses_car_height(void) {
    /* Vertical triangular wall: in its local (y,z) plane the vertices are
     * (0,0), (10,10), (0,10). At z=0.2..2.1 the actual face ends at y=2.1,
     * not y=10. Projecting the entire upper edge onto XY invents a wall at
     * y=8, at least 5.9 m beyond any face at car height. Exercise real broad
     * phase, narrow phase AND response, in rotated/translated coordinates
     * and reversed winding. No texture/category exception may mask the bug. */
    const float angles[] = {0.0f, 0.71f, 1.5707963f};
    for (int a = 0; a < 3; a++) for (int winding = 0; winding < 2; winding++) {
        N2Scene scene; N2Mesh mesh;
        float verts[20] = {0}, obst[1][4], obz[1][2];
        uint16_t idx[6]; int src[1];
        make_vertical_panel(&scene, &mesh, verts, idx, N2_SC_WALL, 10.0f);
        const float local[3][2] = {{0,0},{10,10},{0,10}};
        float c=cosf(angles[a]),s=sinf(angles[a]);
        for(int j=0;j<3;j++) {
            verts[j*5]=20.0f-s*local[j][0];
            verts[j*5+1]=-30.0f+c*local[j][0];
            verts[j*5+2]=100.0f+local[j][1];
            idx[j]=(uint16_t)(winding?2-j:j);
        }
        mesh.nverts=3; mesh.nidx=3;
        assert(collect_one(&scene,obst,src,obz)==1);
        float pos[3]={20.0f+0.5f*c-8.0f*s,-30.0f+0.5f*s+8.0f*c,100.0f};
        float vel[2]={-c,-s};
        float before[3]; memcpy(before,pos,sizeof before);
        assert(collide_walls(pos,vel,obst,obz,1,1.0f,100.2f,102.1f,
                             &scene,src,NULL,0)==0);
        assert(memcmp(pos,before,sizeof pos)==0);
        assert(vel[0]==-c && vel[1]==-s);

        /* At y=1 a face really IS present at car height: keep its normal
         * response, even for a very thin car-height interval. The seam limit
         * describes the authored face, not the height of this clipped slice. */
        pos[0]=20.0f+0.5f*c-s; pos[1]=-30.0f+0.5f*s+c;
        vel[0]=-c; vel[1]=-s;
        PhysWallContact hit={0};
        assert(collide_walls(pos,vel,obst,obz,1,1.0f,101.4f,101.5f,
                             &scene,src,&hit,1)==1);
        assert(fabsf(hit.dist-0.5f)<1e-4f && hit.span>9.99f);
        assert(fabsf(vel[0])<1e-4f && fabsf(vel[1])<1e-4f);
        assert(fabsf((pos[0]-20.0f)*c+(pos[1]+30.0f)*s-1.0f)<1e-4f);
    }
}

static void test_five_vertex_height_slice(void) {
    N2Scene scene; N2Mesh mesh;
    float verts[20]={0},obst[1][4],obz[1][2]; uint16_t idx[6]; int src[1];
    make_vertical_panel(&scene,&mesh,verts,idx,N2_SC_WALL,2.0f);
    const float yz[3][2]={{0,0},{0,1},{10,2}};
    for(int j=0;j<3;j++) { verts[j*5+1]=yz[j][0]; verts[j*5+2]=yz[j][1]; idx[j]=(uint16_t)j; }
    mesh.nverts=mesh.nidx=3;
    assert(collect_one(&scene,obst,src,obz)==1);
    /* Both clipping planes cut off a different vertex: five points remain.
     * Their largest Y is 7.5, not the original triangle's Y=10. */
    assert(!cw_probe_contact(&scene,0,0.5f,9.0f,1.0f,0.5f,1.5f));
    assert(cw_probe_contact(&scene,0,0.5f,6.0f,1.0f,0.5f,1.5f));
    /* Boundary-inclusive slabs must also handle a vertex exactly on a plane,
     * including a zero-width height interval, without division by zero. */
    assert(cw_probe_contact(&scene,0,0.5f,2.0f,1.0f,1.0f,1.0f));
    assert(!cw_probe_contact(&scene,0,0.5f,8.0f,1.0f,1.0f,1.0f));
}

static void test_authored_barriers_and_baked_walls(void) {
    const char *guards[]={"XO_PATHGUARDA_1A_00","XO_PATHGUARDC_1A_00",
                          "XO_TRACKBARRIERB_B_1A_00","XO_ROADBARRIERB_1A_00",
                          "XO_FENCEF_1A_00","XO_CONSTWALLA_1A_00"};
    N2Scene s;N2Mesh m;float verts[20]={0},obst[1][4],obz[1][2];
    uint16_t idx[6];int src[1];
    const float bb[]={-1.97f,-.94f,0,1.97f,.94f,1.37f};
    for(unsigned i=0;i<sizeof guards/sizeof guards[0];i++) {
        make_vertical_panel(&s,&m,verts,idx,n2_scen_class(guards[i]),.65f);
        snprintf(m.sname,sizeof m.sname,"%s",guards[i]);
        assert(collect_one(&s,obst,src,obz)==1);
        float p[]={1.8f,0,0},v[]={-1,.2f};
        assert(collide_body_walls(p,v,0,bb,obst,obz,1,.1f,1.47f,&s,src,NULL,0));
        assert(p[0]>=1.969f && fabsf(v[0])<1e-5f && fabsf(v[1]-.2f)<1e-5f);
    }
    make_vertical_panel(&s,&m,verts,idx,N2_SC_TERRAIN,4.818f);
    m.cat=N2_TERRAIN;strcpy(m.sname,"TRN_TEST_PROPSB_CHOP_ANY");
    assert(collect_one(&s,obst,src,obz)==1);
    float p[]={1.8f,0,0},v[]={-1,.2f};
    assert(collide_body_walls(p,v,0,bb,obst,obz,1,.1f,1.47f,&s,src,NULL,0));
    assert(p[0]>=1.969f && fabsf(v[0])<1e-5f && fabsf(v[1]-.2f)<1e-5f);
    p[0]=1.8f;v[0]=-1;
    assert(!collide_body_walls(p,v,0,bb,obst,obz,1,5,6.4f,&s,src,NULL,0));
    /* A thin sloping curb in that same combined prop mesh stays passable. */
    make_vertical_panel(&s,&m,verts,idx,N2_SC_TERRAIN,.1f);
    m.cat=N2_TERRAIN;strcpy(m.sname,"TRN_TEST_PROPSB_CHOP_ANY");
    verts[7]+=2;verts[12]+=2;
    assert(collect_one(&s,obst,src,obz)==1);
    p[0]=1.8f;v[0]=-1;
    assert(!collide_body_walls(p,v,0,bb,obst,obz,1,.1f,2.2f,&s,src,NULL,0));
    /* Wall-mounted lights are furniture, not a match for barrier semantics. */
    assert(n2_scen_class("XO_STREETLIGHTCBWALL_1A_00")==N2_SC_PROP);
}

/* The optional wall broad phase must reproduce the linear scan bit for bit:
   same hits, same final position/velocity, including long pushes that leave
   the query margin and bodies outside the indexed extent. */
static void test_wall_index_matches_linear(void) {
    enum { NR = 3000 };
    static float obst[NR][4], obz[NR][2];
    unsigned seed = 99u;
    #define RND() (seed = seed*1664525u + 1013904223u, (float)(seed >> 8) / 16777216.0f)
    for (int o = 0; o < NR; o++) {
        float x = RND()*2000.0f - 1000.0f, y = RND()*2000.0f - 1000.0f;
        float w = RND() < 0.02f ? 150.0f + RND()*400.0f : 2.0f + RND()*40.0f;   /* some huge */
        float h = 2.0f + RND()*40.0f;
        obst[o][0] = x; obst[o][1] = y; obst[o][2] = x + w; obst[o][3] = y + h;
        obz[o][0] = RND()*10.0f; obz[o][1] = obz[o][0] + RND()*20.0f;
    }
    int hits_total = 0, far_push = 0;
    for (int q = 0; q < 20000; q++) {
        float p0[3] = {0}, v0[2];
        p0[0] = RND()*2400.0f - 1200.0f;
        p0[1] = RND()*2400.0f - 1200.0f;
        v0[0] = RND()*4.0f - 2.0f;
        v0[1] = RND()*4.0f - 2.0f;
        float r = 0.5f + RND()*3.0f, z0 = RND()*15.0f, z1 = z0 + 1.5f;
        float pa[3], va[2], pb[3], vb[2];
        memcpy(pa, p0, sizeof pa); memcpy(va, v0, sizeof va);
        memcpy(pb, p0, sizeof pb); memcpy(vb, v0, sizeof vb);
        phys_wall_index_build(NULL, NULL, 0);
        int a = collide_walls(pa, va, (const float (*)[4])obst, (const float (*)[2])obz, NR,
                              r, z0, z1, NULL, NULL, NULL, 0);
        phys_wall_index_build((const float (*)[4])obst, (const float (*)[2])obz, NR);
        int b = collide_walls(pb, vb, (const float (*)[4])obst, (const float (*)[2])obz, NR,
                              r, z0, z1, NULL, NULL, NULL, 0);
        assert(a == b && !memcmp(pa, pb, sizeof pa) && !memcmp(va, vb, sizeof va));
        hits_total += a;
        if (fabsf(pa[0]-p0[0]) >= 4.0f || fabsf(pa[1]-p0[1]) >= 4.0f) far_push++;
    }
    #undef RND
    phys_wall_index_build(NULL, NULL, 0);
    assert(hits_total > 1000 && far_push > 50);   /* contacts and margin fallback both covered */
    printf("wall index: 20000 queries identical (%d hits, %d pushes past the margin)\n",
           hits_total, far_push);
}

int main(void) {
    test_authored_barriers_and_baked_walls();
    test_sloping_curb_is_not_a_rail();
    N2Scene scene;
    N2Mesh mesh;
    float verts[20], obst[1][4], obz[1][2];
    uint16_t idx[6];
    int src[1];

    /* The real L4RA XW_SANDSTONEBASE instances are 0.548 m high and carry
     * near-vertical faces. Explicit WALL semantics must reach the geometric
     * narrow phase even though the old 2.5 m heuristic called them "flat". */
    make_vertical_panel(&scene, &mesh, verts, idx, N2_SC_WALL, 0.548f);
    assert(collect_one(&scene, obst, src, obz) == 1);
    assert(src[0] == 0 && fabsf(obz[0][1] - 0.548f) < 1e-6f);

    float pos[3] = {0.5f, 0.0f, 0.0f};
    float vel[2] = {-1.0f, 0.25f};
    PhysWallContact hit = {0};
    assert(collide_walls(pos, vel, obst, obz, 1, 1.0f, 0.28f, 2.10f,
                         &scene, src, &hit, 1) == 1);
    assert(pos[0] >= 0.999f);
    assert(fabsf(vel[0]) < 1e-6f && fabsf(vel[1] - 0.25f) < 1e-6f);
    assert(hit.span >= 0.547f);

    /* Explicit classification does not weaken the proven seam rejection: a
     * 0.10 m panel may enter broad phase, but the face-span narrow phase must
     * still reject it. */
    make_vertical_panel(&scene, &mesh, verts, idx, N2_SC_WALL, 0.10f);
    assert(collect_one(&scene, obst, src, obz) == 1);
    pos[0] = 0.5f; pos[1] = 0.0f; vel[0] = -1.0f; vel[1] = 0.25f;
    assert(collide_walls(pos, vel, obst, obz, 1, 1.0f, 0.0f, 2.10f,
                         &scene, src, NULL, 0) == 0);

    /* The old height heuristic remains valid for non-solid semantic props and
     * unnamed OTHER fallback meshes; this change is not a global lowering. */
    make_vertical_panel(&scene, &mesh, verts, idx, N2_SC_PROP, 0.548f);
    assert(collect_one(&scene, obst, src, obz) == 0);
    make_vertical_panel(&scene, &mesh, verts, idx, N2_SC_NONE, 0.548f);
    assert(collect_one(&scene, obst, src, obz) == 0);
    make_vertical_panel(&scene, &mesh, verts, idx, N2_SC_TERRAIN, 4.0f);
    assert(collect_one(&scene, obst, src, obz) == 0);
    /* The authored divider sign is 1.70 m wide but was classified STRUCT and
       became a permanent wall. Large XS structures still block. */
    make_vertical_panel(&scene, &mesh, verts, idx, N2_SC_STRUCT, 5.18f);
    strcpy(mesh.sname,"XS_WARNDIVIDEEND_1A_00");
    verts[5]=verts[10]=1.70f;
    assert(collect_one(&scene, obst, src, obz) == 0);
    verts[5]=verts[10]=4.0f;
    assert(collect_one(&scene, obst, src, obz) == 1);
    strcpy(mesh.sname,"XB_WALL_1A_00");
    assert(collect_one(&scene, obst, src, obz) == 1);
    /* Overhead arm widens a streetlight's full box; the base stays narrow. */
    float street[8*5]={0};
    for(int j=0;j<8;j++) {
        street[j*5] = j&1 ? (j<4?.4f:3.5f) : 0;
        street[j*5+1] = j&2 ? (j<4?.4f:3.5f) : 0;
        street[j*5+2] = j<4 ? 0 : 8;
    }
    mesh.verts=street;mesh.nverts=8;mesh.scen=N2_SC_PROP;
    strcpy(mesh.sname,"XO_STREETLIGHTS_1A_00");
    assert(collect_one(&scene, obst, src, obz) == 0);
    for(int j=0;j<4;j++) {
        street[j*5]=(j&1)*4.0f;street[j*5+1]=((j>>1)&1)*4.0f;
    }
    assert(collect_one(&scene, obst, src, obz) == 1);

    test_wall_contact_uses_car_height();
    test_five_vertex_height_slice();
    test_body_ends_stay_on_wall_side();
    test_body_feature_edges_and_fallback();
    test_rail_uses_body_footprint();
    test_wall_index_matches_linear();

    puts("district_collision_test: PASS");
    return 0;
}
