/* Real GL regression: a light pass must neither inherit disabled depth testing
 * nor leak its texture, blend factors or uniforms into the next draw. No assets. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "render.h"

static void eq_uniform(const RProg *r, GLint loc, const float *want, int n) {
    float got[16]; glGetUniformfv(r->prog, loc, got);
    for (int i=0; i<n; i++) assert(fabsf(got[i]-want[i]) < 1e-6f);
}

static void footprint_test(void) {
    const float p[]={10,20,3},up[]={0,0,1};
    /* Off-centre model bounds must rotate around the same origin as the car. */
    const float body[]={-2,-1,0,4,1,2},bus[]={-6,-1.6f,0,6,1.6f,4};
    for(int i=0;i<8;i++) {
        float h=i*.785398163f,co=cosf(h),sn=sinf(h),m[16];
        mat_car_footprint(p,h,up,body,1.1f,1.35f,.03f,m);
        float cx=m[12]+.5f*(m[0]+m[4]),cy=m[13]+.5f*(m[1]+m[5]);
        assert(fabsf(cx-(p[0]+co))<1e-5f && fabsf(cy-(p[1]+sn))<1e-5f);
        assert(fabsf(m[0]*co+m[1]*sn-6.6f)<1e-5f);
        assert(fabsf(m[4]*(-sn)+m[5]*co-2.7f)<1e-5f);
        for(int x=0;x<2;x++)for(int y=0;y<2;y++) {
            float bx=body[x?3:0],by=body[y?4:1];
            float dx=p[0]+co*bx-sn*by-m[12],dy=p[1]+sn*bx+co*by-m[13];
            float u=(dx*co+dy*sn)/6.6f,v=(-dx*sn+dy*co)/2.7f;
            assert(u>0 && u<1 && v>0 && v<1); /* all body corners covered */
        }
        mat_car_footprint(p,h,up,bus,1.1f,1.35f,.03f,m);
        assert(fabsf(hypotf(m[0],m[1])-13.2f)<1e-5f);
    }
    float n[]={0,-.6f,.8f},m[16];
    mat_car_footprint(p,1.2f,n,body,1.1f,1.35f,.03f,m);
    for(int x=0;x<2;x++)for(int y=0;y<2;y++) {
        float distance=0;
        for(int a=0;a<3;a++)distance+=(m[12+a]+x*m[a]+y*m[4+a]-p[a])*n[a];
        assert(fabsf(distance-.03f)<1e-5f); /* stays just above sloped road */
    }
}

int main(void) {
    footprint_test();
    assert(SDL_Init(SDL_INIT_VIDEO) == 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_Window *win = SDL_CreateWindow("light-state-test", 0, 0, 32, 32,
                                     SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
    assert(win);
    SDL_GLContext ctx = SDL_GL_CreateContext(win); assert(ctx);
    RProg r = render_program(); GpuMesh quad = make_quad();
    /* A footprint covers the body corners while still fading at its edge.
       UV near (.89,.86) lies beyond the old oval but inside the car bounds. */
    float clip[]={2,0,0,0,0,2,0,0,0,0,1,0,-1,-1,0,1};
    glViewport(0,0,32,32);glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glUniformMatrix4fv(r.uMVP,1,GL_FALSE,clip);
    glUniform1f(r.uUnlit,1);glUniform1f(r.uUseTex,0);glUniform1f(r.uAlpha,.5f);
    glUniform1f(r.uFogDensity,0);glUniform3f(r.uColor,0,0,0);
    for(int soft=1;soft<=2;soft++) {
        glClearColor(1,1,1,1);glClear(GL_COLOR_BUFFER_BIT);
        glUniform1f(r.uSoft,(float)soft);draw_gpumesh(&quad);
        unsigned char inside[4],edge[4];
        glReadPixels(28,27,1,1,GL_RGBA,GL_UNSIGNED_BYTE,inside);
        glReadPixels(0,0,1,1,GL_RGBA,GL_UNSIGNED_BYTE,edge);
        assert(soft==1?inside[0]>250:inside[0]<210);
        assert(edge[0]>250);
    }
    GLuint tex[2]; glGenTextures(2, tex);
    const unsigned char pixel[4] = {128,64,32,255};
    glBindTexture(GL_TEXTURE_2D, tex[0]);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,1,1,0,GL_RGBA,GL_UNSIGNED_BYTE,pixel);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    float cam[3]={0,-3,0}, look[3]={0,1,0}, P[16], V[16], mvp[16], saved[16];
    mat_persp(0.9f,1,0.1f,20,P); mat_lookat(cam,look,V); mat_mul(P,V,mvp);
    mat_trans(2,3,4,saved);
    N2LightSrc lights[2] = {{{0,0,0},2,30,0xff4080ffu},
                            {{1000,1000,1000},2,30,0xffffffffu}};
    const float color[3]={0.2f,0.4f,0.6f};
    const GLint scalars[]={r.uAlpha,r.uUnlit,r.uEmissiveTex,r.uUseTex,r.uSoft};
    const float values[]={0.3f,0.4f,0.2f,0.3f,0.4f};
    glViewport(0,0,32,32); glClearColor(0,0,0,0);
    for (int mode=0; mode<3; mode++) {
        int blocked = mode==1, black_in_fog = mode==2;
        const float fog_color[3]={0.3f,0.2f,0.1f};
        glUniform3fv(r.uFogColor,1,fog_color);
        glUniform1f(r.uFogDensity,black_in_fog ? 1.0f : 0.0f);
        if (black_in_fog) {
            const unsigned char black[4]={0,0,0,255};
            glBindTexture(GL_TEXTURE_2D,tex[0]);
            glTexSubImage2D(GL_TEXTURE_2D,0,0,0,1,1,GL_RGBA,GL_UNSIGNED_BYTE,black);
        }
        glDepthMask(GL_TRUE); glClearDepth(blocked ? 0.0 : 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST); /* the pass must establish depth testing */
        if (blocked) glEnable(GL_BLEND); else glDisable(GL_BLEND);
        glBlendFuncSeparate(GL_ONE,GL_ZERO,GL_ZERO,GL_ONE);
        glBindTexture(GL_TEXTURE_2D,tex[1]);
        glUniform3fv(r.uColor,1,color);
        glUniformMatrix4fv(r.uMVP,1,GL_FALSE,saved);
        for (int i=0;i<5;i++) glUniform1f(scalars[i],values[i]);
        assert(render_district_lights(&r,&quad,tex[0],lights,2,cam,look,mvp,10,1.0f,1.0f)==1);
        eq_uniform(&r,r.uColor,color,3);
        eq_uniform(&r,r.uMVP,saved,16);
        eq_uniform(&r,r.uFogColor,fog_color,3);
        for (int i=0;i<5;i++) eq_uniform(&r,scalars[i],values+i,1);
        GLint v; glGetIntegerv(GL_TEXTURE_BINDING_2D,&v); assert((GLuint)v==tex[1]);
        glGetIntegerv(GL_BLEND_SRC_RGB,&v); assert(v==GL_ONE);
        glGetIntegerv(GL_BLEND_DST_RGB,&v); assert(v==GL_ZERO);
        glGetIntegerv(GL_BLEND_SRC_ALPHA,&v); assert(v==GL_ZERO);
        glGetIntegerv(GL_BLEND_DST_ALPHA,&v); assert(v==GL_ONE);
        assert(!glIsEnabled(GL_DEPTH_TEST));
        assert(!!glIsEnabled(GL_BLEND)==blocked);
        GLboolean mask; glGetBooleanv(GL_DEPTH_WRITEMASK,&mask); assert(mask);
        unsigned char out[4]; glReadPixels(16,16,1,1,GL_RGBA,GL_UNSIGNED_BYTE,out);
        if (blocked || black_in_fog) assert(out[0]==0 && out[1]==0 && out[2]==0);
        else assert(out[0]>0 && out[1]>0 && out[2]>0);
        assert(glGetError()==GL_NO_ERROR);
    }
    /* A wall 20 cm in front of the authored light must occlude its halo.
       A size-scaled camera offset used to pull the sprite through that wall. */
    glBindTexture(GL_TEXTURE_2D,tex[0]);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,0,1,1,GL_RGBA,GL_UNSIGNED_BYTE,pixel);
    glUniform1f(r.uFogDensity,0);
    for(int angle=0;angle<4;angle++)for(int far=0;far<2;far++)for(int blocked=0;blocked<2;blocked++) {
        float co=cosf(angle*1.570796327f),sn=sinf(angle*1.570796327f);
        float distance=far?30:3;
        float eye[]={-co*distance,-sn*distance,0},direction[]={co,sn,0};
        float projection[16],view[16],world[16];
        mat_persp(.9f,1,.1f,200,projection);mat_lookat(eye,direction,view);mat_mul(projection,view,world);
        glDepthMask(GL_TRUE);glClearDepth(1);glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);glDisable(GL_BLEND);
        if(blocked) {
            float wall[]={-sn*4,co*4,0,0, 0,0,4,0, co,sn,0,0,
                          -co*.2f+sn*2,-sn*.2f-co*2,-2,1};
            float wall_mvp[16];mat_mul(world,wall,wall_mvp);
            glUniformMatrix4fv(r.uMVP,1,GL_FALSE,wall_mvp);
            glUniform1f(r.uUseTex,0);glUniform1f(r.uUnlit,1);glUniform1f(r.uEmissiveTex,0);
            glUniform1f(r.uSoft,0);glUniform1f(r.uAlpha,1);glUniform3f(r.uColor,0,0,0);
            draw_gpumesh(&quad);
        }
        N2LightSrc lamp={{0,0,0},10,50,0xffffffffu};
        assert(render_district_lights(&r,&quad,tex[0],&lamp,1,eye,direction,world,100,1,1)==1);
        unsigned char out[4];glReadPixels(16,16,1,1,GL_RGBA,GL_UNSIGNED_BYTE,out);
        assert(blocked?(out[0]==0 && out[1]==0 && out[2]==0):out[0]>50);
    }
    /* A preceding car halo/smoke draw leaves its local MVP behind. World
       glows must remain at their authored position and add no fog rectangle. */
    BatchedVertex bv[4]={0};uint16_t bi[]={0,1,2,0,2,3};
    for(int i=0;i<4;i++) {
        bv[i].pos[0]=(i==1||i==2)?1:-1;bv[i].pos[1]=i>=2?1:-1;
        memset(bv[i].col,255,4);
    }
    N2Batch batch={.index_count=6,.tex=tex[0]};
    glGenBuffers(1,&batch.vbo);glBindBuffer(GL_ARRAY_BUFFER,batch.vbo);
    glBufferData(GL_ARRAY_BUFFER,sizeof bv,bv,GL_STATIC_DRAW);
    glGenBuffers(1,&batch.ibo);glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,batch.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,sizeof bi,bi,GL_STATIC_DRAW);
    float identity[16],wrong[16];mat_trans(0,0,0,identity);mat_trans(100,100,100,wrong);
    for(int mode=0;mode<4;mode++) {
        unsigned char px[]={mode==2?0:128,0,0,255};
        glBindTexture(GL_TEXTURE_2D,tex[0]);
        glTexSubImage2D(GL_TEXTURE_2D,0,0,0,1,1,GL_RGBA,GL_UNSIGNED_BYTE,px);
        glDepthMask(GL_TRUE);glClearDepth(mode==1?0:1);glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glDisable(GL_DEPTH_TEST);glUniformMatrix4fv(r.uMVP,1,GL_FALSE,wrong);
        render_model(&r,wrong);glUniform1f(r.uAlpha,0);glUniform1f(r.uSoft,1);
        const float fog[]={.3f,.2f,.1f};glUniform3fv(r.uFogColor,1,fog);
        glUniform1f(r.uFogDensity,mode==2?1:0);batch.unresolved=mode==3;
        assert(render_world_glows(&r,&batch,1,identity)==(mode==3?0:1));
        eq_uniform(&r,r.uMVP,identity,16);eq_uniform(&r,r.uFogColor,fog,3);
        unsigned char out[4];glReadPixels(16,16,1,1,GL_RGBA,GL_UNSIGNED_BYTE,out);
        if(mode==0)assert(out[0]>100 && out[1]==0 && out[2]==0);
        else assert(out[0]==0 && out[1]==0 && out[2]==0);
        assert(glGetError()==GL_NO_ERROR);
    }
    glDeleteBuffers(1,&batch.vbo);glDeleteBuffers(1,&batch.ibo);
    glDeleteTextures(2,tex); glDeleteProgram(r.prog);
    glDeleteBuffers(1,&quad.vbo); glDeleteBuffers(1,&quad.nbo); glDeleteBuffers(1,&quad.ibo);
    SDL_GL_DeleteContext(ctx); SDL_DestroyWindow(win); SDL_Quit();
    puts("light_state_test: PASS (vehicle footprints, soft shadow coverage, state isolation, depth occlusion, distance culling, additive fog, world-glow transform isolation)");
    return 0;
}
