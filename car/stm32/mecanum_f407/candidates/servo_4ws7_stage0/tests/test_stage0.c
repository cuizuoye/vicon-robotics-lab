/* Real stage0 controller/protocol with synthetic PWM -> encoder dynamics.
 * No test-only enable bypass; same authority code as the ARM target. */
#include "four_link_bench.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static drive_bench_t b;static four_link_bench_t l;
static uint32_t now,seq,session=17;static int32_t counts[4];
static float speed[4],fraction[4];static unsigned groups;
static bool frame(unsigned type,unsigned flags,const void *p,size_t n)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;four_link_request_t r;
    assert(mc_build_frame(type,flags,++seq,session,now,p,(uint16_t)n,raw,sizeof(raw),&used));
    assert(four_link_bench_decode(raw,used,&r));
    return four_link_bench_process(&l,&b,&r,now);
}
static void velocity(int x,int y,bool enabled)
{const mc_cmd_vel_v1_t v={x,y,0,600,0,0};frame(MC_MSG_CMD_VEL,enabled?MC_FLAG_ENABLED:0,&v,sizeof(v));}
static void tick(int x,int y,bool enabled)
{
    now+=10;velocity(x,y,enabled);
    for(unsigned i=0;i<4;i++) {
        const int u=b.group.raw_duty[i]*b.adopted.wheel[i].motor_sign;
        const float target=copysignf(fmaxf(0,(abs(u)-80)/.15f),(float)u);
        speed[i]+=(target-speed[i])*.01f/.11f;
        fraction[i]+=speed[i]*.01f;const int delta=(int)truncf(fraction[i]);fraction[i]-=delta;
        counts[i]+=delta*b.adopted.wheel[i].encoder_sign;
    }
    (void)drive_bench_tick(&b,counts,now);four_link_bench_deadline(&l,&b,now);
    uint32_t ccr[8];assert(four_drive_compares(&b.group,ccr));
    for(unsigned i=0;i<4;i++)assert(abs(b.group.raw_duty[i])<=350);
}
static void init(void)
{
    now=seq=0;session=17;memset(counts,0,sizeof(counts));memset(speed,0,sizeof(speed));
    memset(fraction,0,sizeof(fraction));drive_bench_init(&b,counts,now);four_link_bench_init(&l);
    assert(!b.group.fault && frame(MC_MSG_HELLO,0,NULL,0));
    for(unsigned i=0;i<60;i++)tick(0,0,false);
    assert(b.group.stationary_ms==500 && !l.permitted);
}
static void select_mode(unsigned mode)
{
    const mc_control_enable_v1_t select={.reserved=mode==1?SERVO_TIER_MAGIC:SERVO_BENCH_MAGIC};
    frame(MC_MSG_CONTROL_ENABLE,0,&select,sizeof(select));
    assert(l.tier_selected && l.action_mode==mode && !l.permitted);
}
static void enable(unsigned mode)
{
    const mc_control_enable_v1_t e={.enabled=1,.reserved=mode==1?SERVO_TIER_MAGIC:SERVO_BENCH_MAGIC};
    frame(MC_MSG_CONTROL_ENABLE,0,&e,sizeof(e));assert(l.permitted && !b.group.enabled);
}
static void snapshot(const char *path)
{
    if(!path)return;
    speed_bench_diagnostics_t d={.ms=now,.ticks=now/10,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t t;four_link_bench_status_t s;
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&l,&b,&d,&t,&s));
    assert(s.output_authority_version==3 && s.hardware_released==1 && s.stage_id==0);
    FILE *f=fopen(path,"wb");assert(f);assert(fwrite(&s,1,sizeof(s),f)==sizeof(s));assert(!fclose(f));
}
int main(int argc,char **argv)
{
    assert(argc==1 || argc==4);
    init();snapshot(argc==4?argv[1]:NULL);groups++;
    servo_stage_t p;assert(servo_stage_get(0,&p) && p.duty_permille==350);
    for(unsigned i=1;i<7;i++)assert(!servo_stage_get(i,&p));
    groups++;
    init();select_mode(2);enable(2);
    uint32_t first=0;
    for(unsigned i=0;i<540;i++) {
        tick(200,0,true);
        if(l.motion_started && !first)first=l.first_motion_ms;
        if(first && now-first<5000)assert(l.permitted && !b.group.fault);
        if(i==350)snapshot(argc==4?argv[2]:NULL);
    }
    assert(l.bench_completed && !l.permitted && !b.group.enabled && !b.group.fault);
    assert(l.first_motion_ms==first);snapshot(argc==4?argv[3]:NULL);
    for(unsigned i=0;i<4;i++)assert(!b.group.raw_duty[i]);
    groups++;
    for(unsigned i=0;i<100;i++)tick(200,0,true);
    assert(!l.permitted && !b.group.enabled && !b.group.fault);groups++;
    const mc_control_enable_v1_t again={.enabled=1,.reserved=SERVO_BENCH_MAGIC};
    frame(MC_MSG_CONTROL_ENABLE,0,&again,sizeof(again));
    assert(!l.permitted && b.group.fault);groups++;
    init();select_mode(1);enable(1);
    for(unsigned i=0;i<650;i++){tick(141,141,true);assert(l.permitted && !b.group.fault);}
    assert(!l.bench_completed && b.group.enabled);groups++;
    velocity(201,0,true);assert(!b.group.enabled && b.group.fault);groups++;
    init();select_mode(2);enable(2);for(unsigned i=0;i<20;i++)tick(200,0,true);
    velocity(0,0,false);assert(!l.permitted && !b.group.enabled);
    for(unsigned i=0;i<4;i++)assert(!b.group.raw_duty[i]);
    groups++;
    init();select_mode(2);enable(2);for(unsigned i=0;i<20;i++)tick(200,0,true);
    now+=600;four_link_bench_deadline(&l,&b,now);
    assert(!b.group.enabled && b.group.fault);groups++;
    printf("PASS: %u stage0/mode/5s/stop/lease groups; stage0 only, NO hardware\n",groups);
    return 0;
}
