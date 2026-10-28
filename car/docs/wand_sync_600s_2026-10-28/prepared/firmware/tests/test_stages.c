/* Native production authority, synthetic encoders only; NO execution bypass. */
#include "four_link_bench.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static drive_bench_t b;static four_link_bench_t l;
static uint32_t now,seq,session=31;static int32_t counts[4];
static float speed[4],fraction[4];static unsigned stage_id,groups;
static void command(unsigned type,unsigned flags,const void *p,size_t n)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;four_link_request_t r;
    assert(mc_build_frame(type,flags,++seq,session,now,p,(uint16_t)n,raw,sizeof(raw),&used));
    assert(four_link_bench_decode(raw,used,&r));
    (void)four_link_bench_process(&l,&b,&r,now);
}
static void velocity(int x,int y,bool enable)
{mc_cmd_vel_v1_t v={x,y,0,600,0,0};command(MC_MSG_CMD_VEL,enable?MC_FLAG_ENABLED:0,&v,sizeof(v));}
static void tick(int x,int y,bool enable)
{
    now+=10;velocity(x,y,enable);
    for(unsigned i=0;i<4;i++){
        int u=b.group.raw_duty[i]*b.adopted.wheel[i].motor_sign;
        float target=copysignf(fmaxf(0,(abs(u)-80)/.15f),(float)u);
        speed[i]+=(target-speed[i])*.01f/.11f;fraction[i]+=speed[i]*.01f;
        int delta=(int)truncf(fraction[i]);fraction[i]-=delta;
        counts[i]+=delta*b.adopted.wheel[i].encoder_sign;
    }
    (void)drive_bench_tick(&b,counts,now);four_link_bench_deadline(&l,&b,now);
    servo_stage_t p;assert(servo_stage_get(stage_id,&p));
    for(unsigned i=0;i<4;i++)assert(abs(b.group.raw_duty[i])<=p.duty_permille);
    uint32_t ccr[8];assert(four_drive_compares(&b.group,ccr));
}
static void init(unsigned stage,unsigned mode)
{
    now=seq=0;session=31;stage_id=stage;
    memset(counts,0,sizeof(counts));memset(speed,0,sizeof(speed));memset(fraction,0,sizeof(fraction));
    drive_bench_init(&b,counts,0);four_link_bench_init(&l);
    command(MC_MSG_HELLO,0,NULL,0);
    for(unsigned i=0;i<60;i++)tick(0,0,false);
    assert(!l.permitted && b.group.stationary_ms==500);
    const uint16_t selected=(uint16_t)((mode==1?SERVO_TIER_MAGIC:SERVO_BENCH_MAGIC)+stage);
    mc_control_enable_v1_t e={.reserved=selected};command(MC_MSG_CONTROL_ENABLE,0,&e,sizeof(e));
    assert(l.tier_selected && !l.permitted && b.group.servo_stage==stage && l.action_mode==mode);
    e.enabled=1;command(MC_MSG_CONTROL_ENABLE,0,&e,sizeof(e));assert(l.permitted && !b.group.enabled);
}
static void snapshot(const char *dir,unsigned stage)
{
    if(!dir)return;
    speed_bench_diagnostics_t d={.ms=now,.ticks=now/10,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t t;four_link_bench_status_t s;
    assert(four_drive_compares(&b.group,d.compares));assert(four_link_bench_payloads(&l,&b,&d,&t,&s));
    assert(s.tag==UINT32_C(0x41535734) && s.output_authority_version==6 && s.stage_id==stage);
    char name[1024];assert(snprintf(name,sizeof(name),"%s/active%u.bin",dir,stage)>0);
    FILE *f=fopen(name,"wb");assert(f);assert(fwrite(&s,1,sizeof(s),f)==sizeof(s));assert(!fclose(f));
}
int main(int argc,char **argv)
{
    assert(argc==1 || argc==2);servo_stage_t p;
    for(unsigned s=0;s<7;s++){
        init(s,2);assert(servo_stage_get(s,&p));
        uint32_t first=0;float previous=0;
        for(unsigned i=0;i<540;i++){
            tick((int)lroundf(p.speed_m_s*1000),0,true);
            if(l.motion_started && !first)first=l.first_motion_ms;
            if(l.permitted){
                assert(!b.group.fault);
                float value=b.group.shaped_body.vx_m_s;
                assert(value-previous<=p.accel_m_s2*.0101f+.00001f);previous=value;
            }
            if(i==300)snapshot(argc==2?argv[1]:NULL,s);
        }
        assert(first && l.bench_completed && !l.permitted && !b.group.enabled && !b.group.fault);
        for(unsigned i=0;i<50;i++)tick((int)lroundf(p.speed_m_s*1000),0,true);
        assert(!l.permitted && !b.group.enabled);groups++;
        init(s,1);velocity((int)lroundf(p.speed_m_s*1000)+1,0,true);
        assert(!l.permitted && b.group.fault);groups++;
        init(s,1);for(unsigned i=0;i<80;i++)tick(100,0,true);
        mc_control_enable_v1_t other={.enabled=1,.reserved=(uint16_t)(SERVO_TIER_MAGIC+(s+1)%7)};
        command(MC_MSG_CONTROL_ENABLE,0,&other,sizeof(other));assert(!l.permitted && b.group.fault);groups++;
    }
    assert(!servo_stage_get(7,&p));
    init(6,1);for(unsigned i=0;i<36;i++)tick(700,0,true);
    assert(fabsf(b.group.shaped_body.vx_m_s-.70f)<.0001f && !b.group.fault);groups++;
    /* Inward integer quantization: 495/495 would exceed the 0.70 vector cap. */
    init(6,1);for(unsigned i=0;i<50;i++)tick(494,494,true);
    assert(l.permitted && !b.group.fault);
    for(unsigned i=0;i<100;i++){
        tick(-494,494,true);
        if(b.group.fault){fprintf(stderr,"reverse fault t=%u code=%u wheel=%u shaped=%f,%f target=%f,%f\n",now,b.group.fault,b.group.fault_wheel,b.group.shaped_body.vx_m_s,b.group.shaped_body.vy_m_s,b.group.plan.forward_cps[0],b.group.plan.forward_cps[1]);break;}
    }
    assert(l.permitted && !b.group.fault);groups++;
    velocity(0,0,false);assert(!l.permitted && !b.group.enabled);groups++;
    init(6,2);for(unsigned i=0;i<30;i++)tick(700,0,true);
    now+=600;four_link_bench_deadline(&l,&b,now);assert(!l.permitted && b.group.fault);groups++;
    printf("PASS: %u 4WSA staged/mode/5s/ramp/reversal/lease groups; synthetic only\n",groups);
    return 0;
}
