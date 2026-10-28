/* Static prescribed-state contracts only; no vehicle/motor dynamics. */
#include "four_link_bench.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static const int32_t zero[4]={0};
static void save(const char *path,const four_link_bench_status_t *s)
{
    FILE *f=fopen(path,"wb"); assert(f);
    assert(fwrite(s,1,sizeof(*s),f)==sizeof(*s)); assert(!fclose(f));
}
int main(int argc,char **argv)
{
    assert(argc==3);
    drive_bench_t b; drive_bench_init(&b,zero,1000);
    assert(!b.group.fault && b.adopted.duty_limit_permille==350);
    assert(b.adopted.launch_duty_permille==300 && b.adopted.ramp_cps_per_s==800);
    assert(b.adopted.command_timeout_ms==600 && b.adopted.run_limit_ms==150000);
    const mecanum_body_velocity_t good[]={
        {.2f,0,0},{-.2f,0,0},{0,.2f,0},{0,-.2f,0},
        {.14f,.14f,.1f},{-.14f,.14f,-.1f},{.14f,-.14f,.1f},{-.14f,-.14f,-.1f},{0,0,.1f}};
    const mecanum_body_velocity_t bad[]={
        {.201f,0,0},{.2f,.2f,0},{-.2f,-.01f,0},{0,0,.101f},{NAN,0,0}};
    mecanum_drive_plan_t p;
    for(unsigned j=0;j<sizeof(good)/sizeof(good[0]);j++) {
        assert(mecanum_drive_plan(&b.group.geometry,&b.adopted,&good[j],.3f,&p));
        for(unsigned i=0;i<4;i++) {
            assert(fabsf(p.wheel_m_s[i])<=.3f && fabsf(p.forward_cps[i])<=1700);
            assert(fabsf(p.forward_cps[i])*60/b.adopted.wheel[i].counts_per_rev<=100);
        }
    }
    for(unsigned j=0;j<sizeof(bad)/sizeof(bad[0]);j++)
        assert(!mecanum_drive_plan(&b.group.geometry,&b.adopted,&bad[j],.3f,&p));
    uint32_t a,d;
    assert(four_speed_compare(350,&a,&d) && a==100 && d==65);
    assert(four_speed_compare(-350,&a,&d) && a==65 && d==100);
    assert(!four_speed_compare(360,&a,&d));
    for(unsigned i=0;i<4;i++) {
        four_speed_controller_t c; int32_t raw[4]={0};
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        assert(!four_speed_set_target(&c,1701,1000));
        assert(!c.enabled && (c.fault&FOUR_SPEED_FAULT_COMMAND));
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        assert(four_speed_set_target(&c,1000,1000));
        c.wheel[i].ramped_cps=1000;
        c.wheel[i].raw_duty_permille=300*c.config.wheel[i].motor_sign;
        assert(four_speed_set_target(&c,-1000,1020) && c.reversing && c.run_direction==1);
        assert(c.wheel[i].ramped_cps>0);
        assert(four_speed_set_target(&c,0,1030) && !c.wheel[i].raw_duty_permille);
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        assert(four_speed_set_target(&c,1000,1000));
        raw[i]=24*c.config.wheel[i].encoder_sign;
        assert(!four_speed_tick(&c,raw,1010) && !c.enabled);
        assert(c.first_feedback_fault.reason==FOUR_FEEDBACK_OVERSPEED);
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        assert(four_speed_set_target(&c,1000,1000));
        assert(!four_speed_set_target(&c,1000,1600));
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        assert(four_speed_set_target(&c,1000,1000));
        assert(!four_speed_tick(&c,zero,1031) && (c.fault&FOUR_SPEED_FAULT_TIMING));
    }
    four_link_bench_t link; four_link_bench_init(&link); b.group.stationary_ms=500;
    speed_bench_diagnostics_t diag={.ms=1000,.ticks=1,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t t; four_link_bench_status_t s;
    assert(four_drive_compares(&b.group,diag.compares));
    assert(four_link_bench_payloads(&link,&b,&diag,&t,&s));
    assert(s.tag==0x30325734 && sizeof(s)==156 && s.output_authority_version==1);
    save(argv[1],&s);
    link.permitted=true; b.group.enabled=b.group.run_used=true;
    b.group.plan.applied_body.vx_m_s=.2f;
    for(unsigned i=0;i<4;i++) {
        four_speed_controller_t *c=&b.group.loop[i];
        c->enabled=true; c->selected=i; c->run_direction=1;
        b.group.plan.forward_cps[i]=c->wheel[i].requested_cps=1000;
        c->wheel[i].ramped_cps=1000;
        b.group.raw_duty[i]=c->wheel[i].raw_duty_permille=350*c->config.wheel[i].motor_sign;
    }
    assert(four_drive_compares(&b.group,diag.compares));
    assert(four_link_bench_payloads(&link,&b,&diag,&t,&s)); save(argv[2],&s);
    puts("4W20 static contracts passed; hardware access=false; vehicle model=false");
    return 0;
}
