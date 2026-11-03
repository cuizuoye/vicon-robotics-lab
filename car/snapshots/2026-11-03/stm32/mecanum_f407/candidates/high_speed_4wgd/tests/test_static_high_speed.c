/* Prescribed snapshots and protocol/state assertions only. No motor/vehicle
 * dynamics, ROS, sockets, serial, ST-Link or hardware access. */
#include "four_link_bench.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned checked;
static const int32_t zero[4]={0};

static drive_bench_t bench(void)
{
    drive_bench_t b;
    drive_bench_init(&b,zero,1000U);
    assert(b.group.fault==0 && !b.group.enabled);
    return b;
}

static void test_limits_and_planner(void)
{
    drive_bench_t b=bench();
    assert(b.adopted.duty_limit_permille==800 && b.adopted.launch_duty_permille==300);
    assert(b.adopted.feedback_limit_cps==10000 && b.adopted.ramp_cps_per_s==800);
    assert(b.adopted.command_timeout_ms==600 && b.adopted.run_limit_ms==150000);
    const mecanum_body_velocity_t requests[]={
        {.2f,0,0},{0,.2f,0},{-.2f,0,0},{0,-.2f,0},
        {1.256f,0,0},{-1.256f,0,0},{0,1.256f,0},{0,-1.256f,0},
        {1.256f,1.256f,0},{-1.256f,1.256f,0},{1.256f,-1.256f,0},
        {1.256f,.7f,.2f},{-.9f,-1.2f,-.2f},{0,0,.2f},{0,0,-.2f}};
    for(unsigned j=0;j<sizeof(requests)/sizeof(requests[0]);j++) {
        mecanum_drive_plan_t p;
        assert(mecanum_drive_plan(&b.group.geometry,&b.adopted,&requests[j],FOUR_DRIVE_MAX_WHEEL_M_S,&p));
        const float rotation=.146f*requests[j].wz_rad_s;
        const float raw[4]={requests[j].vx_m_s-requests[j].vy_m_s-rotation,
            requests[j].vx_m_s+requests[j].vy_m_s+rotation,
            requests[j].vx_m_s+requests[j].vy_m_s-rotation,
            requests[j].vx_m_s-requests[j].vy_m_s+rotation};
        for(unsigned i=0;i<4;i++) {
            assert(fabsf(p.wheel_m_s[i]-raw[i]*p.common_scale)<.00001f);
            const float rpm=fabsf(p.forward_cps[i])*60/b.adopted.wheel[i].counts_per_rev;
            assert(rpm<=400.0001f);
            assert(fabsf(p.wheel_m_s[i])<=FOUR_DRIVE_MAX_WHEEL_M_S);
        }
        if(j==8)assert(p.common_scale<.501f); /* One diagonal pair limits all wheels. */
        checked++;
    }
    mecanum_drive_plan_t p;
    const mecanum_body_velocity_t bad[]={{1.257f,0,0},{0,-1.257f,0},{0,0,.201f},{NAN,0,0}};
    for(unsigned i=0;i<4;i++) {
        assert(!mecanum_drive_plan(&b.group.geometry,&b.adopted,&bad[i],FOUR_DRIVE_MAX_WHEEL_M_S,&p));
        assert(!p.forward_cps[0] && !p.forward_cps[3]); checked++;
    }
}

static void test_per_wheel_rpm_and_zero(void)
{
    drive_bench_t b=bench();
    for(unsigned i=0;i<4;i++) {
        four_speed_controller_t c;
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        const float limit=b.adopted.wheel[i].counts_per_rev*400/60;
        assert(four_speed_set_target(&c,limit,1000));
        assert(!four_speed_set_target(&c,limit+.1f,1001));
        assert(!c.enabled && (c.fault&FOUR_SPEED_FAULT_COMMAND));
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        assert(four_speed_set_target(&c,-limit,1000));
        c.wheel[i].ramped_cps=-limit;
        c.wheel[i].raw_duty_permille=-800*c.config.wheel[i].motor_sign;
        assert(four_speed_set_target(&c,0,1010));
        assert(c.wheel[i].raw_duty_permille==0 && c.wheel[i].ramped_cps==0);
        checked+=2;
    }
    uint32_t first,second;
    assert(four_speed_compare(800,&first,&second) && first==100 && second==20);
    assert(four_speed_compare(-800,&first,&second) && first==20 && second==100);
    assert(!four_speed_compare(810,&first,&second));
    assert(four_speed_compare(0,&first,&second) && !first && !second);
    checked++;
}

static void test_feedback_reversal_and_deadlines(void)
{
    drive_bench_t b=bench();
    for(unsigned i=0;i<4;i++) {
        four_speed_controller_t c;
        int32_t raw[4]={0};
        assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,i,zero,1000));
        assert(four_speed_set_target(&c,1000,1000));
        raw[i]=80*c.config.wheel[i].encoder_sign; /* ~452-458rpm, below 10000cps. */
        assert(!four_speed_tick(&c,raw,1010));
        assert(c.fault&FOUR_SPEED_FAULT_FEEDBACK);
        assert(c.first_feedback_fault.reason==FOUR_FEEDBACK_OVERSPEED && !c.enabled);
        assert(c.first_feedback_fault.raw_delta==raw[i]);
        checked++;
    }
    four_speed_controller_t c;
    assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,0,zero,1000));
    const float limit=b.adopted.wheel[0].counts_per_rev*400/60;
    assert(four_speed_set_target(&c,limit,1000));
    c.wheel[0].ramped_cps=limit; c.wheel[0].raw_duty_permille=300;
    assert(four_speed_set_target(&c,-limit,1020));
    assert(c.reversing && c.run_direction==1 && c.reversal_budget_ms>750);
    assert(c.wheel[0].ramped_cps>0); /* Opposite request cannot reverse actual output immediately. */
    assert(!four_speed_set_target(&c,-limit,1620));
    assert(!c.enabled && (c.fault&FOUR_SPEED_FAULT_COMMAND));
    assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,0,zero,1000));
    assert(four_speed_set_target(&c,1000,1000));
    assert(!four_speed_tick(&c,zero,1031));
    assert(c.fault&FOUR_SPEED_FAULT_TIMING);
    checked+=3;
}

static void save(const char *path,const four_link_bench_status_t *s)
{
    FILE *f=fopen(path,"wb");assert(f);
    assert(fwrite(s,1,sizeof(*s),f)==sizeof(*s));assert(fclose(f)==0);
}

static void test_wire(const char *idle_path,const char *fast_path)
{
    drive_bench_t b=bench(); four_link_bench_t link;
    four_link_bench_init(&link);b.group.stationary_ms=500;
    speed_bench_diagnostics_t d={.ms=1000,.ticks=1,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t t;four_link_bench_status_t s;
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&link,&b,&d,&t,&s));
    assert(s.tag==0x44475734 && sizeof(s)==156 && s.output_authority_version==1);
    if(idle_path)save(idle_path,&s);
    /* Snapshot only: no claim that a motor ever attained these speeds. */
    link.permitted=true;b.group.enabled=b.group.run_used=true;
    b.group.plan.applied_body.vx_m_s=1.256f;
    for(unsigned i=0;i<4;i++) {
        four_speed_controller_t *loop=&b.group.loop[i];
        const float cps=loop->config.wheel[i].counts_per_rev*400/60;
        loop->enabled=true;loop->selected=i;loop->run_direction=1;
        b.group.plan.forward_cps[i]=loop->wheel[i].requested_cps=cps;
        loop->wheel[i].ramped_cps=cps;
        b.group.raw_duty[i]=loop->wheel[i].raw_duty_permille=800*loop->config.wheel[i].motor_sign;
    }
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&link,&b,&d,&t,&s));
    assert(s.effective_cps[0]>7000 && s.raw_duty[1]==-800);
    if(fast_path)save(fast_path,&s);
    uint8_t frame[MC_MAX_RAW_FRAME_SIZE];size_t used;mc_frame_header_v1_t header;
    assert(mc_build_frame(MC_MSG_LINK_STATUS,MC_FLAG_RESPONSE,1,1,1000,&s,sizeof(s),frame,sizeof(frame),&used));
    assert(mc_validate_frame(frame,used,&header) && header.payload_length==156);
    checked+=2;
}

int main(int argc,char **argv)
{
    assert(argc==1 || argc==3);
    test_limits_and_planner();test_per_wheel_rpm_and_zero();test_feedback_reversal_and_deadlines();
    test_wire(argc==3?argv[1]:NULL,argc==3?argv[2]:NULL);
    printf("{\"passed\":true,\"static_assertions_groups\":%u,\"motor_target_rpm\":400,"
           "\"hardware_access\":false,\"vehicle_model\":false}\n",checked);
    return 0;
}
