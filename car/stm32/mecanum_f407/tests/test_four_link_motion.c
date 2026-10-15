#include "four_link_bench.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t seq, sid;
static four_link_bench_t link;
static drive_bench_t drive;
static int32_t counts[4];
static bool send(uint8_t type, uint16_t flags, const void *body, uint16_t size, uint32_t now)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE]; size_t n;
    assert(mc_build_frame(type, flags, ++seq, sid, now, body, size, raw, sizeof(raw), &n));
    four_link_request_t r;
    assert(four_link_bench_decode(raw, n, &r));
    return four_link_bench_process(&link, &drive, &r, now);
}
static void velocity(int x, int y, int z, bool enabled, uint32_t now)
{
    const mc_cmd_vel_v1_t v = {.vx_mm_s=x, .vy_mm_s=y, .wz_mrad_s=z, .timeout_ms=150};
    send(MC_MSG_CMD_VEL, enabled ? MC_FLAG_ENABLED : 0, &v, sizeof(v), now);
}
static void check_outputs(bool zero)
{
    uint32_t cc[8]; assert(four_drive_compares(&drive.group, cc));
    for (unsigned i=0; i<4; ++i) {
        assert(abs(drive.group.raw_duty[i]) <= 100);
        if (zero) assert(!cc[2*i] && !cc[2*i+1] && !drive.group.raw_duty[i]);
    }
}
static void tick(uint32_t now, bool progress)
{
    if (progress && drive.group.enabled) for (unsigned i=0;i<4;++i) {
        const float target=drive.group.plan.forward_cps[i];
        counts[i] += ((target>0)-(target<0))*drive.adopted.wheel[i].encoder_sign;
    }
    four_link_bench_deadline(&link, &drive, now);
    (void)drive_bench_tick(&drive, counts, now);
    four_link_bench_deadline(&link, &drive, now);
    check_outputs(!drive.group.enabled);
}
static uint32_t ready(uint32_t start)
{
    seq=0; sid=19; memset(counts,0,sizeof(counts));
    drive_bench_init(&drive, counts, start); four_link_bench_init(&link);
    assert(!drive.group.fault && send(MC_MSG_HELLO,0,NULL,0,start));
    for (unsigned dt=10;dt<=600;dt+=10) {
        tick(start+dt,false);
        if (dt%20==0) velocity(0,0,0,false,start+dt);
    }
    const mc_control_enable_v1_t enable={.enabled=1};
    send(MC_MSG_CONTROL_ENABLE,0,&enable,sizeof(enable),start+600);
    assert(link.permitted && !drive.group.enabled);
    return start+600;
}
static void all_axes_and_updates(void)
{
    const int axes[][3]={{50,0,0},{-50,0,0},{0,50,0},{0,-50,0},{0,0,200},{0,0,-200},
                         {25,25,0},{50,50,200},{1,0,0}};
    for(unsigned a=0;a<sizeof(axes)/sizeof(axes[0]);++a) {
        uint32_t t=ready(0);
        for (unsigned dt=10;dt<=400;dt+=10) {
            velocity(axes[a][0],axes[a][1],axes[a][2],true,t+dt);
            tick(t+dt,true);
            assert(drive.group.enabled && !drive.group.fault);
            for(unsigned i=0;i<4;++i) assert(fabsf(drive.group.plan.wheel_m_s[i])<=0.05f);
        }
        velocity(0,0,0,true,t+410); check_outputs(true);
        tick(t+410,false); assert(link.permitted && drive.group.enabled);
        velocity(0,0,0,false,t+420); check_outputs(true); assert(!link.permitted);
    }
    uint32_t t=ready(0);
    velocity(50,0,0,true,t+10); tick(t+10,true);
    velocity(20,0,0,true,t+20); tick(t+20,true);
    assert(drive.group.enabled && !link.transitioning);
    velocity(0,0,0,true,t+30); tick(t+30,false); check_outputs(true);
}
static void reversal_settle_and_budget(void)
{
    uint32_t t=ready(UINT32_MAX-800);
    velocity(50,0,0,true,t+10); tick(t+10,true);
    const uint32_t first=link.first_motion_ms;
    velocity(0,50,0,true,t+20); tick(t+20,false);
    assert(link.transitioning && !drive.group.enabled); check_outputs(true);
    for(unsigned dt=30;dt<520;dt+=10) {
        velocity(0,50,0,true,t+dt); tick(t+dt,false);
        assert(!drive.group.enabled); check_outputs(true);
    }
    velocity(0,50,0,true,t+520); tick(t+520,false);
    velocity(0,50,0,true,t+530); tick(t+530,true);
    assert(drive.group.enabled && !link.transitioning && link.first_motion_ms==first);
    assert(drive.group.plan.forward_cps[0]<0 && drive.group.plan.forward_cps[1]>0);
    /* No elapsed idle/settling interval or new target can extend the absolute budget. */
    for(uint32_t dt=540;dt<120010;dt+=10) {
        velocity(0,50,0,true,t+dt); tick(t+dt,true);
        assert(drive.group.enabled);
    }
    velocity(0,50,0,true,t+120010); tick(t+120010,false);
    assert(!link.permitted && drive.group.fault & FOUR_SPEED_FAULT_DURATION); check_outputs(true);
}
static void lease_and_latches(void)
{
    for(unsigned mode=0;mode<3;++mode) {
        uint32_t t=ready(0);
        velocity(mode==2 ? 0:50,0,0,true,t+10); tick(t+10,true);
        if(mode==1) {velocity(-50,0,0,true,t+20); tick(t+20,false);}
        uint32_t last=mode==1 ? 20:10;
        for(unsigned dt=last+10;dt<last+150;dt+=10) {
            send(MC_MSG_HEARTBEAT,0,NULL,0,t+dt); tick(t+dt,mode==0);
        }
        tick(t+last+150,false); check_outputs(true);
        assert(link.expired && !link.permitted && drive.group.fault & FOUR_SPEED_FAULT_COMMAND);
        velocity(50,0,0,true,t+last+150); check_outputs(true);
        sid++;seq=0;assert(send(MC_MSG_HELLO,0,NULL,0,t+last+160));
        assert(!link.permitted && drive.group.fault);check_outputs(true);
    }
}
static void rejection_and_decode(void)
{
    const int bad[][3]={{51,0,0},{0,-51,0},{0,0,201}};
    for(unsigned i=0;i<3;++i) {
        uint32_t t=ready(0);velocity(bad[i][0],bad[i][1],bad[i][2],true,t+10);
        assert(drive.group.fault==FOUR_LINK_REJECTED); check_outputs(true);
    }
    uint32_t t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,true);
    send(MC_MSG_ESTOP,0,NULL,0,t+20);check_outputs(true);
    assert(drive.group.fault==SPEED_BENCH_FAULT_ESTOP);
    t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,true);
    const mc_control_enable_v1_t disable={0};send(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),t+20);
    velocity(50,0,0,true,t+30);assert(!drive.group.enabled);check_outputs(true);
    t=ready(0);velocity(50,0,0,false,t+10);assert(drive.group.fault);check_outputs(true);
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t n;four_link_request_t r;
    mc_cmd_vel_v1_t v={.timeout_ms=151};
    assert(mc_build_frame(MC_MSG_CMD_VEL,0,1,1,0,&v,sizeof(v),raw,sizeof(raw),&n));
    assert(!four_link_bench_decode(raw,n,&r));
    v.timeout_ms=150;assert(mc_build_frame(MC_MSG_CMD_VEL,0,1,1,0,&v,sizeof(v),raw,sizeof(raw),&n));
    raw[n-1]^=1;assert(!four_link_bench_decode(raw,n,&r));
}
static void telemetry_and_feedback_faults(void)
{
    uint32_t t=ready(0);velocity(0,50,0,true,t+10);tick(t+10,true);
    speed_bench_diagnostics_t d={.ms=t+10};
    assert(four_drive_compares(&drive.group,d.compares));
    mc_wheel_telemetry_v1_t w;four_link_bench_status_t s;
    assert(four_link_bench_payloads(&link,&drive,&d,&w,&s));
    assert(s.tag==UINT32_C(0x314d5734) && s.body_milli[0]==0 && s.body_milli[1]>=49);
    d.compares[0]^=1;assert(!four_link_bench_payloads(&link,&drive,&d,&w,&s));
    t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,false);
    for(unsigned dt=20;dt<=810;dt+=10) {velocity(50,0,0,true,t+dt);tick(t+dt,false);}
    assert(drive.group.fault & FOUR_SPEED_FAULT_STALL);check_outputs(true);
    t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,true);
    velocity(50,0,0,true,t+50);assert(drive.group.fault & FOUR_SPEED_FAULT_TIMING);check_outputs(true);
}
static void sequence_and_session_guards(void)
{
    uint32_t t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,true);
    --seq;velocity(0,0,0,false,t+20);assert(drive.group.enabled); /* Duplicate ignored. */
    ++sid;velocity(0,0,0,false,t+20);assert(drive.group.enabled); /* Foreign ignored. */
    assert(send(MC_MSG_HELLO,0,NULL,0,t+30));check_outputs(true);
    assert(drive.group.fault==FOUR_LINK_REJECTED && link.motion_started);
    t=ready(0);seq=UINT32_MAX-1;link.last_sequence=UINT32_MAX-2;
    velocity(50,0,0,true,t+10);tick(t+10,true);
    velocity(40,0,0,true,t+20);tick(t+20,true);
    velocity(30,0,0,true,t+30);tick(t+30,true);
    assert(seq==1 && drive.group.enabled && !drive.group.fault);
    t=ready(0);
    const mc_control_enable_v1_t clear={.clear_latched_faults=1};
    send(MC_MSG_CONTROL_ENABLE,0,&clear,sizeof(clear),t+10);check_outputs(true);
    assert(drive.group.fault==FOUR_LINK_REJECTED);
    /* Direction-settling must not accept an arbitrary fixed delay as standstill. */
    t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,true);
    velocity(-50,0,0,true,t+20);tick(t+20,false);
    for(unsigned dt=30;dt<=800;dt+=10) {
        counts[0]++;velocity(-50,0,0,true,t+dt);tick(t+dt,false);
        assert(!drive.group.enabled && link.transitioning);check_outputs(true);
    }
}
int main(void)
{
    all_axes_and_updates();reversal_settle_and_budget();lease_and_latches();
    rejection_and_decode();telemetry_and_feedback_faults();sequence_and_session_guards();
    puts("PASS streaming motion: axes, updates, zero, settle, 120s/wrap, lease, estop, limits, feedback; OFFLINE ONLY");
}
