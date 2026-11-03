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
#ifdef FOUR_LINK_GROUNDB
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_LINEAR_MM_S 150
#define TEST_STATUS_TAG UINT32_C(0x42475734)
#elif defined(FOUR_LINK_GROUNDA)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_LINEAR_MM_S 100
#define TEST_STATUS_TAG UINT32_C(0x41475734)
#elif defined(FOUR_LINK_GROUND9)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_LINEAR_MM_S 200
#define TEST_STATUS_TAG UINT32_C(0x39475734)
#elif defined(FOUR_LINK_GROUND8)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_LINEAR_MM_S 200
#define TEST_STATUS_TAG UINT32_C(0x38475734)
#elif defined(FOUR_LINK_GROUND7)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_STATUS_TAG UINT32_C(0x37475734)
#elif defined(FOUR_LINK_GROUND6)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_STATUS_TAG UINT32_C(0x36475734)
#elif defined(FOUR_LINK_GROUND5)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_STATUS_TAG UINT32_C(0x35475734)
#elif defined(FOUR_LINK_GROUND4)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_STATUS_TAG UINT32_C(0x34475734)
#elif defined(FOUR_LINK_GROUND3)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_STATUS_TAG UINT32_C(0x33475734)
#elif defined(FOUR_LINK_GROUND2)
enum { TEST_DUTY_LIMIT = 350, TEST_STARTUP_DUTY = 300 };
#define TEST_STATUS_TAG UINT32_C(0x32475734)
#elif defined(FOUR_LINK_GROUND)
enum { TEST_DUTY_LIMIT = 200, TEST_STARTUP_DUTY = 150 };
#define TEST_STATUS_TAG UINT32_C(0x31475734)
#else
enum { TEST_DUTY_LIMIT = 100, TEST_STARTUP_DUTY = 80 };
#define TEST_STATUS_TAG UINT32_C(0x314d5734)
#endif
#ifndef TEST_LINEAR_MM_S
#define TEST_LINEAR_MM_S 50
#endif
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
    const mc_cmd_vel_v1_t v = {
        .vx_mm_s=x, .vy_mm_s=y, .wz_mrad_s=z,
        .timeout_ms=FOUR_DRIVE_COMMAND_TIMEOUT_MS,
    };
    send(MC_MSG_CMD_VEL, enabled ? MC_FLAG_ENABLED : 0, &v, sizeof(v), now);
}
static void check_outputs(bool zero)
{
    uint32_t cc[8]; assert(four_drive_compares(&drive.group, cc));
    for (unsigned i=0; i<4; ++i) {
        assert(abs(drive.group.raw_duty[i]) <= TEST_DUTY_LIMIT);
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
#if defined(FOUR_LINK_GROUND6) || defined(FOUR_LINK_GROUND7) || \
    defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9) || \
    defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
    const int smallest=20;
#else
    const int smallest=1;
#endif
    const int axes[][3]={{50,0,0},{-50,0,0},{0,50,0},{0,-50,0},{0,0,200},{0,0,-200},
                         {25,25,0},{50,50,200},{smallest,0,0}};
    for(unsigned a=0;a<sizeof(axes)/sizeof(axes[0]);++a) {
        uint32_t t=ready(0);
        for (unsigned dt=10;dt<=400;dt+=10) {
            velocity(axes[a][0],axes[a][1],axes[a][2],true,t+dt);
            tick(t+dt,true);
            assert(drive.group.enabled && !drive.group.fault);
            for(unsigned i=0;i<4;++i)
                assert(fabsf(drive.group.plan.wheel_m_s[i])<=FOUR_DRIVE_MAX_WHEEL_M_S);
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
    for(uint32_t dt=540;dt<FOUR_LINK_MOTION_LIMIT_MS+10U;dt+=10) {
        velocity(0,50,0,true,t+dt); tick(t+dt,true);
        assert(drive.group.enabled);
    }
    velocity(0,50,0,true,t+FOUR_LINK_MOTION_LIMIT_MS+10U);
    tick(t+FOUR_LINK_MOTION_LIMIT_MS+10U,false);
    assert(!link.permitted && drive.group.fault & FOUR_SPEED_FAULT_DURATION); check_outputs(true);
}
static void lease_and_latches(void)
{
    for(unsigned mode=0;mode<3;++mode) {
        uint32_t t=ready(0);
        velocity(mode==2 ? 0:50,0,0,true,t+10); tick(t+10,true);
        if(mode==1) {velocity(-50,0,0,true,t+20); tick(t+20,false);}
        uint32_t last=mode==1 ? 20:10;
        for(unsigned dt=last+10;dt<last+FOUR_DRIVE_COMMAND_TIMEOUT_MS;dt+=10) {
            send(MC_MSG_HEARTBEAT,0,NULL,0,t+dt); tick(t+dt,mode==0);
        }
        tick(t+last+FOUR_DRIVE_COMMAND_TIMEOUT_MS,false); check_outputs(true);
        assert(link.expired && !link.permitted && drive.group.fault & FOUR_SPEED_FAULT_COMMAND);
        velocity(50,0,0,true,t+last+FOUR_DRIVE_COMMAND_TIMEOUT_MS); check_outputs(true);
        sid++;seq=0;assert(send(MC_MSG_HELLO,0,NULL,0,
            t+last+FOUR_DRIVE_COMMAND_TIMEOUT_MS+10));
        assert(!link.permitted && drive.group.fault);check_outputs(true);
    }
}
static void idle_expiry_and_explicit_disable_are_recoverable(void)
{
    seq=0; sid=41; memset(counts,0,sizeof(counts));
    drive_bench_init(&drive,counts,0); four_link_bench_init(&link);
    assert(send(MC_MSG_HELLO,0,NULL,0,0));
    velocity(0,0,0,false,10);
    for (uint32_t now=20; now<=FOUR_DRIVE_COMMAND_TIMEOUT_MS+10U; now+=10)
        tick(now,false);
    assert(link.expired && !link.permitted && !link.fault && !drive.group.fault);
    check_outputs(true);
    ++sid; seq=0; assert(send(MC_MSG_HELLO,0,NULL,0,
        FOUR_DRIVE_COMMAND_TIMEOUT_MS+20U));
    assert(!link.expired && !link.fault && !drive.group.fault);

    uint32_t t=ready(1000);
    velocity(50,0,0,true,t+10); tick(t+10,true);
    const mc_control_enable_v1_t disable={0};
    send(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),t+20);
    for (uint32_t dt=30; dt<=FOUR_DRIVE_COMMAND_TIMEOUT_MS+20U; dt+=10)
        tick(t+dt,false);
    assert(link.motion_started && link.expired && !link.permitted);
    assert(!link.fault && !drive.group.fault); check_outputs(true);
}
static void rejection_and_decode(void)
{
    const int bad[][3]={{TEST_LINEAR_MM_S+1,0,0},{0,-TEST_LINEAR_MM_S-1,0},{0,0,201}};
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
    mc_cmd_vel_v1_t v={.timeout_ms=FOUR_DRIVE_COMMAND_TIMEOUT_MS+1U};
    assert(mc_build_frame(MC_MSG_CMD_VEL,0,1,1,0,&v,sizeof(v),raw,sizeof(raw),&n));
    assert(!four_link_bench_decode(raw,n,&r));
    v.timeout_ms=FOUR_DRIVE_COMMAND_TIMEOUT_MS;
    assert(mc_build_frame(MC_MSG_CMD_VEL,0,1,1,0,&v,sizeof(v),raw,sizeof(raw),&n));
    raw[n-1]^=1;assert(!four_link_bench_decode(raw,n,&r));
}
static void telemetry_and_feedback_faults(void)
{
    uint32_t t=ready(0);velocity(0,50,0,true,t+10);tick(t+10,true);
    speed_bench_diagnostics_t d={.ms=t+10};
    assert(four_drive_compares(&drive.group,d.compares));
    mc_wheel_telemetry_v1_t w;four_link_bench_status_t s;
    assert(four_link_bench_payloads(&link,&drive,&d,&w,&s));
    assert(s.tag==TEST_STATUS_TAG && s.body_milli[0]==0 && s.body_milli[1]>=49);
    d.compares[0]^=1;assert(!four_link_bench_payloads(&link,&drive,&d,&w,&s));
    t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,false);
    for(unsigned dt=20;dt<=810;dt+=10) {velocity(50,0,0,true,t+dt);tick(t+dt,false);}
    assert(drive.group.fault & FOUR_SPEED_FAULT_STALL);check_outputs(true);
    t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,true);
    velocity(50,0,0,true,t+50);assert(drive.group.fault & FOUR_SPEED_FAULT_TIMING);check_outputs(true);
}
static void startup_profile_is_bounded(void)
{
    uint32_t t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,false);
#if defined(FOUR_LINK_GROUND7) || defined(FOUR_LINK_GROUND8) || \
    defined(FOUR_LINK_GROUND9) || defined(FOUR_LINK_GROUNDA) || \
    defined(FOUR_LINK_GROUNDB)
    int previous=0;
    /* No encoder progress: breakaway rises by a bounded slew, never jumps to
     * 30%, and faults closed at its absolute launch deadline. */
    for(unsigned dt=20;dt<10U+FOUR_DRIVE_STARTUP_MS;dt+=10) {
        velocity(50,0,0,true,t+dt);tick(t+dt,false);
        assert(drive.group.enabled && drive.group.startup_mask==15U);
        const int duty=abs(drive.group.raw_duty[0]);
        assert(duty>=previous && duty<=TEST_STARTUP_DUTY);
        assert(duty-previous<=20);
        previous=duty;
    }
    velocity(50,0,0,true,t+10U+FOUR_DRIVE_STARTUP_MS);
    tick(t+10U+FOUR_DRIVE_STARTUP_MS,false);
    assert(!drive.group.enabled && drive.group.fault==FOUR_SPEED_FAULT_STALL);
    check_outputs(true);
#else
    /* Arming owns the first sample; compensation begins on the next 10ms tick. */
    for(unsigned dt=20;dt<10U+FOUR_DRIVE_STARTUP_MS;dt+=10) {
        velocity(50,0,0,true,t+dt);tick(t+dt,false);
        for(unsigned i=0;i<4;++i) {
            assert(abs(drive.group.raw_duty[i])==TEST_STARTUP_DUTY);
            assert(drive.group.startup_mask&(1U<<i));
        }
    }
    velocity(50,0,0,true,t+10U+FOUR_DRIVE_STARTUP_MS);
    tick(t+10U+FOUR_DRIVE_STARTUP_MS,false);
    assert(drive.group.startup_mask==0);
#if defined(FOUR_LINK_GROUND3) || defined(FOUR_LINK_GROUND4) || \
    defined(FOUR_LINK_GROUND5) || defined(FOUR_LINK_GROUND6)
    for(unsigned i=0;i<4;++i)
        assert(abs(drive.group.raw_duty[i])==FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE);
    for(unsigned dt=20U+FOUR_DRIVE_STARTUP_MS;dt<=200U+FOUR_DRIVE_STARTUP_MS;dt+=10) {
        velocity(50,0,0,true,t+dt);tick(t+dt,false);
        for(unsigned i=0;i<4;++i) {
            assert(abs(drive.group.raw_duty[i])>=FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE);
            assert(abs(drive.group.raw_duty[i])<=TEST_DUTY_LIMIT);
        }
    }
#endif
    check_outputs(false);
#endif
}
static void streamed_ramp_keeps_startup_assist(void)
{
#ifdef FOUR_LINK_GROUND5
    const int axes[][3]={{2,0,0},{-2,0,0},{0,2,0},{0,-2,0},{0,0,10},{0,0,-10}};
    for(unsigned a=0;a<sizeof(axes)/sizeof(axes[0]);++a) {
        uint32_t t=ready(0);
        velocity(axes[a][0],axes[a][1],axes[a][2],true,t+10);
        tick(t+10,false); /* arming owns this sample */
        for(unsigned dt=20;dt<=170;dt+=10) {
            /* Stream a growing target while every wheel produces one valid
             * count per sample.  Filtered speed exceeds the early target, but
             * startup assist must remain until the 16-count budget is met. */
            const int scale=(int)(dt/10);
            velocity(axes[a][0]*scale,axes[a][1]*scale,axes[a][2]*scale,true,t+dt);
            tick(t+dt,true);
            assert(drive.group.enabled && !drive.group.fault);
            if(dt<170) {
                assert(drive.group.startup_mask==15U);
                for(unsigned i=0;i<4;++i)
                    assert(abs(drive.group.raw_duty[i])==TEST_STARTUP_DUTY);
            } else assert(drive.group.startup_mask==0U);
        }
    }
#endif
}
static void cancellation_wheels_below_floor_stay_quiet(void)
{
#if defined(FOUR_LINK_GROUND6) || defined(FOUR_LINK_GROUND7) || \
    defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9) || \
    defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
    uint32_t t=ready(0);
    /* Captured from the real-car failure: wheels 1 and 4 are cancellation
     * terms (~6 cps), while wheels 2 and 3 carry the diagonal motion. */
    velocity(-24,-24,-7,true,t+10); tick(t+10,false);
    assert(drive.group.enabled && !drive.group.fault);
    assert(drive.group.plan.forward_cps[0]==0.0f);
    assert(drive.group.plan.forward_cps[3]==0.0f);
    assert(fabsf(drive.group.plan.forward_cps[1])>=FOUR_DRIVE_MIN_TRACKABLE_CPS);
    assert(fabsf(drive.group.plan.forward_cps[2])>=FOUR_DRIVE_MIN_TRACKABLE_CPS);
    for(unsigned dt=20;dt<=1200;dt+=10) {
        velocity(-24,-24,-7,true,t+dt); tick(t+dt,true);
        assert(drive.group.enabled && !drive.group.fault);
        assert(drive.group.raw_duty[0]==0 && drive.group.raw_duty[3]==0);
    }
    /* A command whose four wheels are all below the floor is an armed zero,
     * not a rejected start and not a consumed motion allowance. */
    t=ready(2000); velocity(1,0,0,true,t+10); tick(t+10,false);
    assert(link.permitted && !drive.group.enabled && !drive.group.fault);
#endif
}
static void subthreshold_start_waits_without_consuming_run(void)
{
#if defined(FOUR_LINK_GROUND6) || defined(FOUR_LINK_GROUND7) || \
    defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9) || \
    defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
    uint32_t t=ready(0);
    for(unsigned dt=10;dt<=300;dt+=10) {
        /* Exact near-start command captured before the 4WG5 lurch.  Its
         * largest wheel demand is about 93 cps, below the start threshold. */
        velocity(-6,-6,32,true,t+dt); tick(t+dt,false);
        assert(link.permitted && !link.motion_started);
        assert(!drive.group.enabled && !drive.group.run_used && !drive.group.fault);
        check_outputs(true);
    }
    velocity(20,0,0,true,t+310); tick(t+310,false);
    assert(link.motion_started && drive.group.enabled && drive.group.run_used);
    assert(!drive.group.fault);
#endif
}
static void ground7_low_wheels_do_not_launch_or_stall(void)
{
#if defined(FOUR_LINK_GROUND7) || defined(FOUR_LINK_GROUND8) || \
    defined(FOUR_LINK_GROUND9) || defined(FOUR_LINK_GROUNDA) || \
    defined(FOUR_LINK_GROUNDB)
    uint32_t t=ready(0);
    /* x=20,y=15 mm/s produces two ~28 cps wheels and two ~195 cps wheels.
     * Only the trackable pair may own breakaway. */
    velocity(20,15,0,true,t+10);tick(t+10,false);
    velocity(20,15,0,true,t+20);tick(t+20,false);
    assert(drive.group.enabled && !drive.group.fault);
    assert(fabsf(drive.group.plan.forward_cps[0])<100.0f);
    assert(fabsf(drive.group.plan.forward_cps[3])<100.0f);
    assert(fabsf(drive.group.plan.forward_cps[1])>=100.0f);
    assert(fabsf(drive.group.plan.forward_cps[2])>=100.0f);
    assert((drive.group.startup_mask&0x9U)==0U);
    assert((drive.group.startup_mask&0x6U)==0x6U);
    /* Give progress only to the high-target pair.  The low pair remains at
     * zero encoder counts for over the legacy 800 ms stall interval. */
    for(unsigned dt=30;dt<=1200;dt+=10) {
        for(unsigned i=1;i<=2;++i) counts[i] +=
            ((drive.group.plan.forward_cps[i]>0)-(drive.group.plan.forward_cps[i]<0))*
            drive.adopted.wheel[i].encoder_sign;
        velocity(20,15,0,true,t+dt);tick(t+dt,false);
        assert(drive.group.enabled && !drive.group.fault);
        assert((drive.group.startup_mask&0x9U)==0U);
        assert(abs(drive.group.raw_duty[0])<drive.adopted.stall_duty_floor_permille);
        assert(abs(drive.group.raw_duty[3])<drive.adopted.stall_duty_floor_permille);
        assert(drive.group.loop[0].wheel[0].integral_permille==0.0f);
        assert(drive.group.loop[3].wheel[3].integral_permille==0.0f);
    }
#endif
}
static void ground8_accepts_only_the_explicit_high_speed_envelope(void)
{
#if defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9)
    uint32_t t=ready(0);
    velocity(200,0,0,true,t+10); tick(t+10,false);
    assert(drive.group.enabled && !drive.group.fault);
    assert(fabsf(drive.group.plan.applied_body.vx_m_s-0.20f)<0.0001f);
    for(unsigned i=0;i<4;++i) {
        assert(fabsf(drive.group.plan.wheel_m_s[i]-0.20f)<0.0001f);
        assert(fabsf(drive.group.plan.forward_cps[i])>1110.0f);
        assert(fabsf(drive.group.plan.forward_cps[i])<1130.0f);
        assert(fabsf(drive.group.plan.forward_cps[i])<=FOUR_DRIVE_TARGET_LIMIT_CPS);
    }
    velocity(201,0,0,true,t+20);
    assert(drive.group.fault==FOUR_LINK_REJECTED); check_outputs(true);
#elif defined(FOUR_LINK_GROUNDB)
    uint32_t t=ready(0);
    velocity(150,0,0,true,t+10); tick(t+10,false);
    assert(drive.group.enabled && !drive.group.fault);
    assert(fabsf(drive.group.plan.applied_body.vx_m_s-0.15f)<0.0001f);
    for(unsigned i=0;i<4;++i) {
        assert(fabsf(drive.group.plan.wheel_m_s[i]-0.15f)<0.0001f);
        assert(fabsf(drive.group.plan.forward_cps[i])>830.0f);
        assert(fabsf(drive.group.plan.forward_cps[i])<850.0f);
        assert(fabsf(drive.group.plan.forward_cps[i])<=FOUR_DRIVE_TARGET_LIMIT_CPS);
    }
    velocity(151,0,0,true,t+20);
    assert(drive.group.fault==FOUR_LINK_REJECTED); check_outputs(true);
#endif
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
    assert(drive.group.fault==FOUR_LINK_REJECTED); /* Permit is still owned. */
    /* Direction-settling must not accept an arbitrary fixed delay as standstill. */
    t=ready(0);velocity(50,0,0,true,t+10);tick(t+10,true);
    velocity(-50,0,0,true,t+20);tick(t+20,false);
    for(unsigned dt=30;dt<=800;dt+=10) {
        counts[0]++;velocity(-50,0,0,true,t+dt);tick(t+dt,false);
        assert(!drive.group.enabled && link.transitioning);check_outputs(true);
    }
}

static void safe_rearm_requires_zero_fault_free_standstill(void)
{
#if defined(FOUR_LINK_GROUND9) || defined(FOUR_LINK_GROUNDA) || \
    defined(FOUR_LINK_GROUNDB)
    uint32_t t=ready(0);
    velocity(50,0,0,true,t+10); tick(t+10,true);
    const mc_control_enable_v1_t disable={0};
    send(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),t+20);
    assert(link.motion_started && drive.group.run_used && !link.permitted);
    for (uint32_t dt=30; dt<=530; dt+=10) tick(t+dt,false);
    assert(drive.group.stationary_ms==500 && !drive.group.fault && !link.fault);

    ++sid; seq=0; assert(send(MC_MSG_HELLO,0,NULL,0,t+540));
    for (uint32_t dt=550; dt<=1150; dt+=10) {
        tick(t+dt,false);
        if (dt%20==0) velocity(0,0,0,false,t+dt);
    }
    const mc_control_enable_v1_t rearm={.clear_latched_faults=1};
    send(MC_MSG_CONTROL_ENABLE,0,&rearm,sizeof(rearm),t+1150);
    assert(!link.motion_started && !drive.group.run_used);
    assert(!link.fault && !drive.group.fault && drive.group.stationary_ms==500);
    /* The host repeats low-rate state datagrams against UDP loss.  A
     * successfully rearmed, still-disabled zero state must be idempotent. */
    for (uint32_t dt=1151; dt<=1154; ++dt)
        send(MC_MSG_CONTROL_ENABLE,0,&rearm,sizeof(rearm),t+dt);
    assert(!link.motion_started && !drive.group.run_used);
    assert(!link.fault && !drive.group.fault && drive.group.stationary_ms==500);

    const mc_control_enable_v1_t enable={.enabled=1};
    send(MC_MSG_CONTROL_ENABLE,0,&enable,sizeof(enable),t+1160);
    assert(link.permitted);
    velocity(50,0,0,true,t+1170); tick(t+1170,true);
    assert(link.motion_started && drive.group.run_used && drive.group.enabled);
    assert(!link.fault && !drive.group.fault);

    /* A real fault remains non-clearable through the same command. */
    send(MC_MSG_ESTOP,0,NULL,0,t+1180);
    ++sid; seq=0; assert(send(MC_MSG_HELLO,0,NULL,0,t+1190));
    for (uint32_t dt=1200; dt<=1800; dt+=10) {
        tick(t+dt,false);
        if (dt%20==0) velocity(0,0,0,false,t+dt);
    }
    send(MC_MSG_CONTROL_ENABLE,0,&rearm,sizeof(rearm),t+1800);
    assert(link.fault && drive.group.fault);
    check_outputs(true);
#endif
}
int main(void)
{
    all_axes_and_updates();reversal_settle_and_budget();lease_and_latches();
    idle_expiry_and_explicit_disable_are_recoverable();
    rejection_and_decode();telemetry_and_feedback_faults();startup_profile_is_bounded();
    streamed_ramp_keeps_startup_assist();
    cancellation_wheels_below_floor_stay_quiet();
    subthreshold_start_waits_without_consuming_run();
    ground7_low_wheels_do_not_launch_or_stall();
    ground8_accepts_only_the_explicit_high_speed_envelope();
    sequence_and_session_guards();
    safe_rearm_requires_zero_fault_free_standstill();
    puts("PASS streaming motion: axes, updates, zero, settle, run-budget/wrap, lease, estop, limits, feedback; OFFLINE ONLY");
}
