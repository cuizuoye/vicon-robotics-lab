/* Native only: real C control/protocol + a SYNTHETIC inertial motor plant.
 * Motor feedback follows PWM, not a copy of the requested wheel target.
 * No serial ports, sockets, device access, or flashing exists in this test.
 */
#include "four_link_bench.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static drive_bench_t d;
static four_link_bench_t link_state;
static int32_t counts[4];
static float speed[4], fraction[4], passive[4];
static uint32_t now_ms, sequence, session=19, first_motion;
static int command[3], old_duty[4], last_nonzero_sign[4];
static unsigned neutral_time[4], polarity_changes[4], tests_run;
static float old_effective[4], max_effective_step;
static unsigned max_output_step;
static float plant_tau=.06f, plant_static=40.0f, plant_slope=.30f;
static const char *dump_status_path;
static const float PI=3.14159265358979323846f;

static void add_count(unsigned wheel, int32_t delta)
{
    uint32_t bits=(uint32_t)counts[wheel]+(uint32_t)delta;
    memcpy(&counts[wheel], &bits, sizeof(bits));
}

static bool frame(uint8_t type, uint16_t flags, const void *payload, uint16_t length)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE]; size_t used;
    assert(mc_build_frame(type,flags,++sequence,session,now_ms,payload,length,raw,sizeof(raw),&used));
    four_link_request_t request;
    assert(four_link_bench_decode(raw,used,&request));
    return four_link_bench_process(&link_state,&d,&request,now_ms);
}

static void velocity(bool enabled)
{
    const mc_cmd_vel_v1_t v={.vx_mm_s=command[0],.vy_mm_s=command[1],
        .wz_mrad_s=command[2],.timeout_ms=600};
    frame(MC_MSG_CMD_VEL,enabled?MC_FLAG_ENABLED:0,&v,sizeof(v));
}

static void safety_assertions(void)
{
    uint32_t compares[8];
    assert(four_drive_compares(&d.group,compares));
    for(unsigned i=0;i<4;i++) {
        const four_speed_wheel_state_t *w=&d.group.loop[i].wheel[i];
        int duty=d.group.raw_duty[i]*d.adopted.wheel[i].motor_sign;
        int direction=(duty>0)-(duty<0);
        assert(abs(duty)<=350);
        assert(compares[2*i]<=100 && compares[2*i+1]<=100);
        assert(fabsf(w->requested_cps)<=1200.001f && fabsf(w->ramped_cps)<=1200.001f);
        assert(fabsf(d.group.plan.wheel_m_s[i])<=.20001f);
        if(!direction) neutral_time[i]+=10;
        else {
            if(last_nonzero_sign[i] && direction!=last_nonzero_sign[i]) {
                assert(neutral_time[i]>=FOUR_CONTINUOUS_NEUTRAL_MS);
                polarity_changes[i]++;
            }
            last_nonzero_sign[i]=direction;
            neutral_time[i]=0;
        }
        unsigned change=(unsigned)abs(duty-old_duty[i]);
        if(duty && old_duty[i]) { assert(change<=20); if(change>max_output_step)max_output_step=change; }
        float delta=fabsf(w->ramped_cps-old_effective[i]);
        /* Explicit zero/disable/ESTOP is an immediate power-removal override. */
        if(w->requested_cps!=0 && d.group.enabled) {
            assert(delta<=8.01f);
            if(delta>max_effective_step)max_effective_step=delta;
        }
        old_duty[i]=duty; old_effective[i]=w->ramped_cps;
        if(!d.group.enabled) assert(duty==0 && compares[2*i]==0 && compares[2*i+1]==0);
    }
}

static void sample(void)
{
    four_link_bench_deadline(&link_state,&d,now_ms);
    drive_bench_tick(&d,counts,now_ms);
    four_link_bench_deadline(&link_state,&d,now_ms);
    safety_assertions();
}

static void reset_at(uint32_t start)
{
    memset(counts,0,sizeof(counts)); memset(speed,0,sizeof(speed));
    memset(fraction,0,sizeof(fraction)); memset(passive,0,sizeof(passive));
    memset(old_duty,0,sizeof(old_duty)); memset(old_effective,0,sizeof(old_effective));
    memset(last_nonzero_sign,0,sizeof(last_nonzero_sign)); memset(neutral_time,0,sizeof(neutral_time));
    memset(command,0,sizeof(command)); sequence=0;session=19;now_ms=start;first_motion=0;
    drive_bench_init(&d,counts,now_ms); four_link_bench_init(&link_state);
    assert(!d.group.fault && frame(MC_MSG_HELLO,0,NULL,0));
    for(unsigned i=0;i<60;i++) {
        now_ms+=10; sample();
        if(i%2==1)velocity(false);
    }
    const mc_control_enable_v1_t enable={.enabled=1};
    frame(MC_MSG_CONTROL_ENABLE,0,&enable,sizeof(enable));
    assert(link_state.permitted && !d.group.enabled && !d.group.fault);
}

static void model_step(bool renew)
{
    now_ms+=10;
    if(renew)velocity(true);
    for(unsigned i=0;i<4;i++) {
        int u=d.group.raw_duty[i]*d.adopted.wheel[i].motor_sign;
        float equilibrium=(u>0?1.0f:-1.0f)*fmaxf(0,(abs(u)-plant_static)/plant_slope);
        equilibrium+=passive[i];
        speed[i]+=(equilibrium-speed[i])*(.01f/(plant_tau+.01f));
        fraction[i]+=speed[i]*.01f;
        int delta=(int)truncf(fraction[i]); fraction[i]-=delta;
        add_count(i,delta*d.adopted.wheel[i].encoder_sign);
    }
    sample();
    if(link_state.motion_started) {
        if(!first_motion) first_motion=link_state.first_motion_ms;
        assert(link_state.first_motion_ms==first_motion);
    }
}

static void drive_for(int x,int y,int z,unsigned ms)
{
    command[0]=x;command[1]=y;command[2]=z;
    for(unsigned t=0;t<ms;t+=10) {
        model_step(true);
        assert(link_state.permitted && d.group.enabled && !d.group.fault && !link_state.transitioning);
    }
}

static void axes_and_diagonals(void)
{
    reset_at(0);
    const int axes[][3]={{50,0,0},{0,50,0},{-50,0,0},{0,-50,0},
        {35,35,0},{-35,35,0},{-35,-35,0},{35,-35,0},{0,0,100},{0,0,-100}};
    for(unsigned a=0;a<sizeof(axes)/sizeof(axes[0]);a++) {
        drive_for(axes[a][0],axes[a][1],axes[a][2],1800);
        float raw[4];
        for(unsigned i=0;i<4;i++)raw[i]=speed[i]*d.adopted.wheel[i].encoder_sign;
        mecanum_body_velocity_t body;
        assert(mecanum_drive_body_from_raw_cps(&d.group.geometry,&d.adopted,raw,&body));
        assert(fabsf(body.vx_m_s-axes[a][0]*.001f)<.025f);
        assert(fabsf(body.vy_m_s-axes[a][1]*.001f)<.025f);
        assert(fabsf(body.wz_rad_s-axes[a][2]*.001f)<.12f);
    }
    for(unsigned i=0;i<4;i++)assert(polarity_changes[i]>0);
}

static void one_wheel_reverses_without_chassis_stop(void)
{
    reset_at(0); drive_for(50,0,0,1600);
    command[0]=30;command[1]=40;command[2]=100;
    bool changed=false;
    for(unsigned t=0;t<1800;t+=10) {
        model_step(true);
        assert(link_state.permitted && d.group.enabled && !d.group.fault && !link_state.transitioning);
        assert(d.group.raw_duty[1]!=0 && d.group.raw_duty[2]!=0);
        if(d.group.raw_duty[0]*d.adopted.wheel[0].motor_sign<0)changed=true;
    }
    assert(changed && d.group.loop[1].run_direction==1 && d.group.loop[2].run_direction==1);
}

static void circular_translation_and_yaw(void)
{
    reset_at(0);
    for(unsigned t=0;t<24000;t+=10) {
        float phase=2*PI*t/8000.0f;
        command[0]=(int)lroundf(50*cosf(phase));command[1]=(int)lroundf(50*sinf(phase));
        command[2]=(int)lroundf(70*sinf(.7f*phase));
        model_step(true);
        assert(link_state.permitted && !d.group.fault && !link_state.transitioning);
    }
}

static void full_envelope_reversals_and_cancel(void)
{
    reset_at(0);
    drive_for(150,150,200,3500);
    drive_for(-150,-150,-200,4000);
    drive_for(150,-150,200,4000);
    /* Latest command may cancel a per-wheel reversal; it must not revive
     * old integral or require the whole chassis to stop/rearm. */
    command[0]=-150;command[1]=150;command[2]=-200;
    for(unsigned t=0;t<300;t+=10) {model_step(true);assert(!d.group.fault && link_state.permitted);}
    drive_for(150,-150,200,2500);
}

static void zero_bands_and_zero_crossing_jitter(void)
{
    reset_at(0); drive_for(50,0,0,1000);
    for(unsigned j=0;j<1000;j++) {
        command[0]=35;command[1]=-35+(j%4==0?5:j%4==1?-5:0);command[2]=0;
        model_step(true);assert(!d.group.fault && link_state.permitted && d.group.enabled);
    }
    memset(command,0,sizeof(command));model_step(true);
    for(unsigned i=0;i<4;i++)assert(d.group.raw_duty[i]==0);
    assert(link_state.permitted);
}

static void seeded_random_changes(void)
{
    for(unsigned seed=1;seed<=8;seed++) {
        reset_at(seed*1000000U); uint32_t state=seed;
        command[0]=50;
        for(unsigned t=0;t<10000;t+=10) {
            state=state*1664525U+1013904223U;
            unsigned axis=state%3;
            int delta=(int)((state>>8)%7)-3;
            command[axis]+=delta;
            int limit=axis==2?100:50;
            if(command[axis]>limit)command[axis]=limit;
            if(command[axis]<-limit)command[axis]=-limit;
            model_step(true);
            assert(link_state.permitted && !d.group.fault && !link_state.transitioning);
        }
    }
}

static void inertia_and_static_load_sweep(void)
{
    const float inertia[]={.03f,.06f,.12f,.20f};
    const float friction[]={30.0f,60.0f,90.0f};
    for(unsigned i=0;i<4;i++)for(unsigned j=0;j<3;j++) {
        plant_tau=inertia[i];plant_static=friction[j];plant_slope=.25f+.05f*j;
        reset_at(0);drive_for(50,0,0,2200);drive_for(-20,40,80,2500);
        drive_for(35,-35,-100,2500);
    }
    plant_tau=.06f;plant_static=40.0f;plant_slope=.30f;
}

static void old_failure_and_passive_low_speed(void)
{
    reset_at(0);
    command[0]=32;command[1]=-39; now_ms+=10;velocity(true);
    add_count(0,1);add_count(1,1);add_count(2,-1);add_count(3,-1);sample();
    assert(!d.group.fault);
    command[0]=36;command[1]=-34; now_ms+=10;velocity(true);add_count(2,1);sample();
    assert(!d.group.fault && d.group.raw_duty[2]==0);
    command[0]=31;command[1]=-39;now_ms+=10;velocity(true);add_count(2,1);sample();
    assert(!d.group.fault && d.group.enabled && link_state.permitted);
    /* The low-torque cancellation wheel may roll either way; this does not
     * manufacture correct-direction progress for meaningful-drive stall. */
    reset_at(0); drive_for(50,0,0,1000);
    command[0]=36;command[1]=-34; passive[1]=60;passive[2]=-60;
    for(unsigned t=0;t<2000;t+=10) {model_step(true);assert(!d.group.fault && link_state.permitted);}
}

static four_speed_controller_t wheel;
static int32_t wheel_counts[4];
static uint32_t wheel_now;

static void wheel_ready(void)
{
    int32_t zero[4]={0}; drive_bench_t template;
    drive_bench_init(&template,zero,0);
    memset(wheel_counts,0,sizeof(wheel_counts));wheel_now=1000;
    assert(four_speed_init(&wheel,&template.adopted));
    assert(four_speed_arm(&wheel,2,wheel_counts,wheel_now));
    assert(four_speed_set_target(&wheel,400,wheel_now));
    for(unsigned t=0;t<700;t+=10) {
        wheel_now+=10;wheel_counts[2]+=4;
        assert(four_speed_set_target(&wheel,400,wheel_now));
        assert(four_speed_tick(&wheel,wheel_counts,wheel_now));
    }
    assert(wheel.wheel[2].ramped_cps==400 && abs(wheel.wheel[2].raw_duty_permille)>=80);
}

static bool wheel_sample(int target,int delta,unsigned dt)
{
    wheel_now+=dt;wheel_counts[2]+=delta;
    if(!four_speed_set_target(&wheel,(float)target,wheel_now))return false;
    return four_speed_tick(&wheel,wheel_counts,wheel_now);
}

static void one_count_noise_not_a_fault(void)
{
    wheel_ready(); assert(wheel_sample(400,-1,10));
    assert(wheel.opposite_counts==1 && !wheel.first_feedback_fault.valid);
    assert(wheel_sample(400,4,10));assert(wheel.opposite_counts==0 && !wheel.fault);
}

static void brief_pickup_transient_then_correct_progress(void)
{
    wheel_ready();
    for(unsigned i=0;i<3;i++)assert(wheel_sample(400,-1,10));
    for(unsigned i=0;i<3;i++)assert(wheel_sample(400,2,10));
    assert(!wheel.fault && !wheel.opposite_pending);
    wheel_ready();assert(!wheel_sample(400,-12,10));
    assert(wheel.first_feedback_fault.reason==FOUR_FEEDBACK_OPPOSED);
}

static void persistent_opposed_and_command_chatter(void)
{
    wheel_ready();
    unsigned i;
    for(i=0;i<11;i++) {
        bool ok=wheel_sample(i%2?390:400,-1,10);
        if(!ok)break;
    }
    assert(i>=6 && i<=10);
    assert(wheel.fault==FOUR_SPEED_FAULT_FEEDBACK);
    four_feedback_snapshot_t f=wheel.first_feedback_fault;
    assert(f.valid && f.reason==FOUR_FEEDBACK_OPPOSED && f.raw_delta==-1 &&
        f.opposite_counts>=FOUR_CONTINUOUS_OPPOSED_COUNTS);
    four_speed_stop(&wheel,FOUR_SPEED_FAULT_COMMAND);
    assert(memcmp(&f,&wheel.first_feedback_fault,sizeof(f))==0);
    assert(!four_speed_set_target(&wheel,400,wheel_now+10));
}

static void sparse_persistent_opposed_timeout(void)
{
    wheel_ready(); assert(wheel_sample(400,-1,10));
    for(unsigned t=0;t<110;t+=10)assert(wheel_sample(400,0,10));
    assert(!wheel_sample(400,-1,10));
    assert(wheel.first_feedback_fault.reason==FOUR_FEEDBACK_OPPOSED);
}

static void reversal_requires_measured_neutral_and_has_deadline(void)
{
    wheel_ready(); uint32_t start=wheel_now;
    assert(four_speed_set_target(&wheel,-400,wheel_now));
    uint32_t budget=wheel.reversal_budget_ms;
    assert(budget==1250);
    while(wheel_now-start<budget-10) {
        assert(wheel_sample(-400,5,10));
        assert(wheel.run_direction==1 && wheel.wheel[2].raw_duty_permille>=0);
    }
    assert(!wheel_sample(-400,5,10));
    assert(wheel.first_feedback_fault.reason==FOUR_FEEDBACK_REVERSAL_TIMEOUT);
    assert(wheel.first_feedback_fault.raw_delta==5);
}

static void overspeed_in_all_phases(void)
{
    for(unsigned phase=0;phase<3;phase++) {
        wheel_ready();
        if(phase==1)assert(four_speed_set_target(&wheel,0,wheel_now));
        if(phase==2)assert(four_speed_set_target(&wheel,-400,wheel_now));
        wheel_now+=10;wheel_counts[2]+=101;
        assert(!four_speed_tick(&wheel,wheel_counts,wheel_now));
        assert(wheel.first_feedback_fault.reason==FOUR_FEEDBACK_OVERSPEED);
        assert(wheel.first_feedback_fault.raw_delta==101 && wheel.first_feedback_fault.dt_ms==10);
    }
}

static void launch_and_tracking_stall(void)
{
    reset_at(0);command[0]=50;
    for(unsigned t=0;t<550 && !d.group.fault;t+=10) {now_ms+=10;velocity(true);sample();}
    assert(d.group.fault&FOUR_SPEED_FAULT_STALL);assert(!link_state.permitted);
    reset_at(0);drive_for(100,0,0,1500);
    for(unsigned t=0;t<4000 && !d.group.fault;t+=10) {now_ms+=10;velocity(true);sample();}
    assert(d.group.fault&FOUR_SPEED_FAULT_STALL);assert(!link_state.permitted);
}

static void lease_deadline_late_packet_and_replay(void)
{
    reset_at(0);drive_for(50,0,0,1000);uint32_t last=link_state.last_command_ms;
    for(unsigned t=0;t<590;t+=10) {model_step(false);assert(link_state.permitted);}
    now_ms=last+600;velocity(true);
    assert(!link_state.permitted && (d.group.fault&FOUR_SPEED_FAULT_COMMAND));safety_assertions();
    reset_at(0);drive_for(50,0,0,1000);
    uint32_t prior=link_state.last_sequence, stamp=link_state.last_command_ms;
    sequence=prior-1;now_ms+=10;velocity(true);assert(link_state.last_command_ms==stamp);
    sequence=prior;session=999;velocity(true);assert(link_state.last_command_ms==stamp);session=19;
}

static void stop_estop_fault_cannot_rearm(void)
{
    reset_at(0);drive_for(50,0,0,1000);memset(command,0,sizeof(command));velocity(false);
    assert(!link_state.permitted && !d.group.enabled);safety_assertions();
    reset_at(0);drive_for(50,0,0,1000);frame(MC_MSG_ESTOP,0,NULL,0);
    assert(!link_state.permitted && (d.group.fault&SPEED_BENCH_FAULT_ESTOP));safety_assertions();
    four_link_bench_init(&link_state);session++;sequence=0;frame(MC_MSG_HELLO,0,NULL,0);
    const mc_control_enable_v1_t rearm={.clear_latched_faults=1};
    frame(MC_MSG_CONTROL_ENABLE,0,&rearm,sizeof(rearm));assert(d.group.fault && !link_state.permitted);
}

static void timing_invalid_and_atomic_validation(void)
{
    wheel_ready();assert(!wheel_sample(400,0,31));assert(wheel.fault&FOUR_SPEED_FAULT_TIMING);
    reset_at(0);drive_for(50,0,0,1000);
    mecanum_drive_plan_t bad=d.group.plan;bad.forward_cps[3]=NAN;
    mecanum_drive_plan_t before=d.group.plan;
    assert(!four_drive_retarget(&d.group,&bad,now_ms));
    assert(memcmp(&before,&d.group.plan,sizeof(before))==0);
    for(unsigned i=0;i<4;i++)assert(d.group.loop[i].wheel[i].requested_cps==before.forward_cps[i]);
    command[0]=151;velocity(true);assert(d.group.fault&FOUR_LINK_REJECTED);safety_assertions();
}

static void valid_20_30ms_samples_keep_control_safe(void)
{
    wheel_ready();
    for(unsigned i=0;i<20;i++) {
        assert(wheel_sample(400,8,20));
        assert(wheel_sample(400,12,30));
    }
    assert(!wheel.fault);
}

static void selected_channel_and_output_invariants(void)
{
    wheel_ready();wheel.selected=255;
    assert(!four_speed_set_target(&wheel,400,wheel_now));
    assert(wheel.fault&FOUR_SPEED_FAULT_CONFIG);
    reset_at(0);drive_for(50,0,0,1000);
    int original=d.group.raw_duty[0];
    d.group.raw_duty[0]=(int16_t)-original;
    uint32_t compares[8];
    assert(!four_drive_compares(&d.group,compares));
    for(unsigned i=0;i<8;i++)assert(compares[i]==0);
    four_drive_stop(&d.group,FOUR_SPEED_FAULT_FEEDBACK,now_ms);
    safety_assertions();
}

static void healthy_rearm_requires_disabled_still_and_new_enable(void)
{
    reset_at(0);drive_for(50,0,0,1000);
    memset(command,0,sizeof(command));velocity(false);
    for(unsigned j=0;j<100;j++) {
        model_step(false);
        if(j%2==0)velocity(false);
    }
    assert(!d.group.fault && d.group.stationary_ms==500 && !link_state.permitted);
    const mc_control_enable_v1_t reset={.clear_latched_faults=1};
    frame(MC_MSG_CONTROL_ENABLE,0,&reset,sizeof(reset));
    assert(!d.group.fault && !d.group.run_used && !link_state.motion_started && !link_state.permitted);
    const mc_control_enable_v1_t enable={.enabled=1};
    frame(MC_MSG_CONTROL_ENABLE,0,&enable,sizeof(enable));
    assert(link_state.permitted && !d.group.enabled);
    first_motion=0;drive_for(0,50,0,1000);
}

static void wire_snapshot_and_crc(void)
{
    reset_at(0);command[0]=50;now_ms+=10;velocity(true);
    sample(); now_ms+=10; velocity(true);
    add_count(2,101);sample();
    assert(d.group.fault_wheel==3);
    speed_bench_diagnostics_t diag={.ms=now_ms};
    assert(four_drive_compares(&d.group,diag.compares));
    mc_wheel_telemetry_v1_t telemetry;four_link_bench_status_t status;
    assert(four_link_bench_payloads(&link_state,&d,&diag,&telemetry,&status));
    assert(status.tag==UINT32_C(0x43475734) && sizeof(status)==156);
    assert(status.first_feedback_fault.reason==FOUR_FEEDBACK_OVERSPEED);
    assert(status.first_feedback_fault.wheel==3 && status.first_feedback_fault.raw_delta==101);
    if(dump_status_path) {
        FILE *capture=fopen(dump_status_path,"wb");assert(capture);
        assert(fwrite(&status,1,sizeof(status),capture)==sizeof(status));assert(fclose(capture)==0);
    }
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;four_link_request_t request;
    assert(mc_build_frame(MC_MSG_HELLO,0,1,1,now_ms,NULL,0,raw,sizeof(raw),&used));
    raw[used-1]^=1;assert(!four_link_bench_decode(raw,used,&request));
}

static void absolute_budget_not_reset_by_reversal_or_zero(void)
{
    reset_at(0);drive_for(50,0,0,1000);
    uint32_t start=link_state.first_motion_ms;
    while(now_ms-start<FOUR_LINK_MOTION_LIMIT_MS-10U) {
        if((now_ms-start)%2000==0)command[0]=-command[0];
        model_step(true);assert(!d.group.fault && link_state.permitted);
    }
    model_step(true);
    assert(!link_state.permitted && (d.group.fault&FOUR_SPEED_FAULT_DURATION));safety_assertions();
}

static void uint32_time_and_encoder_wrap(void)
{
    reset_at(UINT32_MAX-900U);drive_for(50,0,0,2500);
    wheel_ready();wheel_counts[2]=INT32_MAX-1;wheel.wheel[2].previous_count=wheel_counts[2];
    uint32_t bits=(uint32_t)wheel_counts[2]+4U;memcpy(&wheel_counts[2],&bits,sizeof(bits));
    wheel_now+=10;assert(four_speed_set_target(&wheel,400,wheel_now));
    assert(four_speed_tick(&wheel,wheel_counts,wheel_now));assert(!wheel.fault);
}

static void run(const char *name,void (*test)(void))
{
    printf("RUN %s\n",name);fflush(stdout);test();tests_run++;printf("PASS %s\n",name);
}

int main(int argc,char **argv)
{
    if(argc==3 && strcmp(argv[1],"--dump-status")==0)dump_status_path=argv[2];
    else assert(argc==1);
    run("axes_diagonals_yaw_continuous",axes_and_diagonals);
    run("single_wheel_reversal_others_keep_running",one_wheel_reverses_without_chassis_stop);
    run("circular_translation_yaw_24s",circular_translation_and_yaw);
    run("full_envelope_reversals_latest_command_cancel",full_envelope_reversals_and_cancel);
    run("zero_deadband_repeated_crossing_jitter",zero_bands_and_zero_crossing_jitter);
    run("eight_seed_random_walks_80s",seeded_random_changes);
    run("twelve_inertia_static_load_combinations",inertia_and_static_load_sweep);
    run("yesterday_zero_restart_and_passive_wheel",old_failure_and_passive_low_speed);
    run("single_reverse_count_then_recovery",one_count_noise_not_a_fault);
    run("brief_pickup_transient_and_large_reverse_excursion",brief_pickup_transient_then_correct_progress);
    run("persistent_opposed_not_reset_by_target_updates",persistent_opposed_and_command_chatter);
    run("sparse_opposed_100ms_bound",sparse_persistent_opposed_timeout);
    run("measured_neutral_and_bounded_reversal",reversal_requires_measured_neutral_and_has_deadline);
    run("overspeed_including_zero_and_reversal",overspeed_in_all_phases);
    run("launch_and_running_stall",launch_and_tracking_stall);
    run("600ms_deadline_late_packet_replay_foreign_session",lease_deadline_late_packet_and_replay);
    run("disable_estop_fault_cannot_rearm",stop_estop_fault_cannot_rearm);
    run("timing_nan_atomicity_body_limits",timing_invalid_and_atomic_validation);
    run("valid_20_30ms_encoder_sampling",valid_20_30ms_samples_keep_control_safe);
    run("selected_channel_and_actuator_invariants",selected_channel_and_output_invariants);
    run("healthy_rearm_requires_still_disabled_new_enable",healthy_rearm_requires_disabled_still_and_new_enable);
    run("distinct_identity_first_fault_snapshot_crc",wire_snapshot_and_crc);
    run("150s_absolute_budget_not_renewable",absolute_budget_not_reset_by_reversal_or_zero);
    run("uint32_clock_encoder_wrap",uint32_time_and_encoder_wrap);
    printf("{\"passed\":true,\"test_groups\":%u,\"max_effective_step_cps_per_10ms\":%.4f,"
        "\"max_nonzero_pwm_step_permille_per_10ms\":%u,\"hardware_access\":false}\n",
        tests_run,(double)max_effective_step,max_output_step);
    return 0;
}
