/* HAL-free protocol + actual controller + synthetic PWM-driven dynamics.
 * No motors, current/temperature model, network, ROS or device access. */
#include "four_link_bench.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static drive_bench_t b;
static four_link_bench_t l;
static uint32_t now,seq,session;
static int32_t counts[4];
static float speed[4],fraction[4],plant_tau=.10f,plant_k=.11f;
static int command[3];
static unsigned tests;
static float max_vector_step;
static uint32_t straight_target_ramp_ms,diagonal_target_ramp_ms;
static const char *active_fixture,*reverse_fixture;

static void save_status(const char *path)
{
    if(!path)return;
    mc_wheel_telemetry_v1_t t;four_link_bench_status_t s;
    speed_bench_diagnostics_t d={.ms=now,.ticks=100,.dt_min=10,.dt_max=10};
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&l,&b,&d,&t,&s));
    FILE *f=fopen(path,"wb");assert(f);
    assert(fwrite(&s,1,sizeof(s),f)==sizeof(s));assert(fclose(f)==0);
}

static bool frame(unsigned type,unsigned flags,const void *payload,size_t length,int source_shift)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;four_link_request_t r;
    assert(mc_build_frame(type,flags,++seq,session,now+(uint32_t)source_shift,
                         payload,(uint16_t)length,raw,sizeof(raw),&used));
    assert(four_link_bench_decode(raw,used,&r));
    return four_link_bench_process(&l,&b,&r,now);
}
static void velocity(bool enabled)
{
    const mc_cmd_vel_v1_t v={command[0],command[1],command[2],600,0,0};
    frame(MC_MSG_CMD_VEL,enabled?MC_FLAG_ENABLED:0,&v,sizeof(v),0);
}
static void add_count(unsigned i,int delta)
{
    const uint32_t value=(uint32_t)counts[i]+(uint32_t)delta;
    counts[i]=value<=INT32_MAX ? (int32_t)value : (int32_t)((int64_t)value-INT64_C(4294967296));
}
static void check_outputs(void)
{
    uint32_t ccr[8];assert(four_drive_compares(&b.group,ccr));
    servo_stage_t p;assert(servo_stage_get(b.group.servo_stage,&p));
    for(unsigned i=0;i<4;i++) {
        const four_speed_controller_t *c=&b.group.loop[i];
        const four_speed_wheel_state_t *w=&c->wheel[i];
        assert(abs(b.group.raw_duty[i])<=p.duty_permille);
        assert(fabsf(w->ramped_cps)*60/c->config.wheel[i].counts_per_rev<=400.001f);
        assert(!c->reversing || !b.group.raw_duty[i]);
        if(!b.group.enabled) assert(!ccr[2*i] && !ccr[2*i+1]);
    }
}
static void tick(bool renew,unsigned dt)
{
    const mecanum_body_velocity_t before=b.group.shaped_body;
    float effective_before[4];int16_t duty_before[4];
    for(unsigned i=0;i<4;i++) {
        effective_before[i]=b.group.loop[i].wheel[i].ramped_cps;
        duty_before[i]=b.group.raw_duty[i];
    }
    now+=dt;
    if(renew)velocity(true);
    for(unsigned i=0;i<4;i++) {
        const int u=b.group.raw_duty[i]*b.adopted.wheel[i].motor_sign;
        const float equilibrium=copysignf(fmaxf(0,(abs(u)-80)/plant_k),(float)u);
        speed[i]+=(equilibrium-speed[i])*(dt*.001f/(plant_tau+dt*.001f));
        fraction[i]+=speed[i]*dt*.001f;
        const int delta=(int)truncf(fraction[i]);fraction[i]-=delta;
        add_count(i,delta*b.adopted.wheel[i].encoder_sign);
    }
    (void)drive_bench_tick(&b,counts,now);four_link_bench_deadline(&l,&b,now);
    if(b.group.enabled && (command[0] || command[1] || command[2])) {
        servo_stage_t p;assert(servo_stage_get(b.group.servo_stage,&p));
        const float step=hypotf(b.group.shaped_body.vx_m_s-before.vx_m_s,
                                b.group.shaped_body.vy_m_s-before.vy_m_s);
        assert(step<=p.accel_m_s2*dt*.001f+.00001f);
        assert(fabsf(b.group.shaped_body.wz_rad_s-before.wz_rad_s)<=.4f*dt*.001f+.00001f);
        if(step>max_vector_step)max_vector_step=step;
    }
    check_outputs();
    if(b.group.enabled)for(unsigned i=0;i<4;i++) {
        const four_speed_controller_t *c=&b.group.loop[i];
        const four_speed_wheel_state_t *w=&c->wheel[i];
        if(c->rejoining && w->ramped_cps)
            assert(fabsf(w->ramped_cps-effective_before[i])<=c->config.resume_cps_per_s*dt*.001f+.001f);
        if(w->raw_duty_permille)
            assert(abs(w->raw_duty_permille-duty_before[i])<=c->config.launch_slew_permille_per_s*dt*.001f+10.01f);
    }
}
static void reset_at(unsigned stage,uint32_t start)
{
    now=start;seq=0;session=17;
    memset(counts,0,sizeof(counts));memset(speed,0,sizeof(speed));
    memset(fraction,0,sizeof(fraction));memset(command,0,sizeof(command));
    drive_bench_init(&b,counts,now);four_link_bench_init(&l);
    assert(!b.group.fault && frame(MC_MSG_HELLO,0,NULL,0,0));
    for(unsigned i=0;i<60;i++){now+=10;assert(drive_bench_tick(&b,counts,now));velocity(false);}
    const mc_control_enable_v1_t select={.reserved=SERVO_TIER_MAGIC+stage};
    frame(MC_MSG_CONTROL_ENABLE,0,&select,sizeof(select),0);
    assert(l.tier_selected && b.group.servo_stage==stage && !l.permitted);
    const mc_control_enable_v1_t enable={.enabled=1,.reserved=SERVO_TIER_MAGIC+stage};
    frame(MC_MSG_CONTROL_ENABLE,0,&enable,sizeof(enable),0);
    assert(l.permitted && !b.group.enabled && !b.group.fault);
}
static void run_for(int x,int y,int z,unsigned duration)
{
    command[0]=x;command[1]=y;command[2]=z;
    for(unsigned i=0;i<duration;i+=10) {
        tick(true,10);
        if(b.group.fault || !l.permitted) {
            fprintf(stderr,"fault=%x link=%x at %u command %d %d %d wheel=%u reason=%u\n",
                b.group.fault,l.fault,now,x,y,z,b.group.fault_wheel,
                b.group.loop[b.group.fault_wheel ? b.group.fault_wheel-1 : 0].first_feedback_fault.reason);
            assert(!b.group.fault && l.permitted);
        }
    }
}
static void vector_ramp_and_stages(void)
{
    for(unsigned stage=0;stage<SERVO_STAGE_COUNT;stage++) {
        reset_at(stage,0);servo_stage_t p;assert(servo_stage_get(stage,&p));
        const int x=(int)floorf(p.speed_m_s*1000);
        run_for(x,0,0,(unsigned)ceilf(p.speed_m_s/p.accel_m_s2*1000)+100);
        assert(fabsf(b.group.shaped_body.vx_m_s-x*.001f)<.0001f);
        assert(!four_drive_select_stage(&b.group,(stage+1)%7,now));
    }
    reset_at(6,0);command[0]=700;
    for(unsigned n=0;n<50;n++) {
        tick(true,10);
        if(b.group.shaped_body.vx_m_s>=.699999f) {
            straight_target_ramp_ms=now-l.first_motion_ms;break;
        }
    }
    assert(straight_target_ramp_ms==350);
    assert(b.group.shaped_body.vx_m_s>.699f);
    reset_at(6,0);command[0]=command[1]=494;command[2]=100;
    for(unsigned n=0;n<50;n++) {
        tick(true,10);
        if(b.group.shaped_body.vx_m_s>=.493999f && b.group.shaped_body.vy_m_s>=.493999f) {
            diagonal_target_ramp_ms=now-l.first_motion_ms;break;
        }
    }
    assert(diagonal_target_ramp_ms==350);
    assert(hypotf(b.group.shaped_body.vx_m_s,b.group.shaped_body.vy_m_s)>.698f);
    assert(b.group.loop[1].wheel[1].requested_cps>5500);
    tests++;
}
static void all_axes_and_circle(void)
{
    reset_at(6,0);
    const int path[][3]={{700,0,0},{0,700,0},{-700,0,0},{0,-700,0},
        {494,494,0},{-494,494,0},{-494,-494,0},{494,-494,0},{0,0,100},{0,0,-100}};
    for(unsigned i=0;i<10;i++)run_for(path[i][0],path[i][1],path[i][2],1800);
    for(unsigned i=0;i<2000;i++) {
        const float a=i*.004f;
        command[0]=(int)(450*cosf(a));command[1]=(int)(450*sinf(a));command[2]=(int)(90*sinf(a*.5f));
        tick(true,10);assert(!b.group.fault && l.permitted);
    }
    tests++;
}
static void immediate_zero_and_partial_reversal(void)
{
    reset_at(6,0);run_for(400,0,0,1500);
    command[0]=0;velocity(true);
    for(unsigned i=0;i<4;i++)assert(!b.group.raw_duty[i] && !b.group.loop[i].wheel[i].ramped_cps);
    run_for(400,0,0,1500);
    bool partial=false;
    command[0]=300;command[1]=400;command[2]=100;
    for(unsigned n=0;n<150;n++) {
        tick(true,10);assert(!b.group.fault && l.permitted);
        if(b.group.loop[0].reversing) {
            partial=true;
            assert(!b.group.raw_duty[0]);
            assert(b.group.raw_duty[1] && b.group.raw_duty[2]);
        }
    }
    assert(partial && b.group.loop[0].run_direction==-1);
    tests++;
}
static void zero_crossing_chatter_and_random(void)
{
    reset_at(6,0);
    uint32_t rng=419;
    for(unsigned t=0;t<6000;t++) {
        if(t%70==0) {
            rng=rng*1664525U+1013904223U;command[0]=(int)(rng%801)-400;
            rng=rng*1664525U+1013904223U;command[1]=(int)(rng%801)-400;
        }
        if(t%700<12) command[0]=command[1]=(t%2 ? 2 : -2);
        tick(true,10);assert(!b.group.fault && l.permitted);
    }
    tests++;
}
static void tier_and_source_faults(void)
{
    reset_at(0,0);command[0]=201;velocity(true);
    assert(l.fault && !l.permitted && !b.group.enabled);
    reset_at(6,0);run_for(300,0,0,500);
    const mc_control_enable_v1_t select={.reserved=SERVO_TIER_MAGIC+5};
    frame(MC_MSG_CONTROL_ENABLE,0,&select,sizeof(select),0);
    assert(l.fault && !b.group.enabled);
    reset_at(6,0);run_for(300,0,0,500);
    const mc_cmd_vel_v1_t v={300,0,0,600,0,0};
    frame(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),-120);
    assert((l.fault&SERVO_LINK_SOURCE_AGE) && !b.group.enabled);
    reset_at(6,0);frame(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),61);
    assert(l.fault&SERVO_LINK_SOURCE_AGE);
    tests++;
}
static void safety_faults(void)
{
    reset_at(6,0);run_for(300,0,0,500);
    for(unsigned t=0;t<60;t++)tick(false,10);
    assert(b.group.fault && !b.group.enabled);check_outputs();
    reset_at(6,0);run_for(300,0,0,500);
    frame(MC_MSG_ESTOP,0,NULL,0,0);assert(!b.group.enabled && b.group.fault);
    reset_at(6,0);run_for(300,0,0,500);
    now+=31;(void)drive_bench_tick(&b,counts,now);
    assert(b.group.fault&FOUR_SPEED_FAULT_TIMING);
    reset_at(6,0);run_for(300,0,0,500);
    now+=10;velocity(true);add_count(2,80*b.adopted.wheel[2].encoder_sign);
    (void)drive_bench_tick(&b,counts,now);
    assert(b.group.fault&FOUR_SPEED_FAULT_FEEDBACK);check_outputs();
    reset_at(6,0);command[0]=300;
    for(unsigned t=0;t<100;t++) {now+=10;velocity(true);(void)drive_bench_tick(&b,counts,now);four_link_bench_deadline(&l,&b,now);}
    assert(b.group.fault&FOUR_SPEED_FAULT_STALL);check_outputs();
    tests++;
}
static void reversal_deadline_and_neutral(void)
{
    reset_at(6,0);run_for(300,0,0,1500);
    four_speed_controller_t *c=&b.group.loop[0];
    const float old=c->wheel[0].requested_cps;
    assert(old>1000 && c->run_direction==1);
    /* A coherent opposite target crosses zero; chatter cannot reset budget. */
    command[0]=-300;
    for(unsigned t=0;t<500 && !c->reversing;t++)tick(true,10);
    assert(c->reversing && !c->wheel[0].raw_duty_permille);
    save_status(reverse_fixture);
    const uint32_t began=c->reversal_started_ms;
    for(unsigned t=0;t<75;t++) {
        now+=10;
        const float target=t%2 ? 20 : -20;
        (void)four_speed_set_target(c,target,now);
        int32_t isolated[4];
        for(unsigned i=0;i<4;i++)isolated[i]=c->wheel[i].previous_count;
        const int64_t next=(int64_t)isolated[0]+5*c->config.wheel[0].encoder_sign;
        isolated[0]=(int32_t)next;
        (void)four_speed_tick(c,isolated,now);
        assert(c->reversal_started_ms==began);
        assert(!c->wheel[0].raw_duty_permille);
    }
    assert(c->fault&FOUR_SPEED_FAULT_FEEDBACK);
    assert(c->first_feedback_fault.reason==FOUR_FEEDBACK_REVERSAL_TIMEOUT);
    tests++;
}
static void snapshots_and_atomicity(void)
{
    reset_at(6,0);run_for(300,200,50,1000);
    mc_wheel_telemetry_v1_t t;four_link_bench_status_t s;
    speed_bench_diagnostics_t d={.ms=now,.ticks=100,.dt_min=10,.dt_max=10};
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&l,&b,&d,&t,&s));
    assert(sizeof(s)==184 && s.tag==UINT32_C(0x37535734));
    assert(s.output_authority_version==2 && s.stage_id==6 && !s.hardware_released);
    assert(s.vector_accel_limit==2 && s.stage_duty_limit==800);
    save_status(active_fixture);
    b.group.raw_duty[2]=810;uint32_t ccr[8];
    assert(!four_drive_compares(&b.group,ccr));
    for(unsigned i=0;i<8;i++)assert(ccr[i]==0);
    tests++;
}
static void time_wrap_and_sample_dt(void)
{
    reset_at(6,UINT32_MAX-1000);
    for(unsigned n=0;n<200;n++){command[0]=200;tick(true,n%2?20:30);assert(!b.group.fault);}
    tests++;
}
static void malformed_replay_and_wrong_session(void)
{
    reset_at(6,0);run_for(200,0,0,500);
    four_link_request_t r={.header={.message_type=MC_MSG_CMD_VEL,.flags=MC_FLAG_ENABLED,
        .sequence=seq,.session_id=session,.sender_time_ms=now},
        .body.velocity={200,0,0,600,0,0}};
    const uint32_t lease=l.last_command_ms;
    assert(!four_link_bench_process(&l,&b,&r,now+10));
    assert(l.last_command_ms==lease && !l.fault);
    r.header.sequence=seq+1;r.header.session_id=session+1;
    assert(!four_link_bench_process(&l,&b,&r,now+20));
    assert(l.last_command_ms==lease && !l.fault);
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;
    assert(mc_build_frame(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,seq+1,session,now,&r.body.velocity,
                          sizeof(r.body.velocity),raw,sizeof(raw),&used));
    raw[used-1]^=1;assert(!four_link_bench_decode(raw,used,&r));
    raw[used-1]^=1;assert(!four_link_bench_decode(raw,used-1,&r));
    mc_cmd_vel_v1_t bad={200,0,0,600,1,0};
    assert(mc_build_frame(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,seq+1,session,now,&bad,
                          sizeof(bad),raw,sizeof(raw),&used));
    assert(!four_link_bench_decode(raw,used,&r));
    tests++;
}
static void fault_rearm_is_not_a_clear(void)
{
    reset_at(6,0);run_for(200,0,0,500);
    frame(MC_MSG_ESTOP,0,NULL,0,0);
    const uint32_t fault=b.group.fault;
    for(unsigned i=0;i<150;i++)tick(false,10);
    memset(command,0,sizeof(command));velocity(false);
    const mc_control_enable_v1_t rearm={.clear_latched_faults=1};
    frame(MC_MSG_CONTROL_ENABLE,0,&rearm,sizeof(rearm),0);
    assert(b.group.fault&fault && !b.group.enabled && !l.permitted);
    assert(!four_drive_rearm(&b.group,now));
    check_outputs();tests++;
}
static void healthy_rearm_and_new_enable(void)
{
    reset_at(6,0);run_for(200,0,0,500);
    const mc_control_enable_v1_t disable={0};
    frame(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),0);
    memset(command,0,sizeof(command));
    for(unsigned i=0;i<200;i++){tick(false,10);velocity(false);}
    assert(b.group.stationary_ms==500 && !b.group.fault);
    const mc_control_enable_v1_t rearm={.clear_latched_faults=1};
    frame(MC_MSG_CONTROL_ENABLE,0,&rearm,sizeof(rearm),0);
    assert(!b.group.run_used && !l.permitted && !l.motion_started);
    const mc_control_enable_v1_t enable={.enabled=1,.reserved=SERVO_TIER_MAGIC+6};
    frame(MC_MSG_CONTROL_ENABLE,0,&enable,sizeof(enable),0);
    assert(l.permitted && !b.group.enabled);
    for(unsigned i=0;i<20;i++)tick(true,10);
    assert(!b.group.enabled); /* Re-preparing with zero never auto-moves. */
    run_for(200,0,0,500);tests++;
}
static void absolute_budget_and_bad_config(void)
{
    reset_at(6,0);run_for(200,0,0,1000);
    const uint32_t began=l.first_motion_ms;
    run_for(0,200,0,1000);run_for(0,0,0,100);
    assert(l.first_motion_ms==began);
    l.first_motion_ms=now-150000;
    four_link_bench_deadline(&l,&b,now);
    assert(b.group.fault&FOUR_SPEED_FAULT_DURATION);check_outputs();
    reset_at(0,0);four_speed_config_t config=b.adopted;four_speed_controller_t c;
    config.ramp_cps_per_s=800;assert(!four_speed_init(&c,&config));
    config=b.adopted;config.feedback_filter_ms=50;assert(!four_speed_init(&c,&config));
    assert(four_speed_init(&c,&b.adopted) && four_speed_arm(&c,0,counts,now));
    assert(!four_speed_set_target(&c,3000,now+10));
    assert(c.fault&FOUR_SPEED_FAULT_COMMAND);
    tests++;
}
static void opposed_feedback_and_passive_zero(void)
{
    reset_at(6,0);run_for(300,0,0,1000);
    now+=10;velocity(true);add_count(0,-12);
    (void)drive_bench_tick(&b,counts,now);
    assert(b.group.fault&FOUR_SPEED_FAULT_FEEDBACK);
    assert(b.group.loop[0].first_feedback_fault.reason==FOUR_FEEDBACK_OPPOSED);
    check_outputs();
    reset_at(6,0);run_for(200,200,0,1500);
    assert(!b.group.plan.forward_cps[0] && !b.group.raw_duty[0]);
    for(unsigned i=0;i<10;i++) {
        now+=10;velocity(true);add_count(0,-1);
        /* Let the other wheels progress; zero FL can be passively back-driven. */
        for(unsigned j=1;j<4;j++)add_count(j,8*b.adopted.wheel[j].encoder_sign);
        (void)drive_bench_tick(&b,counts,now);
        assert(!b.group.fault);check_outputs();
    }
    tests++;
}
static void inertia_sweep_and_counter_wrap(void)
{
    const float taus[]={.04f,.10f,.18f}, slopes[]={.11f,.15f,.18f};
    for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++) {
        plant_tau=taus[i];plant_k=slopes[j];reset_at(6,0);
        run_for(450,300,0,1600);run_for(-300,400,50,1800);run_for(300,-400,-50,1800);
    }
    plant_tau=.10f;plant_k=.11f;
    reset_at(6,0);
    for(unsigned i=0;i<4;i++) {
        counts[i]=b.counts[i]=b.group.counts[i]=INT32_MAX-5;
        for(unsigned j=0;j<4;j++)b.group.loop[j].wheel[i].previous_count=counts[i];
    }
    run_for(200,0,0,500);
    assert(counts[0]<0 && counts[2]<0);tests++;
}
int main(int argc,char **argv)
{
    assert(argc==1 || argc==3);
    if(argc==3){active_fixture=argv[1];reverse_fixture=argv[2];}
    vector_ramp_and_stages();all_axes_and_circle();immediate_zero_and_partial_reversal();
    zero_crossing_chatter_and_random();tier_and_source_faults();safety_faults();
    reversal_deadline_and_neutral();snapshots_and_atomicity();time_wrap_and_sample_dt();
    malformed_replay_and_wrong_session();fault_rearm_is_not_a_clear();healthy_rearm_and_new_enable();
    absolute_budget_and_bad_config();opposed_feedback_and_passive_zero();inertia_sweep_and_counter_wrap();
    printf("PASS: %u controller/protocol/model groups; max vector step %.6f m/s; straight/diagonal TARGET ramp %u/%u ms; NO hardware/electrical validation\n",
           tests,max_vector_step,straight_target_ramp_ms,diagonal_target_ramp_ms);
    return 0;
}
