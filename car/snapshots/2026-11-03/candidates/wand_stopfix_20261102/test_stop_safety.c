/* Production C, synthetic encoders; no HAL/motor/network access. */
#define main inherited_controller_main
#include "test_controller.c"
#undef main

static unsigned stop_groups;
static void zero_outputs(void)
{
    assert(!l.permitted && !b.group.enabled);check_outputs();
    for(unsigned i=0;i<4;i++)assert(!b.group.raw_duty[i]);
}
static void start_stage(unsigned stage)
{
    reset_at(stage,0);servo_stage_t p;assert(servo_stage_get(stage,&p));
    run_for((int)fminf(p.speed_m_s*1000,200),0,0,1800);
    assert(l.permitted && b.group.enabled && !l.fault && !b.group.fault);
}
static void delayed(unsigned type,unsigned flags,const void *p,size_t n,unsigned lag,int shift)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;four_link_request_t r;
    assert(mc_build_frame(type,flags,++seq,session,now+(uint32_t)shift,
        p,(uint16_t)n,raw,sizeof(raw),&used));
    assert(four_link_bench_decode(raw,used,&r));
    for(unsigned elapsed=0;elapsed<lag;elapsed+=10)tick(false,10);
    (void)four_link_bench_process(&l,&b,&r,now);
}
static void safe_revocations(void)
{
    const unsigned delays[]={0,100,120,140,300,590};
    for(unsigned stage=0;stage<SERVO_STAGE_COUNT;stage++)for(unsigned k=0;k<6;k++){
        start_stage(stage);
        const uint32_t last_ms=l.last_command_ms,last_cmd=l.last_command_sequence,anchor=l.source_anchor_ms;
        const mc_control_enable_v1_t disable={0};
        delayed(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),delays[k],0);
        zero_outputs();assert(!l.fault && !b.group.fault);
        assert(l.last_command_ms==last_ms && l.last_command_sequence==last_cmd);
        assert(l.source_anchor_ms==anchor && !l.zero_seen && l.motion_started);
        /* No stale/fresh old enabled stream can reacquire a revoked permit. */
        velocity(true);zero_outputs();assert(l.fault && b.group.fault);stop_groups++;

        start_stage(stage);
        const uint32_t prior_ms=l.last_command_ms,prior_cmd=l.last_command_sequence;
        const mc_cmd_vel_v1_t zero={0,0,0,600,0,0};
        delayed(MC_MSG_CMD_VEL,0,&zero,sizeof(zero),delays[k],0);
        zero_outputs();assert(!l.fault && !b.group.fault);
        if(delays[k]>=120){
            assert(l.last_command_ms==prior_ms && l.last_command_sequence==prior_cmd && !l.zero_seen);
        }else assert(l.zero_seen && l.last_command_ms==now && l.last_command_sequence==seq);
        stop_groups++;
    }
    /* A late zero AFTER a completed fresh stop is not an actuator fault. */
    start_stage(6);const mc_control_enable_v1_t disable={0};
    frame(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),0);zero_outputs();
    const uint32_t before=l.last_command_ms,barrier=l.last_source_elapsed;
    const mc_cmd_vel_v1_t zero={0,0,0,600,0,0};
    delayed(MC_MSG_CMD_VEL,0,&zero,sizeof(zero),140,0);
    zero_outputs();assert(!l.fault && !b.group.fault && !l.zero_seen);
    assert(l.last_command_ms==before && l.last_source_elapsed==barrier);stop_groups++;
}
static void source_boundaries_and_restrictions(void)
{
    const mc_control_enable_v1_t disable={0};
    for(unsigned kind=0;kind<2;kind++)for(unsigned shift=0;shift<2;shift++){
        start_stage(6);const mc_cmd_vel_v1_t zero={0,0,0,600,0,0};
        const uint32_t ms=l.last_command_ms,barrier=l.last_source_elapsed;
        delayed(kind?MC_MSG_CMD_VEL:MC_MSG_CONTROL_ENABLE,0,kind?(const void*)&zero:(const void*)&disable,
            kind?sizeof(zero):sizeof(disable),0,shift?61:-1);
        zero_outputs();assert(!l.fault && !b.group.fault && !l.zero_seen);
        assert(l.last_command_ms==ms && l.last_source_elapsed==barrier);stop_groups++;
    }
    /* Exact original lag/lead guard on ENABLED requests, including zero. */
    const int shifts[]={0,1,60,61,-1};
    const unsigned lags[]={100,120,0,0,0};
    const bool faults[]={false,false,false,true,true};
    for(unsigned i=0;i<5;i++)for(unsigned zero=0;zero<2;zero++){
        start_stage(6);const mc_cmd_vel_v1_t v={zero?0:200,0,0,600,0,0};
        delayed(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),lags[i],shifts[i]);
        if(faults[i]){zero_outputs();assert(l.fault&SERVO_LINK_SOURCE_AGE);}
        else assert(l.permitted && !l.fault && !b.group.fault);
        stop_groups++;
    }
    for(unsigned zero=0;zero<2;zero++){
        start_stage(6);const mc_cmd_vel_v1_t v={zero?0:200,0,0,600,0,0};
        delayed(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),120,0);
        zero_outputs();assert(l.fault&SERVO_LINK_SOURCE_AGE);stop_groups++;
    }
    /* A delayed select/rearm/enable is never interpreted as safe disable. */
    const mc_control_enable_v1_t controls[]={
        {.reserved=SERVO_TIER_MAGIC+6},{.clear_latched_faults=1},
        {.enabled=1,.reserved=SERVO_TIER_MAGIC+6}};
    for(unsigned i=0;i<3;i++){
        start_stage(6);const uint32_t budget=l.first_motion_ms;
        delayed(MC_MSG_CONTROL_ENABLE,0,&controls[i],sizeof(controls[i]),140,0);
        zero_outputs();assert(l.fault&SERVO_LINK_SOURCE_AGE);assert(l.first_motion_ms==budget);stop_groups++;
    }
    start_stage(6);const mc_cmd_vel_v1_t illegal={200,0,0,600,0,0};
    delayed(MC_MSG_CMD_VEL,0,&illegal,sizeof(illegal),140,0);
    zero_outputs();assert(l.fault&SERVO_LINK_SOURCE_AGE);stop_groups++;
}
static void latches_sessions_and_deadlines(void)
{
    const mc_control_enable_v1_t disable={0};const mc_cmd_vel_v1_t zero={0,0,0,600,0,0};
    for(unsigned fault=0;fault<4;fault++){
        start_stage(6);
        if(fault==0)frame(MC_MSG_ESTOP,0,NULL,0,0);
        else if(fault==1)for(unsigned i=0;i<60;i++)tick(false,10);
        else if(fault==2){now+=31;(void)drive_bench_tick(&b,counts,now);}
        else {const mc_cmd_vel_v1_t v={200,0,0,600,0,0};frame(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),61);}
        const uint32_t link=l.fault,group=b.group.fault;assert(link || group);
        delayed(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),140,0);
        zero_outputs();assert((l.fault&link)==link && (b.group.fault&group)==group);
        delayed(MC_MSG_CMD_VEL,0,&zero,sizeof(zero),140,0);
        zero_outputs();assert((l.fault&link)==link && (b.group.fault&group)==group);stop_groups++;
    }
    /* The 600ms deadline runs BEFORE any late revocation and remains a fault. */
    start_stage(6);delayed(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),600,0);
    zero_outputs();assert(b.group.fault&FOUR_SPEED_FAULT_COMMAND);stop_groups++;
    for(unsigned foreign=0;foreign<2;foreign++){
        start_stage(6);four_link_request_t r={0};
        r.header.message_type=MC_MSG_CONTROL_ENABLE;r.header.session_id=foreign?session+1:session;
        r.header.sequence=foreign?seq+1:seq;r.header.sender_time_ms=now;
        const uint32_t before=l.last_sequence;
        (void)four_link_bench_process(&l,&b,&r,now);
        assert(l.permitted && !l.fault && l.last_sequence==before);stop_groups++;
    }
    /* uint32 time wrap does not change fresh command/disable semantics. */
    reset_at(6,UINT32_MAX-1000);run_for(200,0,0,1500);
    frame(MC_MSG_CONTROL_ENABLE,0,&disable,sizeof(disable),0);zero_outputs();assert(!l.fault);stop_groups++;
}
int main(void)
{
    safe_revocations();source_boundaries_and_restrictions();latches_sessions_and_deadlines();
    printf("PASS: %u STOPFIX production-C revocation/lease/order/fault/boundary groups; software only\n",stop_groups);
    return 0;
}
