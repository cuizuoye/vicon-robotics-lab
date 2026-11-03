#include "four_link_bench.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#ifndef DRIVE_STARTUP_OBSERVE
#error Wireless bench must retain the 10-second bounded startup profile.
#endif

void four_link_bench_init(four_link_bench_t *c) { memset(c,0,sizeof(*c)); }

bool four_link_bench_decode(const uint8_t *raw, size_t length, four_link_request_t *r)
{
    if (!r) return false;
    memset(r,0,sizeof(*r));
    if (!mc_validate_frame(raw,length,&r->header) || !r->header.session_id ||
        (r->header.flags & ~(MC_FLAG_ACK_REQUIRED|MC_FLAG_ENABLED))) return false;
    const mc_frame_header_v1_t *h=&r->header;
    const uint8_t *p=raw+sizeof(*h);
    switch(h->message_type) {
    case MC_MSG_CMD_VEL:
        if(h->payload_length!=sizeof(r->body.velocity)) return false;
        memcpy(&r->body.velocity,p,sizeof(r->body.velocity));
        return r->body.velocity.timeout_ms==150 && !r->body.velocity.reserved &&
               !r->body.velocity.control_mode;
    case MC_MSG_CONTROL_ENABLE:
        if(h->payload_length!=sizeof(r->body.control) || (h->flags&MC_FLAG_ENABLED)) return false;
        memcpy(&r->body.control,p,sizeof(r->body.control));
        return r->body.control.enabled<=1 && r->body.control.clear_latched_faults<=1 &&
               !r->body.control.reserved;
    case MC_MSG_HELLO: case MC_MSG_HEARTBEAT: case MC_MSG_ESTOP:
        return h->payload_length==0 && !(h->flags&MC_FLAG_ENABLED);
    default: return false;
    }
}

static void stop(four_link_bench_t *c, drive_bench_t *b, uint32_t fault, uint32_t now)
{
    c->fault|=fault; c->permitted=false; c->zero_seen=false;
    four_drive_stop(&b->group,fault,now); b->mode=DRIVE_MODE_IDLE;
}

void four_link_bench_deadline(four_link_bench_t *c, drive_bench_t *b, uint32_t now)
{
    if(c->session_active && c->command_seen && !c->expired && now-c->last_command_ms>=150) {
        c->expired=true; c->fault|=FOUR_LINK_EXPIRED;
        /* 运行期超时永久消耗额度并锁存。空闲链路超时只能新session重新建立。 */
        if(b->group.enabled) stop(c,b,FOUR_SPEED_FAULT_COMMAND,now);
        else { c->permitted=false; c->zero_seen=false; }
    }
    if(b->group.fault || (b->group.run_used && !b->group.enabled)) c->permitted=false;
}

bool four_link_bench_process(four_link_bench_t *c, drive_bench_t *b,
                             const four_link_request_t *r, uint32_t now)
{
    four_link_bench_deadline(c,b,now); /* 先截止，迟到包不能复活动作。 */
    const mc_frame_header_v1_t *h=&r->header;
    if(h->message_type==MC_MSG_HELLO) {
        if(c->session_active && h->session_id==c->session_id &&
           (!mc_sequence_is_newer(h->sequence,c->last_sequence) || c->expired)) return false;
        if(b->group.enabled) stop(c,b,FOUR_LINK_REJECTED,now);
        c->session_active=true; c->session_id=h->session_id; c->last_sequence=h->sequence;
        c->last_command_sequence=0; c->command_seen=false; c->zero_seen=false;
        c->expired=false; c->permitted=false;
        c->fault&=~FOUR_LINK_EXPIRED; /* 不清执行器故障或单次额度。 */
        return true;
    }
    if(!c->session_active || h->session_id!=c->session_id ||
       !mc_sequence_is_newer(h->sequence,c->last_sequence)) return false;
    c->last_sequence=h->sequence;
    if(h->message_type==MC_MSG_ESTOP) {
        stop(c,b,SPEED_BENCH_FAULT_ESTOP,now); return false;
    }
    if(h->message_type==MC_MSG_HEARTBEAT) return false; /* 不能续租CMD_VEL。 */
    if(h->message_type==MC_MSG_CONTROL_ENABLE) {
        const mc_control_enable_v1_t *e=&r->body.control;
        if(e->clear_latched_faults) { stop(c,b,FOUR_LINK_REJECTED,now); return false; }
        if(!e->enabled) {
            stop(c,b,0,now); return false;
        }
        if(c->expired || c->fault || b->group.fault || b->group.run_used || !c->zero_seen ||
           !c->command_seen || now-c->last_command_ms>=100 ||
           b->group.stationary_ms<500 || now-b->sampled_ms>30) {
            stop(c,b,FOUR_LINK_REJECTED,now); return false;
        }
        c->permitted=true; return false; /* 只开门，不施加PWM。 */
    }
    if(h->message_type!=MC_MSG_CMD_VEL) return false;
    const mc_cmd_vel_v1_t *v=&r->body.velocity;
    const bool zero=!v->vx_mm_s && !v->vy_mm_s && !v->wz_mrad_s;
    if(!(h->flags&MC_FLAG_ENABLED)) {
        if(!zero) { stop(c,b,FOUR_LINK_REJECTED,now); return false; }
        if(b->group.enabled) stop(c,b,0,now);
        if(!c->expired) {
            c->last_command_ms=now; c->last_command_sequence=h->sequence;
            c->command_seen=true; c->zero_seen=true;
        }
        return false;
    }
    /* 初次无线动作只开放F50；不在线改变方向、速度或重启。 */
    if(v->vx_mm_s!=50 || v->vy_mm_s || v->wz_mrad_s || c->expired || c->fault ||
       b->group.fault || !c->permitted || (!b->group.enabled && b->group.run_used)) {
        stop(c,b,FOUR_LINK_REJECTED,now); return false;
    }
    if(!four_drive_heartbeat(&b->group,now)) {
        stop(c,b,b->group.fault ? b->group.fault : FOUR_LINK_REJECTED,now); return false;
    }
    if(!b->group.enabled) {
        const mecanum_body_velocity_t forward={.vx_m_s=0.05f};
        if(!four_drive_arm(&b->group,&forward,now)) {
            stop(c,b,FOUR_LINK_REJECTED,now); return false;
        }
        b->mode=DRIVE_MODE_FORWARD;
    }
    c->last_command_ms=now; c->last_command_sequence=h->sequence; c->command_seen=true;
    return false;
}

static int16_t clipped(float value)
{
    return value>INT16_MAX ? INT16_MAX : value<INT16_MIN ? INT16_MIN : (int16_t)value;
}

bool four_link_bench_payloads(const four_link_bench_t *c, const drive_bench_t *b,
    const speed_bench_diagnostics_t *d, mc_wheel_telemetry_v1_t *t, four_link_bench_status_t *s)
{
    if(!c || !b || !d || !t || !s) return false;
    uint32_t expected[8];
    if(!four_drive_compares(&b->group,expected)) return false;
    for(unsigned i=0;i<8;++i) if(expected[i]!=d->compares[i]) return false;
    memset(t,0,sizeof(*t)); memset(s,0,sizeof(*s));
    t->sample_time_ms=d->ms;
    t->state_flags=FOUR_LINK_CALIBRATED|FOUR_LINK_FOUR|FOUR_LINK_BENCH;
    if(c->session_active) t->state_flags|=FOUR_LINK_SESSION;
    if(b->group.enabled) t->state_flags|=FOUR_LINK_ENABLED;
    else t->state_flags|=FOUR_LINK_ZERO;
    if(c->permitted) t->state_flags|=FOUR_LINK_PERMIT;
    if(c->expired) t->state_flags|=FOUR_LINK_TIMED_OUT;
    if(b->group.run_used) t->state_flags|=FOUR_LINK_USED;
    t->fault_bits=c->fault|b->group.fault;
    t->last_command_sequence=c->last_command_sequence;
    s->tag=FOUR_LINK_BENCH_TAG; s->ticks=d->ticks; s->updates=b->updates;
    s->dt_min=d->dt_min; s->dt_max=d->dt_max; s->irq_us_max=d->irq_us_max;
    s->started_ms=b->group.started_ms; s->stationary_ms=b->group.stationary_ms;
    s->watchdog_reset=d->watchdog_reset; s->permitted=c->permitted;
    s->startup_mask=b->group.startup_mask; s->fault_wheel=b->group.fault_wheel;
    memcpy(s->compares,d->compares,sizeof(s->compares));
    for(unsigned i=0;i<4;++i) {
        const four_speed_wheel_state_t *w=&b->group.loop[i].wheel[i];
        if(!isfinite(w->requested_cps) || !isfinite(w->filtered_cps)) return false;
        t->encoder_counts[i]=b->counts[i]; t->wheel_rpm_x10[i]=clipped(b->rpm_milli[i]/100.0f);
        s->raw_duty[i]=b->group.raw_duty[i];
        s->target_cps[i]=clipped(w->requested_cps); s->filtered_cps[i]=clipped(w->filtered_cps);
    }
    if(b->group.enabled) s->body_milli[0]=50; /* 不是Vicon位姿或电池ADC测量。 */
    return true;
}
