#include "four_link_readonly.h"
#include <limits.h>
#include <string.h>

void four_ro_init(four_ro_control_t *c) { memset(c,0,sizeof(*c)); }

bool four_ro_tick(four_ro_control_t *c, uint32_t now)
{
    if (!c->session_active || !c->command_seen || c->expired ||
        now-c->last_command_ms < 150U) return false;
    c->expired=true; c->fault|=FOUR_RO_TIMEOUT; return true;
}

four_ro_event_t four_ro_accept(four_ro_control_t *c, const uint8_t *frame,
                               size_t length, uint32_t now)
{
    mc_frame_header_v1_t h;
    if (!mc_validate_frame(frame,length,&h) ||
        (h.flags & ~(MC_FLAG_ACK_REQUIRED|MC_FLAG_ENABLED))) return FOUR_RO_NONE;
    (void)four_ro_tick(c,now); /* 迟到数据不能掩盖已过期会话。 */
    if (h.message_type==MC_MSG_HELLO) {
        if (h.payload_length || !h.session_id || (h.flags&MC_FLAG_ENABLED) ||
            (c->session_active && h.session_id==c->session_id &&
             !mc_sequence_is_newer(h.sequence,c->last_sequence))) return FOUR_RO_NONE;
        if (c->session_active && h.session_id==c->session_id && c->expired)
            return FOUR_RO_NONE;
        c->session_active=true; c->session_id=h.session_id; c->last_sequence=h.sequence;
        c->last_command_sequence=0; c->command_seen=false; c->expired=false;
        c->fault&=~FOUR_RO_TIMEOUT; /* 仅新握手恢复链路；UART/急停/禁止动作故障不清。 */
        return FOUR_RO_HELLO;
    }
    if (!c->session_active || h.session_id!=c->session_id ||
        !mc_sequence_is_newer(h.sequence,c->last_sequence)) return FOUR_RO_NONE;
    const uint8_t *p=frame+sizeof(h);
    if (h.message_type==MC_MSG_CMD_VEL) {
        mc_cmd_vel_v1_t cmd;
        if (h.payload_length!=sizeof(cmd)) return FOUR_RO_NONE;
        memcpy(&cmd,p,sizeof(cmd));
        if (cmd.timeout_ms!=150U || cmd.control_mode || cmd.reserved) return FOUR_RO_NONE;
        c->last_sequence=h.sequence;
        if ((h.flags&MC_FLAG_ENABLED) || cmd.vx_mm_s || cmd.vy_mm_s || cmd.wz_mrad_s) {
            c->fault|=FOUR_RO_FORBIDDEN; return FOUR_RO_CONTROL;
        }
        if (c->expired) return FOUR_RO_NONE;
        c->last_command_sequence=h.sequence; c->last_command_ms=now; c->command_seen=true;
        return FOUR_RO_COMMAND;
    }
    if (h.message_type==MC_MSG_CONTROL_ENABLE) {
        mc_control_enable_v1_t cmd;
        if (h.payload_length!=sizeof(cmd) || (h.flags&MC_FLAG_ENABLED)) return FOUR_RO_NONE;
        memcpy(&cmd,p,sizeof(cmd));
        if (cmd.enabled>1 || cmd.clear_latched_faults>1 || cmd.reserved) return FOUR_RO_NONE;
        c->last_sequence=h.sequence;
        if (cmd.enabled || cmd.clear_latched_faults) c->fault|=FOUR_RO_FORBIDDEN;
        return FOUR_RO_CONTROL;
    }
    if (h.payload_length || (h.flags&MC_FLAG_ENABLED)) return FOUR_RO_NONE;
    if (h.message_type==MC_MSG_ESTOP) {
        c->last_sequence=h.sequence; c->fault|=FOUR_RO_ESTOP; return FOUR_RO_CONTROL;
    }
    if (h.message_type==MC_MSG_HEARTBEAT) {
        c->last_sequence=h.sequence; return FOUR_RO_CONTROL; /* 不续租速度数据。 */
    }
    return FOUR_RO_NONE;
}

bool four_ro_payloads(const four_ro_control_t *c, const speed_bench_t *b,
                      const speed_bench_diagnostics_t *d,
                      mc_wheel_telemetry_v1_t *t, four_ro_status_t *s)
{
    if (!c || !b || !d || !t || !s || !b->read_only || b->pi.enabled ||
        !d->gpio_low) return false;
    for(unsigned i=0;i<8;++i) if(d->compares[i]) return false;
    memset(t,0,sizeof(*t)); memset(s,0,sizeof(*s));
    t->sample_time_ms=d->ms;
    t->state_flags=FOUR_RO_CALIBRATED|FOUR_RO_HIGH_Z|FOUR_RO_READONLY|FOUR_RO_FOUR_WHEELS;
    if(c->session_active) t->state_flags|=FOUR_RO_SESSION;
    if(c->expired) t->state_flags|=FOUR_RO_EXPIRED;
    t->fault_bits=c->fault;
    if(b->pi.fault || d->watchdog_reset) t->fault_bits|=FOUR_RO_SAMPLE;
    t->last_command_sequence=c->last_command_sequence;
    for(unsigned i=0;i<4;++i) {
        if(b->pi.wheel[i].raw_duty_permille || b->pi.wheel[i].requested_cps ||
           b->pi.wheel[i].integral_permille) return false;
        const int32_t rpm_x10=b->rpm_milli[i]/100;
        t->wheel_rpm_x10[i]=(int16_t)(rpm_x10>INT16_MAX ? INT16_MAX :
                                     rpm_x10<INT16_MIN ? INT16_MIN : rpm_x10);
        t->encoder_counts[i]=b->counts[i];
    }
    /* 没有电池ADC，不将0mV当作真实测量；主机显示unknown。 */
    s->tag=FOUR_RO_STATUS_TAG; s->ticks=d->ticks; s->updates=b->updates;
    s->dt_min=d->dt_min; s->dt_max=d->dt_max; s->irq_us_max=d->irq_us_max;
    memcpy(s->compares,d->compares,sizeof(s->compares));
    s->gpio_low=d->gpio_low; s->watchdog_reset=d->watchdog_reset;
    return true;
}
