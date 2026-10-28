/* Separate, not yet flashed streaming controller. Existing bench images unchanged. */
#include "four_link_bench.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error This module belongs only to the bounded streaming target.
#endif

static unsigned requested_mode(uint16_t value)
{ return value>=SERVO_TIER_MAGIC && value<SERVO_TIER_MAGIC+SERVO_STAGE_COUNT ? 1U :
    value>=SERVO_BENCH_MAGIC && value<SERVO_BENCH_MAGIC+SERVO_STAGE_COUNT ? 2U : 0U; }
static unsigned requested_stage(uint16_t value)
{ return requested_mode(value)==1U ? (unsigned)value-(unsigned)SERVO_TIER_MAGIC : (unsigned)value-SERVO_BENCH_MAGIC; }

void four_link_bench_init(four_link_bench_t *c) { memset(c, 0, sizeof(*c)); }

bool four_link_bench_decode(const uint8_t *raw, size_t n, four_link_request_t *r)
{
    if (!r) return false;
    memset(r, 0, sizeof(*r));
    if (!mc_validate_frame(raw, n, &r->header) || !r->header.session_id ||
        (r->header.flags & ~(MC_FLAG_ACK_REQUIRED | MC_FLAG_ENABLED))) return false;
    const mc_frame_header_v1_t *h = &r->header;
    const uint8_t *p = raw + sizeof(*h);
    switch (h->message_type) {
    case MC_MSG_CMD_VEL:
        if (h->payload_length != sizeof(r->body.velocity)) return false;
        memcpy(&r->body.velocity, p, sizeof(r->body.velocity));
        return r->body.velocity.timeout_ms == FOUR_DRIVE_COMMAND_TIMEOUT_MS &&
               !r->body.velocity.reserved &&
               !r->body.velocity.control_mode;
    case MC_MSG_CONTROL_ENABLE:
        if (h->payload_length != sizeof(r->body.control) || (h->flags & MC_FLAG_ENABLED)) return false;
        memcpy(&r->body.control, p, sizeof(r->body.control));
        return r->body.control.enabled <= 1 && r->body.control.clear_latched_faults <= 1 &&
               (!r->body.control.reserved ||
                requested_mode(r->body.control.reserved));
    case MC_MSG_HELLO: case MC_MSG_HEARTBEAT: case MC_MSG_ESTOP:
        return !h->payload_length && !(h->flags & MC_FLAG_ENABLED);
    default: return false;
    }
}

static void stop(four_link_bench_t *c, drive_bench_t *b, uint32_t fault, uint32_t now)
{
    c->fault |= fault;
    c->permitted = c->zero_seen = c->transitioning = false;
    four_drive_stop(&b->group, fault, now);
    b->mode = DRIVE_MODE_IDLE;
}

void four_link_bench_deadline(four_link_bench_t *c, drive_bench_t *b, uint32_t now)
{
    if (c->session_active && c->command_seen && !c->expired &&
        now - c->last_command_ms >= FOUR_DRIVE_COMMAND_TIMEOUT_MS) {
        c->expired = true;
        /* A lease gap is a fault only while the actuator gate is owned.  An
         * unpermitted disabled-zero session may expire while the operator is
         * still positioning the car; a new HELLO must recover it without a
         * host-side fault reset.  Direction settling and armed zero retain the
         * permit, so they still fail closed on the profile's fixed deadline. */
        if (c->permitted || b->group.enabled) {
            c->fault |= FOUR_LINK_EXPIRED;
            stop(c, b, FOUR_SPEED_FAULT_COMMAND, now);
        } else {
            c->zero_seen = false;
        }
    }
    if (c->permitted && c->motion_started && c->action_mode==2U &&
        now-c->first_motion_ms>=SERVO_BENCH_LIMIT_MS) {
        c->bench_completed=true;
        stop(c,b,0,now); /* Normal one-shot completion, NOT a fault clear. */
    }
    if (c->permitted && c->motion_started && now - c->first_motion_ms >= FOUR_LINK_MOTION_LIMIT_MS)
        stop(c, b, FOUR_SPEED_FAULT_DURATION, now);
    if (b->group.fault) stop(c, b, b->group.fault, now);
}

static bool plan_for(const drive_bench_t *b, const mc_cmd_vel_v1_t *v, mecanum_drive_plan_t *p)
{
    if (v->vx_mm_s < -FOUR_DRIVE_MAX_BODY_MM_S ||
        v->vx_mm_s > FOUR_DRIVE_MAX_BODY_MM_S ||
        v->vy_mm_s < -FOUR_DRIVE_MAX_BODY_MM_S ||
        v->vy_mm_s > FOUR_DRIVE_MAX_BODY_MM_S ||
        v->wz_mrad_s < -FOUR_DRIVE_MAX_YAW_MRAD_S ||
        v->wz_mrad_s > FOUR_DRIVE_MAX_YAW_MRAD_S) return false;
    const mecanum_body_velocity_t body = {v->vx_mm_s / 1000.0f, v->vy_mm_s / 1000.0f,
                                         v->wz_mrad_s / 1000.0f};
    if (!mecanum_drive_plan(&b->group.geometry, &b->adopted, &body,
                            FOUR_DRIVE_MAX_WHEEL_M_S, p)) return false;
    servo_stage_t stage;
    if (!servo_stage_get(b->group.servo_stage,&stage) ||
        hypotf(body.vx_m_s,body.vy_m_s)>stage.speed_m_s+.000001f) return false;
    four_drive_normalize_plan(p);
    return true;
}

bool four_link_bench_process(four_link_bench_t *c, drive_bench_t *b,
                             const four_link_request_t *r, uint32_t now)
{
    if (!c || !b || !r) return false;
    four_link_bench_deadline(c, b, now); /* No late packet may revive an expired action. */
    const mc_frame_header_v1_t *h = &r->header;
    if (h->message_type == MC_MSG_HELLO) {
        if (c->session_active && h->session_id == c->session_id &&
            (!mc_sequence_is_newer(h->sequence, c->last_sequence) || c->expired)) return false;
        if (c->permitted || b->group.enabled) stop(c, b, FOUR_LINK_REJECTED, now);
        c->session_active = true; c->session_id = h->session_id; c->last_sequence = h->sequence;
        c->command_seen = c->zero_seen = c->expired = c->permitted = false;
        c->last_command_sequence = 0;
        c->tier_selected=false; c->action_mode=0;
        c->source_anchor_ms=h->sender_time_ms; c->local_anchor_ms=now;
        c->last_source_elapsed=0;
        /* Idle timeout can be recovered with HELLO, never actuator faults/run allowance. */
        c->fault &= ~FOUR_LINK_EXPIRED;
        return true;
    }
    if (!c->session_active || h->session_id != c->session_id ||
        !mc_sequence_is_newer(h->sequence, c->last_sequence)) return false;
    c->last_sequence = h->sequence;
    if (h->message_type == MC_MSG_ESTOP) { stop(c, b, SPEED_BENCH_FAULT_ESTOP, now); return false; }
    if (h->message_type == MC_MSG_HEARTBEAT) return false; /* Not a velocity lease. */
    /* Relative source/receive skew guard, not synchronised exposure-to-output
     * latency. An old HELLO's initial delay remains unknown. No lease renewal
     * by a newer-sequence packet after a >120ms additional queue delay. */
    const uint32_t source_elapsed=h->sender_time_ms-c->source_anchor_ms;
    const uint32_t local_elapsed=now-c->local_anchor_ms;
    const int64_t skew=(int64_t)local_elapsed-source_elapsed;
    if (source_elapsed>INT32_MAX || local_elapsed>INT32_MAX ||
        source_elapsed<c->last_source_elapsed ||
        skew>=(int64_t)SERVO_SOURCE_LAG_LIMIT_MS ||
        skew<-(int64_t)SERVO_SOURCE_LEAD_LIMIT_MS) {
        stop(c,b,SERVO_LINK_SOURCE_AGE,now); return false;
    }
    c->last_source_elapsed=source_elapsed;
    if (h->message_type == MC_MSG_CONTROL_ENABLE) {
        const mc_control_enable_v1_t *e = &r->body.control;
        const unsigned mode=requested_mode(e->reserved);
        const bool tier_request=mode!=0;
        if (tier_request && !e->enabled) {
            if (e->clear_latched_faults || c->tier_selected || c->permitted ||
                c->expired || c->fault || b->group.fault || c->motion_started ||
                !c->zero_seen || !c->command_seen || now-c->last_command_ms>=100 ||
                !four_drive_select_stage(&b->group,requested_stage(e->reserved),now)) {
                stop(c,b,FOUR_LINK_REJECTED,now); return false;
            }
            servo_stage_t stage;
            (void)servo_stage_get(b->group.servo_stage,&stage);
            b->adopted.duty_limit_permille=stage.duty_permille;
            b->adopted.launch_slew_permille_per_s=stage.output_slew_permille_s;
            c->tier_selected=true; c->action_mode=(uint8_t)mode;
            c->action_session_id=h->session_id;
            return false;
        }
        if (e->reserved && (!tier_request || e->clear_latched_faults)) {
            stop(c,b,FOUR_LINK_REJECTED,now); return false;
        }
        if (e->clear_latched_faults) {
#if defined(FOUR_LINK_GROUND9) || defined(FOUR_LINK_GROUNDA) || \
    defined(FOUR_LINK_GROUNDB)
            /* This command is a safe run-budget rearm, not a general fault
             * clear.  It requires a fresh disabled-zero lease plus measured
             * standstill, and refuses every real actuator/link fault. */
            if ((c->bench_completed && h->session_id==c->action_session_id) ||
                e->enabled || c->permitted || c->expired || c->fault ||
                b->group.fault || !c->zero_seen || !c->command_seen ||
                now - c->last_command_ms >= 100U ||
                !four_drive_rearm(&b->group, now)) {
                stop(c, b, FOUR_LINK_REJECTED, now); return false;
            }
            c->motion_started = c->transitioning = false;
            c->bench_completed=false;
            c->first_motion_ms = 0U;
            return false;
#else
            stop(c, b, FOUR_LINK_REJECTED, now); return false;
#endif
        }
        if (!e->enabled) { stop(c, b, 0, now); return false; }
        if (!tier_request || !c->tier_selected ||
            mode!=c->action_mode || b->group.servo_stage!=requested_stage(e->reserved)) {
            stop(c,b,FOUR_LINK_REJECTED,now); return false;
        }
        /* Fixed stage/mode must match this fresh session; never auto-promote. */
        /* Repeated enables are not keepalives, and cannot restart/reset a run. */
        if (c->permitted) return false;
        if (c->expired || c->fault || b->group.fault || c->motion_started ||
            !c->zero_seen || !c->command_seen || now - c->last_command_ms >= 100 ||
            b->group.stationary_ms < 500 || now - b->sampled_ms > 30) {
            stop(c, b, FOUR_LINK_REJECTED, now); return false;
        }
        c->permitted = true;
        return false;
    }
    if (h->message_type != MC_MSG_CMD_VEL) return false;
    const mc_cmd_vel_v1_t *v = &r->body.velocity;
    const bool zero = !v->vx_mm_s && !v->vy_mm_s && !v->wz_mrad_s;
    if ((h->flags & MC_FLAG_ENABLED) && c->bench_completed) return false;
    if (!(h->flags & MC_FLAG_ENABLED)) {
        if (!zero) { stop(c, b, FOUR_LINK_REJECTED, now); return false; }
        if (c->permitted || b->group.enabled) stop(c, b, 0, now);
        if (!c->expired) {
            c->last_command_ms = now; c->last_command_sequence = h->sequence;
            c->command_seen = c->zero_seen = true;
        }
        return false;
    }
    mecanum_drive_plan_t plan;
    if (!c->permitted || c->expired || c->fault || b->group.fault || !plan_for(b, v, &plan)) {
        stop(c, b, FOUR_LINK_REJECTED, now); return false;
    }
    bool effective_zero = true;
    for (unsigned i = 0; i < 4; ++i)
        effective_zero &= plan.forward_cps[i] == 0.0f;
    if (now - b->sampled_ms > 30) { stop(c, b, FOUR_SPEED_FAULT_TIMING, now); return false; }
    c->last_command_ms = now; c->last_command_sequence = h->sequence; c->command_seen = true;
    if (b->group.enabled) {
        /* Atomic four-wheel demand commit; each loop owns its bounded,
         * measured-neutral reversal. A normal sign change is not a fault
         * and cannot reset the independent absolute motion budget. */
        if (!four_drive_retarget(&b->group, &plan, now))
            stop(c, b, b->group.fault ? b->group.fault : FOUR_LINK_REJECTED, now);
        return false;
    }
    if (zero || effective_zero)
        return false; /* A deadband-only command renews the lease without starting a run. */
    if (c->transitioning) {
        /* Require measured standstill, not merely elapsed dead time. Latest command wins. */
        if (b->group.stationary_ms < 500 || now - b->group.stopped_ms < 500) return false;
        const mecanum_drive_geometry_t geometry = b->group.geometry;
        const uint32_t stationary_ms = b->group.stationary_ms, stopped_ms = b->group.stopped_ms;
        if (!four_drive_init(&b->group, &geometry, &b->adopted, b->counts, now)) {
            stop(c, b, FOUR_SPEED_FAULT_CONFIG, now); return false;
        }
        b->group.stationary_ms = stationary_ms;
        b->group.stopped_ms = stopped_ms;
        c->transitioning = false;
    }
#if defined(FOUR_LINK_GROUND6) || defined(FOUR_LINK_GROUND7) || \
    defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9) || \
    defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
    float largest_target = 0.0f;
    for (unsigned i = 0; i < 4; ++i) {
        const float magnitude = fabsf(plan.forward_cps[i]);
        if (magnitude > largest_target) largest_target = magnitude;
    }
    if (largest_target < FOUR_DRIVE_START_THRESHOLD_CPS)
        return false; /* Fresh lease, but no output and no consumed run allowance. */
#endif
    if (!four_drive_heartbeat(&b->group, now) ||
        !four_drive_arm(&b->group, &plan.applied_body, now)) {
        stop(c, b, b->group.fault ? b->group.fault : FOUR_LINK_REJECTED, now); return false;
    }
    b->mode = DRIVE_MODE_FORWARD; /* Generic enabled marker; body fields carry real axes. */
    if (!c->motion_started) { c->first_motion_ms = now; c->motion_started = true; }
    return false;
}

static int16_t clipped(float v)
{ return v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : (int16_t)v; }

static int32_t milli_cps(float v)
{
    const double scaled = (double)v * 1000.0;
    return scaled > INT32_MAX ? INT32_MAX : scaled < INT32_MIN ? INT32_MIN : (int32_t)scaled;
}

bool four_link_bench_payloads(const four_link_bench_t *c, const drive_bench_t *b,
    const speed_bench_diagnostics_t *d, mc_wheel_telemetry_v1_t *t, four_link_bench_status_t *s)
{
    if (!c || !b || !d || !t || !s) return false;
    uint32_t compares[8];
    if (!four_drive_compares(&b->group, compares) || memcmp(compares, d->compares, sizeof(compares)))
        return false;
    memset(t, 0, sizeof(*t)); memset(s, 0, sizeof(*s));
    t->sample_time_ms = d->ms;
    t->state_flags = FOUR_LINK_CALIBRATED | FOUR_LINK_FOUR | FOUR_LINK_BENCH;
    if (c->session_active) t->state_flags |= FOUR_LINK_SESSION;
    t->state_flags |= b->group.enabled ? FOUR_LINK_ENABLED : FOUR_LINK_ZERO;
    if (c->permitted) t->state_flags |= FOUR_LINK_PERMIT;
    if (c->expired) t->state_flags |= FOUR_LINK_TIMED_OUT;
    if (c->motion_started) t->state_flags |= FOUR_LINK_USED;
    t->fault_bits = c->fault | b->group.fault;
    t->last_command_sequence = c->last_command_sequence;
    s->tag = FOUR_LINK_BENCH_TAG; s->ticks = d->ticks; s->updates = b->updates;
    s->dt_min = d->dt_min; s->dt_max = d->dt_max; s->irq_us_max = d->irq_us_max;
    s->started_ms = c->first_motion_ms; s->stationary_ms = b->group.stationary_ms;
    s->watchdog_reset = d->watchdog_reset; s->permitted = c->permitted;
    s->startup_mask = b->group.startup_mask; s->fault_wheel = b->group.fault_wheel;
    servo_stage_t stage;
    if (!servo_stage_get(b->group.servo_stage,&stage)) return false;
    s->output_authority_version = 4U;
    s->servo_reserved=(uint8_t)(c->action_mode | (c->bench_completed ? 0x80U : 0U));
    s->stage_id=b->group.servo_stage; s->tier_selected=c->tier_selected;
    s->stage_duty_limit=(uint16_t)stage.duty_permille;
    s->vector_accel_limit=stage.accel_m_s2;
    s->output_slew_limit=stage.output_slew_permille_s;
    s->shaped_body_si[0]=b->group.shaped_body.vx_m_s;
    s->shaped_body_si[1]=b->group.shaped_body.vy_m_s;
    s->shaped_body_si[2]=b->group.shaped_body.wz_rad_s;
    s->hardware_released=SERVO_EXECUTION_RELEASED;
    for (unsigned i = 0; i < 4; ++i) {
        const four_feedback_snapshot_t *f = &b->group.loop[i].first_feedback_fault;
        if (!f->valid) continue;
        four_feedback_wire_t *w = &s->first_feedback_fault;
        w->ms=f->ms; w->dt_ms=f->dt_ms; w->raw_delta=f->raw_delta;
        w->measured_milli_cps=milli_cps(f->measured_cps);
        w->requested_milli_cps=milli_cps(f->requested_cps);
        w->effective_milli_cps=milli_cps(f->effective_cps);
        w->reason=f->reason; w->wheel=(uint16_t)(i+1U); w->duty_permille=f->duty_permille;
        w->opposite_counts=f->opposite_counts>UINT16_MAX ? UINT16_MAX : (uint16_t)f->opposite_counts;
        break;
    }
    memcpy(s->compares, compares, sizeof(compares));
    for (unsigned i = 0; i < 4; ++i) {
        const four_speed_wheel_state_t *w = &b->group.loop[i].wheel[i];
        if (!isfinite(w->requested_cps) || !isfinite(w->filtered_cps)) return false;
        s->effective_cps[i] = w->ramped_cps;
        if (w->output_saturated) s->saturated_mask |= (uint8_t)(1U<<i);
        if (b->group.loop[i].rejoining) s->rejoining_mask |= (uint8_t)(1U<<i);
        s->granted_direction[i] = b->group.loop[i].run_direction;
        if (b->group.loop[i].reversing) s->reversing_mask |= (uint8_t)(1U << i);
        t->encoder_counts[i] = b->counts[i]; t->wheel_rpm_x10[i] = clipped(b->rpm_milli[i] / 100.0f);
        s->raw_duty[i] = b->group.raw_duty[i];
        s->target_cps[i] = clipped(w->requested_cps); s->filtered_cps[i] = clipped(w->filtered_cps);
    }
    if (b->group.enabled) {
        s->body_milli[0] = clipped(b->group.plan.applied_body.vx_m_s * 1000);
        s->body_milli[1] = clipped(b->group.plan.applied_body.vy_m_s * 1000);
        s->body_milli[2] = clipped(b->group.plan.applied_body.wz_rad_s * 1000);
    }
    return true;
}
