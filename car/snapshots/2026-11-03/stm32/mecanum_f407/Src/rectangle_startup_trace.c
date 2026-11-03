#include "rectangle_startup_trace.h"
#include <math.h>
#include <string.h>

void rectangle_trace_init(rectangle_startup_trace_t *t)
{
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));
    t->magic = RECT_TRACE_MAGIC; t->version = 1U;
    t->row_bytes = sizeof(rectangle_trace_row_t); t->capacity = RECT_TRACE_CAPACITY;
}

bool rectangle_startup_diag_tick(rectangle_demo_t *r, const int32_t counts[4],
                                bool pressed, uint32_t now)
{
    if (r == NULL) return false;
    /* 原采样/按键/失速/期限检查优先，诊断只能加严，不能救活故障。 */
    const bool ok = rectangle_demo_tick(r, counts, pressed, now);
    if (r->fault == 0U && (r->completed_legs != 0U ||
        (r->state == RECT_MOVE && now - r->phase_ms >= RECT_TRACE_RUN_LIMIT_MS))) {
        rectangle_demo_stop(r, RECT_TRACE_FAULT_LIMIT, now);
        return false;
    }
    return ok;
}

bool rectangle_trace_capture(rectangle_startup_trace_t *t, const rectangle_demo_t *r,
                             const int32_t counts[4], const uint32_t cc[8],
                             uint8_t before, bool pressed, uint32_t now)
{
    if (t == NULL || r == NULL || counts == NULL || cc == NULL) return false;
    if (t->magic != RECT_TRACE_MAGIC || t->version != 1U ||
        t->row_bytes != sizeof(rectangle_trace_row_t) || t->capacity != RECT_TRACE_CAPACITY ||
        t->count > RECT_TRACE_CAPACITY || t->post_stop_rows > RECT_TRACE_POST_STOP_ROWS)
        return false;
    if (t->complete) return !t->overflow;
    if (!t->capturing && r->state != RECT_MOVE && before != RECT_MOVE) return true;
    if (t->count >= RECT_TRACE_CAPACITY) {
        t->overflow = t->complete = 1U; t->capturing = 0U;
        return false;
    }
    rectangle_trace_row_t row = {.ms = now, .fault = r->fault,
        .state = (uint8_t)r->state, .enabled = r->group.enabled, .key = pressed,
        .assist_mask = r->group.startup_mask, .fault_wheel = r->group.fault_wheel,
        .legs = r->completed_legs, .before_state = before};
    for (unsigned i = 0; i < 4; ++i) {
        const four_speed_wheel_state_t *s = &r->group.loop[i].wheel[i];
        if (!isfinite(s->filtered_cps) || fabsf(s->filtered_cps) > 10000.0f ||
            !isfinite(s->ramped_cps) || fabsf(s->ramped_cps) > FOUR_DRIVE_TARGET_LIMIT_CPS ||
            !isfinite(s->integral_permille) || fabsf(s->integral_permille) > 100.0f ||
            cc[2U*i] > FOUR_SPEED_PWM_COUNTS || cc[2U*i+1U] > FOUR_SPEED_PWM_COUNTS)
            return false;
        row.counts[i] = counts[i];
        row.filtered_cps[i] = s->filtered_cps;
        row.ramped_cps[i] = s->ramped_cps;
        row.integral_permille[i] = s->integral_permille;
        row.duty_permille[i] = r->group.raw_duty[i];
        row.compares[2U*i] = (uint16_t)cc[2U*i];
        row.compares[2U*i+1U] = (uint16_t)cc[2U*i+1U];
    }
    if (!t->capturing) {
        t->capturing = 1U; t->started_ms = r->phase_ms;
        memcpy(t->target_cps, r->group.plan.forward_cps, sizeof(t->target_cps));
    }
    t->rows[t->count++] = row;
    if (!r->group.enabled) {
        if (t->post_stop_rows == 0U) t->stopped_ms = now;
        if (++t->post_stop_rows >= RECT_TRACE_POST_STOP_ROWS) {
            t->capturing = 0U; t->complete = 1U;
        }
    }
    return true;
}
