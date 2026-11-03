#include "rr_diagnostic_trace.h"
#include <math.h>
#include <stdio.h>

bool rr_trace_capture(rr_trace_t *t, const drive_bench_t *b, const uint32_t cc[8], uint32_t now)
{
    if (t == NULL || b == NULL || cc == NULL) return false;
    if (t->complete || (!b->group.enabled && !t->capturing)) return true;
    t->capturing = true;
    if (t->count >= RR_TRACE_CAPACITY) {
        t->overflow = t->complete = true; t->capturing = false; return false;
    }
    const four_speed_wheel_state_t *s = &b->group.loop[3].wheel[3];
    const float integral = s->integral_permille * 1000.0f;
    if (!isfinite(integral) || fabsf(integral) > 1000000.0f ||
        !isfinite(s->filtered_cps) || fabsf(s->filtered_cps) > 10000.0f ||
        !isfinite(s->ramped_cps) || fabsf(s->ramped_cps) > FOUR_DRIVE_TARGET_LIMIT_CPS ||
        cc[6] > FOUR_SPEED_PWM_COUNTS || cc[7] > FOUR_SPEED_PWM_COUNTS)
        return false;
    uint8_t other = 0;
    for (unsigned i = 0; i < 3; ++i)
        if (cc[2*i] || cc[2*i+1]) other |= (uint8_t)(1U << i);
    t->rows[t->count++] = (rr_trace_row_t){
        .ms=now, .fault=b->group.fault, .enc=b->counts[3], .cps=b->raw_cps[3],
        .fcps=(int32_t)s->filtered_cps, .target=(int32_t)s->ramped_cps,
        .integral_milli=(int32_t)integral, .duty=b->group.raw_duty[3],
        .p1=(uint16_t)cc[6], .p2=(uint16_t)cc[7], .enabled=b->group.enabled,
        .assist=b->group.startup_mask, .other_outputs=other};
    if (!b->group.enabled) { t->capturing = false; t->complete = true; }
    return other == 0;
}

bool rr_trace_format(const rr_trace_t *t, unsigned i, char *out, size_t capacity)
{
    if (t == NULL || i >= t->count || out == NULL || !capacity) return false;
    const rr_trace_row_t *r = &t->rows[i];
    const int n = snprintf(out, capacity,
        "RRTRACE,i=%u,ms=%lu,enc=%ld,cps=%ld,fcps=%ld,target=%ld,im=%ld,"
        "d=%d,p1=%u,p2=%u,en=%u,assist=%u,other=%u,fault=0x%08lx\r\n",
        i, (unsigned long)r->ms, (long)r->enc, (long)r->cps, (long)r->fcps,
        (long)r->target, (long)r->integral_milli, (int)r->duty,
        (unsigned)r->p1, (unsigned)r->p2, (unsigned)r->enabled,
        (unsigned)r->assist, (unsigned)r->other_outputs, (unsigned long)r->fault);
    return n >= 0 && (size_t)n < capacity;
}
