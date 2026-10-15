#include "rectangle_startup_trace.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const int signs[4] = {1, -1, 1, -1};
static rectangle_startup_trace_t trace;

static void step(rectangle_demo_t *r, int32_t raw[4], bool key, uint32_t *now)
{
    const uint8_t before = (uint8_t)r->state;
    *now += 10U;
    (void)rectangle_startup_diag_tick(r, raw, key, *now);
    uint32_t cc[8]; assert(rectangle_demo_compares(r, cc));
    assert(rectangle_trace_capture(&trace, r, raw, cc, before, key, *now));
    for (unsigned i = 0; i < 4; ++i) assert(abs(r->group.raw_duty[i]) <= 100);
}

static void prepare(rectangle_demo_t *r, int32_t raw[4], uint32_t *now)
{
    rectangle_trace_init(&trace);
    assert(rectangle_demo_init(r, raw, *now, false));
    for (unsigned i = 0; i < 60; ++i) step(r, raw, false, now);
    assert(r->state == RECT_READY && trace.count == 0);
    for (unsigned i = 0; i < 110; ++i) step(r, raw, true, now);
    for (unsigned i = 0; i < 10; ++i) step(r, raw, false, now);
    while (r->state == RECT_COUNTDOWN) step(r, raw, false, now);
    assert(r->state == RECT_MOVE && trace.count == 1U);
    assert(trace.rows[0].before_state == RECT_COUNTDOWN);
}

static void feedback(rectangle_demo_t *r, int32_t raw[4], uint32_t *now, int counts)
{
    for (unsigned i = 0; i < 4; ++i) raw[i] += counts * signs[i];
    step(r, raw, false, now);
}

static void idle_and_countdown_do_not_record(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0;
    rectangle_trace_init(&trace); assert(rectangle_demo_init(&r, raw, now, false));
    for (unsigned i = 0; i < 60; ++i) step(&r, raw, false, &now);
    for (unsigned i = 0; i < 110; ++i) step(&r, raw, true, &now);
    for (unsigned i = 0; i < 10; ++i) step(&r, raw, false, &now);
    step(&r, raw, true, &now);
    assert(r.fault == RECT_FAULT_STOP && trace.count == 0U && !trace.capturing);
    assert(!rectangle_startup_diag_tick(NULL, raw, false, now));
}

static void identical_outputs_until_added_deadline(void)
{
    rectangle_demo_t r, reference; int32_t raw[4] = {0}; uint32_t now = 0;
    prepare(&r, raw, &now); reference = r;
    for (unsigned n = 0; n < 100; ++n) {
        for (unsigned i = 0; i < 4; ++i) raw[i] += signs[i];
        (void)rectangle_demo_tick(&reference, raw, false, now + 10U);
        step(&r, raw, false, &now);
        assert(memcmp(&r, &reference, sizeof(r)) == 0);
    }
    step(&r, raw, true, &now);
    assert(r.fault == RECT_FAULT_STOP && !r.group.enabled);
    for (unsigned n = 0; n < 25; ++n) step(&r, raw, false, &now);
    assert(trace.complete && !trace.overflow && !trace.capturing);
    assert(trace.post_stop_rows == RECT_TRACE_POST_STOP_ROWS);
    const uint32_t frozen_count = trace.count;
    for (unsigned n = 0; n < 100; ++n) step(&r, raw, true, &now);
    assert(trace.count == frozen_count && trace.rows[frozen_count - 1U].fault == RECT_FAULT_STOP);
}

static void first_leg_maximum_five_seconds(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0;
    prepare(&r, raw, &now); const uint32_t start = now;
    while (!r.fault) feedback(&r, raw, &now, 3);
    assert(now - start == RECT_TRACE_RUN_LIMIT_MS && r.fault == RECT_TRACE_FAULT_LIMIT);
    assert(r.completed_legs == 0 && !r.group.enabled);
    for (unsigned n = 0; n < 25; ++n) step(&r, raw, false, &now);
    assert(trace.complete && !trace.overflow && trace.count < RECT_TRACE_CAPACITY);
    for (unsigned i = 0; i < 8; ++i) assert(trace.rows[trace.count - 1U].compares[i] == 0);
}

static void never_switches_to_second_leg(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0;
    prepare(&r, raw, &now);
    while (!r.fault) feedback(&r, raw, &now, 20);
    assert(r.completed_legs == 1U && r.fault == RECT_TRACE_FAULT_LIMIT && !r.group.enabled);
    for (unsigned n = 0; n < 300; ++n) step(&r, raw, false, &now);
    assert(r.completed_legs == 1U && !r.group.enabled);
}

static void observed_rr_no_progress_replay(void)
{
    rectangle_demo_t r; int32_t raw[4] = {84, -2, 1, 0}; uint32_t now = 0;
    prepare(&r, raw, &now); const uint32_t start = now;
    while (!r.fault) {
        if (now - start == 80U) raw[3] -= 4;
        step(&r, raw, false, &now);
    }
    assert(r.fault == FOUR_SPEED_FAULT_STALL && r.group.fault_wheel == 4U);
    assert(now - start == 590U && r.group.loop[3].last_progress_ms == start + 90U);
    assert(trace.rows[1].assist_mask == 15U);
    assert(trace.rows[1].duty_permille[3] == -80);
    assert(trace.rows[20].assist_mask == 0U);
    /* 合成无进度输入重现期限；并不证明真实电机阻力或动力电压。 */
    for (unsigned n = 0; n < 25; ++n) step(&r, raw, false, &now);
    assert(trace.complete && !trace.overflow);
}

static void capture_bounds_and_wrap(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = UINT32_MAX - 200U;
    prepare(&r, raw, &now);
    assert(!rectangle_trace_capture(NULL, &r, raw, (uint32_t[8]){0}, 5, false, now));
    assert(!rectangle_trace_capture(&trace, NULL, raw, (uint32_t[8]){0}, 5, false, now));
    assert(!rectangle_trace_capture(&trace, &r, NULL, (uint32_t[8]){0}, 5, false, now));
    assert(!rectangle_trace_capture(&trace, &r, raw, NULL, 5, false, now));
    assert(!rectangle_trace_capture(&trace, &r, raw, (uint32_t[8]){101}, 5, false, now));
    r.group.loop[0].wheel[0].filtered_cps = NAN;
    assert(!rectangle_trace_capture(&trace, &r, raw, (uint32_t[8]){0}, 5, false, now));
    r.group.loop[0].wheel[0].filtered_cps = 0;
    trace.count = RECT_TRACE_CAPACITY;
    assert(!rectangle_trace_capture(&trace, &r, raw, (uint32_t[8]){0}, 5, false, now));
    assert(trace.complete && trace.overflow);
    assert(!rectangle_trace_capture(&trace, &r, raw, (uint32_t[8]){0}, 5, false, now));
    memset(raw, 0, sizeof(raw)); now = UINT32_MAX - 200U;
    prepare(&r, raw, &now); const uint32_t start = now;
    while (!r.fault) feedback(&r, raw, &now, 3);
    assert(now - start == RECT_TRACE_RUN_LIMIT_MS && r.fault == RECT_TRACE_FAULT_LIMIT);
}

int main(void)
{
    idle_and_countdown_do_not_record();
    identical_outputs_until_added_deadline();
    first_leg_maximum_five_seconds();
    never_switches_to_second_leg();
    observed_rr_no_progress_replay();
    capture_bounds_and_wrap();
    puts("rectangle_startup_trace: 6 groups passed (offline synthetic inputs only)");
    return 0;
}
