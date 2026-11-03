#include "rectangle_demo.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const int signs[4] = {1, -1, 1, -1};

static void zero(const rectangle_demo_t *r)
{
    uint32_t cc[8];
    assert(rectangle_demo_compares(r, cc));
    assert(!r->group.enabled);
    for (unsigned i = 0; i < 8; ++i) assert(cc[i] == 0);
}

static void step(rectangle_demo_t *r, int32_t raw[4], bool pressed, uint32_t *now)
{
    *now += 10U;
    (void)rectangle_demo_tick(r, raw, pressed, *now);
    uint32_t cc[8]; assert(rectangle_demo_compares(r, cc));
    for (unsigned i = 0; i < 4; ++i) assert(abs(r->group.raw_duty[i]) <= 100);
}

static void wait(rectangle_demo_t *r, int32_t raw[4], bool pressed, uint32_t *now, unsigned ms)
{
    for (unsigned dt = 0; dt < ms; dt += 10) step(r, raw, pressed, now);
}

static void prepare(rectangle_demo_t *r, int32_t raw[4], uint32_t *now)
{
    assert(rectangle_demo_init(r, raw, *now, false));
    wait(r, raw, false, now, 600);
    assert(r->state == RECT_READY); zero(r);
    wait(r, raw, true, now, 1100);
    assert(r->state == RECT_ARMED_RELEASE); zero(r);
    wait(r, raw, false, now, 100);
    assert(r->state == RECT_COUNTDOWN); zero(r);
    while (r->state == RECT_COUNTDOWN) step(r, raw, false, now);
    assert(r->state == RECT_MOVE && r->completed_legs == 0 && r->used);
}

static void advance(rectangle_demo_t *r, int32_t raw[4], uint32_t *now, float cps)
{
    for (unsigned i = 0; i < 4; ++i) {
        const float target = r->group.plan.forward_cps[i];
        raw[i] += (target > 0 ? 1 : -1) * signs[i] * (int32_t)cps;
    }
    step(r, raw, false, now);
}

static void startup_and_one_complete_rectangle(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0;
    assert(rectangle_demo_init(&r, raw, now, false));
    /* 启动时一直按住：不能在上电或看门狗复位后意外启动。 */
    wait(&r, raw, true, &now, 10000); assert(r.state == RECT_BOOT_RELEASE); zero(&r);
    wait(&r, raw, false, &now, 600); assert(r.state == RECT_READY);
    wait(&r, raw, true, &now, 500); wait(&r, raw, false, &now, 100);
    assert(r.state == RECT_READY && !r.used); zero(&r);
    prepare(&r, raw, &now);
    for (unsigned leg = 0; leg < RECT_LEGS; ++leg) {
        assert(r.state == RECT_MOVE && r.completed_legs == leg);
        const int expected[4][4] = {{1,1,1,1}, {-1,1,1,-1}, {-1,-1,-1,-1}, {1,-1,-1,1}};
        for (unsigned i = 0; i < 4; ++i)
            assert(r.group.plan.forward_cps[i] * (float)expected[leg][i] > 0);
        unsigned samples = 0;
        /* 理想整数编码器输入；不是地面动力学或1米实测。 */
        while (r.state == RECT_MOVE && samples++ < 3000) advance(&r, raw, &now, 3);
        assert(r.completed_legs == leg + 1 && r.fault == 0);
        assert(r.progress_m >= 1.0f && r.progress_m < 1.01f); zero(&r);
        if (leg + 1 < RECT_LEGS) {
            assert(r.state == RECT_PAUSE);
            while (r.state == RECT_PAUSE) step(&r, raw, false, &now);
            assert(r.state == RECT_MOVE);
        }
    }
    assert(r.state == RECT_COMPLETE); zero(&r);
    wait(&r, raw, true, &now, 10000); wait(&r, raw, false, &now, 10000);
    assert(r.state == RECT_COMPLETE); zero(&r); /* 不自动第二圈、不按键重试。 */
}

static void button_stop_and_countdown(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0;
    assert(rectangle_demo_init(&r, raw, 0, false));
    wait(&r, raw, false, &now, 600); wait(&r, raw, true, &now, 1100);
    wait(&r, raw, false, &now, 100); assert(r.state == RECT_COUNTDOWN);
    step(&r, raw, true, &now); assert(r.fault == RECT_FAULT_STOP); zero(&r);
    wait(&r, raw, false, &now, 10000); assert(r.state == RECT_FAULT); zero(&r);
    now = 0; memset(raw, 0, sizeof(raw)); prepare(&r, raw, &now);
    advance(&r, raw, &now, 3); step(&r, raw, true, &now);
    assert(r.fault == RECT_FAULT_STOP); zero(&r);
    now = 0; memset(raw, 0, sizeof(raw)); prepare(&r, raw, &now);
    while (r.state == RECT_MOVE) advance(&r, raw, &now, 3);
    assert(r.state == RECT_PAUSE); step(&r, raw, true, &now);
    assert(r.fault == RECT_FAULT_STOP); zero(&r);
}

static void faults_and_deadlines(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0;
    assert(!rectangle_demo_init(&r, raw, 0, true));
    assert(r.fault == RECT_FAULT_WATCHDOG); zero(&r);
    assert(!rectangle_demo_init(&r, NULL, 0, false)); zero(&r);
    assert(!rectangle_demo_init(NULL, raw, 0, false));
    assert(!rectangle_demo_tick(NULL, raw, false, 0));
    prepare(&r, raw, &now); now += 31;
    assert(!rectangle_demo_tick(&r, raw, false, now));
    assert(r.fault == FOUR_SPEED_FAULT_TIMING); zero(&r);
    now = 0; prepare(&r, raw, &now);
    assert(!rectangle_demo_tick(&r, NULL, false, now + 10)); zero(&r);
    now = 0; prepare(&r, raw, &now);
    wait(&r, raw, false, &now, 900);
    assert(r.fault & FOUR_SPEED_FAULT_STALL); zero(&r);
    now = 0; prepare(&r, raw, &now); raw[0] -= 1;
    step(&r, raw, false, &now); assert(r.fault & FOUR_SPEED_FAULT_FEEDBACK); zero(&r);
    now = 0; memset(raw,0,sizeof(raw)); prepare(&r, raw, &now);
    r.phase_ms = now - RECT_LEG_LIMIT_MS + 10U;
    for(unsigned i=0;i<4;++i) raw[i] += 10000 * signs[i];
    step(&r, raw, false, &now); assert(r.fault == RECT_FAULT_LEG_TIMEOUT); zero(&r);
    now = 0; memset(raw,0,sizeof(raw)); prepare(&r, raw, &now);
    r.sequence_ms = now - RECT_TOTAL_LIMIT_MS + 10U;
    step(&r, raw, false, &now); assert(r.fault == RECT_FAULT_TOTAL_TIMEOUT); zero(&r);
    now = 0; prepare(&r, raw, &now); r.state = (rectangle_state_t)99;
    step(&r, raw, false, &now); assert(r.fault == RECT_FAULT_STATE); zero(&r);
    now = 0; prepare(&r, raw, &now); r.completed_legs = 4;
    step(&r, raw, false, &now); assert(r.fault == RECT_FAULT_STATE); zero(&r);
    now = 0; prepare(&r, raw, &now); r.progress_m = NAN;
    advance(&r, raw, &now, 3); assert(r.fault == RECT_FAULT_ODOMETRY); zero(&r);
    now = 0; prepare(&r, raw, &now); r.yaw_rad = 0.36f;
    advance(&r, raw, &now, 3); assert(r.fault == RECT_FAULT_ODOMETRY); zero(&r);
}

static void time_wrap_and_physical_release(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = UINT32_MAX - 300;
    prepare(&r, raw, &now); advance(&r, raw, &now, 3);
    assert(r.group.enabled && r.fault == 0);
    rectangle_demo_stop(&r, 0, now); assert(r.state == RECT_STOPPED); zero(&r);
    /* 软复位/重新初始化时，即使KEY仍按下也不能重启运动。 */
    assert(rectangle_demo_init(&r, raw, now, false));
    wait(&r, raw, true, &now, 20000); assert(r.state == RECT_BOOT_RELEASE); zero(&r);
    wait(&r, raw, false, &now, 600); assert(r.state == RECT_READY); zero(&r);
}

static void one_wheel_fault_stops_whole_group(void)
{
    for (unsigned bad = 0; bad < 4; ++bad) {
        rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0;
        prepare(&r, raw, &now);
        /* 每个独立轮子的反向反馈都必须让整组停止。 */
        raw[bad] -= signs[bad]; step(&r, raw, false, &now);
        assert(r.fault & FOUR_SPEED_FAULT_FEEDBACK); zero(&r);
        memset(raw, 0, sizeof(raw)); now = 0; prepare(&r, raw, &now);
        for (unsigned n = 0; n < 90 && !r.fault; ++n) {
            for (unsigned i = 0; i < 4; ++i) if (i != bad) raw[i] += signs[i];
            step(&r, raw, false, &now);
        }
        assert(r.fault & FOUR_SPEED_FAULT_STALL); zero(&r);
    }
}

static void output_bounds_and_corner_guards(void)
{
    rectangle_demo_t r; int32_t raw[4] = {0}; uint32_t now = 0, cc[8];
    prepare(&r, raw, &now); advance(&r, raw, &now, 3);
    r.group.raw_duty[0] = 110;
    assert(!rectangle_demo_compares(&r, cc));
    for (unsigned i = 0; i < 8; ++i) assert(cc[i] == 0);
    rectangle_demo_stop(&r, RECT_FAULT_STATE, now); zero(&r);
    now = 0; memset(raw, 0, sizeof(raw)); prepare(&r, raw, &now);
    r.total_yaw_rad = 0.36f; advance(&r, raw, &now, 3);
    assert(r.fault == RECT_FAULT_ODOMETRY); zero(&r);
    now = 0; memset(raw, 0, sizeof(raw)); prepare(&r, raw, &now);
    while (r.state == RECT_MOVE) advance(&r, raw, &now, 3);
    const uint32_t corner = now;
    /* 转角期间轮子持续被带动，不允许直接换方向重新启动。 */
    while (!r.fault && now - corner < 3100U) {
        raw[0] += 1; step(&r, raw, false, &now);
    }
    assert(r.fault == RECT_FAULT_NOT_SETTLED); zero(&r);
    assert(!rectangle_demo_compares(NULL, cc));
    assert(!rectangle_demo_compares(&r, NULL));
    now = 0; memset(raw, 0, sizeof(raw));
    assert(rectangle_demo_init(&r, raw, now, false));
    wait(&r, raw, false, &now, 600); wait(&r, raw, true, &now, 1100);
    assert(r.state == RECT_ARMED_RELEASE);
    wait(&r, raw, true, &now, 30000);
    assert(r.fault == RECT_FAULT_NOT_SETTLED); zero(&r);
}

int main(void)
{
    startup_and_one_complete_rectangle();
    button_stop_and_countdown();
    faults_and_deadlines();
    time_wrap_and_physical_release();
    one_wheel_fault_stops_whole_group();
    output_bounds_and_corner_guards();
    puts("rectangle_demo: 6 scenario groups passed (synthetic encoders only)");
    return 0;
}
