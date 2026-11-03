#include "drive_bench_control.h"
#include "speed_uart_queue.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef DRIVE_STARTUP_OBSERVE
#error This test requires the isolated startup observation build.
#endif

static const char *const commands[] = {"run F50 S10", "run L50 S10", "run Y200 S10", "run D25 S10"};

static void zero(const drive_bench_t *b)
{
    uint32_t cc[8];
    assert(!b->group.enabled && b->group.startup_mask == 0);
    assert(four_drive_compares(&b->group, cc));
    for (unsigned i = 0; i < 4; ++i) {
        const four_speed_wheel_state_t *s = &b->group.loop[i].wheel[i];
        assert(b->group.raw_duty[i] == 0 && !b->group.loop[i].enabled);
        assert(s->requested_cps == 0 && s->ramped_cps == 0 &&
               s->integral_permille == 0 && s->raw_duty_permille == 0);
        assert(cc[2*i] == 0 && cc[2*i+1] == 0);
    }
}

static uint32_t ready(drive_bench_t *b, int32_t raw[4], unsigned mode, uint32_t start)
{
    drive_bench_init(b, raw, start);
    assert(b->group.fault == 0);
    for (uint32_t dt = 10; dt <= 500; dt += 10) assert(drive_bench_tick(b, raw, start + dt));
    const uint32_t now = start + 500;
    assert(!strcmp(drive_bench_accept(b, "hb", now), "OK,hb_drive\r\n"));
    assert(!strcmp(drive_bench_accept(b, commands[mode], now), "OK,running_drive\r\n"));
    return now;
}

static void progress(drive_bench_t *b, int32_t raw[4], int excluded)
{
    for (unsigned i = 0; i < 4; ++i) {
        const float want = b->group.plan.forward_cps[i];
        if ((int)i != excluded && want != 0) {
            const int d = ((want > 0) - (want < 0)) * b->adopted.wheel[i].encoder_sign;
            const uint32_t next = (uint32_t)raw[i] + (uint32_t)d;
            raw[i] = next <= INT32_MAX ? (int32_t)next : (int32_t)((int64_t)next - INT64_C(4294967296));
        }
    }
}

static void test_profile_and_command_isolation(void)
{
    drive_bench_t b; int32_t raw[4] = {0};
    drive_bench_init(&b, raw, 0);
    assert(b.adopted.run_limit_ms == 10000 && b.adopted.duty_limit_permille == 100);
    assert(b.adopted.ramp_cps_per_s == 800 && b.adopted.command_timeout_ms == 150);
    const char *bad[] = {"run L50 S5", "run L50 S11", "run L50 S30", "run F60 S10",
                        "duty 120", "clear 5A", "cmd_vel 0 0.05 0", "run L50 S10 "};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) {
        assert(strstr(drive_bench_accept(&b, bad[i], 0), "ERR,"));
        assert(!b.group.run_used); zero(&b);
    }
    four_speed_controller_t c;
    four_speed_config_t invalid = b.adopted;
    invalid.run_limit_ms = 10001;
    assert(!four_speed_init(&c, &invalid));
    invalid = b.adopted; invalid.duty_limit_permille = 110;
    assert(!four_speed_init(&c, &invalid));
    invalid = b.adopted; invalid.ramp_cps_per_s = 200;
    four_drive_group_t g;
    assert(!four_drive_init(&g, &b.group.geometry, &invalid, raw, 0));
}

static void test_bounded_assist_all_modes_and_zero_wheels(void)
{
    for (unsigned m = 0; m < 4; ++m) {
        drive_bench_t b; int32_t raw[4] = {0};
        const uint32_t start = ready(&b, raw, m, 0);
        for (uint32_t dt = 10; dt <= 200; dt += 10) {
            if (dt % 50 == 0) drive_bench_accept(&b, "hb", start + dt);
            assert(drive_bench_tick(&b, raw, start + dt));
            const uint8_t mask = m == 3 ? 6U : 15U;
            assert(b.group.startup_mask == (dt < 200 ? mask : 0U));
            uint32_t cc[8]; assert(four_drive_compares(&b.group, cc));
            for (unsigned i = 0; i < 4; ++i) {
                const float want = b.group.plan.forward_cps[i];
                if (want == 0) {
                    assert(!b.group.raw_duty[i] && cc[2*i] == 0 && cc[2*i+1] == 0);
                } else if (dt < 200) {
                    const int sign = ((want > 0)-(want < 0)) * b.adopted.wheel[i].motor_sign;
                    assert(b.group.raw_duty[i] == 80 * sign);
                    assert(b.group.loop[i].wheel[i].integral_permille == 0);
                }
            }
        }
        drive_bench_accept(&b, "stop", start + 200); zero(&b);
        assert(b.group.run_used && b.group.fault == 0);
        assert(!strcmp(drive_bench_accept(&b, "hb", start + 201), "ERR,one_shot_used\r\n"));
    }
}

static void test_early_finish_no_retrigger(void)
{
    drive_bench_t b; int32_t raw[4] = {0};
    uint32_t start = ready(&b, raw, 1, 0);
    for (uint32_t dt = 10; dt <= 90; dt += 10) {
        if (dt > 10) progress(&b, raw, -1);
        if (dt % 50 == 0) drive_bench_accept(&b, "hb", start + dt);
        drive_bench_tick(&b, raw, start + dt);
        assert(b.group.startup_mask == (dt < 90 ? 15U : 0U));
    }
    assert(b.group.startup_done_mask == 15);
    for (uint32_t dt = 100; dt <= 200; dt += 10) {
        if (dt % 50 == 0) drive_bench_accept(&b, "hb", start + dt);
        drive_bench_tick(&b, raw, start + dt);
        assert(b.group.startup_mask == 0);
    }
    /* 到达最终目标速度时提前退出；随后低通回落也不再次补偿。 */
    start = ready(&b, raw, 1, 0);
    assert(drive_bench_tick(&b, raw, start + 10));
    raw[0] -= 6; /* 600cps / 6 = 100cps, below final target; not done on tiny counts. */
    assert(drive_bench_tick(&b, raw, start + 20));
    assert(b.group.startup_mask & 1U);
    /* yaw较低目标，7计数/10ms经低通逐步到目标但累计未满8也可退出。 */
    memset(raw, 0, sizeof(raw)); start = ready(&b, raw, 2, 0);
    b.group.loop[0].wheel[0].filtered_cps = -164.0f;
    raw[0] -= 2; /* filtered remains above 162cps final target in intended direction */
    assert(drive_bench_tick(&b, raw, start + 10));
    assert((b.group.startup_done_mask & 1U) && !(b.group.startup_mask & 1U));
    assert(drive_bench_tick(&b, raw, start + 20));
    assert(!(b.group.startup_mask & 1U));
}

static void test_no_progress_deadline_is_not_extended(void)
{
    for (int excluded = -1; excluded < 4; ++excluded) {
        drive_bench_t b; int32_t raw[4] = {0};
        const uint32_t start = ready(&b, raw, 1, 0);
        for (uint32_t dt = 10; dt <= 800; dt += 10) {
            if (excluded >= 0) progress(&b, raw, excluded);
            if (dt % 50 == 0) drive_bench_accept(&b, "hb", start + dt);
            drive_bench_tick(&b, raw, start + dt);
            assert(b.group.enabled == (dt < 800));
        }
        assert(b.group.fault == FOUR_SPEED_FAULT_STALL);
        assert(b.group.fault_wheel == (excluded < 0 ? 1 : excluded + 1)); zero(&b);
        assert(!strcmp(drive_bench_accept(&b, "hb", start + 801), "ERR,fault_latched\r\n"));
        drive_bench_accept(&b, "stop", start + 802);
        assert(b.group.fault == FOUR_SPEED_FAULT_STALL && b.group.run_used); zero(&b);
    }
}

static void test_heartbeat_and_timing_during_assist(void)
{
    drive_bench_t b; int32_t raw[4] = {0};
    uint32_t start = ready(&b, raw, 1, 0);
    for (uint32_t dt = 10; dt < 150; dt += 10) {
        progress(&b, raw, -1); drive_bench_tick(&b, raw, start + dt);
    }
    assert(!strcmp(drive_bench_accept(&b, "hb", start + 150), "ERR,heartbeat_late\r\n"));
    assert(b.group.fault == FOUR_SPEED_FAULT_COMMAND); zero(&b);
    assert(!strcmp(drive_bench_accept(&b, "hb", start + 151), "ERR,fault_latched\r\n"));
    start = ready(&b, raw, 1, 0);
    assert(drive_bench_tick(&b, raw, start + 10) && b.group.startup_mask == 15);
    drive_bench_tick(&b, raw, start + 41);
    assert(b.group.fault == FOUR_SPEED_FAULT_TIMING); zero(&b);
}

static void test_direction_speed_and_zero_target_faults(void)
{
    for (unsigned failed = 0; failed < 4; ++failed) for (unsigned large = 0; large < 2; ++large) {
        drive_bench_t b; int32_t raw[4] = {0};
        const uint32_t start = ready(&b, raw, 1, 0);
        const int sign = ((b.group.plan.forward_cps[failed] > 0) ? 1 : -1) *
                         b.adopted.wheel[failed].encoder_sign;
        raw[failed] = sign * (large ? 101 : -1);
        drive_bench_tick(&b, raw, start + 10);
        assert(b.group.fault == FOUR_SPEED_FAULT_FEEDBACK && b.group.fault_wheel == failed + 1);
        zero(&b);
    }
    for (unsigned failed = 0; failed < 4; failed += 3) {
        drive_bench_t b; int32_t raw[4] = {0};
        const uint32_t start = ready(&b, raw, 3, 0);
        raw[failed] = 1; drive_bench_tick(&b, raw, start + 10);
        assert(b.group.fault == FOUR_SPEED_FAULT_FEEDBACK && b.group.fault_wheel == failed + 1);
        zero(&b);
    }
}

static void test_ten_second_limit_wrap_and_stop(void)
{
    for (unsigned m = 0; m < 4; ++m) {
        drive_bench_t b; int32_t raw[4] = {INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX};
        const uint32_t start = ready(&b, raw, m, UINT32_MAX - 600U);
        for (uint32_t dt = 10; dt < 10000; dt += 10) {
            progress(&b, raw, -1);
            if (dt % 50 == 0) drive_bench_accept(&b, "hb", start + dt);
            drive_bench_tick(&b, raw, start + dt);
            assert(b.group.enabled && b.group.fault == 0);
            for (unsigned i = 0; i < 4; ++i) assert(abs(b.group.raw_duty[i]) <= 100);
        }
        assert(!strcmp(drive_bench_accept(&b, "hb", start + 10000), "ERR,heartbeat_late\r\n"));
        assert(b.group.fault == FOUR_SPEED_FAULT_DURATION && b.group.run_used); zero(&b);
        assert(strstr(drive_bench_accept(&b, commands[m], start + 10001), "busy_or_fault"));
    }
}

static void test_output_and_telemetry_validation(void)
{
    drive_bench_t b; int32_t raw[4] = {0};
    const uint32_t start = ready(&b, raw, 1, 0);
    drive_bench_tick(&b, raw, start + 10);
    speed_bench_diagnostics_t d = {.ms=start+10, .ticks=51, .dt_min=10, .dt_max=10};
    assert(four_drive_compares(&b.group, d.compares));
    char frame[SPEED_UART_FRAME_BYTES], small[8];
    assert(drive_bench_format(&b, &d, "TELSP", frame, sizeof(frame)));
    assert(strstr(frame, "fw=drive_pi_start8_v1") && strstr(frame, "start_mask=15,fault_wheel=0"));
    assert(!drive_bench_format(&b, &d, "TELSP", small, sizeof(small)));
    b.group.startup_done_mask |= 1U;
    assert(!four_drive_compares(&b.group, d.compares));
    four_drive_stop(&b.group, FOUR_SPEED_FAULT_FEEDBACK, start + 11); zero(&b);
    b.group.fault_wheel = 3;
    d.ms = UINT32_MAX; d.ticks = UINT32_MAX; b.updates = UINT32_MAX;
    for (unsigned i = 0; i < 4; ++i) {
        b.counts[i] = b.raw_cps[i] = INT32_MIN;
        b.rpm_milli[i] = -10000000; b.speed_mm_s[i] = -1000000;
    }
    assert(drive_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame)));
    assert(strlen(frame) < SPEED_UART_FRAME_BYTES && strstr(frame, "start_mask=0,fault_wheel=3"));
}

int main(void)
{
    test_profile_and_command_isolation(); test_bounded_assist_all_modes_and_zero_wheels();
    test_early_finish_no_retrigger(); test_no_progress_deadline_is_not_extended();
    test_heartbeat_and_timing_during_assist(); test_direction_speed_and_zero_target_faults();
    test_ten_second_limit_wrap_and_stop(); test_output_and_telemetry_validation();
    puts("startup observation: 8 offline safety test groups passed (not hardware)");
    return 0;
}
