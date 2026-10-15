#include "rr_diagnostic_trace.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static rr_trace_t trace;
static void zero(const drive_bench_t *b)
{
    uint32_t cc[8]; assert(!b->group.enabled && four_drive_compares(&b->group, cc));
    for (unsigned i = 0; i < 8; ++i) assert(cc[i] == 0);
    for (unsigned i = 0; i < 4; ++i) {
        assert(b->group.raw_duty[i] == 0);
        assert(b->group.loop[i].wheel[i].integral_permille == 0);
    }
}
static uint32_t ready(drive_bench_t *b, int32_t raw[4], uint32_t origin)
{
    drive_bench_init(b, raw, origin);
    for (uint32_t dt = 10; dt <= 500; dt += 10) assert(drive_bench_tick(b, raw, origin+dt));
    assert(b->group.fault == 0);
    assert(!strcmp(drive_bench_accept(b, "hb", origin+500), "OK,hb_drive\r\n"));
    const mecanum_body_velocity_t body = {.vx_m_s=.05f};
    assert(!four_drive_arm(&b->group, &body, origin+500));
    assert(!strcmp(drive_bench_accept(b, RR_DIAGNOSTIC_RUN_COMMAND, origin+500), "OK,running_drive\r\n"));
    assert(b->mode == DRIVE_MODE_RR_DIAGNOSTIC);
    for (unsigned i = 0; i < 4; ++i) assert(b->group.plan.forward_cps[i] == (i == 3 ? RR_DIAGNOSTIC_TARGET_CPS : 0));
    return origin+500;
}
static void test_isolation_and_profile(void)
{
    drive_bench_t b; int32_t raw[4]={0}; drive_bench_init(&b, raw, 0);
    const char *bad[]={"run F50 S10", "run Y200 S10", "run L50 S10", "run D25 S10",
        "run RR164 S5", "run RR200 S10", "run RR164 S10 ", "clear 5A", "duty 100"};
    for (unsigned i=0; i<sizeof(bad)/sizeof(bad[0]); ++i) {
        assert(strstr(drive_bench_accept(&b, bad[i], 0), "ERR,"));
        assert(!b.group.run_used); zero(&b);
    }
    const uint32_t start=ready(&b, raw, 0);
    for (uint32_t dt=10; dt<=200; dt+=10) {
        if (dt%50==0) drive_bench_accept(&b,"hb",start+dt);
        assert(drive_bench_tick(&b,raw,start+dt));
        uint32_t cc[8]; assert(four_drive_compares(&b.group,cc));
        for(unsigned i=0;i<6;++i) assert(!cc[i]);
        assert(b.group.startup_mask == (dt<200 ? 8 : 0));
        if(dt<200) assert(cc[6]==FOUR_SPEED_PWM_COUNTS*92/100 &&
                         cc[7]==FOUR_SPEED_PWM_COUNTS && b.group.raw_duty[3]==-80);
    }
    drive_bench_accept(&b,"stop",start+200); zero(&b);
    assert(b.group.run_used && !b.group.fault);
    assert(!strcmp(drive_bench_accept(&b,"hb",start+210),"ERR,one_shot_used\r\n"));
    assert(!strcmp(drive_bench_accept(&b,RR_DIAGNOSTIC_RUN_COMMAND,start+210),"ERR,one_shot_used\r\n"));
}
static void test_zero_wheel_and_corrupt_plan(void)
{
    for(unsigned w=0;w<3;++w) {
        drive_bench_t b; int32_t raw[4]={0}; const uint32_t start=ready(&b,raw,0);
        raw[w]++; drive_bench_tick(&b,raw,start+10);
        assert(b.group.fault==FOUR_SPEED_FAULT_FEEDBACK && b.group.fault_wheel==w+1); zero(&b);
        memset(raw,0,sizeof(raw)); ready(&b,raw,0);
        b.group.plan.forward_cps[w]=164;
        uint32_t cc[8]; memset(cc,1,sizeof(cc)); assert(!four_drive_compares(&b.group,cc));
        for(unsigned i=0;i<8;++i) assert(!cc[i]);
    }
    drive_bench_t b; int32_t raw[4]={0}; ready(&b,raw,0); b.group.plan.forward_cps[3]=RR_DIAGNOSTIC_TARGET_CPS+1;
    uint32_t cc[8]; assert(!four_drive_compares(&b.group,cc));
}
static void test_faults_and_deadlines(void)
{
    for(unsigned test=0;test<6;++test) {
        drive_bench_t b; int32_t raw[4]={0}; const uint32_t start=ready(&b,raw,UINT32_MAX-800);
        uint32_t expected=0;
        if(test==0) { raw[3]=1; expected=FOUR_SPEED_FAULT_FEEDBACK; drive_bench_tick(&b,raw,start+10); }
        else if(test==1) { raw[3]=-101; expected=FOUR_SPEED_FAULT_FEEDBACK; drive_bench_tick(&b,raw,start+10); }
        else if(test==2) { expected=FOUR_SPEED_FAULT_TIMING; drive_bench_tick(&b,raw,start+31); }
        else {
            const uint32_t limit=test==3 ? 150 : test==4 ? 800 : 10000;
            expected=test==3 ? FOUR_SPEED_FAULT_COMMAND : test==4 ? FOUR_SPEED_FAULT_STALL : FOUR_SPEED_FAULT_DURATION;
            for(uint32_t dt=10;dt<=limit;dt+=10) {
                if(test!=4) --raw[3];
                if(test!=3 && dt%50==0) drive_bench_accept(&b,"hb",start+dt);
                drive_bench_tick(&b,raw,start+dt);
                assert(b.group.enabled==(dt<limit));
            }
        }
        assert(b.group.fault==expected); zero(&b);
        drive_bench_accept(&b,"stop",start+10010); assert(b.group.fault==expected); zero(&b);
    }
}
static void test_full_capture_freeze_and_capacity(void)
{
    drive_bench_t b; int32_t raw[4]={0}; memset(&trace,0,sizeof(trace));
    const uint32_t start=ready(&b,raw,0); uint32_t cc[8];
    for(uint32_t dt=10;dt<=10000;dt+=10) {
        --raw[3]; if(dt%50==0) drive_bench_accept(&b,"hb",start+dt);
        drive_bench_tick(&b,raw,start+dt); assert(four_drive_compares(&b.group,cc));
        assert(rr_trace_capture(&trace,&b,cc,start+dt));
    }
    assert(trace.count==1000 && trace.complete && !trace.capturing && !trace.overflow);
    assert(!trace.rows[999].enabled && trace.rows[999].fault==FOUR_SPEED_FAULT_DURATION);
    assert(rr_trace_capture(&trace,&b,cc,start+10010) && trace.count==1000);
    char row[256]; assert(rr_trace_format(&trace,0,row,sizeof(row)) && strlen(row)<sizeof(row));
    assert(!rr_trace_format(&trace,1000,row,sizeof(row)));
    assert(!rr_trace_format(&trace,0,row,10));
    memset(raw,0,sizeof(raw)); ready(&b,raw,0); memset(&trace,0,sizeof(trace));
    trace.count=RR_TRACE_CAPACITY; trace.capturing=true;
    assert(four_drive_compares(&b.group,cc));
    assert(!rr_trace_capture(&trace,&b,cc,510) && trace.overflow && trace.complete);
    assert(rr_trace_capture(&trace,&b,cc,520)); /* 满只报一次，不能每tick重新stop阻止导出。 */
}
int main(void)
{
    test_isolation_and_profile(); test_zero_wheel_and_corrupt_plan();
    test_faults_and_deadlines(); test_full_capture_freeze_and_capacity();
    puts("RR diagnostic safety/trace tests passed"); return 0;
}
