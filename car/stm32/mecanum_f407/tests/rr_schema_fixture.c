#include "rr_diagnostic_trace.h"
#include <assert.h>
#include <stdio.h>

static rr_trace_t trace;
int main(void)
{
    drive_bench_t b; int32_t counts[4]={0}; drive_bench_init(&b,counts,0);
    for(uint32_t t=10;t<=500;t+=10) drive_bench_tick(&b,counts,t);
    drive_bench_accept(&b,"hb",500); drive_bench_accept(&b,RR_DIAGNOSTIC_RUN_COMMAND,500);
    drive_bench_tick(&b,counts,510);
    speed_bench_diagnostics_t d={.ms=510,.ticks=51,.dt_min=10,.dt_max=10};
#ifdef RR_PWM_FINE
    d.pwm_rr_arr=8399; d.pwm_rr_psc=0;
#endif
    assert(four_drive_compares(&b.group,d.compares)); char line[1152];
    assert(drive_bench_format(&b,&d,"TELSP",line,sizeof(line))); fputs(line,stdout);
    assert(rr_trace_capture(&trace,&b,d.compares,510));
    assert(rr_trace_format(&trace,0,line,sizeof(line))); fputs(line,stdout);
    drive_bench_accept(&b,"stop",510); drive_bench_tick(&b,counts,520);
    assert(four_drive_compares(&b.group,d.compares)); d.ms=520; d.ticks=52;
    assert(drive_bench_format(&b,&d,"STATUSSP",line,sizeof(line))); fputs(line,stdout);
    assert(rr_trace_capture(&trace,&b,d.compares,520));
    assert(rr_trace_format(&trace,1,line,sizeof(line))); fputs(line,stdout);
    return 0;
}
