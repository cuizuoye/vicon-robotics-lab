/* Static serializer fixtures ONLY: no time stepping, motor model or motion.
 * Exercise the actual C payload builder and host-readable ABI. */
#include "four_link_bench.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static void save(const char *path, const four_link_bench_status_t *s)
{
    FILE *file=fopen(path,"wb"); assert(file);
    assert(fwrite(s,1,sizeof(*s),file)==sizeof(*s)); assert(fclose(file)==0);
}

int main(int argc, char **argv)
{
    assert(argc==3);
    _Static_assert(offsetof(four_link_bench_status_t, first_feedback_fault)==100,"snapshot offset");
    _Static_assert(offsetof(four_link_bench_status_t, effective_cps)==132,"live authority offset");
    const int32_t counts[4]={0}; drive_bench_t b; four_link_bench_t c;
    drive_bench_init(&b,counts,1000); four_link_bench_init(&c);
    assert(!b.group.fault); b.group.stationary_ms=500;
    speed_bench_diagnostics_t d={.ms=1000,.ticks=1,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t t; four_link_bench_status_t s;
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&c,&b,&d,&t,&s));
    assert(sizeof(s)==156 && s.tag==0x43475734 && s.output_authority_version==1);
    save(argv[1],&s);
    /* Construct a valid instantaneous reversal snapshot. These assignments
     * do not evolve a simulated controller or claim any physical motion. */
    c.permitted=true; b.group.enabled=b.group.run_used=true;
    for(unsigned i=0;i<4;i++) {
        four_speed_controller_t *loop=&b.group.loop[i];
        four_speed_wheel_state_t *w=&loop->wheel[i];
        loop->enabled=true; loop->selected=i; loop->run_direction=1; loop->reversing=true;
        b.group.plan.forward_cps[i]=w->requested_cps=-100;
        w->ramped_cps=80; w->filtered_cps=50;
        b.group.raw_duty[i]=w->raw_duty_permille=80*loop->config.wheel[i].motor_sign;
    }
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&c,&b,&d,&t,&s));
    assert(s.reversing_mask==15 && s.target_cps[2]==-100 && s.effective_cps[2]==80);
    save(argv[2],&s);
    uint8_t frame[MC_MAX_RAW_FRAME_SIZE]; size_t used; mc_frame_header_v1_t header;
    assert(mc_build_frame(MC_MSG_LINK_STATUS,MC_FLAG_RESPONSE,1,1,1000,
        &s,sizeof(s),frame,sizeof(frame),&used));
    assert(mc_validate_frame(frame,used,&header) && header.payload_length==156);
    puts("PASS static C serializer + output-authority ABI; NO DYNAMIC SIMULATION");
    return 0;
}
