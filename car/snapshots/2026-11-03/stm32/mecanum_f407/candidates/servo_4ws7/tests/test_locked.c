#include "four_link_bench.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    drive_bench_t b; four_link_bench_t l;
    int32_t counts[4]={0};
    drive_bench_init(&b,counts,0); four_link_bench_init(&l);
    four_link_request_t r={.header={.message_type=MC_MSG_HELLO,.sequence=1,.session_id=17}};
    assert(four_link_bench_process(&l,&b,&r,0));
    for(unsigned t=10;t<=600;t+=10) assert(drive_bench_tick(&b,counts,t));
    r.header=(mc_frame_header_v1_t){.message_type=MC_MSG_CMD_VEL,.sequence=2,.session_id=17,.sender_time_ms=600};
    r.body.velocity=(mc_cmd_vel_v1_t){.timeout_ms=600};
    four_link_bench_process(&l,&b,&r,600);
    r.header.message_type=MC_MSG_CONTROL_ENABLE;r.header.sequence=3;
    r.body.control=(mc_control_enable_v1_t){.reserved=SERVO_TIER_MAGIC};
    four_link_bench_process(&l,&b,&r,600);
    assert(l.tier_selected);
    r.header.sequence=4;r.body.control.enabled=1;
    four_link_bench_process(&l,&b,&r,600);
    assert(!l.permitted && !b.group.enabled && (l.fault&SERVO_LINK_OFFLINE_LOCK));
    uint32_t ccr[8];assert(four_drive_compares(&b.group,ccr));
    for(unsigned i=0;i<8;i++)assert(ccr[i]==0);
    puts("PASS: production/default OFFLINE lock refuses enable; all outputs zero");
    return 0;
}
