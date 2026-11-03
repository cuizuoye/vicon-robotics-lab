/* Native host/real-C protocol integration, never linked into firmware. */
#include "four_link_bench.h"
#include <string.h>
static four_link_bench_t link;
static drive_bench_t drive;
static int32_t counts[4];
static uint32_t ticks;

void fixture_init(void)
{
    memset(counts,0,sizeof(counts));ticks=0;
    drive_bench_init(&drive,counts,0);four_link_bench_init(&link);
}
int fixture_receive(const uint8_t *data,size_t n,uint32_t now)
{
    four_link_request_t r;
    if(!four_link_bench_decode(data,n,&r))return -1;
    return four_link_bench_process(&link,&drive,&r,now) ? 1:0;
}
void fixture_tick(uint32_t now)
{
    if(drive.group.enabled)for(unsigned i=0;i<4;++i) {
        const float t=drive.group.plan.forward_cps[i];
        counts[i]+=((t>0)-(t<0))*drive.adopted.wheel[i].encoder_sign;
    }
    four_link_bench_deadline(&link,&drive,now);
    (void)drive_bench_tick(&drive,counts,now);
    four_link_bench_deadline(&link,&drive,now);++ticks;
}
size_t fixture_feedback(uint8_t *out,uint8_t kind,uint32_t sequence,uint32_t now)
{
    speed_bench_diagnostics_t d={.ms=now,.ticks=ticks,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t t;four_link_bench_status_t s;
    if(!four_drive_compares(&drive.group,d.compares) ||
       !four_link_bench_payloads(&link,&drive,&d,&t,&s))return 0;
    const uint16_t flags=(t.fault_bits?MC_FLAG_FAULT:0)|(drive.group.enabled?MC_FLAG_ENABLED:0);
    const void *p=kind==MC_MSG_WHEEL_TELEMETRY?(const void*)&t:(const void*)&s;
    const uint16_t n=kind==MC_MSG_WHEEL_TELEMETRY?sizeof(t):sizeof(s);
    size_t length=0;
    if(!mc_build_frame(kind,flags,sequence,link.session_id,now,p,n,out,MC_MAX_RAW_FRAME_SIZE,&length))return 0;
    return length;
}
int fixture_enabled(void){return drive.group.enabled;}
int fixture_permitted(void){return link.permitted;}
uint32_t fixture_fault(void){return drive.group.fault;}
