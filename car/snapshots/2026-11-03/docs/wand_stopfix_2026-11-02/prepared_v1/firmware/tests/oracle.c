/* Native protocol/controller oracle. Not a device API and never built for ARM. */
#include "four_link_bench.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
static drive_bench_t b;static four_link_bench_t l;static int32_t counts[4];
static float speed[4],fraction[4];static uint32_t sampled,tx;
void oracle_init(void)
{
    memset(counts,0,sizeof(counts));memset(speed,0,sizeof(speed));memset(fraction,0,sizeof(fraction));
    sampled=tx=0;drive_bench_init(&b,counts,0);four_link_bench_init(&l);
}
size_t oracle_request(const uint8_t *raw,size_t n,uint32_t now,uint8_t *out)
{
    four_link_request_t r;size_t used=0;
    if (four_link_bench_decode(raw,n,&r) && four_link_bench_process(&l,&b,&r,now))
        (void)mc_build_frame(MC_MSG_HELLO_ACK,MC_FLAG_RESPONSE,r.header.sequence,r.header.session_id,
                            now,NULL,0,out,MC_MAX_RAW_FRAME_SIZE,&used);
    return used;
}
void oracle_tick(uint32_t now)
{
    while(now-sampled>=10) {
        sampled+=10;
        for(unsigned i=0;i<4;i++) {
            const int u=b.group.raw_duty[i]*b.adopted.wheel[i].motor_sign;
            const float target=copysignf(fmaxf(0,(abs(u)-80)/.15f),(float)u);
            speed[i]+=(target-speed[i])*.01f/.11f;
            fraction[i]+=speed[i]*.01f;const int delta=(int)truncf(fraction[i]);fraction[i]-=delta;
            counts[i]+=delta*b.adopted.wheel[i].encoder_sign;
        }
        (void)drive_bench_tick(&b,counts,sampled);four_link_bench_deadline(&l,&b,sampled);
    }
}
size_t oracle_packet(unsigned kind,uint8_t *out)
{
    if(!l.session_active)return 0;
    speed_bench_diagnostics_t d={.ms=sampled,.ticks=sampled/10,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t telemetry;four_link_bench_status_t s;size_t used=0;
    if(!four_drive_compares(&b.group,d.compares) || !four_link_bench_payloads(&l,&b,&d,&telemetry,&s))return 0;
    /* Keep this expression identical to production publish(), not a reply. */
    const uint16_t flags=(telemetry.fault_bits ? MC_FLAG_FAULT : 0) |
                         ((telemetry.state_flags & FOUR_LINK_ENABLED) ? MC_FLAG_ENABLED : 0);
    (void)mc_build_frame(kind,flags,++tx,l.session_id,sampled,
        kind==MC_MSG_LINK_STATUS?(void*)&s:(void*)&telemetry,
        kind==MC_MSG_LINK_STATUS?sizeof(s):sizeof(telemetry),out,MC_MAX_RAW_FRAME_SIZE,&used);
    return used;
}
