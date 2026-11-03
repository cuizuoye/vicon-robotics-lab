#include "four_link_readonly.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t frame[MC_MAX_RAW_FRAME_SIZE];
static size_t length;
static four_ro_event_t send(four_ro_control_t *c, uint8_t type, uint16_t flags,
    uint32_t seq, uint32_t sid, const void *p, uint16_t n, uint32_t now)
{
    assert(mc_build_frame(type,flags,seq,sid,now,p,n,frame,sizeof(frame),&length));
    return four_ro_accept(c,frame,length,now);
}
static void sessions(void)
{
    four_ro_control_t c; four_ro_init(&c);
    mc_cmd_vel_v1_t z={.timeout_ms=150};
    assert(send(&c,MC_MSG_CMD_VEL,0,1,7,&z,sizeof(z),0)==FOUR_RO_NONE);
    assert(send(&c,MC_MSG_HELLO,MC_FLAG_ACK_REQUIRED,1,7,NULL,0,0)==FOUR_RO_HELLO);
    assert(send(&c,MC_MSG_CMD_VEL,0,2,7,&z,sizeof(z),10)==FOUR_RO_COMMAND);
    assert(send(&c,MC_MSG_CMD_VEL,0,2,7,&z,sizeof(z),20)==FOUR_RO_NONE);
    assert(send(&c,MC_MSG_CMD_VEL,0,3,8,&z,sizeof(z),20)==FOUR_RO_NONE);
    assert(send(&c,MC_MSG_HEARTBEAT,0,3,7,NULL,0,100)==FOUR_RO_CONTROL);
    assert(!four_ro_tick(&c,159)); assert(four_ro_tick(&c,160));
    assert(c.expired && c.fault==FOUR_RO_TIMEOUT);
    assert(send(&c,MC_MSG_CMD_VEL,0,4,7,&z,sizeof(z),161)==FOUR_RO_NONE);
    assert(send(&c,MC_MSG_HELLO,0,5,7,NULL,0,162)==FOUR_RO_NONE);
    assert(send(&c,MC_MSG_HELLO,0,1,8,NULL,0,170)==FOUR_RO_HELLO);
    assert(!c.expired && c.fault==0);
    z.reserved=1;
    assert(send(&c,MC_MSG_CMD_VEL,0,2,8,&z,sizeof(z),180)==FOUR_RO_NONE);
    z.reserved=0; z.timeout_ms=200;
    assert(send(&c,MC_MSG_CMD_VEL,0,2,8,&z,sizeof(z),180)==FOUR_RO_NONE);
    z.timeout_ms=150;
    assert(send(&c,MC_MSG_CMD_VEL,MC_FLAG_RESPONSE,2,8,&z,sizeof(z),180)==FOUR_RO_NONE);
    assert(mc_build_frame(MC_MSG_CMD_VEL,0,2,8,180,&z,sizeof(z),frame,sizeof(frame),&length));
    frame[length-1]^=1;
    assert(four_ro_accept(&c,frame,length,180)==FOUR_RO_NONE);
    assert(send(&c,MC_MSG_CMD_VEL,0,2,8,&z,sizeof(z),180)==FOUR_RO_COMMAND);
    four_ro_init(&c);
    assert(send(&c,MC_MSG_HELLO,0,UINT32_MAX-1,9,NULL,0,UINT32_MAX-20)==FOUR_RO_HELLO);
    assert(send(&c,MC_MSG_CMD_VEL,0,UINT32_MAX,9,&z,sizeof(z),UINT32_MAX-10)==FOUR_RO_COMMAND);
    assert(send(&c,MC_MSG_CMD_VEL,0,0,9,&z,sizeof(z),5)==FOUR_RO_COMMAND);
    assert(!four_ro_tick(&c,154)); assert(four_ro_tick(&c,155));
}
static void forbidden(void)
{
    four_ro_control_t c; four_ro_init(&c);
    mc_cmd_vel_v1_t z={.timeout_ms=150};
    mc_control_enable_v1_t en={.enabled=1};
    assert(send(&c,MC_MSG_HELLO,0,1,1,NULL,0,0)==FOUR_RO_HELLO);
    assert(send(&c,MC_MSG_CMD_VEL,MC_FLAG_ENABLED,2,1,&z,sizeof(z),1)==FOUR_RO_CONTROL);
    assert(c.fault==FOUR_RO_FORBIDDEN && !c.command_seen);
    z.vx_mm_s=1;
    assert(send(&c,MC_MSG_CMD_VEL,0,3,1,&z,sizeof(z),2)==FOUR_RO_CONTROL);
    assert(send(&c,MC_MSG_CONTROL_ENABLE,0,4,1,&en,sizeof(en),3)==FOUR_RO_CONTROL);
    assert(send(&c,MC_MSG_ESTOP,0,5,1,NULL,0,4)==FOUR_RO_CONTROL);
    c.fault|=FOUR_RO_UART;
    assert(send(&c,MC_MSG_HELLO,0,1,2,NULL,0,5)==FOUR_RO_HELLO);
    assert(c.fault==(FOUR_RO_FORBIDDEN|FOUR_RO_ESTOP|FOUR_RO_UART));
    en.enabled=0; en.clear_latched_faults=1;
    assert(send(&c,MC_MSG_CONTROL_ENABLE,0,2,2,&en,sizeof(en),6)==FOUR_RO_CONTROL);
    assert(c.fault==(FOUR_RO_FORBIDDEN|FOUR_RO_ESTOP|FOUR_RO_UART));
}
static void telemetry(void)
{
    four_ro_control_t c; four_ro_init(&c);
    speed_bench_t b; const int32_t counts[4]={0}; speed_bench_init(&b,true,counts,0);
    speed_bench_diagnostics_t d={.ms=10,.ticks=1,.dt_min=10,.dt_max=10,.gpio_low=true};
    const int32_t next[4]={10,-10,10,-10};
    assert(speed_bench_tick(&b,next,10));
    mc_wheel_telemetry_v1_t t; four_ro_status_t s;
    assert(four_ro_payloads(&c,&b,&d,&t,&s));
    assert(t.state_flags==108 && t.fault_bits==0 && t.battery_mv==0);
    for(unsigned i=0;i<4;++i) {
        assert(t.encoder_counts[i]==next[i]);
        assert(t.wheel_rpm_x10[i]>550 && t.wheel_rpm_x10[i]<580);
        assert(!s.compares[2*i] && !s.compares[2*i+1]);
    }
    assert(s.tag==FOUR_RO_STATUS_TAG && s.updates==1 && s.gpio_low==1);
    assert(mc_build_frame(MC_MSG_LINK_STATUS,0,1,1,10,&s,sizeof(s),frame,sizeof(frame),&length));
    assert(length==84 && mc_validate_frame(frame,length,NULL));
    d.compares[7]=1; assert(!four_ro_payloads(&c,&b,&d,&t,&s)); d.compares[7]=0;
    d.gpio_low=false; assert(!four_ro_payloads(&c,&b,&d,&t,&s)); d.gpio_low=true;
    b.read_only=false; assert(!four_ro_payloads(&c,&b,&d,&t,&s)); b.read_only=true;
    b.pi.enabled=true; assert(!four_ro_payloads(&c,&b,&d,&t,&s)); b.pi.enabled=false;
    b.pi.wheel[3].requested_cps=1; assert(!four_ro_payloads(&c,&b,&d,&t,&s));
    b.pi.wheel[3].requested_cps=0; d.watchdog_reset=true;
    assert(four_ro_payloads(&c,&b,&d,&t,&s) && t.fault_bits==FOUR_RO_SAMPLE);
}
int main(void)
{
    assert(mc_protocol_self_test()); sessions(); forbidden(); telemetry();
    puts("four_link_readonly: session, CRC, replay, wrap, timeout, forbidden motion, telemetry OK");
}
