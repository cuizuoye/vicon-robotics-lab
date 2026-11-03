#include "four_link_bench.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t sequence=0, sid=7;
static bool send(four_link_bench_t *c, drive_bench_t *b, uint8_t type,
                 uint16_t flags, const void *p, uint16_t n, uint32_t now)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE]; size_t size=0;
    assert(mc_build_frame(type,flags,++sequence,sid,now,p,n,raw,sizeof(raw),&size));
    four_link_request_t r; assert(four_link_bench_decode(raw,size,&r));
    return four_link_bench_process(c,b,&r,now);
}
static void zero(const drive_bench_t *b)
{
    uint32_t cc[8]; assert(!b->group.enabled && four_drive_compares(&b->group,cc));
    for(unsigned i=0;i<8;++i) assert(!cc[i]);
    for(unsigned i=0;i<4;++i) assert(!b->group.raw_duty[i]);
}
static void tick(four_link_bench_t *c,drive_bench_t *b,int32_t raw[4],uint32_t now,bool progress)
{
    if(progress && b->group.enabled) for(unsigned i=0;i<4;++i)
        raw[i]+=2*b->adopted.wheel[i].encoder_sign;
    four_link_bench_deadline(c,b,now); (void)drive_bench_tick(b,raw,now);
    four_link_bench_deadline(c,b,now);
}
static uint32_t ready(four_link_bench_t *c,drive_bench_t *b,int32_t raw[4],uint32_t start)
{
    sequence=0; sid=7; memset(raw,0,4*sizeof(*raw));
    drive_bench_init(b,raw,start); four_link_bench_init(c);
    assert(send(c,b,MC_MSG_HELLO,0,NULL,0,start)); zero(b);
    const mc_cmd_vel_v1_t z={.timeout_ms=150};
    for(unsigned dt=10;dt<=600;dt+=10) {
        tick(c,b,raw,start+dt,false);
        if(dt%20==0) send(c,b,MC_MSG_CMD_VEL,0,&z,sizeof(z),start+dt);
    }
    const mc_control_enable_v1_t e={.enabled=1};
    send(c,b,MC_MSG_CONTROL_ENABLE,0,&e,sizeof(e),start+600);
    assert(c->permitted && !b->group.run_used); zero(b);
    return start+600;
}
static void start(four_link_bench_t *c,drive_bench_t *b,uint32_t now)
{
    const mc_cmd_vel_v1_t v={.vx_mm_s=50,.timeout_ms=150};
    send(c,b,MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),now);
    assert(b->group.enabled && b->group.run_used);
}
static void decoding(void)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE]; size_t n; four_link_request_t r;
    mc_cmd_vel_v1_t v={.timeout_ms=150};
    for(unsigned i=0;i<5;++i) {
        uint16_t flags=0; uint32_t session=1;
        v.timeout_ms=150; v.reserved=0; v.control_mode=0;
        if(i==0) v.timeout_ms=151;
        if(i==1) v.reserved=1;
        if(i==2) v.control_mode=1;
        if(i==3) flags=MC_FLAG_RESPONSE;
        if(i==4) session=0;
        assert(mc_build_frame(MC_MSG_CMD_VEL,flags,1,session,0,&v,sizeof(v),raw,sizeof(raw),&n));
        assert(!four_link_bench_decode(raw,n,&r));
    }
    v=(mc_cmd_vel_v1_t){.timeout_ms=150};
    assert(mc_build_frame(MC_MSG_CMD_VEL,0,1,1,0,&v,sizeof(v),raw,sizeof(raw),&n));
    raw[n-1]^=1; assert(!four_link_bench_decode(raw,n,&r));
}
static void gates_and_normal_stop(void)
{
    four_link_bench_t c; drive_bench_t b; int32_t raw[4];
    uint32_t t=ready(&c,&b,raw,0); start(&c,&b,t+10);
    const mc_cmd_vel_v1_t v={.vx_mm_s=50,.timeout_ms=150}, z={.timeout_ms=150};
    for(unsigned dt=20;dt<=1000;dt+=10) {
        if(dt%20==0) send(&c,&b,MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),t+dt);
        tick(&c,&b,raw,t+dt,true);
        uint32_t cc[8]; assert(four_drive_compares(&b.group,cc));
        for(unsigned i=0;i<4;++i) assert(b.group.raw_duty[i]<=100 && b.group.raw_duty[i]>=-100);
    }
    send(&c,&b,MC_MSG_CMD_VEL,0,&z,sizeof(z),t+1001); zero(&b);
    assert(!c.permitted && !b.group.fault && b.group.run_used);
    sid=8; sequence=0; assert(send(&c,&b,MC_MSG_HELLO,0,NULL,0,t+1100));
    assert(b.group.run_used); zero(&b);
    const mc_control_enable_v1_t e={.enabled=1};
    send(&c,&b,MC_MSG_CONTROL_ENABLE,0,&e,sizeof(e),t+1101); zero(&b);
    assert(b.group.fault==FOUR_LINK_REJECTED);
}
static void timeout_and_heartbeat(void)
{
    four_link_bench_t c; drive_bench_t b; int32_t raw[4];
    uint32_t t=ready(&c,&b,raw,0); start(&c,&b,t+10);
    for(unsigned dt=20;dt<=150;dt+=10) {
        if(dt%20==0) send(&c,&b,MC_MSG_HEARTBEAT,0,NULL,0,t+dt);
        tick(&c,&b,raw,t+dt,true);
        assert(b.group.enabled);
    }
    tick(&c,&b,raw,t+160,true); zero(&b);
    assert(c.expired && b.group.fault==FOUR_SPEED_FAULT_COMMAND);
    sid=8; sequence=0; assert(send(&c,&b,MC_MSG_HELLO,0,NULL,0,t+170));
    assert(b.group.run_used && b.group.fault==FOUR_SPEED_FAULT_COMMAND); zero(&b);
}
static void hard_deadline_and_unsupported(void)
{
    four_link_bench_t c; drive_bench_t b; int32_t raw[4];
    uint32_t t=ready(&c,&b,raw,UINT32_MAX-1000U); start(&c,&b,t+10);
    const mc_cmd_vel_v1_t v={.vx_mm_s=50,.timeout_ms=150};
    for(unsigned dt=20;dt<10010;dt+=10) {
        if(dt%20==0) send(&c,&b,MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&v,sizeof(v),t+dt);
        tick(&c,&b,raw,t+dt,true); assert(b.group.enabled);
    }
    tick(&c,&b,raw,t+10010,true); zero(&b);
    assert(b.group.fault==FOUR_SPEED_FAULT_DURATION && !c.permitted);
    for(unsigned bad=0;bad<4;++bad) {
        t=ready(&c,&b,raw,0);
        mc_cmd_vel_v1_t x=v;
        if(bad==0) x.vx_mm_s=51;
        if(bad==1) x.vy_mm_s=50;
        if(bad==2) x.wz_mrad_s=200;
        if(bad==3) x.vx_mm_s=0;
        send(&c,&b,MC_MSG_CMD_VEL,MC_FLAG_ENABLED,&x,sizeof(x),t+10);
        assert(b.group.fault==FOUR_LINK_REJECTED); zero(&b);
    }
}
static void session_and_estop(void)
{
    four_link_bench_t c; drive_bench_t b; int32_t raw[4];
    uint32_t t=ready(&c,&b,raw,0); start(&c,&b,t+10);
    --sequence;
    const mc_cmd_vel_v1_t z={.timeout_ms=150};
    send(&c,&b,MC_MSG_CMD_VEL,0,&z,sizeof(z),t+20); assert(b.group.enabled);
    sid=8; send(&c,&b,MC_MSG_CMD_VEL,0,&z,sizeof(z),t+20); assert(b.group.enabled);
    sid=7; send(&c,&b,MC_MSG_ESTOP,0,NULL,0,t+30); zero(&b);
    assert(b.group.fault==SPEED_BENCH_FAULT_ESTOP);
    t=ready(&c,&b,raw,0); start(&c,&b,t+10);
    sid=8; assert(send(&c,&b,MC_MSG_HELLO,0,NULL,0,t+20)); zero(&b);
    assert(b.group.fault==FOUR_LINK_REJECTED && b.group.run_used);
    t=ready(&c,&b,raw,0);
    const mc_control_enable_v1_t clear={.clear_latched_faults=1};
    send(&c,&b,MC_MSG_CONTROL_ENABLE,0,&clear,sizeof(clear),t+10); zero(&b);
    assert(b.group.fault==FOUR_LINK_REJECTED);
}
static void payloads(void)
{
    four_link_bench_t c; drive_bench_t b; int32_t raw[4];
    uint32_t t=ready(&c,&b,raw,0);
    speed_bench_diagnostics_t d={.ms=t,.ticks=60,.dt_min=10,.dt_max=10};
    mc_wheel_telemetry_v1_t w; four_link_bench_status_t s;
    assert(four_link_bench_payloads(&c,&b,&d,&w,&s));
    assert(s.tag==FOUR_LINK_BENCH_TAG && s.permitted && w.state_flags==461);
    d.compares[7]=1; assert(!four_link_bench_payloads(&c,&b,&d,&w,&s)); d.compares[7]=0;
    start(&c,&b,t+10); tick(&c,&b,raw,t+20,false);
    assert(four_drive_compares(&b.group,d.compares));
    assert(four_link_bench_payloads(&c,&b,&d,&w,&s));
    assert(w.state_flags==967 && s.body_milli[0]==50);
    for(unsigned i=0;i<4;++i) assert(s.raw_duty[i]==80*b.adopted.wheel[i].motor_sign);
}
int main(void)
{
    decoding(); gates_and_normal_stop(); timeout_and_heartbeat();
    hard_deadline_and_unsupported(); session_and_estop(); payloads();
    puts("four_link_bench: decode, enable/zero gates, one-shot, timeout, hard deadline, wrap, estop, telemetry OK");
}
