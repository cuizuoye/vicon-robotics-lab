#include <assert.h>
#include <stdio.h>
#include "mc_wifi_echo.h"
static bool accept(uint8_t type, uint16_t flags, uint32_t sid, const void *p, size_t n)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t length=0;mc_frame_header_v1_t h;
    assert(mc_build_frame(type,flags,2,sid,0,p,n,raw,sizeof(raw),&length));
    return mc_wifi_echo_allowed(raw,length,&h);
}
int main(void)
{
    mc_cmd_vel_v1_t c={.timeout_ms=150};
    assert(accept(MC_MSG_HELLO,MC_FLAG_ACK_REQUIRED,1,NULL,0));
    assert(!accept(MC_MSG_HELLO,MC_FLAG_ENABLED,1,NULL,0));
    assert(!accept(MC_MSG_HELLO,0,0,NULL,0));
    assert(accept(MC_MSG_CMD_VEL,0,1,&c,sizeof(c)));
    assert(!accept(MC_MSG_CMD_VEL,MC_FLAG_ENABLED,1,&c,sizeof(c)));
    c.vx_mm_s=1;assert(!accept(MC_MSG_CMD_VEL,0,1,&c,sizeof(c)));c.vx_mm_s=0;
    c.vy_mm_s=-1;assert(!accept(MC_MSG_CMD_VEL,0,1,&c,sizeof(c)));c.vy_mm_s=0;
    c.wz_mrad_s=1;assert(!accept(MC_MSG_CMD_VEL,0,1,&c,sizeof(c)));c.wz_mrad_s=0;
    c.reserved=1;assert(!accept(MC_MSG_CMD_VEL,0,1,&c,sizeof(c)));c.reserved=0;
    c.timeout_ms=0;assert(!accept(MC_MSG_CMD_VEL,0,1,&c,sizeof(c)));
    assert(!accept(MC_MSG_CONTROL_ENABLE,0,1,NULL,0));
    assert(!accept(MC_MSG_ESTOP,0,1,NULL,0));
    puts("PASS: Wi-Fi-only schema accepts HELLO/disabled zero, rejects all motor/state commands");
}
