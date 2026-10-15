#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mc_uart_packet.h"

int main(void)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE], packet[MC_MAX_COBS_FRAME_SIZE+2u];
    uint8_t decoded[MC_MAX_RAW_FRAME_SIZE], payload[MC_MAX_PAYLOAD_SIZE];
    memset(payload, 0x55, sizeof(payload));
    const uint8_t types[] = {MC_MSG_HELLO, MC_MSG_ESTOP, MC_MSG_CMD_VEL};
    for (unsigned i=0; i<3; ++i) {
        size_t n=0, p=0, d=0;
        const size_t body = i==2 ? sizeof(payload) : 0u;
        assert(mc_build_frame(types[i], 0, 1, 7, 0, payload, body,
                               raw, sizeof(raw), &n));
        assert(mc_uart_packet_encode(raw,n,packet,sizeof(packet),&p));
        const size_t prefix = i==0 ? 1u : 0u;
        assert((packet[0]==0u)==(prefix!=0u));
        assert(packet[p-1]==0u && p<=sizeof(packet));
        assert(mc_cobs_decode(packet+prefix,p-prefix-1u,decoded,sizeof(decoded),&d));
        assert(d==n && memcmp(raw,decoded,n)==0);
        assert(!mc_uart_packet_encode(raw,n,packet,1u,&p));
        raw[n-1]^=1u;
        assert(!mc_uart_packet_encode(raw,n,packet,sizeof(packet),&p));
    }
    size_t p=0;
    assert(!mc_uart_packet_encode(NULL,0,packet,sizeof(packet),&p));
    assert(!mc_uart_packet_encode(raw,1,NULL,sizeof(packet),&p));
    assert(!mc_uart_packet_encode(raw,1,packet,sizeof(packet),NULL));
    puts("PASS: HELLO resync delimiter, unchanged other packets, CRC and capacity guards");
    return 0;
}
