/* Compile against the preserved ESP32 protocol/consumer source, no ESP write. */
#include "mc_protocol.h"
#include "mc_uart_consumer.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
    uint8_t payload[200],raw[MC_MAX_RAW_FRAME_SIZE],cobs[MC_MAX_COBS_FRAME_SIZE],back[MC_MAX_RAW_FRAME_SIZE];
    for(unsigned i=0;i<sizeof(payload);i++)payload[i]=(uint8_t)i;
    size_t n,encoded,decoded;mc_frame_header_v1_t h;
    assert(mc_build_frame(MC_MSG_LINK_STATUS,MC_FLAG_ENABLED,8,123,1000,payload,sizeof(payload),raw,sizeof(raw),&n));
    assert(mc_cobs_encode(raw,n,cobs,sizeof(cobs),&encoded));
    assert(mc_cobs_decode(cobs,encoded,back,sizeof(back),&decoded));
    assert(n==decoded && !memcmp(raw,back,n) && mc_validate_frame(back,decoded,&h));
    assert(h.payload_length==200 && mc_uart_consumer_accepts(123,&h));
    assert(!mc_uart_consumer_accepts(0,&h) && !mc_uart_consumer_accepts(124,&h));
    back[30]^=1;assert(!mc_validate_frame(back,decoded,&h));
    puts("PASS: preserved ESP32 CRC/COBS/consumer accepts intact 200-byte status; no hardware or network");
    return 0;
}
