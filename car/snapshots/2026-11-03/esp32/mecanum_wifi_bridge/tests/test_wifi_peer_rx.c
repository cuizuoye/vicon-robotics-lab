#include <assert.h>
#include <stdio.h>
#include "mc_wifi_peer_rx.h"
int main(void)
{
    const uint8_t own[6]={1,2,3,4,5,6},ap[6]={7,8,9,10,11,12},pc[6]={13,14,15,16,17,18};
    uint8_t frame[28]={0x88,0x42};uint16_t sequence=0;bool retry=false;
    memcpy(frame+4,own,6);memcpy(frame+10,ap,6);memcpy(frame+16,pc,6);
    frame[22]=0x30;frame[23]=0x12;
    assert(mc_wifi_peer_rx(frame,sizeof(frame),own,ap,pc,&sequence,&retry));
    assert(sequence==0x123 && !retry);
    frame[24]=6;
    assert(mc_wifi_peer_tid(frame,28)==6);
    assert(mc_wifi_peer_tid(frame,24)==255);
    assert(mc_wifi_peer_tid(NULL,28)==255);
    frame[0]=0x08;assert(mc_wifi_peer_tid(frame,28)==255);frame[0]=0x88;
    frame[1]|=8;assert(mc_wifi_peer_rx(frame,sizeof(frame),own,ap,pc,&sequence,&retry));assert(retry);
    for (size_t n=0;n<24;++n) assert(!mc_wifi_peer_rx(frame,n,own,ap,pc,&sequence,&retry));
    frame[4]^=1;assert(!mc_wifi_peer_rx(frame,28,own,ap,pc,&sequence,&retry));frame[4]^=1;
    frame[10]^=1;assert(!mc_wifi_peer_rx(frame,28,own,ap,pc,&sequence,&retry));frame[10]^=1;
    frame[16]^=1;assert(!mc_wifi_peer_rx(frame,28,own,ap,pc,&sequence,&retry));frame[16]^=1;
    frame[1]=3;assert(!mc_wifi_peer_rx(frame,28,own,ap,pc,&sequence,&retry));frame[1]=2;
    frame[0]=0x80;assert(!mc_wifi_peer_rx(frame,28,own,ap,pc,&sequence,&retry));
    assert(!mc_wifi_peer_rx(NULL,28,own,ap,pc,&sequence,&retry));
    puts("PASS: own-PC MAC filter, encrypted QoS headers, retry, bounds and unrelated frames");
}
