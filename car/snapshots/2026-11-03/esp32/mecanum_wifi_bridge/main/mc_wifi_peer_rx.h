#ifndef MC_WIFI_PEER_RX_H
#define MC_WIFI_PEER_RX_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
/* QoS control precedes any optional HT control in a three-address frame.
 * 255 means no observable QoS TID; never read encrypted payload bytes. */
static inline uint32_t mc_wifi_peer_tid(const uint8_t *frame, size_t length)
{
    if (!frame || length < 26 || !(frame[0] & 0x80u)) return 255u;
    return frame[24] & 15u;
}
/* Observe only AP->our station data whose original source is our PC.
 * Payloads are neither decoded nor stored; encrypted frames are supported. */
static inline bool mc_wifi_peer_rx(const uint8_t *frame, size_t length,
    const uint8_t own[6], const uint8_t ap[6], const uint8_t pc[6],
    uint16_t *sequence, bool *retry)
{
    if (!frame || !own || !ap || !pc || !sequence || !retry || length<24)
        return false;
    if ((frame[0]&0x0fu)!=0x08u || (frame[1]&3u)!=2u
        || memcmp(frame+4,own,6) || memcmp(frame+10,ap,6)
        || memcmp(frame+16,pc,6)) return false;
    *sequence=(uint16_t)(((uint16_t)frame[23]<<8|frame[22])>>4);
    *retry=(frame[1]&8u)!=0;
    return true;
}
#endif
