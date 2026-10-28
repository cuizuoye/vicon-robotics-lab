#ifndef MC_WIFI_ECHO_H
#define MC_WIFI_ECHO_H
#include <string.h>
#include "mc_protocol.h"
#define MC_WIFI_ECHO_TAG UINT32_C(0x31454657) /* WFE1, NOT MCU telemetry */
typedef struct MC_PACKED {
    uint32_t tag, sequence, pc_sender_ms, esp_rx_ms, rx_gap_ms, loop_max_gap_ms, send_failures;
} mc_wifi_echo_status_t;
_Static_assert(sizeof(mc_wifi_echo_status_t)==28,"Wi-Fi-only diagnostic schema");
#define MC_WIFI_ECHO_V2_TAG UINT32_C(0x32454657) /* WFE2 */
typedef struct MC_PACKED {
    mc_wifi_echo_status_t base;
    uint32_t last_errno, internal_free, internal_largest, ps_type;
    int32_t rssi;
    uint32_t disconnects;
} mc_wifi_echo_status_v2_t;
_Static_assert(sizeof(mc_wifi_echo_status_v2_t)==52,"Wi-Fi health schema");
#define MC_WIFI_ECHO_V3_TAG UINT32_C(0x33454657) /* WFE3 */
typedef struct MC_PACKED {
    uint32_t count, last_callback_ms, max_callback_gap_ms;
    uint32_t last_hardware_us, max_hardware_gap_us, last_sequence, retries;
} mc_wifi_peer_rx_stats_t;
typedef struct MC_PACKED {
    mc_wifi_echo_status_v2_t health;
    mc_wifi_peer_rx_stats_t peer_rx;
} mc_wifi_echo_status_v3_t;
_Static_assert(sizeof(mc_wifi_echo_status_v3_t)==80,"Wi-Fi RX observation schema");
static inline bool mc_wifi_echo_allowed(const uint8_t *raw, size_t length,
                                        mc_frame_header_v1_t *h)
{
    if (!h || !mc_validate_frame(raw,length,h) || !h->session_id) return false;
    if (h->message_type==MC_MSG_HELLO)
        return h->payload_length==0 && !(h->flags & ~MC_FLAG_ACK_REQUIRED);
    if (h->message_type!=MC_MSG_CMD_VEL || h->flags
        || h->payload_length!=sizeof(mc_cmd_vel_v1_t)) return false;
    mc_cmd_vel_v1_t c;
    memcpy(&c,raw+sizeof(*h),sizeof(c));
    return !c.vx_mm_s && !c.vy_mm_s && !c.wz_mrad_s && !c.control_mode && !c.reserved
        && c.timeout_ms>=20 && c.timeout_ms<=2000;
}
#endif
