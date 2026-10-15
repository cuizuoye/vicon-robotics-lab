#ifndef MC_WIFI_RESOURCE_STATS_H
#define MC_WIFI_RESOURCE_STATS_H
#include <stdint.h>
/* Low-rate diagnostic snapshots, not motor/control telemetry. Protocol counters
 * can wrap; SDK system/pool stats are observational, not globally transactional. */
typedef struct {
    uint32_t sample_ms,callback_delay_ms;
    uint32_t echo_stack_free_min_bytes,tcpip_stack_free_min_bytes;
    uint32_t udp_rx,udp_drop,udp_memerr,udp_chkerr,ip_drop,ip_memerr,arp_memerr;
    uint32_t pool_stats_available,pool_allocation_errors;
    uint32_t pbuf_used,pbuf_peak,pbuf_allocation_errors;
    uint32_t arp_queue_used,arp_queue_peak,arp_queue_allocation_errors;
} mc_wifi_resource_stats_t;
#endif
