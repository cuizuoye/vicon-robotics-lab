#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "mc_protocol.h"
#include "mc_uart_diagnostic.h"
#include "mc_uart_consumer.h"
#include "mc_uart_packet.h"
#include "mc_link_trace.h"
#include "mc_wifi_echo.h"
#include "mc_wifi_peer_rx.h"
#include "mc_wifi_bssid.h"
#include "mc_wifi_ap.h"
#include "mc_radio_trace.h"
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
#include "esp_vfs_eventfd.h"
#include "mc_event_forward.h"
static int s_uart_wakeup_fd = -1;
static _Atomic bool s_uart_wakeup_error;

static bool forward_wakeup_init(void)
{
    const esp_vfs_eventfd_config_t config = {.max_fds=1};
    if (esp_vfs_eventfd_register(&config) != ESP_OK) return false;
    s_uart_wakeup_fd = eventfd(0, 0);
    if (s_uart_wakeup_fd < 0 || s_uart_wakeup_fd >= FD_SETSIZE) {
        if (s_uart_wakeup_fd >= 0) close(s_uart_wakeup_fd);
        s_uart_wakeup_fd = -1;
        (void)esp_vfs_eventfd_unregister();
        return false;
    }
    return true;
}
#endif

#ifdef CONFIG_MC_WIFI_RESOURCE_DIAGNOSTICS
#include "lwip/tcpip.h"
#include "lwip/stats.h"
#include "mc_wifi_resource_stats.h"
static _Atomic(TaskHandle_t) s_echo_task_handle;
static _Atomic bool s_resource_pending;
static _Atomic uint32_t s_resource_requested_ms, s_resource_enqueue_failures;
static portMUX_TYPE s_resource_mux=portMUX_INITIALIZER_UNLOCKED;
static mc_wifi_resource_stats_t s_resources;
#endif


#define WIFI_CONNECTED_BIT BIT0
#define UART_RX_CHUNK_SIZE 128u
#define TELEMETRY_QUEUE_LENGTH 12u
#define UART_LINK_TEST_SESSION_ID 0x53544D32u
#define UART_LINK_TEST_SEQUENCE 1u
#define UART_LINK_TEST_TIMEOUT_MS 500u
#define UDP_BURST_DRAIN_LIMIT 64u

static const char *TAG = "mc_bridge";
static EventGroupHandle_t s_wifi_events;
static QueueHandle_t s_uart_to_udp_queue;
static bool s_uart_enabled;
#if defined(CONFIG_MC_UART_STARTUP_PROBE) || defined(CONFIG_MC_UART_DIAGNOSTIC_ONLY)
static uint32_t s_uart_test_session_id;
#endif
/* Written by the self-test/UDP reader and read by the UART RX task on another
 * core. Do not use an unsynchronized bool/session pair here. */
static _Atomic uint32_t s_uart_consumer_session;
#ifdef CONFIG_MC_WIFI_SOFTAP
/* A quick leave/rejoin must not resurrect a socket's old session merely
 * because the availability bit was cleared and set between task polls. */
static _Atomic uint32_t s_ap_disconnect_epoch;
#endif

typedef struct {
    size_t length;
    uint8_t data[MC_MAX_RAW_FRAME_SIZE];
} mc_frame_item_t;


static uint32_t monotonic_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000u);
}

#ifdef CONFIG_MC_WIFI_RESOURCE_DIAGNOSTICS
static void resource_snapshot(void *argument)
{
    (void)argument;
    mc_wifi_resource_stats_t snapshot={.sample_ms=monotonic_time_ms(),
        .tcpip_stack_free_min_bytes=uxTaskGetStackHighWaterMark(NULL)};
    snapshot.callback_delay_ms=snapshot.sample_ms-atomic_load(&s_resource_requested_ms);
    TaskHandle_t echo=atomic_load(&s_echo_task_handle);
    snapshot.echo_stack_free_min_bytes=echo?uxTaskGetStackHighWaterMark(echo):0;
#if UDP_STATS
    snapshot.udp_rx=lwip_stats.udp.recv;snapshot.udp_drop=lwip_stats.udp.drop;
    snapshot.udp_memerr=lwip_stats.udp.memerr;snapshot.udp_chkerr=lwip_stats.udp.chkerr;
#endif
#if IP_STATS
    snapshot.ip_drop=lwip_stats.ip.drop;snapshot.ip_memerr=lwip_stats.ip.memerr;
#endif
#if ETHARP_STATS
    snapshot.arp_memerr=lwip_stats.etharp.memerr;
#endif
#if MEMP_STATS
    snapshot.pool_stats_available=1;
    for (size_t i=0;i<MEMP_MAX;++i) {
        const struct stats_mem *pool=lwip_stats.memp[i];
        if (pool) snapshot.pool_allocation_errors+=pool->err;
    }
    const struct stats_mem *pbuf=lwip_stats.memp[MEMP_PBUF_POOL];
    if (pbuf) {
        snapshot.pbuf_used=pbuf->used;snapshot.pbuf_peak=pbuf->max;
        snapshot.pbuf_allocation_errors=pbuf->err;
    }
#if ARP_QUEUEING
    const struct stats_mem *arp=lwip_stats.memp[MEMP_ARP_QUEUE];
    if (arp) {
        snapshot.arp_queue_used=arp->used;snapshot.arp_queue_peak=arp->max;
        snapshot.arp_queue_allocation_errors=arp->err;
    }
#endif
#endif
    portENTER_CRITICAL(&s_resource_mux);s_resources=snapshot;portEXIT_CRITICAL(&s_resource_mux);
    atomic_store(&s_resource_pending,false);
}
static void resource_observe(void)
{
    mc_wifi_resource_stats_t snapshot;
    portENTER_CRITICAL(&s_resource_mux);snapshot=s_resources;portEXIT_CRITICAL(&s_resource_mux);
    ESP_LOGI(TAG,"WIFI_RESOURCES ms=%lu sample=%lu callback_delay=%lu pending=%u enqueue_errors=%lu echo_stack=%lu tcpip_stack=%lu udp_rx=%lu udp_drop=%lu udp_memerr=%lu udp_chkerr=%lu ip_drop=%lu ip_memerr=%lu arp_memerr=%lu pools_available=%lu pool_errors=%lu pbuf_used=%lu pbuf_peak=%lu pbuf_errors=%lu arp_used=%lu arp_peak=%lu arp_errors=%lu",
        (unsigned long)monotonic_time_ms(),(unsigned long)snapshot.sample_ms,
        (unsigned long)snapshot.callback_delay_ms,(unsigned)atomic_load(&s_resource_pending),
        (unsigned long)atomic_load(&s_resource_enqueue_failures),
        (unsigned long)snapshot.echo_stack_free_min_bytes,(unsigned long)snapshot.tcpip_stack_free_min_bytes,
        (unsigned long)snapshot.udp_rx,(unsigned long)snapshot.udp_drop,
        (unsigned long)snapshot.udp_memerr,(unsigned long)snapshot.udp_chkerr,
        (unsigned long)snapshot.ip_drop,(unsigned long)snapshot.ip_memerr,
        (unsigned long)snapshot.arp_memerr,(unsigned long)snapshot.pool_stats_available,
        (unsigned long)snapshot.pool_allocation_errors,(unsigned long)snapshot.pbuf_used,
        (unsigned long)snapshot.pbuf_peak,(unsigned long)snapshot.pbuf_allocation_errors,
        (unsigned long)snapshot.arp_queue_used,(unsigned long)snapshot.arp_queue_peak,
        (unsigned long)snapshot.arp_queue_allocation_errors);
    bool idle=false;
    if (atomic_compare_exchange_strong(&s_resource_pending,&idle,true)) {
        atomic_store(&s_resource_requested_ms,monotonic_time_ms());
        if (tcpip_try_callback(resource_snapshot,NULL)!=ERR_OK) {
            atomic_fetch_add(&s_resource_enqueue_failures,1u);
            atomic_store(&s_resource_pending,false);
        }
    }
}
#endif

#ifdef CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS
static uint8_t s_rx_own[6], s_rx_ap[6], s_rx_pc[6];
static portMUX_TYPE s_peer_rx_mux=portMUX_INITIALIZER_UNLOCKED;
static mc_wifi_peer_rx_stats_t s_peer_rx;
#ifdef CONFIG_MC_LINK_TIMING_TRACE
static mc_radio_trace_t s_radio_trace;
#endif
static void peer_radio_rx(void *buffer, wifi_promiscuous_pkt_type_t type)
{
    if (type!=WIFI_PKT_DATA || !buffer) return;
    const wifi_promiscuous_pkt_t *packet=buffer;
    uint16_t sequence;bool retry;
    if (!mc_wifi_peer_rx(packet->payload,packet->rx_ctrl.sig_len,
        s_rx_own,s_rx_ap,s_rx_pc,&sequence,&retry)) return;
    const uint32_t now=monotonic_time_ms(), hardware=packet->rx_ctrl.timestamp;
    portENTER_CRITICAL(&s_peer_rx_mux);
    if (s_peer_rx.count) {
        uint32_t gap=now-s_peer_rx.last_callback_ms;
        uint32_t hardware_gap=hardware-s_peer_rx.last_hardware_us;
        if (gap>s_peer_rx.max_callback_gap_ms) s_peer_rx.max_callback_gap_ms=gap;
        if (hardware_gap>s_peer_rx.max_hardware_gap_us) s_peer_rx.max_hardware_gap_us=hardware_gap;
    }
    ++s_peer_rx.count;s_peer_rx.last_callback_ms=now;s_peer_rx.last_hardware_us=hardware;
    s_peer_rx.last_sequence=sequence;s_peer_rx.retries+=(uint32_t)retry;
#ifdef CONFIG_MC_LINK_TIMING_TRACE
    mc_radio_push(&s_radio_trace, now, hardware, sequence, retry, packet->rx_ctrl.sig_len,
                  mc_wifi_peer_tid(packet->payload, packet->rx_ctrl.sig_len));
#endif
    portEXIT_CRITICAL(&s_peer_rx_mux);
}
static void peer_radio_start(void)
{
    ESP_ERROR_CHECK(mc_wifi_parse_bssid(CONFIG_MC_WIFI_DIAGNOSTIC_PC_MAC, s_rx_pc)
                    ? ESP_OK : ESP_ERR_INVALID_ARG);
    wifi_ap_record_t ap;ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap));
    memcpy(s_rx_ap,ap.bssid,6);ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA,s_rx_own));
    const wifi_promiscuous_filter_t filter={.filter_mask=WIFI_PROMIS_FILTER_MASK_DATA};
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(peer_radio_rx));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_LOGI(TAG,"WIFI_RX_METADATA_READY: only own PC->STA headers, no payload capture");
}
#endif

#ifdef CONFIG_MC_WIFI_HEALTH_DIAGNOSTICS
static _Atomic uint32_t s_internal_free, s_internal_largest, s_ps_type;
static _Atomic int32_t s_rssi;
static _Atomic uint32_t s_disconnects, s_disconnect_reason;
static void wifi_health_task(void *argument)
{
    (void)argument;
    ESP_LOGI(TAG,"WIFI_HEALTH_BOOT reset=%d heap_integrity=%d",
             esp_reset_reason(),heap_caps_check_integrity_all(false));
    for (;;) {
        const uint32_t caps=MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT;
        const uint32_t free_bytes=heap_caps_get_free_size(caps);
        const uint32_t largest=heap_caps_get_largest_free_block(caps);
        wifi_ps_type_t ps;wifi_ap_record_t ap;
        uint32_t ps_value=esp_wifi_get_ps(&ps)==ESP_OK?(uint32_t)ps:UINT32_MAX;
        int32_t rssi=esp_wifi_sta_get_ap_info(&ap)==ESP_OK?ap.rssi:INT32_MIN;
        atomic_store(&s_internal_free,free_bytes);atomic_store(&s_internal_largest,largest);
        atomic_store(&s_ps_type,ps_value);atomic_store(&s_rssi,rssi);
        ESP_LOGI(TAG,"WIFI_HEALTH ms=%lu free=%lu largest=%lu min=%lu ps=%lu rssi=%ld disconnects=%lu reason=%lu",
            (unsigned long)monotonic_time_ms(),(unsigned long)free_bytes,(unsigned long)largest,
            (unsigned long)heap_caps_get_minimum_free_size(caps),(unsigned long)ps_value,
            (long)rssi,(unsigned long)atomic_load(&s_disconnects),
            (unsigned long)atomic_load(&s_disconnect_reason));
#ifdef CONFIG_MC_WIFI_RESOURCE_DIAGNOSTICS
        resource_observe();
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

#ifdef CONFIG_MC_LINK_TIMING_TRACE
static mc_link_trace_t s_link_trace;
static void trace_uart(const uint8_t *raw, size_t length, uint32_t started, bool ok)
{
    mc_frame_header_v1_t h;
    if (mc_validate_frame(raw,length,&h))
        mc_trace_push(&s_link_trace,MC_TRACE_UART,h.sequence,started,
                      monotonic_time_ms()-started,ok);
}
static void trace_fault(const mc_frame_item_t *item, const mc_frame_header_v1_t *h)
{
    if (s_link_trace.dumped || h->message_type!=MC_MSG_WHEEL_TELEMETRY
        || h->payload_length!=sizeof(mc_wheel_telemetry_v1_t)) return;
    mc_wheel_telemetry_v1_t t;
    memcpy(&t,item->data+sizeof(*h),sizeof(t));
    if (!t.fault_bits) return;
    s_link_trace.dumped=true;
#ifdef CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS
    portENTER_CRITICAL(&s_peer_rx_mux);
    s_radio_trace.frozen=true;
    portEXIT_CRITICAL(&s_peer_rx_mux);
#endif
    ESP_LOGI(TAG,"TRACE_FAULT sid=%lu esp_ms=%lu mcu_ms=%lu last_cmd=%lu fault=%lu",
        (unsigned long)h->session_id,(unsigned long)monotonic_time_ms(),
        (unsigned long)t.sample_time_ms,(unsigned long)t.last_command_sequence,
        (unsigned long)t.fault_bits);
    for (uint32_t i=0;i<s_link_trace.count;++i) {
        const mc_trace_event_t *e=mc_trace_get(&s_link_trace,i);
        ESP_LOGI(TAG,"TRACE_EVENT kind=%lu seq=%lu at_ms=%lu duration_ms=%lu detail=%lu",
            (unsigned long)e->kind,(unsigned long)e->sequence,(unsigned long)e->at_ms,
            (unsigned long)e->duration_ms,(unsigned long)e->detail);
    }
#ifdef CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS
    for (uint32_t i=0;i<s_radio_trace.count;++i) {
        const mc_radio_event_t *e=mc_radio_get(&s_radio_trace,i);
        ESP_LOGI(TAG,"TRACE_RADIO cb_ms=%lu hw_us=%lu seq=%lu retry=%lu len=%lu tid=%lu",
            (unsigned long)e->callback_ms,(unsigned long)e->hardware_us,
            (unsigned long)e->sequence,(unsigned long)e->retry,(unsigned long)e->frame_length,
            (unsigned long)e->tid);
    }
    ESP_LOGI(TAG,"TRACE_RADIO_END count=%lu",(unsigned long)s_radio_trace.count);
#endif
    ESP_LOGI(TAG,"TRACE_END");
}
#endif


static void wifi_event_handler(
    void *argument,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    (void)argument;
#ifdef CONFIG_MC_WIFI_SOFTAP
    if (event_base != WIFI_EVENT) return;
    (void)event_data;
    mc_wifi_ap_event_t event = MC_AP_OTHER;
    switch (event_id) {
    case WIFI_EVENT_AP_START: event = MC_AP_STARTED; break;
    case WIFI_EVENT_AP_STACONNECTED: event = MC_AP_JOINED; break;
    case WIFI_EVENT_AP_STADISCONNECTED: event = MC_AP_LEFT; break;
    case WIFI_EVENT_AP_STOP: event = MC_AP_STOPPED; break;
    default: break;
    }
    const int action = mc_wifi_ap_availability(event);
    if (action < 0) {
        atomic_fetch_add(&s_ap_disconnect_epoch, 1u);
        atomic_store(&s_uart_consumer_session, 0u);
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "SoftAP client/link unavailable; new HELLO handshake required");
    } else if (action > 0) {
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "SoftAP available at 192.168.50.1 (event=%ld)", (long)event_id);
    }
#else
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
#ifdef CONFIG_MC_WIFI_HEALTH_DIAGNOSTICS
        const wifi_event_sta_disconnected_t *disconnected=event_data;
        atomic_fetch_add(&s_disconnects,1u);
        atomic_store(&s_disconnect_reason,disconnected->reason);
#endif
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting");
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "Wi-Fi ready, IP=" IPSTR, IP2STR(&event->ip_info.ip));
#ifdef CONFIG_MC_WIFI_AP_PIN
        wifi_ap_record_t ap;
        ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap));
        ESP_LOGI(TAG, "Pinned AP connected: %02x:%02x:%02x:%02x:%02x:%02x channel=%u",
            ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5],
            (unsigned)ap.primary);
#endif
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
#ifdef CONFIG_MC_WIFI_ECHO_ONLY
        uint8_t protocols=0;
        ESP_ERROR_CHECK(esp_wifi_get_protocol(WIFI_IF_STA,&protocols));
        ESP_LOGI(TAG,"WIFI_PROTOCOL_BITMAP=%u",(unsigned)protocols);
#endif
    }
#endif
}


static void wifi_start(void)
{
    s_wifi_events = xEventGroupCreate();
    assert(s_wifi_events != NULL);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#ifdef CONFIG_MC_WIFI_SOFTAP
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif != NULL);
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    esp_netif_ip_info_t ap_ip = {0};
    IP4_ADDR(&ap_ip.ip, 192, 168, 50, 1);
    IP4_ADDR(&ap_ip.gw, 192, 168, 50, 1);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ap_ip));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));
#else
    assert(esp_netif_create_default_wifi_sta() != NULL);
#endif

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
#ifndef CONFIG_MC_WIFI_SOFTAP
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));
#endif

#ifdef CONFIG_MC_WIFI_SOFTAP
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t ap_config = {.ap = {
        .channel = CONFIG_MC_WIFI_SOFTAP_CHANNEL,
        .max_connection = 1,
        .authmode = WIFI_AUTH_WPA2_PSK,
        .pmf_cfg = {.required = false},
    }};
    const size_t ssid_length = strlen(CONFIG_MC_WIFI_SOFTAP_SSID);
    memcpy(ap_config.ap.ssid, CONFIG_MC_WIFI_SOFTAP_SSID, ssid_length);
    ap_config.ap.ssid_len = ssid_length;
    memcpy(ap_config.ap.password, CONFIG_MC_WIFI_SOFTAP_PASSWORD,
           strlen(CONFIG_MC_WIFI_SOFTAP_PASSWORD));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "MC_LINK_SOFTAP_READY ssid=%s ip=192.168.50.1 channel=%d HT20 WPA2 RAM_CONFIG no_NAT",
             CONFIG_MC_WIFI_SOFTAP_SSID, CONFIG_MC_WIFI_SOFTAP_CHANNEL);
#else
    wifi_config_t station_config = {0};
    strlcpy((char *)station_config.sta.ssid, CONFIG_MC_WIFI_SSID,
            sizeof(station_config.sta.ssid));
    strlcpy((char *)station_config.sta.password, CONFIG_MC_WIFI_PASSWORD,
            sizeof(station_config.sta.password));
    station_config.sta.threshold.authmode =
        CONFIG_MC_WIFI_PASSWORD[0] == '\0' ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
#ifdef CONFIG_MC_WIFI_AP_PIN
    ESP_ERROR_CHECK(mc_wifi_parse_bssid(CONFIG_MC_WIFI_AP_BSSID, station_config.sta.bssid)
                    ? ESP_OK : ESP_ERR_INVALID_ARG);
    station_config.sta.bssid_set = true;
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &station_config));
#ifdef CONFIG_MC_WIFI_ECHO_ONLY
    /* Driver protocol settings can survive in NVS. Every bench image must
     * explicitly select its intended bitmap, not inherit the last experiment. */
#ifdef CONFIG_MC_WIFI_LEGACY_BG_DIAGNOSTIC
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G));
#else
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N));
#endif
#endif
    /* Disable power saving to reduce command jitter on the control link. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
#endif
#ifdef CONFIG_MC_WIFI_HEALTH_DIAGNOSTICS
    assert(xTaskCreate(wifi_health_task,"mc_wifi_health",4096,NULL,1,NULL)==pdPASS);
#endif
}


static bool uart_send_raw_frame(const uint8_t *raw, size_t raw_length)
{
    if (!s_uart_enabled) {
        return false;
    }

    uint8_t encoded[MC_MAX_COBS_FRAME_SIZE + 2u];
    size_t encoded_length = 0u;
    if (!mc_uart_packet_encode(raw, raw_length, encoded, sizeof(encoded),
                               &encoded_length)) {
        ESP_LOGE(TAG, "COBS encode failed");
        return false;
    }
#ifdef CONFIG_MC_LINK_TIMING_TRACE
    const uint32_t trace_started = monotonic_time_ms();
#endif
    const int written = uart_write_bytes(
        (uart_port_t)CONFIG_MC_UART_PORT, encoded, encoded_length);
#ifdef CONFIG_MC_LINK_TIMING_TRACE
    trace_uart(raw,raw_length,trace_started,written==(int)encoded_length);
#endif
    if (written != (int)encoded_length) {
        ESP_LOGE(TAG, "UART short write: %d/%u", written, (unsigned)encoded_length);
        return false;
    }
    return true;
}


static void uart_rx_task(void *argument)
{
    (void)argument;
    uint8_t chunk[UART_RX_CHUNK_SIZE];
    uint8_t encoded[MC_MAX_COBS_FRAME_SIZE];
    size_t encoded_length = 0u;

    for (;;) {
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
        int received = uart_read_bytes((uart_port_t)CONFIG_MC_UART_PORT,
                                      chunk, 1u, portMAX_DELAY);
        if (received > 0) {
            size_t buffered = 0;
            if (uart_get_buffered_data_len((uart_port_t)CONFIG_MC_UART_PORT, &buffered) != ESP_OK) {
                atomic_store(&s_uart_wakeup_error, true);
                vTaskDelete(NULL); return;
            }
            const size_t take = mc_forward_drain_count(buffered, sizeof(chunk));
            if (take) {
                const int extra = uart_read_bytes((uart_port_t)CONFIG_MC_UART_PORT, chunk+1u, take, 0);
                if (extra < 0) { atomic_store(&s_uart_wakeup_error, true); vTaskDelete(NULL); return; }
                received += extra;
            }
        } else if (received < 0) {
            atomic_store(&s_uart_wakeup_error, true); vTaskDelete(NULL); return;
        }
#else
        const int received = uart_read_bytes(
            (uart_port_t)CONFIG_MC_UART_PORT,
            chunk,
            sizeof(chunk),
            pdMS_TO_TICKS(20));
#endif
        for (int index = 0; index < received; ++index) {
            const uint8_t byte = chunk[index];
            if (byte == 0u) {
                if (encoded_length > 0u) {
                    mc_frame_item_t item = {0};
                    mc_frame_header_v1_t header;
                    if (mc_cobs_decode(encoded, encoded_length, item.data,
                                       sizeof(item.data), &item.length)
                        && mc_validate_frame(item.data, item.length, &header)) {
                        /* No host/self-test reader: discard valid telemetry
                         * rather than filling a queue that nobody drains. */
                        if (!mc_uart_consumer_accepts(
                                atomic_load(&s_uart_consumer_session), &header)) {
                            encoded_length = 0u;
                            continue;
                        }
                        if (xQueueSend(s_uart_to_udp_queue, &item, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "Telemetry queue full; frame dropped");
                        }
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
                        else if (!mc_forward_notify(s_uart_wakeup_fd)) {
                            atomic_store(&s_uart_wakeup_error, true);
                        }
#endif
                    } else {
                        ESP_LOGW(TAG, "Invalid UART frame dropped");
                    }
                }
                encoded_length = 0u;
            } else if (encoded_length < sizeof(encoded)) {
                encoded[encoded_length++] = byte;
            } else {
                /* Drop an overlong frame and resynchronize at the next delimiter. */
                encoded_length = 0u;
                ESP_LOGW(TAG, "Overlong UART frame dropped");
            }
        }
    }
}


static void uart_start(void)
{
    if (CONFIG_MC_UART_TX_GPIO < 0 || CONFIG_MC_UART_RX_GPIO < 0) {
        s_uart_enabled = false;
        ESP_LOGW(TAG, "UART bridge disabled; set TX/RX GPIO after checking the board pinout");
        return;
    }

    const uart_config_t uart_config = {
        .baud_rate = CONFIG_MC_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(
        (uart_port_t)CONFIG_MC_UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config((uart_port_t)CONFIG_MC_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(
        (uart_port_t)CONFIG_MC_UART_PORT,
        CONFIG_MC_UART_TX_GPIO,
        CONFIG_MC_UART_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE));

    s_uart_to_udp_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(mc_frame_item_t));
    assert(s_uart_to_udp_queue != NULL);
    s_uart_enabled = true;
    assert(xTaskCreate(uart_rx_task, "mc_uart_rx", 4096, NULL, 10, NULL) == pdPASS);
    ESP_LOGI(TAG, "UART bridge enabled: UART%d TX=%d RX=%d baud=%d",
             CONFIG_MC_UART_PORT, CONFIG_MC_UART_TX_GPIO,
             CONFIG_MC_UART_RX_GPIO, CONFIG_MC_UART_BAUD_RATE);
}


#if defined(CONFIG_MC_UART_STARTUP_PROBE) || defined(CONFIG_MC_UART_DIAGNOSTIC_ONLY)
static bool uart_link_self_test(void)
{
    if (!s_uart_enabled) {
        return false;
    }

    uint8_t request[MC_MAX_RAW_FRAME_SIZE];
    size_t request_length = 0u;
    /* Fresh session allows an ESP-only reboot to re-handshake a running MCU. */
    s_uart_test_session_id = esp_random();
    if (!s_uart_test_session_id) s_uart_test_session_id = UART_LINK_TEST_SESSION_ID;
    if (!mc_build_frame(
            MC_MSG_HELLO,
            0u,
            UART_LINK_TEST_SEQUENCE,
            s_uart_test_session_id,
            monotonic_time_ms(),
            NULL,
            0u,
            request,
            sizeof(request),
            &request_length)) {
        ESP_LOGE(TAG, "UART link self-test FAIL: HELLO build failed");
        return false;
    }

    /* Clear stale telemetry so the result belongs to this one-shot probe. */
    atomic_store(&s_uart_consumer_session, s_uart_test_session_id);
    xQueueReset(s_uart_to_udp_queue);
    if (!uart_send_raw_frame(request, request_length)) {
        ESP_LOGE(TAG, "UART link self-test FAIL: HELLO transmit failed");
        return false;
    }

    const uint32_t started = monotonic_time_ms();
    unsigned skipped = 0;
    for (;;) {
        const uint32_t elapsed = monotonic_time_ms() - started;
        if (elapsed >= UART_LINK_TEST_TIMEOUT_MS) break;
        const uint32_t remaining = UART_LINK_TEST_TIMEOUT_MS - elapsed;
        mc_frame_item_t response;
        if (xQueueReceive(s_uart_to_udp_queue, &response, pdMS_TO_TICKS(remaining)) != pdTRUE)
            break;
        if (mc_uart_ack_matches(response.data, response.length,
                                s_uart_test_session_id, UART_LINK_TEST_SEQUENCE)) {
            ESP_LOGI(TAG, "UART link self-test PASS: STM32 HELLO_ACK received (skipped=%u)", skipped);
            return true;
        }
        ++skipped;
    }
    ESP_LOGW(TAG, "UART link self-test WAITING: STM32 HELLO_ACK timeout (skipped=%u)", skipped);
    return false;
}
#endif


#ifdef CONFIG_MC_UART_DIAGNOSTIC_ONLY
static void uart_readonly_diagnostic(void)
{
    mc_uart_diagnostic_t state = {0};
    uint32_t report_started = monotonic_time_ms();
    uint32_t previous_wheel = 0, previous_status = 0;
    uint32_t previous_sample_ms = 0, previous_ticks = 0;
    ESP_LOGI(TAG, "UART_DIAG_ONLY: one HELLO only; Wi-Fi/UDP/NVS and motor commands disabled");
    for (;;) {
        mc_frame_item_t item;
        if (xQueueReceive(s_uart_to_udp_queue, &item, pdMS_TO_TICKS(50)) == pdTRUE)
            (void)mc_uart_diagnostic_accept(&state, item.data, item.length, s_uart_test_session_id);
        if (monotonic_time_ms() - report_started >= 1000u) {
            const bool fresh = state.wheel_frames > previous_wheel && state.status_frames > previous_status
                && state.wheel.sample_time_ms - previous_sample_ms > 0u
                && state.status.ticks - previous_ticks > 0u;
            ESP_LOGI(TAG, "UART_DIAG: wheel=%lu status=%lu invalid=%lu safe=%d fresh=%d ms=%lu ticks=%lu gpio_low=%u fault=%lu",
                (unsigned long)state.wheel_frames, (unsigned long)state.status_frames,
                (unsigned long)state.invalid_frames, mc_uart_diagnostic_safe(&state), fresh,
                (unsigned long)state.wheel.sample_time_ms, (unsigned long)state.status.ticks,
                state.status.gpio_low, (unsigned long)state.wheel.fault_bits);
            previous_wheel = state.wheel_frames; previous_status = state.status_frames;
            previous_sample_ms = state.wheel.sample_time_ms; previous_ticks = state.status.ticks;
            report_started = monotonic_time_ms();
        }
    }
}
#endif


static bool command_payload_is_valid(
    const uint8_t *frame,
    const mc_frame_header_v1_t *header)
{
    const uint8_t *payload = frame + sizeof(*header);
    switch (header->message_type) {
    case MC_MSG_HELLO:
    case MC_MSG_HEARTBEAT:
        return header->payload_length == 0u;
    case MC_MSG_CMD_VEL: {
        if (header->payload_length != sizeof(mc_cmd_vel_v1_t)) {
            return false;
        }
        mc_cmd_vel_v1_t command;
        memcpy(&command, payload, sizeof(command));
        return command.timeout_ms >= 20u && command.timeout_ms <= 2000u
            && command.control_mode == 0u && command.reserved == 0u;
    }
    case MC_MSG_CONTROL_ENABLE:
        return header->payload_length == sizeof(mc_control_enable_v1_t);
    case MC_MSG_ESTOP:
        return header->payload_length == 0u;
    default:
        return false;
    }
}


static void send_timeout_disable(
    uint32_t session_id,
    uint32_t last_sequence)
{
    const mc_cmd_vel_v1_t zero_command = {
        .vx_mm_s = 0,
        .vy_mm_s = 0,
        .wz_mrad_s = 0,
        .timeout_ms = 150,
        .control_mode = 0,
        .reserved = 0,
    };
    uint8_t frame[MC_MAX_RAW_FRAME_SIZE];
    size_t frame_length = 0u;
    if (mc_build_frame(
            MC_MSG_CMD_VEL, 0u, last_sequence + 1u, session_id,
            monotonic_time_ms(), &zero_command, sizeof(zero_command),
            frame, sizeof(frame), &frame_length)) {
        (void)uart_send_raw_frame(frame, frame_length);
    }
}


static bool same_ipv4_host(
    const struct sockaddr_in *left,
    const struct sockaddr_in *right)
{
    return left->sin_family == AF_INET && right->sin_family == AF_INET
        && left->sin_addr.s_addr == right->sin_addr.s_addr;
}


static bool send_hello_ack(
    int socket_fd,
    const struct sockaddr_in *peer_address,
    const mc_frame_header_v1_t *hello_header)
{
    uint8_t frame[MC_MAX_RAW_FRAME_SIZE];
    size_t frame_length = 0u;
    if (!mc_build_frame(
            MC_MSG_HELLO_ACK, MC_FLAG_RESPONSE,
            hello_header->sequence, hello_header->session_id,
            monotonic_time_ms(), NULL, 0u,
            frame, sizeof(frame), &frame_length)) {
        ESP_LOGE(TAG, "Failed to build HELLO_ACK");
        return false;
    }

    struct sockaddr_in destination = *peer_address;
    destination.sin_port = htons(CONFIG_MC_UDP_TELEMETRY_PORT);
    const ssize_t sent = sendto(
        socket_fd, frame, frame_length, 0,
        (const struct sockaddr *)&destination, sizeof(destination));
    if (sent != (ssize_t)frame_length) {
        ESP_LOGW(TAG, "HELLO_ACK send failed: errno=%d", errno);
        return false;
    }
    ESP_LOGI(TAG, "HELLO_ACK sent to %s:%d",
             inet_ntoa(destination.sin_addr), CONFIG_MC_UDP_TELEMETRY_PORT);
    return true;
}


#ifdef CONFIG_MC_WIFI_ECHO_ONLY
static void wifi_echo_task(void *argument)
{
    (void)argument;
#ifdef CONFIG_MC_WIFI_RESOURCE_DIAGNOSTICS
    atomic_store(&s_echo_task_handle,xTaskGetCurrentTaskHandle());
#endif
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE],out[MC_MAX_RAW_FRAME_SIZE];
    for (;;) {
        xEventGroupWaitBits(s_wifi_events,WIFI_CONNECTED_BIT,pdFALSE,pdTRUE,portMAX_DELAY);
        int fd=socket(AF_INET,SOCK_DGRAM,IPPROTO_IP);
        if (fd<0) { vTaskDelay(pdMS_TO_TICKS(1000));continue; }
        const struct timeval timeout={.tv_sec=0,.tv_usec=20000};
        (void)setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        const struct sockaddr_in local={.sin_family=AF_INET,
            .sin_port=htons(CONFIG_MC_UDP_COMMAND_PORT),.sin_addr.s_addr=htonl(INADDR_ANY)};
        if (bind(fd,(const struct sockaddr *)&local,sizeof(local))<0) {
            close(fd);vTaskDelay(pdMS_TO_TICKS(1000));continue;
        }
#ifdef CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS
        peer_radio_start();
#endif
        ESP_LOGI(TAG,"WIFI_ECHO_ONLY_READY: UART not initialized, WFE1 is not MCU telemetry");
        uint32_t sid=0,last_seq=0,last_rx=0,loop_previous=monotonic_time_ms(),loop_max=0,failures=0,last_errno=0;
        bool seen=false;
        struct sockaddr_in peer={0};
        while ((xEventGroupGetBits(s_wifi_events)&WIFI_CONNECTED_BIT)!=0u) {
            uint32_t now=monotonic_time_ms();
            uint32_t loop_gap=now-loop_previous;loop_previous=now;
            if (loop_gap>loop_max) loop_max=loop_gap;
            struct sockaddr_in source={0};socklen_t source_size=sizeof(source);
            ssize_t n=recvfrom(fd,raw,sizeof(raw),0,(struct sockaddr *)&source,&source_size);
            if (n<0) {
                if (errno!=EAGAIN && errno!=EWOULDBLOCK) break;
                continue;
            }
            now=monotonic_time_ms();mc_frame_header_v1_t h;
            if (!mc_wifi_echo_allowed(raw,(size_t)n,&h)) continue;
            if (h.message_type==MC_MSG_HELLO) {
                peer=source;sid=h.session_id;last_seq=h.sequence;seen=false;loop_max=0;failures=0;last_errno=0;
#ifdef CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS
                portENTER_CRITICAL(&s_peer_rx_mux);
                memset(&s_peer_rx,0,sizeof(s_peer_rx));
                portEXIT_CRITICAL(&s_peer_rx_mux);
#endif
                (void)send_hello_ack(fd,&peer,&h);continue;
            }
            if (!sid || h.session_id!=sid || !same_ipv4_host(&source,&peer)
                || !mc_sequence_is_newer(h.sequence,last_seq)) continue;
            mc_wifi_echo_status_t status={.tag=MC_WIFI_ECHO_TAG,.sequence=h.sequence,
                .pc_sender_ms=h.sender_time_ms,.esp_rx_ms=now,.rx_gap_ms=seen?now-last_rx:0,
                .loop_max_gap_ms=loop_max,.send_failures=failures};
            last_rx=now;last_seq=h.sequence;seen=true;
            size_t length=0;
#ifdef CONFIG_MC_WIFI_HEALTH_DIAGNOSTICS
            status.tag=MC_WIFI_ECHO_V2_TAG;
            mc_wifi_echo_status_v2_t detailed={.base=status,.last_errno=last_errno,
                .internal_free=atomic_load(&s_internal_free),.internal_largest=atomic_load(&s_internal_largest),
                .ps_type=atomic_load(&s_ps_type),.rssi=atomic_load(&s_rssi),
                .disconnects=atomic_load(&s_disconnects)};
#ifdef CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS
            detailed.base.tag=MC_WIFI_ECHO_V3_TAG;
            mc_wifi_echo_status_v3_t observed={.health=detailed};
            portENTER_CRITICAL(&s_peer_rx_mux);
            observed.peer_rx=s_peer_rx;
            portEXIT_CRITICAL(&s_peer_rx_mux);
            const void *payload=&observed;const size_t payload_length=sizeof(observed);
#else
            const void *payload=&detailed;const size_t payload_length=sizeof(detailed);
#endif
#else
            const void *payload=&status;const size_t payload_length=sizeof(status);
            (void)last_errno;
#endif
            if (mc_build_frame(MC_MSG_LINK_STATUS,MC_FLAG_RESPONSE,h.sequence,sid,now,
                payload,payload_length,out,sizeof(out),&length)) {
                struct sockaddr_in destination=peer;
                destination.sin_port=htons(CONFIG_MC_UDP_TELEMETRY_PORT);
                const ssize_t written=sendto(fd,out,length,0,(const struct sockaddr *)&destination,sizeof(destination));
                if (written!=(ssize_t)length) {
                    last_errno=written<0?(uint32_t)errno:(uint32_t)EIO;
                    ++failures;
                }
            }
        }
        close(fd);
    }
}
#endif

static void udp_bridge_task(void *argument)
{
    (void)argument;
    uint8_t receive_buffer[MC_MAX_RAW_FRAME_SIZE];

    for (;;) {
        xEventGroupWaitBits(
            s_wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        const int socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (socket_fd < 0) {
            ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        const struct timeval receive_timeout = {
            .tv_sec = 0,
            .tv_usec = 20000,
        };
        (void)setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                         &receive_timeout, sizeof(receive_timeout));

        const struct sockaddr_in listen_address = {
            .sin_family = AF_INET,
            .sin_port = htons(CONFIG_MC_UDP_COMMAND_PORT),
            .sin_addr.s_addr = htonl(INADDR_ANY),
        };
        if (bind(socket_fd, (const struct sockaddr *)&listen_address,
                 sizeof(listen_address)) < 0) {
            ESP_LOGE(TAG, "bind(%d) failed: errno=%d",
                     CONFIG_MC_UDP_COMMAND_PORT, errno);
            close(socket_fd);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
#if defined(CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS) && defined(CONFIG_MC_LINK_TIMING_TRACE)
        peer_radio_start();
#endif
        ESP_LOGI(TAG, "UDP command listener ready on port %d",
                 CONFIG_MC_UDP_COMMAND_PORT);

        bool session_active = false;
        bool timeout_latched = false;
        bool command_fresh = false;
        uint32_t session_id = 0u;
        uint32_t last_sequence = 0u;
        TickType_t last_command_tick = 0u;
        struct sockaddr_in peer_address = {0};
#ifdef CONFIG_MC_WIFI_SOFTAP
        const uint32_t ap_epoch = atomic_load(&s_ap_disconnect_epoch);
#endif

#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
        bool event_failure = false;
#endif
        while ((xEventGroupGetBits(s_wifi_events) & WIFI_CONNECTED_BIT) != 0u
#ifdef CONFIG_MC_WIFI_SOFTAP
               && atomic_load(&s_ap_disconnect_epoch) == ap_epoch
#endif
        ) {
#ifdef CONFIG_MC_LINK_TIMING_TRACE
            mc_trace_push(&s_link_trace,MC_TRACE_LOOP,last_sequence,
                          monotonic_time_ms(),0u,0u);
#endif
            if (s_uart_enabled && session_active) {
                mc_frame_item_t telemetry;
                while (xQueueReceive(s_uart_to_udp_queue, &telemetry, 0) == pdTRUE) {
                    mc_frame_header_v1_t telemetry_header;
                    if (!mc_validate_frame(telemetry.data, telemetry.length,
                                           &telemetry_header)
                        || telemetry_header.session_id != session_id) {
                        ESP_LOGW(TAG, "Telemetry from another session dropped");
                        continue;
                    }
                    struct sockaddr_in destination = peer_address;
                    destination.sin_port = htons(CONFIG_MC_UDP_TELEMETRY_PORT);
#ifdef CONFIG_MC_LINK_TIMING_TRACE
                    const uint32_t trace_started=monotonic_time_ms();
#endif
                    const ssize_t sent = sendto(
                        socket_fd, telemetry.data, telemetry.length, 0,
                        (const struct sockaddr *)&destination, sizeof(destination));
#ifdef CONFIG_MC_LINK_TIMING_TRACE
                    mc_trace_push(&s_link_trace,MC_TRACE_TELEMETRY,telemetry_header.sequence,
                        trace_started,monotonic_time_ms()-trace_started,
                        sent==(ssize_t)telemetry.length);
                    trace_fault(&telemetry,&telemetry_header);
#endif
                    if (sent != (ssize_t)telemetry.length) {
                        ESP_LOGW(TAG, "UDP telemetry send failed: errno=%d", errno);
                    }
                }
            }

            if (command_fresh
                && (xTaskGetTickCount() - last_command_tick)
                    > pdMS_TO_TICKS(CONFIG_MC_UDP_COMMAND_TIMEOUT_MS)) {
                /* Send one disabled zero command before locking the expired
                 * session.  The STM32 still owns the independent 150 ms
                 * command watchdog and final high-Z motor shutdown. */
                send_timeout_disable(session_id, last_sequence);
                command_fresh = false;
                timeout_latched = true;
                ESP_LOGE(TAG, "UDP command timeout: output disabled, new HELLO session required");
            }

#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
            if (atomic_load(&s_uart_wakeup_error)) {
                ESP_LOGE(TAG, "UART event forwarding failed; close UDP, no automatic enable or clear");
                event_failure = true; break;
            }
            const int event_ready = mc_forward_wait(socket_fd, s_uart_wakeup_fd);
            if (event_ready < 0) { ESP_LOGE(TAG, "Forwarding select failed"); event_failure = true; break; }
            if (!event_ready) continue; /* UART wake: drain validated queue at loop top. */
#endif
            struct sockaddr_in sender_address = {0};
            socklen_t sender_length = sizeof(sender_address);
            ssize_t received = recvfrom(
                socket_fd, receive_buffer, sizeof(receive_buffer),
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
                MSG_DONTWAIT,
#else
                0,
#endif
                (struct sockaddr *)&sender_address, &sender_length);
            if (received < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                ESP_LOGW(TAG, "recvfrom() failed: errno=%d", errno);
                break;
            }

            /* Mobile hotspots can release several delayed UDP datagrams in one
             * burst.  Preserve state/safety frames, but collapse consecutive
             * CMD_VEL frames to the newest one before writing the slower UART
             * side.  Sequence gaps are valid in MC-Link. */
            uint8_t pending_velocity[MC_MAX_RAW_FRAME_SIZE];
            size_t pending_velocity_length = 0u;
            unsigned drained_datagrams = 0u;
            for (;;) {
#ifdef CONFIG_MC_WIFI_SOFTAP
                if (atomic_load(&s_ap_disconnect_epoch) != ap_epoch) {
                    pending_velocity_length = 0u;
                    break;
                }
#endif
                mc_frame_header_v1_t header;
                if (!mc_validate_frame(receive_buffer, (size_t)received, &header)
                    || !command_payload_is_valid(receive_buffer, &header)
                    || header.session_id == 0u) {
                    ESP_LOGW(TAG, "Invalid UDP command frame dropped");
                } else {
                    bool accepted = false;
                    if (header.message_type == MC_MSG_HELLO) {
                        if (timeout_latched && header.session_id == session_id) {
                            ESP_LOGW(TAG, "Timed-out session rejected; PC must choose a new session_id");
                        } else {
                            peer_address = sender_address;
                            session_id = header.session_id;
#ifdef CONFIG_MC_LINK_TIMING_TRACE
                            s_link_trace=(mc_link_trace_t){0};
#ifdef CONFIG_MC_WIFI_RX_TIMING_DIAGNOSTICS
                            portENTER_CRITICAL(&s_peer_rx_mux);
                            s_radio_trace=(mc_radio_trace_t){0};
                            portEXIT_CRITICAL(&s_peer_rx_mux);
#endif
#endif
                            last_sequence = header.sequence;
                            session_active = true;
                            timeout_latched = false;
                            command_fresh = true;
                            last_command_tick = xTaskGetTickCount();
                            pending_velocity_length = 0u;
                            if (s_uart_enabled) {
                                atomic_store(&s_uart_consumer_session, header.session_id);
                                xQueueReset(s_uart_to_udp_queue);
                            }
                            ESP_LOGI(TAG, "HELLO accepted: session=0x%08lx",
                                     (unsigned long)session_id);
                            if (s_uart_enabled) {
                                ESP_LOGI(TAG, "HELLO will be acknowledged by STM32 over UART");
                            } else {
                                /* Wi-Fi-only bring-up keeps a local ACK path.
                                 * With UART enabled, only the STM32 may ACK. */
                                (void)send_hello_ack(
                                    socket_fd, &peer_address, &header);
                            }
                            accepted = true;
                        }
                    } else if (!session_active || timeout_latched
                               || header.session_id != session_id
                               || !same_ipv4_host(&sender_address, &peer_address)
                               || !mc_sequence_is_newer(
                                   header.sequence, last_sequence)) {
                        ESP_LOGW(TAG, "Stale or foreign UDP command dropped");
                    } else {
                        last_sequence = header.sequence;
                        command_fresh = true;
                        last_command_tick = xTaskGetTickCount();
                        accepted = true;
                    }

                    if (accepted && s_uart_enabled) {
#ifdef CONFIG_MC_LINK_TIMING_TRACE
                        mc_trace_push(&s_link_trace,MC_TRACE_RX,header.sequence,
                            monotonic_time_ms(),0u,header.sender_time_ms);
#endif
                        if (header.message_type == MC_MSG_CMD_VEL) {
                            memcpy(pending_velocity, receive_buffer,
                                   (size_t)received);
                            pending_velocity_length = (size_t)received;
                        } else {
                            /* A newer state frame supersedes any older pending
                             * velocity frame.  Forward it immediately so ESTOP
                             * and disable commands never wait behind a burst. */
                            pending_velocity_length = 0u;
                            (void)uart_send_raw_frame(
                                receive_buffer, (size_t)received);
                        }
                    }
                }

                ++drained_datagrams;
                if (drained_datagrams >= UDP_BURST_DRAIN_LIMIT) {
                    ESP_LOGW(TAG, "UDP burst drain limit reached");
                    break;
                }
                sender_length = sizeof(sender_address);
                received = recvfrom(
                    socket_fd, receive_buffer, sizeof(receive_buffer),
                    MSG_DONTWAIT,
                    (struct sockaddr *)&sender_address, &sender_length);
                if (received < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        ESP_LOGW(TAG, "burst recvfrom() failed: errno=%d", errno);
                    }
                    break;
                }
            }

            if (pending_velocity_length > 0u) {
#ifdef CONFIG_MC_WIFI_SOFTAP
                if (atomic_load(&s_ap_disconnect_epoch) != ap_epoch) continue;
#endif
                (void)uart_send_raw_frame(
                    pending_velocity, pending_velocity_length);
            }
        }

        atomic_store(&s_uart_consumer_session, 0u);
        if (s_uart_enabled) {
            xQueueReset(s_uart_to_udp_queue);
        }
        close(socket_fd);
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
        if (event_failure) {
            ESP_LOGE(TAG, "Event forwarder stopped; manual inspection required, no automatic restart");
            vTaskDelete(NULL); return;
        }
#endif
        ESP_LOGW(TAG, "UDP bridge stopped; waiting for Wi-Fi");
    }
}


void app_main(void)
{
    ESP_ERROR_CHECK(mc_protocol_self_test() ? ESP_OK : ESP_FAIL);

#ifdef CONFIG_MC_UART_DIAGNOSTIC_ONLY
    uart_start();
    if (!s_uart_enabled) {
        ESP_LOGE(TAG, "UART-only diagnosis requires checked TX/RX GPIO settings");
        return;
    }
    /* Bench-only window: TX is idle, so a powered-off-motor STM32 can be
       explicitly reset by the host after ESP boot/programming disturbances.
       This is not fault auto-clear and is not used in the Wi-Fi control mode. */
    ESP_LOGI(TAG, "UART_DIAG_IDLE_READY: stable TX; waiting 2000ms before HELLO");
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (!uart_link_self_test()) {
        atomic_store(&s_uart_consumer_session, 0u);
        ESP_LOGE(TAG, "UART-only diagnosis stopped: no matching HELLO_ACK; no retry or commands");
        return;
    }
    uart_readonly_diagnostic();
    return;
#endif

    esp_err_t nvs_result = nvs_flash_init();
#ifndef CONFIG_MC_WIFI_SOFTAP
    if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES
        || nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_result = nvs_flash_init();
    }
#endif
    ESP_ERROR_CHECK(nvs_result);

#ifdef CONFIG_MC_WIFI_SOFTAP
    if (!mc_wifi_ap_valid(CONFIG_MC_WIFI_SOFTAP_SSID,
                          CONFIG_MC_WIFI_SOFTAP_PASSWORD,
                          CONFIG_MC_WIFI_SOFTAP_CHANNEL)) {
        ESP_LOGE(TAG, "Invalid SoftAP configuration; no UART or open-AP fallback");
        return;
    }
#else
    if (CONFIG_MC_WIFI_SSID[0] == '\0') {
        ESP_LOGE(TAG, "Wi-Fi SSID is empty; configure it in SDK Configuration Editor");
        return;
    }
#endif

#ifdef CONFIG_MC_WIFI_ECHO_ONLY
    /* Return before uart_start: isolation mode has no path to the STM32. */
    wifi_start();
    assert(xTaskCreate(wifi_echo_task,"mc_wifi_echo",6144,NULL,9,NULL)==pdPASS);
    return;
#endif
#ifdef CONFIG_MC_EVENT_DRIVEN_FORWARD
    if (!forward_wakeup_init()) { ESP_LOGE(TAG, "Forward wake initialization failed; no UART/Wi-Fi start"); return; }
#endif
    uart_start();
    if (s_uart_enabled) {
#ifdef CONFIG_MC_UART_STARTUP_PROBE
        (void)uart_link_self_test();
#else
        ESP_LOGI(TAG, "UART idle: waiting for PC HELLO; no startup commands sent");
#endif
        /* Only the diagnostic-only branch retains the boot probe reader.
         * Normal operation waits for a PC HELLO, even if the hotspot is off. */
        atomic_store(&s_uart_consumer_session, 0u);
        xQueueReset(s_uart_to_udp_queue);
    }
    wifi_start();
    assert(xTaskCreate(udp_bridge_task, "mc_udp_bridge", 6144, NULL, 9, NULL) == pdPASS);
}
