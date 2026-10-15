/* Adapted from ESP-IDF v6.1 examples/wifi/getting_started/softAP and
 * examples/protocols/sockets/udp_server (Public Domain / CC0).
 * Intentionally independent from mecanum_wifi_bridge: no UART1, MC-Link,
 * motor commands, NAPT, STA uplink, or NVS erase/recovery.
 */
#include <errno.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "bench_wire.h"
static const char *TAG="sap_bench";
static void wifi_event(void *arg,esp_event_base_t base,int32_t id,void *data)
{
    (void)arg;(void)base;(void)data;
    if(id==WIFI_EVENT_AP_STACONNECTED) ESP_LOGI(TAG,"BENCH_STATION_JOINED");
    if(id==WIFI_EVENT_AP_STADISCONNECTED) ESP_LOGW(TAG,"BENCH_STATION_LEFT");
}
static void udp_task(void *arg)
{
    (void)arg;
    int fd=socket(AF_INET,SOCK_DGRAM,IPPROTO_IP);
    if(fd<0) { ESP_LOGE(TAG,"socket errno=%d",errno);vTaskDelete(NULL);return; }
    struct sockaddr_in local={.sin_family=AF_INET,.sin_port=htons(SAP_PORT)};
    inet_pton(AF_INET,"192.168.50.1",&local.sin_addr);
    if(bind(fd,(struct sockaddr*)&local,sizeof(local))<0) {
        ESP_LOGE(TAG,"bind errno=%d",errno);close(fd);vTaskDelete(NULL);return;
    }
    uint32_t boot=esp_random(),session=0,count=0;
    uint64_t last=0,max_gap=0;
    ESP_LOGI(TAG,"BENCH_UDP_READY ip=192.168.50.1 port=%d boot=%lu UART1_DISABLED",
        SAP_PORT,(unsigned long)boot);
    for(;;) {
        uint8_t in[128],out[SAP_REPLY_SIZE];struct sockaddr_in peer;
        socklen_t size=sizeof(peer);
        int n=recvfrom(fd,in,sizeof(in),0,(struct sockaddr*)&peer,&size);
        uint64_t rx=(uint64_t)esp_timer_get_time();
        if(n<0) { ESP_LOGE(TAG,"recv errno=%d",errno);break; }
        if(!sap_valid(in,(size_t)n)) continue;
        uint32_t sid=sap_u32(in+12);
        if(sid!=session) { session=sid;last=0;max_gap=0;count=0; }
        if(last && rx-last>max_gap) max_gap=rx-last;
        last=rx;++count;
        sap_reply(out,in,rx,(uint64_t)esp_timer_get_time(),boot,count,max_gap);
        if(sendto(fd,out,sizeof(out),0,(struct sockaddr*)&peer,size)<0)
            ESP_LOGW(TAG,"send errno=%d",errno);
    }
    close(fd);vTaskDelete(NULL);
}
void app_main(void)
{
    size_t plen=strlen(CONFIG_SAP_PASSWORD),slen=strlen(CONFIG_SAP_SSID);
    if(plen<8 || plen>63 || slen==0 || slen>32) {
        ESP_LOGE(TAG,"Configure SSID and WPA2 passphrase; no open fallback");return;
    }
    /* Deliberately fail instead of erasing a previous bridge's NVS. */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap=esp_netif_create_default_wifi_ap();assert(ap);
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap));
    esp_netif_ip_info_t info={0};
    IP4_ADDR(&info.ip,192,168,50,1);IP4_ADDR(&info.gw,192,168,50,1);
    IP4_ADDR(&info.netmask,255,255,255,0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap,&info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap));
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,
        wifi_event,NULL,NULL));
    wifi_config_t conf={.ap={.channel=CONFIG_SAP_CHANNEL,.max_connection=1,
        .authmode=WIFI_AUTH_WPA2_PSK,.pmf_cfg={.required=false}}};
    memcpy(conf.ap.ssid,CONFIG_SAP_SSID,slen);conf.ap.ssid_len=slen;
    memcpy(conf.ap.password,CONFIG_SAP_PASSWORD,plen);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&conf));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP,WIFI_BW20));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG,"BENCH_AP_READY ssid=%s channel=%d HT20 WPA2 RAM_CONFIG no_NAT no_UART1",
        CONFIG_SAP_SSID,CONFIG_SAP_CHANNEL);
    assert(xTaskCreate(udp_task,"sap_udp",4096,NULL,5,NULL)==pdPASS);
}
