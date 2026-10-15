#include <assert.h>
#include "mc_wifi_ap.h"
int main(void)
{
    assert(mc_wifi_ap_valid("Mecanum-Bench-S3", "bench-pass", 6));
    assert(!mc_wifi_ap_valid(NULL, "bench-pass", 6));
    assert(!mc_wifi_ap_valid("x", NULL, 6));
    assert(!mc_wifi_ap_valid("", "bench-pass", 6));
    assert(!mc_wifi_ap_valid("x", "", 6));
    assert(!mc_wifi_ap_valid("x", "short", 6));
    assert(!mc_wifi_ap_valid("x", "bench-pass", 0));
    assert(!mc_wifi_ap_valid("x", "bench-pass", 12));
    char ssid[34], pass[65];
    memset(ssid, 's', sizeof(ssid)); ssid[32] = 0;
    memset(pass, 'p', sizeof(pass)); pass[63] = 0;
    assert(mc_wifi_ap_valid(ssid, pass, 11));
    ssid[32] = 's'; ssid[33] = 0;
    assert(!mc_wifi_ap_valid(ssid, pass, 11));
    ssid[32] = 0; pass[63] = 'p'; pass[64] = 0;
    assert(!mc_wifi_ap_valid(ssid, pass, 11));
    assert(mc_wifi_ap_availability(MC_AP_STARTED) == 1);
    assert(mc_wifi_ap_availability(MC_AP_JOINED) == 1);
    assert(mc_wifi_ap_availability(MC_AP_LEFT) == -1);
    assert(mc_wifi_ap_availability(MC_AP_STOPPED) == -1);
    assert(mc_wifi_ap_availability(MC_AP_OTHER) == 0);
    return 0;
}
