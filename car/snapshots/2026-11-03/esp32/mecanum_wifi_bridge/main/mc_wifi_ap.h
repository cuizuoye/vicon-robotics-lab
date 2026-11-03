#ifndef MC_WIFI_AP_H
#define MC_WIFI_AP_H
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* AP-only configuration. Never silently fall back to an open hotspot. */
static inline bool mc_wifi_ap_valid(const char *ssid, const char *password, int channel)
{
    if (ssid == NULL || password == NULL) return false;
    const size_t sn = strlen(ssid), pn = strlen(password);
    return sn > 0 && sn <= 32 && pn >= 8 && pn <= 63 && channel >= 1 && channel <= 11;
}

typedef enum {
    MC_AP_STARTED, MC_AP_JOINED, MC_AP_LEFT, MC_AP_STOPPED, MC_AP_OTHER
} mc_wifi_ap_event_t;

/* Stateless event action: -1 clears availability, +1 sets it, 0 ignores it.
 * This never clears application faults, creates a session or enables control. */
static inline int mc_wifi_ap_availability(mc_wifi_ap_event_t event)
{
    switch (event) {
    case MC_AP_STARTED:
    case MC_AP_JOINED: return 1;
    case MC_AP_LEFT:
    case MC_AP_STOPPED: return -1;
    default: return 0;
    }
}
#endif
