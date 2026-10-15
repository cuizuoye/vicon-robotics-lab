#ifndef MC_WIFI_BSSID_H
#define MC_WIFI_BSSID_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static inline int mc_wifi_hex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Reject malformed, multicast and zero AP addresses; no partial writes. */
static inline bool mc_wifi_parse_bssid(const char *text, uint8_t output[6])
{
    if (!text || !output || strlen(text) != 17) return false;
    uint8_t parsed[6];
    unsigned nonzero = 0;
    for (unsigned i = 0; i < 6; ++i) {
        const int hi = mc_wifi_hex(text[3 * i]);
        const int lo = mc_wifi_hex(text[3 * i + 1]);
        if (hi < 0 || lo < 0 || (i < 5 && text[3 * i + 2] != ':')) return false;
        parsed[i] = (uint8_t)((hi << 4) | lo);
        nonzero |= parsed[i];
    }
    if (!nonzero || (parsed[0] & 1)) return false;
    memcpy(output, parsed, sizeof(parsed));
    return true;
}
#endif
