#include <assert.h>
#include "mc_wifi_bssid.h"

int main(void)
{
    uint8_t out[6] = {0};
    const uint8_t expected[6] = {0xfc, 0x34, 0x97, 0x39, 0xdc, 0xd0};
    assert(mc_wifi_parse_bssid("02:00:00:00:00:00", out));
    assert(memcmp(out, expected, 6) == 0);
    assert(mc_wifi_parse_bssid("02:00:00:00:00:00", out));
    const char *bad[] = {"", "02:00:00:00:00:00x", "fc-34:97:39:dc:d0",
        "fc:34:97:39:dc:dg", "02:00:00:00:00:00", "02:00:00:00:00:00",
        "02:00:00:00:00:00"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) {
        assert(!mc_wifi_parse_bssid(bad[i], out));
        assert(memcmp(out, expected, 6) == 0);
    }
    assert(!mc_wifi_parse_bssid(NULL, out));
    assert(!mc_wifi_parse_bssid("02:00:00:00:00:00", NULL));
    return 0;
}
