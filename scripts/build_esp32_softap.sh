#!/usr/bin/env bash
# Build only with untracked private config. Never opens COM or erases NVS.
set -euo pipefail
handoff_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command -v idf.py >/dev/null || { printf 'Activate the pinned ESP-IDF v6.1 environment first.\n' >&2; exit 2; }
: "${MC_AP_PASSWORD:?Set a new private MC_AP_PASSWORD locally; never commit it}"
export MC_AP_SSID=${MC_AP_SSID:-Mecanum-Bench-S3}
export MC_AP_PASSWORD
mkdir -p "$handoff_root/.local/esp32"
chmod 700 "$handoff_root/.local/esp32"
python3 - "$handoff_root/.local/esp32/softap.defaults" <<'PY'
import json, os, sys
ssid=os.environ['MC_AP_SSID']; password=os.environ['MC_AP_PASSWORD']
if not 1<=len(ssid.encode())<=32 or not 8<=len(password.encode())<=63:
    raise SystemExit('SSID must be 1..32 bytes and WPA2 password 8..63 bytes')
config='\n'.join(['CONFIG_MC_WIFI_SOFTAP=y',
    'CONFIG_MC_WIFI_SOFTAP_SSID='+json.dumps(ssid),
    'CONFIG_MC_WIFI_SOFTAP_PASSWORD='+json.dumps(password),
    'CONFIG_MC_WIFI_SOFTAP_CHANNEL=6','CONFIG_MC_UART_PORT=1',
    'CONFIG_MC_UART_TX_GPIO=17','CONFIG_MC_UART_RX_GPIO=18',
    'CONFIG_MC_UART_BAUD_RATE=921600','CONFIG_MC_UDP_COMMAND_TIMEOUT_MS=750',''])
fd=os.open(sys.argv[1],os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
with os.fdopen(fd,'w') as stream:stream.write(config)
PY
handoff_project="$handoff_root/car/esp32/mecanum_wifi_bridge"
idf.py -C "$handoff_project" -B "$handoff_root/.local/build/esp32-softap" \
    -D "SDKCONFIG=$handoff_root/.local/esp32/sdkconfig" \
    -D "SDKCONFIG_DEFAULTS=$handoff_project/sdkconfig.defaults;$handoff_root/.local/esp32/softap.defaults" build
printf 'Built only. Private config is ignored by Git. No flashing or device reset.\n'
