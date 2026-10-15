#!/usr/bin/env bash
set -Eeuo pipefail
# shellcheck source=tools/lib/wifi_power_save_state.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/../lib/wifi_power_save_state.sh"
task_on=$'wlp114s0 IEEE 802.11\n          Power Management:on\n          Link Quality=70/70\n'
task_off=$'wlp114s0 IEEE 802.11\n          Encryption key:off\n          Power Management:off\n'
wifi_power_save_matches "$task_on" on
wifi_power_save_matches "$task_off" off
assert_rejected() {
    if wifi_power_save_matches "$1" "$2"; then
        printf 'FAIL: unexpectedly accepted power state.\n' >&2
        exit 1
    fi
}
assert_rejected "$task_on" off
assert_rejected "$task_off" on
assert_rejected 'Power Management:onboard' on
assert_rejected 'missing power state' on
assert_rejected 'NotPower Management:on' on
[[ "$(wifi_power_save_state "$task_on")" == on ]]
[[ "$(wifi_power_save_state "$task_off")" == off ]]
printf 'PASS: on/off extraction, mismatch, unknown state, word boundary; no rg dependency.\n'
