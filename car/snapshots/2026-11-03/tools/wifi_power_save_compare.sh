#!/usr/bin/env bash
# One approved transient interface setting; always restore it on exit.
set -Eeuo pipefail
if [[ ${EUID} -ne 0 || $# -ne 1 || $1 != --confirm-battery-disconnected ]]; then
    printf 'Run with sudo and --confirm-battery-disconnected; motor battery must be unplugged.\n' >&2
    exit 2
fi
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$task_workspace" == /home/PRIMARY_USER/vicon_mecanum_ws ]] || exit 2
# shellcheck source=tools/lib/wifi_power_save_state.sh
source "$task_workspace/tools/lib/wifi_power_save_state.sh"
task_interface=wlp114s0
task_iw=/usr/sbin/iwconfig
task_cli=/home/PRIMARY_USER/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI
[[ "$(nmcli -g GENERAL.CONNECTION device show "$task_interface")" == OMGGZ-2.4G ]] || {
    printf 'Expected network is not active; nothing changed.\n' >&2; exit 2;
}
[[ "$(nmcli -g IP4.ADDRESS device show "$task_interface")" == 192.168.100.37/24 ]] || {
    printf 'PC address changed; update the test deliberately before running.\n' >&2; exit 2;
}
task_original="$(wifi_power_save_state "$($task_iw "$task_interface" 2>/dev/null)")"
[[ "$task_original" == on || "$task_original" == off ]] || exit 2
umask 077
task_result_dir="$(mktemp -d "$task_workspace/tmp/wifi_power_save_local_20261014_XXXXXXXX")"
task_probe_rc=not_started
# Called indirectly by the EXIT trap; older ShellCheck cannot follow it.
# shellcheck disable=SC2317
cleanup() {
    task_exit_rc=$?
    trap - EXIT
    set +e
    "$task_iw" "$task_interface" power "$task_original" > "$task_result_dir/restore.log" 2>&1
    task_restore_rc=$?
    "$task_iw" "$task_interface" > "$task_result_dir/after.log" 2>&1
    if ! wifi_power_save_matches "$(< "$task_result_dir/after.log")" "$task_original"; then
        task_restore_rc=1
    fi
    printf 'original=%s\nprobe_exit=%s\nrestore_exit=%s\n' \
        "$task_original" "$task_probe_rc" "$task_restore_rc" > "$task_result_dir/result.txt"
    if [[ ${SUDO_UID:-} =~ ^[0-9]+$ && ${SUDO_GID:-} =~ ^[0-9]+$ ]]; then
        chown -R -- "${SUDO_UID}:${SUDO_GID}" "$task_result_dir"
    fi
    printf '\nResults: %s\n' "$task_result_dir"
    if [[ $task_restore_rc -ne 0 ]]; then
        printf 'WARNING: power-save restoration failed; see restore.log.\n' >&2
        exit 1
    fi
    printf 'Wi-Fi power saving restored to %s. No profile, proxy, IP or virtual interface changed.\n' "$task_original"
    exit "$task_exit_rc"
}
trap 'cleanup' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$task_iw" "$task_interface" > "$task_result_dir/before.log" 2>&1
# Verify the exact no-PWM MCU image before sending even disabled zero commands.
timeout --kill-after=2s 15s "$task_cli" -q -c port=SWD sn=000000000000000000000000 mode=HOTPLUG freq=1000 \
    -u 0x08000000 26988 "$task_result_dir/stm32_readback.bin" > "$task_result_dir/readback.log" 2>&1
task_hash="$(sha256sum "$task_result_dir/stm32_readback.bin" | cut -d ' ' -f 1)"
[[ "$task_hash" == 12d4224f5337b42d3bdc3eef30f4142e957d65d900d61b64d8b8fb22033e9aa6 ]] || {
    printf 'MCU is not the verified readonly image; comparison refused.\n' >&2; exit 2;
}
"$task_iw" "$task_interface" power off
"$task_iw" "$task_interface" > "$task_result_dir/during.log" 2>&1
wifi_power_save_matches "$(< "$task_result_dir/during.log")" off
printf 'Power saving temporarily off. Running one 15-second zero-only comparison.\n'
set +e
timeout --signal=TERM --kill-after=2s 25s /usr/bin/python3 \
    "$task_workspace/stm32/mecanum_f407/tools/four_link_ro_probe.py" \
    --esp-ip 192.168.100.26 --bind-ip 192.168.100.37 --interface "$task_interface" \
    --seconds 15 --check-timeout --confirm-battery-disconnected \
    --output "$task_result_dir/udp_readonly.json" > "$task_result_dir/probe.log" 2>&1
task_probe_rc=$?
set -e
if [[ $task_probe_rc -eq 0 ]]; then
    printf 'Readonly comparison passed.\n'
else
    printf 'Readonly comparison failed or timed out (exit %s); retained logs.\n' "$task_probe_rc"
fi
exit "$task_probe_rc"
