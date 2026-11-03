#!/usr/bin/env bash
# Capture only the checked car endpoint; do not modify any network setting.
set -Eeuo pipefail
if [[ ${EUID} -ne 0 || $# -ne 1 || $1 != --confirm-battery-disconnected ]]; then
    printf 'Run with sudo and --confirm-battery-disconnected; motor battery must be unplugged.\n' >&2
    exit 2
fi
[[ ${SUDO_UID:-} =~ ^[0-9]+$ && ${SUDO_GID:-} =~ ^[0-9]+$ && ${SUDO_UID} -ne 0 ]] || exit 2
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$task_workspace" == /home/PRIMARY_USER/vicon_mecanum_ws ]] || exit 2
# shellcheck source=tools/lib/capture_process.sh
source "$task_workspace/tools/lib/capture_process.sh"
task_user="$(id -un "$SUDO_UID")"
task_interface=wlp114s0
task_pc_ip=10.236.223.61
task_esp_ip=10.236.223.148
[[ "$(nmcli -g GENERAL.CONNECTION device show "$task_interface")" == 'Vivo X90s' ]] || {
    printf 'Expected phone hotspot is not active; nothing changed.\n' >&2; exit 2;
}
[[ "$(nmcli -g IP4.ADDRESS device show "$task_interface")" == "$task_pc_ip/24" ]] || {
    printf 'PC address changed; exact capture refused.\n' >&2; exit 2;
}
umask 077
task_result_dir="$(mktemp -d "$task_workspace/tmp/packet_capture_20261014_XXXXXXXX")"
chown -- "$SUDO_UID:$SUDO_GID" "$task_result_dir"
task_capture_pid=''
task_probe_rc=not_started
task_analysis_rc=not_started
task_capture_rc=not_started
task_stage=capture_start
task_error_line=none
task_error_status=none
# Indirectly invoked by traps; ShellCheck cannot follow that invocation.
# shellcheck disable=SC2317
cleanup() {
    task_exit_rc=$?
    trap - EXIT ERR
    set +e
    if [[ -n "$task_capture_pid" ]]; then
        if capture_stop "$task_capture_pid"; then task_capture_rc=0; else task_capture_rc=$?; fi
    fi
    chown -R -- "$SUDO_UID:$SUDO_GID" "$task_result_dir"
    if [[ -s "$task_result_dir/trial/probe.json" && -s "$task_result_dir/link.pcap" ]]; then
        /usr/bin/setpriv --reuid="$SUDO_UID" --regid="$SUDO_GID" --init-groups \
            /usr/bin/python3 "$task_workspace/stm32/mecanum_f407/tools/packet_timing.py" \
            --pcap "$task_result_dir/link.pcap" --probe "$task_result_dir/trial/probe.json" \
            --pc-ip "$task_pc_ip" --esp-ip "$task_esp_ip" \
            > "$task_result_dir/analysis.json" 2> "$task_result_dir/analysis.log"
        task_analysis_rc=$?
    fi
    printf 'probe_exit=%s\nanalysis_exit=%s\ncapture_exit=%s\nmain_exit=%s\nstage=%s\nerror_line=%s\nerror_status=%s\n' \
        "$task_probe_rc" "$task_analysis_rc" "$task_capture_rc" "$task_exit_rc" \
        "$task_stage" "$task_error_line" "$task_error_status" > "$task_result_dir/result.txt"
    chown -R -- "$SUDO_UID:$SUDO_GID" "$task_result_dir"
    printf '\nResults: %s\nCapture stopped. No proxy, IP, routing or virtual interface setting changed.\n' "$task_result_dir"
    exit "$task_exit_rc"
}
trap 'cleanup' EXIT
trap 'task_error_status=$? task_error_line=$LINENO' ERR
trap 'exit 130' INT
trap 'exit 143' TERM
# Create synchronously before the child starts. Reading a nonexistent file via
# Bash's $(< file) shortcut can abort the shell even inside an if condition.
: > "$task_result_dir/capture.log"
/usr/bin/tcpdump -p -n -U -i "$task_interface" -s 256 -Z "$task_user" \
    -w "$task_result_dir/link.pcap" \
    "udp and host $task_pc_ip and host $task_esp_ip and (port 46000 or port 46001)" \
    > "$task_result_dir/capture.log" 2>&1 &
task_capture_pid=$!
task_stage=capture_wait
task_ready=false
for ((task_i=0;task_i<50;task_i++)); do
    kill -0 "$task_capture_pid" 2>/dev/null || break
    if capture_log_ready "$task_result_dir/capture.log" "$task_interface"; then
        task_ready=true; break
    fi
    sleep 0.1
done
[[ "$task_ready" == true ]] || { printf 'Capture failed to start; no UDP probe sent.\n' >&2; exit 2; }
printf 'Filtered capture ready. Running verified no-PWM zero-only timing probe.\n'
task_stage=readonly_probe
if timeout --signal=TERM --kill-after=2s 60s \
    /usr/bin/setpriv --reuid="$SUDO_UID" --regid="$SUDO_GID" --init-groups \
    /usr/bin/python3 "$task_workspace/stm32/mecanum_f407/tools/link_timing_bench.py" \
    --esp-ip "$task_esp_ip" --bind-ip "$task_pc_ip" --interface "$task_interface" \
    --esp-log-serial /dev/serial/by-id/usb-1a86_USB_Single_Serial_00000000-if00 \
    --result-dir "$task_result_dir/trial" --confirm-battery-disconnected \
    > "$task_result_dir/probe_console.log" 2>&1; then
    task_probe_rc=0
else
    task_probe_rc=$?
fi
task_stage=probe_finished
printf 'Probe finished (exit %s); collecting capture and analysis results.\n' "$task_probe_rc"
exit "$task_probe_rc"
