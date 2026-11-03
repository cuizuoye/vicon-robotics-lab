#!/usr/bin/env bash
# Only one bounded USB capture. NO UDP commands, GUI, network/driver reset or flash.
set -euo pipefail
umask 077
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_parent="$task_root/docs/tenda_usb_capture_2026-10-27"
task_iface=wlx000000000000
task_mode="${1:---check}"
if [ "$#" -gt 1 ] || { [ "$task_mode" != --check ] && [ "$task_mode" != --capture ]; }; then
    printf '用法：%s --check 或 --capture\n' "$0" >&2; exit 2
fi
task_device=$(readlink -f "/sys/class/net/$task_iface/device")
task_usb=$(dirname -- "$task_device")
IFS= read -r task_mac < "/sys/class/net/$task_iface/address"
IFS= read -r task_vendor < "$task_usb/idVendor"
IFS= read -r task_product < "$task_usb/idProduct"
IFS= read -r task_bus < "$task_usb/busnum"
IFS= read -r task_address < "$task_usb/devnum"
if [ "$task_mac" != 02:00:00:00:00:00 ] || [ "$task_vendor:$task_product" != 0bda:c811 ]; then
    printf '设备身份不符，拒绝抓包。未加载模块或打开抓包接口。\n' >&2; exit 2
fi
[[ "$task_bus" =~ ^[0-9]+$ && "$task_address" =~ ^[0-9]+$ ]] || exit 2
task_bus=$((10#$task_bus)); task_address=$((10#$task_address))
/usr/bin/python3 "$task_root/tools/verify_tenda_usb_filter.py" --bus "$task_bus" --device "$task_address"
task_filter="link[11] = $task_address and link[12] = $((task_bus & 255)) and link[13] = $((task_bus >> 8))"
task_monitor="usbmon$task_bus"
if [ "$task_mode" = --check ]; then
    printf 'PASS：只读设备核对和离线过滤测试；目标%s，总线%s设备%s。未加载usbmon、未抓包。\n' "$task_iface" "$task_bus" "$task_address"
    exit 0
fi
if [ "$EUID" -ne 0 ] || [ "${SUDO_UID:-}" != 1000 ] || [ "${SUDO_USER:-}" != czy ]; then
    printf '请由czy用户使用sudo执行；不要从其他账号或root登录会话运行。\n' >&2; exit 2
fi
if [ -n "$(/usr/bin/ss -H -lunp 'sport = :46001')" ]; then
    printf '小车通信端口正被占用，请先正常关闭原操作台。不会抢占、停止或杀进程。\n' >&2; exit 2
fi
if [ ! -d "$task_parent" ] || [ -L "$task_parent" ] || [ "$(stat -c %u "$task_parent")" != 1000 ]; then
    printf '抓包输出目录未准备好，停止。\n' >&2; exit 2
fi
# This is the ONLY system instrumentation change. Does not unload/reload the
# Tenda driver, disconnect WiFi, scan, change power policy, or reset USB.
/usr/sbin/modprobe usbmon
task_folder=$(mktemp -d "$task_parent/capture_XXXXXXXX")
/usr/bin/chown 1000:1000 -- "$task_folder"
/usr/bin/install -m 600 -o 1000 -g 1000 /dev/null "$task_folder/capture.log"
task_before=$(/usr/sbin/ip -4 route show default)
set +e
{
    printf 'CAPTURE_DIR=%s\n' "$task_folder"
    printf 'START_UNIX_S=%s\nMAX_DURATION_S=120\nUSB_DEVICE=%s:%s\nFILTER=%s\n' "$(date +%s)" "$task_bus" "$task_address" "$task_filter"
    printf '仅抓Tenda USB；不会启动小车。看到listening后立即回复“抓包已开始”，不要等待本窗口结束。\n'
    /usr/bin/timeout --signal=INT --kill-after=5s 120s \
        /usr/bin/tcpdump -i "$task_monitor" -y USB_LINUX_MMAPPED -p -nn -s 0 -U -Z czy \
        -w "$task_folder/tenda_usb.pcap" "$task_filter"
    task_status=$?
    printf 'CAPTURE_EXIT_CODE=%s\nEND_UNIX_S=%s\n' "$task_status" "$(date +%s)"
    exit "$task_status"
} 2>&1 | /usr/bin/tee "$task_folder/capture.log"
task_result=${PIPESTATUS[0]}
set -e
task_after=$(/usr/sbin/ip -4 route show default)
if [ "$task_before" != "$task_after" ]; then
    printf '提醒：观察到默认路由变化；本脚本没有执行网络修改，不自动修复。\n' >&2
fi
printf '抓包已结束，文件：%s/tenda_usb.pcap\nusbmon模块保留加载，不主动卸载；未改网络和固件。\n' "$task_folder"
if [ "$task_result" -ne 0 ] && [ "$task_result" -ne 124 ] && [ "$task_result" -ne 130 ]; then exit "$task_result"; fi
