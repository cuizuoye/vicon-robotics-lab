#!/usr/bin/env python3
"""诊断：飞控 USB 口（/dev/ttyACM0）上为什么一个字节都没有。

背景（px4_firmware/src/drivers/cdcacm_autostart/cdcacm_autostart.cpp:214-242）：
`SYS_USB_AUTO` 决定 USB 口上跑什么 ——

    SYS_USB_AUTO = 2  MAVLink   连上就【立刻】起 mavlink，不等主机说话（默认值）
    SYS_USB_AUTO = 0  Disabled  什么都不起，口是【完全静默】的
    SYS_USB_AUTO = 1  Auto      要主机【先发 ≥3 字节】才决定跑 mavlink 还是 nsh

所以"打开端口后收不到任何字节"有三种完全不同的原因，被动地读是分不出来的。
这个脚本按顺序主动试：

  A. 发 MAVLink GCS 心跳 -> 若飞控回心跳，说明是 Auto(1)，且 mavlink 可用
  B. 发回车换行      -> 若回 "nsh> "，说明是 Auto(1)，而且拿到了飞控 shell
  C. 都不行          -> 大概率 SYS_USB_AUTO=0（或飞控没在跑），得换条路进去改参数

    python3 usb_probe.py
    python3 usb_probe.py --port /dev/ttyACM0 --secs 8
"""

import argparse
import sys
import time

import serial
from pymavlink import mavutil


def show(tag, data):
    if not data:
        return
    print(f"    {tag} 收到 {len(data)} 字节")
    print(f"      hex  : {data[:32].hex(' ')}")
    print(f"      repr : {data[:80]!r}")


def drain(port, secs, tag):
    """读干端口里的东西，返回收到的全部字节。"""
    buf, t0 = b"", time.time()
    while time.time() - t0 < secs:
        d = port.read(4096)
        if d:
            buf += d
    show(tag, buf)
    return buf


def main():
    ap = argparse.ArgumentParser(description="飞控 USB 口诊断")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="CDC-ACM 上波特率无意义，随便写")
    ap.add_argument("--secs", type=float, default=6.0, help="每步等多久")
    args = ap.parse_args()

    print(f"[probe] 端口 {args.port}  每步等 {args.secs:.0f} 秒\n")

    # ---------- A. MAVLink 心跳 ----------
    print("[A] 发 MAVLink GCS 心跳，看飞控回不回 …")
    got_a = b""
    try:
        m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                       source_system=250, source_component=190)
        hb = mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID
        t0, n_msg = time.time(), 0
        while time.time() - t0 < args.secs:
            m.mav.heartbeat_send(hb[0], hb[1], 0, 0, 0)
            msg = m.recv_match(blocking=True, timeout=0.4)
            if msg is not None:
                n_msg += 1
                if n_msg <= 5:
                    print(f"    <- {msg.get_type():24} {str(msg)[:90]}")
        print(f"    A 收到 {n_msg} 条 MAVLink 消息")
        got_a = b"x" * n_msg
        m.close()
    except Exception as e:
        print(f"    A 失败：{type(e).__name__}: {e}")

    time.sleep(1.0)

    # ---------- B. 回车换行 -> NSH shell ----------
    print("\n[B] 发回车换行，看能不能拿到飞控 shell …")
    got_b = b""
    try:
        p = serial.Serial(args.port, args.baud, timeout=0.3)
        p.dtr = True
        drain(p, 1.0, "[B] 发之前的残留")
        for _ in range(6):
            p.write(b"\r\n")
            time.sleep(0.4)
        got_b = drain(p, args.secs, "[B] 发 \\r\\n 之后")
        if b"nsh>" in got_b:
            print("    >>> 拿到 NSH shell 了！可以进去 `param show SYS_USB_AUTO`")
            p.write(b"param show SYS_USB_AUTO\r\n")
            time.sleep(1.0)
            drain(p, 2.0, "[B] param show 回复")
        p.close()
    except Exception as e:
        print(f"    B 失败：{type(e).__name__}: {e}")

    # ---------- 结论 ----------
    print("\n=== 结论 ===")
    if got_a:
        print("  USB 上跑的是 MAVLink（SYS_USB_AUTO=1 自动探测，被我们的心跳唤醒了）。")
        print("  -> 正常用 gamepad_rc.py / px4_params.py 就行；")
        print("     但注意每次连接可能都要先发一帧才会开口。")
    elif got_b:
        print("  USB 上跑的是 NSH 控制台（SYS_USB_AUTO=1 自动探测，被回车唤醒了）。")
        print("  -> 可以进 nsh 里 `param set SYS_USB_AUTO 2` + `param save`，")
        print("     重启后 USB 就直接是 MAVLink 了。")
    else:
        print("  主动说话也没任何回应。最可能是：")
        print("   1) SYS_USB_AUTO=0（USB 被关掉了）—— 这个口救不回来，")
        print("      得换一条链路进去改参数（TRS 接收机那路 / 重刷固件）；")
        print("   2) 飞控根本没在跑（卡在 boot / 挂了）；")
        print("   3) USB 线只有电源没有数据 —— 但设备能枚举出来，基本排除。")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
