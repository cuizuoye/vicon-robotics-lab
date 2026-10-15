#!/usr/bin/env python3
"""量 MAVLink 往返时延，用来判断一条链路是【无线】还是【有线】。

**为什么要这个**：光"收到了飞控数据"分不出链路性质。导线上是 1 ms 量级，
2.4G LoRa 数传上是几百毫秒。差两个数量级，量一次就定性了。

做法：发 MAVLink `PING`(#4)，飞控会回一个同 seq 的 PING，量两次之间的墙钟时间。
飞控的应答走它自己的 mavlink 栈，所以**往返里包含了空中那一段两次**。

    python3 rtt_probe.py --link serial:/dev/ttyUSB0:57600     # 待判定的口
    python3 rtt_probe.py --link serial:/dev/ttyACM0           # 对照组：纯 USB，肯定不是无线

⚠ TRS 的标准模式本身就有几十 ms 量级的转发延迟，再加上 LoRa 空口排队，
   平均值骗人，**看 min 和中位数**。丢几个包是正常的。
"""

import argparse
import statistics
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from mavlink_link import open_link                     # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="MAVLink 往返时延探测")
    ap.add_argument("--link", required=True, help="pymavlink 连接串或串口路径")
    ap.add_argument("--count", type=int, default=12, help="打几发")
    ap.add_argument("--interval", type=float, default=0.5, help="两发之间隔几秒")
    ap.add_argument("--timeout", type=float, default=2.0, help="每发等多久放弃")
    args = ap.parse_args()

    mav = open_link(args.link, source_system=255)
    mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=5.0)
    if hb is None:
        print("没等到飞控心跳，先确认链路通不通")
        return 1
    print(f"对端 sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}\n")

    rtts, pending = [], {}
    for seq in range(args.count):
        sent = time.time()
        mav.mav.ping_send(int(sent * 1e6), seq, hb.get_srcSystem(), hb.get_srcComponent())
        pending[seq] = sent
        deadline = sent + args.timeout
        while time.time() < deadline:
            m = mav.recv_match(type="PING", blocking=True, timeout=0.2)
            if m is None:
                continue
            s = m.seq
            if s in pending:
                rtts.append((time.time() - pending.pop(s)) * 1e3)
        if args.interval:
            time.sleep(args.interval)

    if not rtts:
        print(f"{args.count} 发全丢 —— 这个口上没人应答 PING")
        return 1

    ms = lambda x: f"{x:7.2f} ms"                          # noqa: E731
    print(f"回收 {len(rtts)}/{args.count} 发")
    print(f"  min    {ms(min(rtts))}")
    print(f"  中位数 {ms(statistics.median(rtts))}")
    print(f"  均值   {ms(statistics.mean(rtts))}")
    print(f"  max    {ms(max(rtts))}")
    print()
    med = statistics.median(rtts)
    if med < 10:
        print(f"判读：中位数 {med:.1f} ms —— **有线**（导线/直连的量级）")
    else:
        print(f"判读：中位数 {med:.1f} ms —— **无线**（射频空口转发，单程就有几十到上百 ms）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
