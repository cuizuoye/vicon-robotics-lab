#!/usr/bin/env python3
"""连续记录飞控的 `DISTANCE_SENSOR`(#132)，用来判断测距模块的读数是真是假。

**为什么不用 nsh 的 `listener distance_sensor`**：那个 topic 是 ~64 Hz，
一条记录 250 字节上下，就是 16 KB/s。而 nsh 控制台是走 MAVLink 的
`SERIAL_CONTROL`(#126) 隧道出来的，PX4 那边有节流，这个量级必然被丢消息，
看到的时间序列是假的。走 MAVLink 请求 `DISTANCE_SENSOR` 流反而干净。

判据：**把飞机抬起来，读数应该跟着变大**（orientation 25 = 朝下，
抬得越高离地越远）。读数纹丝不动 = 传感器坏的或没在测。

    python3 watch_distance.py --link serial:/dev/ttyACM0 --secs 80 --hz 10
"""

import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from mavlink_link import KeepAlive, open_link              # noqa: E402

MSID = 132                                                 # DISTANCE_SENSOR


def main():
    ap = argparse.ArgumentParser(description="连续录 distance_sensor 读数")
    ap.add_argument("--link", default="serial:/dev/ttyACM0")
    ap.add_argument("--secs", type=float, default=80.0)
    ap.add_argument("--hz", type=float, default=10.0, help="请求的流频率")
    ap.add_argument("--print-hz", type=float, default=4.0, help="打印抽稀到几 Hz")
    args = ap.parse_args()

    mav = open_link(args.link, source_system=255)
    ka = KeepAlive(mav)

    def pump(seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            ka.tick()
            m = mav.recv_match(blocking=True, timeout=0.2)
            if m is not None:
                yield m

    # 先握手，拿到 sysid/compid 才好指定目标
    tgt = None
    for m in pump(6.0):
        if m.get_type() == "HEARTBEAT" and m.get_srcSystem() != 255:
            tgt = (m.get_srcSystem(), m.get_srcComponent())
            break
    if tgt is None:
        print("没拿到飞控心跳")
        return 1
    print(f"飞控 sysid={tgt[0]} compid={tgt[1]}；请求 DISTANCE_SENSOR @ {args.hz:g} Hz\n")

    mav.mav.command_long_send(
        tgt[0], tgt[1], 511, 0,          # MAV_CMD_SET_MESSAGE_INTERVAL
        MSID, 1e6 / args.hz, 0, 0, 0, 0, 0)

    print(f"{'t(s)':>6} {'当前(m)':>9} {'最小(m)':>9} {'最大(m)':>9} {'id':>10} {'朝向':>5}")
    print("-" * 56)

    t0 = time.time()
    next_print = 0.0
    vals, ids = [], set()
    for m in pump(args.secs):
        if m.get_type() != "DISTANCE_SENSOR":
            continue
        d = m.current_distance / 100.0          # MAVLink 里是厘米
        vals.append(d)
        ids.add((m.id, m.orientation))
        el = time.time() - t0
        if el >= next_print:
            next_print = el + 1.0 / args.print_hz
            print(f"{el:6.1f} {d:9.3f} {m.min_distance/100:9.3f} "
                  f"{m.max_distance/100:9.3f} {m.id:10} {m.orientation:5}")

    print("-" * 56)
    if not vals:
        print("\n一条 DISTANCE_SENSOR 都没收到 —— 飞控没往这个口推这个流。")
        print("（不代表传感器坏，只是这个 MAVLink 口上没开这个流。）")
        return 1

    lo, hi = min(vals), max(vals)
    print(f"\n共 {len(vals)} 个样本；读数范围 {lo:.3f} ~ {hi:.3f} m，跨度 {hi - lo:.3f} m")
    print(f"来源 (id, orientation)：{sorted(ids)}")
    if hi - lo < 0.02:
        print("\n判读：**读数基本不动** —— 抬升/放下都没有反应。")
        print("      传感器可能是坏的、被挡住，或者根本不是测距模块。")
    else:
        print(f"\n判读：**读数跟着动，跨度 {hi - lo:.2f} m** —— 测距模块是在真实测量的。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
