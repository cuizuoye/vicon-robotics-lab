#!/usr/bin/env python3
"""只读观察飞控的 RC 输入 —— 上机验证 gamepad_rc.py 到底有没有生效。

**只发两种帧**：GCS 心跳（唤醒 USB 口用）和 SET_MESSAGE_INTERVAL（让飞控按
我们要的频率推 RC_CHANNELS）。**一个参数都不改，一个 override 都不发**，
所以可以随便跑，不影响飞控状态。

看三样东西：

  1. `RC_CHANNELS` (#65) —— 就是飞控侧 `input_rc` 的内容。这就是**判据**：
     如果这里出现的通道值等于 gamepad_rc.py 正在发的那组，说明整条
     MAVLink override 链路通了（rc_channels_override -> input_rc）。
     **一个字节都没收到过的话，这里会是全 0**（PX4 没发布 input_rc 时不推）。

  2. `HEARTBEAT.custom_mode` —— 模式档变没变。按 L1/R1 切档，这里应该跟着跳。
     ⚠ 它是个 uint32 的**打包结构**（px4_custom_mode: main_mode<<16 | sub_mode<<24），
     既不是 COM_FLTMODE* 参数值，也不是 nav_state 枚举值。换算见 px4_modes.py。
     本项目 COM_FLTMODE1..6 = 2/1/8/7/4/11，对应的期望回读值见下面汇总。

  3. `HEARTBEAT.base_mode` 的 bit7 (SAFETY_ARMED) —— 按 PLUS（解锁键）时应该变。
     地面推桨状态下大概率**解不了锁**（EKF 没有位置源、没 GPS），
     但 STATUSTEXT 会告诉你被哪条 prearm 检查拦下的 —— 那本身就证明
     解锁开关被正确解码了。

    python3 rc_watch.py --link serial:/dev/ttyACM0                # 看 15 秒
    python3 rc_watch.py --link serial:/dev/ttyACM0 --secs 60      # 边按手柄边看
    python3 rc_watch.py --link serial:/dev/ttyACM0 --raw          # 连没映射的值也打

⚠ 和 px4_params.py / gamepad_rc.py 一样：**同一个串口只能被一个进程打开**。
  跑这个的时候别同时跑那个（除非走 UDP，把 MAVLink 转出来再看）。

  **所以"边发 override 边看回读"不要用这个脚本** —— 用 gamepad_rc.py 自己：
  它的主循环里已经带 drain()，会把飞控回读的 RC_CHANNELS 和刚发出去的值逐路
  对比（`--echo` 每帧都打）。那才是同一个进程里做得到的验证。
  本脚本留给"没人在发 override"的时候用：看有没有别的 input_rc 发布者、
  看真实接收机在不在、看模式/解锁位的基线。
"""

import argparse
import sys
import time

from pymavlink import mavutil

import mavlink_link
import px4_modes      # HEARTBEAT.custom_mode 的解码（三层编号，见模块 docstring）

RC_CHANNELS_ID = mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS   # 65
SET_MESSAGE_INTERVAL = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL  # 511

# 本项目 gamepad_rc.py 的通道分配（见 docs/GAMEPAD_RC.md）
CH_NAME = {1: "roll", 2: "pitch", 3: "thr", 4: "yaw",
           5: "FLTMODE", 6: "ARM", 7: "KILL", 8: "OFFB"}

# 判定"这个通道有东西"的容差：1000/1500/2000 之外，或者偏离中点
MID, TOL = 1500, 30


def main():
    ap = argparse.ArgumentParser(description="只读观察飞控的 RC 输入")
    ap.add_argument("--link", required=True,
                    help="serial:/dev/ttyACM0 或 udpin:127.0.0.1:14551")
    ap.add_argument("--secs", type=float, default=15.0, help="看多久（默认 15s）")
    ap.add_argument("--hz", type=float, default=10.0,
                    help="要飞控推 RC_CHANNELS 的频率（默认 10Hz）")
    ap.add_argument("--raw", action="store_true",
                    help="把没映射的通道（9-18）也打出来")
    args = ap.parse_args()

    print(f"[watch] 连接 {args.link} （{mavlink_link.describe(args.link)}）…")
    mav = mavlink_link.open_link(args.link, source_system=250)
    hb = mavlink_link.wake_and_wait_heartbeat(mav, timeout=20)
    if hb is None:
        raise SystemExit("[watch] 20 秒没等到心跳。线插了没 / 口对不对 / 飞控上电没。")
    print(f"[watch] 心跳 OK  sysid={mav.target_system} compid={mav.target_component}")

    # 让飞控按我们要的频率推 RC_CHANNELS。
    # PX4 走 MAV_CMD_SET_MESSAGE_INTERVAL（1.14 支持），interval 单位是微秒。
    interval_us = int(1e6 / max(args.hz, 0.1))
    mav.mav.command_long_send(
        mav.target_system, mav.target_component, SET_MESSAGE_INTERVAL, 0,
        RC_CHANNELS_ID, interval_us, 0, 0, 0, 0, 0)
    print(f"[watch] 已请求 RC_CHANNELS 以 {args.hz:.0f}Hz 推送。开始观察 "
          f"{args.secs:.0f} 秒，现在按手柄。\n")

    seen_channels = False
    chan_min, chan_max = {}, {}
    mode_seen, armed_seen = {}, set()
    last_mode = None
    last_armed = None
    texts = []
    n_rc = 0
    t0 = last_print = time.time()
    next_hb = 0.0

    while time.time() - t0 < args.secs:
        # 边看边补心跳：飞控在收到主机字节前 USB 口是静的，而且 GCS 心跳
        # 也是 PX4 判定"有地面站"、愿意推流的依据。
        now = time.time()
        if now >= next_hb:
            try:
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            next_hb = now + 1.0

        msg = mav.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        t = msg.get_type()

        if t == "RC_CHANNELS":
            n_rc += 1
            vals = [getattr(msg, f"chan{i}_raw") for i in range(1, 19)]
            if any(v not in (0, 65535, 1500) for v in vals[:8]):
                seen_channels = True
            for i, v in enumerate(vals, 1):
                if v in (0, 65535):
                    continue
                chan_min[i] = min(chan_min.get(i, v), v)
                chan_max[i] = max(chan_max.get(i, v), v)
            if args.raw or seen_channels:
                print(f"  RC_CH rssi={msg.rssi:>4} ch={msg.chancount:>2} | "
                      + " ".join(f"c{i}={v:>4}" for i, v in enumerate(vals[:8], 1)))
            elif n_rc == 1:
                print(f"  RC_CH 收到第 1 帧，chancount={msg.chancount}，"
                      f"但前 8 路全是 0/1500 —— 说明飞控侧 input_rc 是空的。")

        elif t == "HEARTBEAT" and msg.get_srcSystem() == mav.target_system:
            cm, bm = msg.custom_mode, msg.base_mode
            if cm != last_mode:
                print(f"\n  >>> 模式变了: custom_mode={cm}  "
                      f"[{px4_modes.describe(cm)}]")
                mode_seen[cm] = mode_seen.get(cm, 0) + 1
                last_mode = cm
            armed = bool(bm & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed != last_armed:
                print(f"  >>> {'已解锁 ARMED' if armed else '已上锁 DISARMED'}")
                if armed:
                    armed_seen.add("armed")
                last_armed = armed

        elif t == "STATUSTEXT":
            sev = msg.severity
            txt = str(msg.text).rstrip("\x00").strip()
            if txt:
                texts.append(txt)
                tag = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR",
                       4: "WARN", 5: "NOTICE", 6: "INFO"}.get(sev, str(sev))
                print(f"  [{tag}] {txt}")

    # ------------------------------------------------------------ 汇总
    print(f"\n=== 汇总（{args.secs:.0f} 秒，收到 {n_rc} 帧 RC_CHANNELS）===")
    if seen_channels:
        print("  前 8 路观察到的范围：")
        for i in range(1, 9):
            lo, hi = chan_min.get(i), chan_max.get(i)
            if lo is None:
                print(f"    ch{i} {CH_NAME.get(i, ''):>8}: 一直 0 / 没出现")
                continue
            mark = ""
            if hi - lo < TOL:
                mark = "  （基本不动）"
            print(f"    ch{i} {CH_NAME.get(i, ''):>8}: {lo:>5} .. {hi:>5}{mark}")
        print("\n  判据：某路的值应等于 gamepad_rc.py 那一时刻发的值。"
              "对得上 = override 链路通。")
    else:
        print("  ⚠ 一次都没看到非 0 的 RC 通道值。这说明飞控侧 `input_rc` 是空的：")
        print("    a) 手柄脚本没在发（没跑 / 没连上飞控 / dry-run）；或")
        print("    b) 飞控压根没有在推 RC_CHANNELS —— 看看上面有没有收到过帧。")

    if mode_seen:
        print(f"\n  观察到的模式: "
              + ", ".join(f"{k}={px4_modes.decode(k)[0]}" for k in sorted(mode_seen)))
    else:
        print("\n  模式档一次都没变过（没按 L1/R1？）")
    print(f"  解锁状态: {'出现过 ARMED' if armed_seen else '始终未解锁'}")
    if texts:
        print(f"\n  飞控文字消息 {len(texts)} 条（解锁被拦时原因就在这里）：")
        for s in texts[-8:]:
            print(f"    - {s}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
