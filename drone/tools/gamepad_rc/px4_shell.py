#!/usr/bin/env python3
"""在飞控上跑 PX4 的 nsh shell（= QGC 那个 "MAVLink Console"）—— 看 MAVLink 层看不到的内部状态。

**为什么需要它**：MAVLink 上只看得到 HEARTBEAT / RC_CHANNELS 这类"对外"消息。而本次
要查的问题是"ch7 拉到 2000，飞控到底认没认"，涉及的全是内部量：

    listener input_rc -n 12                -> timestamp_last_signal 的更新频率 /
                                              input_source / channel_count
    listener manual_control_switches -n 40 -> rc_update 解出来的 kill_switch 到底变没变
    listener actuator_armed -n 40          -> Commander 的 manual_lockdown 是不是被 latch 住了

这些只有飞控内部知道，在外面猜是猜不出来的。

⚠ **`listener` 的语法是 `listener <主题> -n <条数>`，主题在前**。写成
`listener <主题> <条数>`（没带 `-n`）会**静默地只打一条缓存样本就退出** ——
看起来像"这个主题没在发布"，实际是你把它写成了别的意思。这个坑吃过一次。

**实现依据**（PX4 v1.14.3 `src/modules/mavlink/mavlink_receiver.cpp:1741-1771`
`handle_message_serial_control`）：

  - `device` 必须是 `SERIAL_CONTROL_DEV_SHELL`(**10**)，而且**不能带
    `SERIAL_CONTROL_FLAG_REPLY`** —— 带了直接 `return`，静默丢弃（这是最容易踩的坑）
  - 必须带 `SERIAL_CONTROL_FLAG_RESPOND`(**2**)，否则飞控以为没人用了会 `close_shell()`
  - `data` 固定 70 字节，`count` 才是有效长度

**用法**

    # 最简单：读几个参数，确认通道通了
    python3 px4_shell.py --link serial:/dev/ttyACM0 \\
        --cmd "param show RC_MAP_KILL_SW" --cmd "param show RC_KILLSWITCH_TH"

    # 看内部主题（注意 -n 的位置）
    python3 px4_shell.py --link serial:/dev/ttyACM0 --secs 20 \\
        --cmd "listener input_rc -n 12"

    # 关键诊断：一边把 ch7 拉高，一边看飞控内部到底收到什么
    python3 px4_shell.py --link serial:/dev/ttyACM0 --secs 25 --pulse-kill \\
        --cmd "listener manual_control_switches -n 40"

⚠ 和 px4_params.py / gamepad_rc.py 一样：**同一根串口只能被一个进程打开**。
  跑这个的时候别同时跑别的。`--pulse-kill` 就是为"必须在同一个进程里边发边看"准备的。

⚠ **连上之后必须持续补 GCS 心跳**：本文件里已经做了（主循环里那段 1 Hz
  `heartbeat_send`，等价于 `mavlink_link.KeepAlive`）。不补的话几秒后飞控就
  **不再下发任何 `STATUSTEXT`**，而 shell 回显照常 —— 看起来像"飞控什么都没报"，
  其实是链路被降级了。详见 `mavlink_link.KeepAlive`。
"""

import argparse
import re
import sys
import time

import mavlink_link
from pymavlink import mavutil

import gamepad_rc as g            # noqa: E402

DEV_SHELL = getattr(mavutil.mavlink, "SERIAL_CONTROL_DEV_SHELL", 10)
FLAG_RESPOND = getattr(mavutil.mavlink, "SERIAL_CONTROL_FLAG_RESPOND", 2)

DATA_LEN = 70                     # SERIAL_CONTROL.data 固定长度

ANSI = re.compile(r"\x1b\[[0-9;?]*[a-zA-Z]|\x1b[=>]|\x1b\][^\x07]*\x07")


def build_rc_timeline(pre_sweep_secs=0.0):
    """给 --pulse-kill 用的 RC 时间线：[(时长s, ch5值, ch7是否拉高), ...]，从 t=0 起依次执行。

    `pre_sweep_secs > 0` 时先按 6 个档各停留这么久扫一遍（复现"先扫档再打急停"），
    然后连打两次急停脉冲 —— 连打两次是为了分辨"飞控根本没动作"和
    "动作了但 STATUSTEXT 没显示出来"。
    """
    steps = []
    for i in range(g.MODE_SLOTS):
        pwm = int(round(g.PWM_MIN + (g.PWM_MAX - g.PWM_MIN) * (i + 0.5) / g.MODE_SLOTS))
        steps.append((pre_sweep_secs, pwm, False))

    steps += [(2.0, 1083, False),      # 基线
              (5.0, 1083, True),       # 第 1 次拉高
              (3.0, 1083, False),      # 松开
              (5.0, 1083, True),       # 第 2 次拉高
              (3.0, 1083, False)]      # 松开
    return steps


def frame_at(steps, rel):
    """按经过时间 rel(秒) 在时间线里找到当前该发的 (ch5, ch7高?)。"""
    t = 0.0
    for secs, pwm, kill in steps:
        if rel < t + secs:
            return pwm, kill
        t += secs
    return steps[-1][1], steps[-1][2]


def send_shell(mav, payload: bytes):
    """把命令字节按 70 字节一段塞进 SERIAL_CONTROL 发出去（不足补 0）。

    ⚠ 这里**必须用关键字参数**。pymavlink 生成出来的签名是
        serial_control_send(device, flags, timeout, baudrate, count, data)
    字段顺序跟 MAVLink XML 里的声明顺序一致，**不是**直觉上的
    baudrate/timeout 打头。6 个参数全是 int/bytes，位置传错不会报任何错，
    只会静默发一条垃圾消息 —— 本文件第一版就这么错的（device 收到 0，
    PX4 在 mavlink_receiver.cpp:1755 一句 `device != SERIAL_CONTROL_DEV_SHELL`
    就直接 return，一个字都不回）。
    """
    if not payload:
        mav.mav.serial_control_send(device=DEV_SHELL, flags=FLAG_RESPOND, timeout=0,
                                    baudrate=0, count=0, data=b"\x00" * DATA_LEN)
        return

    for off in range(0, len(payload), DATA_LEN):
        chunk = payload[off:off + DATA_LEN]
        mav.mav.serial_control_send(device=DEV_SHELL, flags=FLAG_RESPOND, timeout=0,
                                    baudrate=0, count=len(chunk),
                                    data=chunk.ljust(DATA_LEN, b"\x00"))
        if off + DATA_LEN < len(payload):
            time.sleep(0.02)      # 别把 nsh 的输入缓冲灌爆


def main():
    ap = argparse.ArgumentParser(description="跑 PX4 nsh shell（MAVLink Console）")
    ap.add_argument("--link", default="serial:/dev/ttyACM0")
    ap.add_argument("--cmd", action="append", default=[],
                    help="要执行的 nsh 命令，可重复给多次（按顺序串成一行行发）")
    ap.add_argument("--secs", type=float, default=10.0, help="总共看多久")
    ap.add_argument("--raw", action="store_true", help="把收到的原始 SERIAL_CONTROL 字节打出来")
    ap.add_argument("--pulse-kill", action="store_true",
                    help="诊断用：期间把急停 ch7 连打两次（低2s->高5s->低3s->高5s->低3s），"
                         "同时照常把回读收干净。全程 ch6(ARM) 钉在最低，不会解锁。")
    ap.add_argument("--pre-sweep", type=float, default=0.0, metavar="秒",
                    help="配合 --pulse-kill：先按 6 个模式档各停留这么久扫一遍，再打急停。"
                         "用来复现/排除'先扫档把后面的急停吃掉'这个怀疑。0 = 不扫。")
    args = ap.parse_args()

    if not args.cmd and not args.pulse_kill:
        ap.error("至少给一个 --cmd，或者用 --pulse-kill")

    print(f"[shell] 连接 {args.link} （{mavlink_link.describe(args.link)}）…")
    mav = mavlink_link.open_link(args.link, source_system=250)
    hb = mavlink_link.wake_and_wait_heartbeat(mav, timeout=20)
    if hb is None:
        raise SystemExit("[shell] 20 秒没等到心跳。线插了没 / 口对不对 / 飞控上电没。")
    print(f"[shell] 心跳 OK  sysid={mav.target_system} compid={mav.target_component}")

    # 一条命令一行，最后补个换行让 nsh 立刻执行
    if args.cmd:
        payload = ("\n".join(args.cmd) + "\n").encode()
        send_shell(mav, payload)
        for c in args.cmd:
            print(f"[shell] >>> {c}")
    print()

    t0 = time.time()
    next_keepalive = t0 + 1.0
    next_hb = t0
    next_rc = t0
    if args.pulse_kill:
        # 连打两次，专门用来分辨"飞控没动作"和"动作了但 STATUSTEXT 没显示出来"
        if args.pre_sweep > 0:
            print(f"[shell] 先扫 6 个模式档，每档 {args.pre_sweep:.1f}s")
        print("[shell] 急停时间线：2s 低，5s 拉高(第1次)，3s 低，5s 拉高(第2次)，3s 低\n")

    timeline = build_rc_timeline(args.pre_sweep) if args.pulse_kill else []
    need = sum(s for s, _, _ in timeline)
    if args.pulse_kill and args.secs < need + 1:
        print(f"[shell] ⚠ 时间线要 {need:.0f}s，--secs 只有 {args.secs:.0f}s，后半段会被截断。\n")

    buf = b""
    while time.time() - t0 < args.secs:
        now = time.time()

        # --- 补 GCS 心跳：PX4 判定"有地面站"、愿意推流的依据
        if now >= next_hb:
            try:
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            next_hb = now + 1.0

        # --- 保活：不补的话飞控可能 close_shell()
        if now >= next_keepalive:
            try:
                send_shell(mav, b"")
            except Exception:
                pass
            next_keepalive = now + 1.0

        # --- 可选：一边发 RC 覆盖，一边看
        if args.pulse_kill and now >= next_rc:
            ch5, kill_high = frame_at(timeline, now - t0)
            chans = [g.PWM_MID] * g.NUM_CH
            chans[g.CH_FLTMODE - 1] = ch5
            chans[g.CH_ARM - 1] = g.PWM_MIN         # 钉住：不尝试解锁
            chans[g.CH_OFFB - 1] = g.PWM_MIN
            chans[g.CH_KILL - 1] = g.PWM_MAX if kill_high else g.PWM_MIN
            try:
                mav.mav.rc_channels_override_send(
                    mav.target_system, mav.target_component, *chans[:8])
            except Exception as e:
                print(f"[shell] !! 发 override 失败: {e}", flush=True)
            next_rc = now + 0.05                     # 20 Hz

        msg = mav.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue

        if msg.get_type() == "STATUSTEXT":
            txt = str(msg.text).rstrip("\x00").strip()
            if txt:
                print(f"\n[fc!] {txt}", flush=True)

        elif msg.get_type() == "SERIAL_CONTROL":
            chunk = bytes(msg.data[:msg.count])
            if args.raw:
                print(f"[raw] count={msg.count} {chunk!r}", flush=True)
            if chunk:
                buf += chunk
                text = ANSI.sub("", chunk.decode("utf-8", "replace"))
                sys.stdout.write(text)
                sys.stdout.flush()

    print("\n\n[shell] 结束。再发一次空写并去掉 RESPOND，让飞控把 shell 收掉。")
    try:
        mav.mav.serial_control_send(0, 0, DEV_SHELL, 0, 0, b"\x00" * DATA_LEN)
    except Exception:
        pass
    if not buf:
        print("[shell] ⚠ 一个字节都没收到。检查：飞控是不是把这个 MAVLink 实例的 shell 关了"
              "（MAV_x_MODE / MAV_x_CONFIG），或者 device 常量不对。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
