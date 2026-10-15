#!/usr/bin/env python3
"""台架验证：把 6 个模式档逐个扫一遍，看飞控的 HEARTBEAT.custom_mode 跟不跟。

为什么要单独一个脚本：手柄按键验证模式档得有人守着按 L1/R1，而且**同一根串口只能
被一个进程打开**，没法"跑 gamepad_rc.py 的同时再开 rc_watch.py 看"。这里直接合成
RC 覆盖帧发下去，一个进程里发完就看回读，全自动。

**安全边界（重要）**
  - 只发 RC_CHANNELS_OVERRIDE，不改任何参数、不发解锁命令。
  - 全程 ch6(ARM) 钉在 PWM_MIN、ch7(KILL) 钉在 PWM_MIN = 急停释放，
    所以飞控**从头到尾是 disarmed，电机不会转**。
  - 模式档在未解锁时也能切（ManualControl.cpp:354-394 evaluateModeSlot 没有
    armed 门），所以拿它做验证最安全、信息量也最大。
  - 结束前把档位拨回 slot1（Position）、offboard 关，给下一次留个干净状态。

    python3 bench_rc_sweep.py --link serial:/dev/ttyACM0
"""

import argparse
import sys
import time

import gamepad_rc as g
import px4_modes


def slot_pwm(i):
    """第 i 档（0-based）的 PWM —— 和 gamepad_rc.Mapper.channels 用的公式一致。"""
    return int(round(g.PWM_MIN + (g.PWM_MAX - g.PWM_MIN) * (i + 0.5) / g.MODE_SLOTS))


def build(slot, offboard=False):
    """合成一帧 8 路：摇杆居中、模式档=slot、arm=低、kill=低、offboard 可选。"""
    chans = [g.PWM_MID] * g.NUM_CH
    chans[g.CH_FLTMODE - 1] = slot_pwm(slot)
    chans[g.CH_ARM - 1] = g.PWM_MIN          # 钉住：不尝试解锁
    chans[g.CH_KILL - 1] = g.PWM_MIN         # 钉住：急停释放
    chans[g.CH_OFFB - 1] = g.PWM_MAX if offboard else g.PWM_MIN
    return chans


def hold(link, chans, slot, secs, rate=20.0):
    """按 rate 持续发 secs 秒，同时把飞控回读收干净。返回期间最后一次看到的 custom_mode。"""
    period = 1.0 / rate
    next_tx = time.time()
    end = next_tx + secs
    while time.time() < end:
        now = time.time()
        if now >= next_tx:
            link.send_rc(chans)
            link.drain(slot)
            next_tx += period
            if next_tx < now - 0.5:
                next_tx = now + period
        else:
            time.sleep(max(0.0, min(0.02, next_tx - now)))
    return link._last_mode


def main():
    ap = argparse.ArgumentParser(description="台架扫模式档")
    ap.add_argument("--link", default="serial:/dev/ttyACM0")
    ap.add_argument("--secs", type=float, default=1.6,
                    help="每档保持多久（默认 1.6s；太短飞控来不及切）")
    ap.add_argument("--kill", action="store_true",
                    help="额外测一次急停开关 ch7：拉高 2s 再松开。"
                         "预期看到 STATUSTEXT 'Kill-switch engaged' / 'disengaged'。"
                         "安全：急停只会让飞控上锁，不会转电机。")
    ap.add_argument("--only-kill", action="store_true",
                    help="跳过模式档扫描，只做急停测试（用来隔离变量）")
    args = ap.parse_args()

    link = g.Link(args.link)

    if args.only_kill:
        args.kill = True
        print("[bench] --only-kill：跳过模式档扫描。\n")

    results = []
    if not args.only_kill:
        print(f"\n[bench] 开始扫 {g.MODE_SLOTS} 个档，每档 {args.secs:.1f}s。"
              f"ch6/ch7 全程钉住 -> 不会解锁、不会转电机。\n")
    for i in () if args.only_kill else range(g.MODE_SLOTS):
        param = g.SLOT_PARAM[i]
        want = px4_modes.custom_mode_from_param(param)
        print(f"[bench] --- slot{i+1}  ch5={slot_pwm(i)}  "
              f"COM_FLTMODE{i+1}={param}  期望 {px4_modes.describe(want)} ---")
        got = hold(link, build(i), i, args.secs)
        ok = (got == want)
        results.append((i + 1, param, want, got, ok))
        print(f"[bench]     -> 实得 {px4_modes.describe(got) if got is not None else '没回读'}"
              f"   {'✔' if ok else '✘'}\n")

    # offboard 开关（ch8）单独试一下。实测（2026-10-10）disarmed 时【也会切过去】，
    # 不是被拒 —— commander 允许未解锁就进 OFFBOARD。
    if not args.only_kill:
        print("[bench] --- 附加 ch8 = OFFBOARD 开关 ON ---")
        hold(link, build(0, offboard=True), 0, args.secs)
        print()
        hold(link, build(0), 0, 1.0)          # 关掉，回到 slot1

    # 急停开关（ch7）—— 本项目【唯一】的手动兜底，单独确认。
    # Commander.cpp:1532-1550 ACTION_KILL 没有 armed 门，disarmed 时也照样发 STATUSTEXT
    # （landed -> INFO "Kill-switch engaged"），所以拉高一定能观测到。
    # 这一步同时是那个"解锁/急停对调"隐患的判决性测试：
    # 映射若还是旧的（RC_MAP_KILL_SW=6 -> 功能挂在 ch6），拉 ch7 会【毫无反应】。
    kill_result = None
    if args.kill:
        print("[bench] --- 急停开关 ch7：基线 -> 拉高 2s -> 松开 ---")
        hold(link, build(0), 0, 1.2)                  # 基线（ch7 低）
        mark = len(link._texts)
        k = build(0)
        k[g.CH_KILL - 1] = g.PWM_MAX
        link._n_rc = 0            # 让 drain() 把这一阶段的第一帧回读打出来（看 ch7 到没到）
        hold(link, k, 0, 2.0)                         # 拉高
        hold(link, build(0), 0, 1.5)                  # 松开
        new = link._texts[mark:]
        # 注意 "Kill-switch disengaged" 里也含 "engaged"，得排掉
        engaged = any("engaged" in s and "disengaged" not in s for s in new)
        disengaged = any("disengaged" in s for s in new)
        kill_result = (engaged, disengaged, new)
        print(f"[bench]     -> {'✔ ch7 确实映射到 KILL' if engaged else '✘ 拉高 ch7 没反应'}"
              f"；松开后{'有' if disengaged else '没有'} disengaged\n")

    # 回到安全初值
    print("[bench] 恢复初值：slot1 / offboard 关 / ch7 松开 / disarmed。")
    hold(link, build(0), 0, 1.2)

    # ---------------------------------------------------------------- 汇总
    print("\n=== 汇总 ===")
    print(f"{'档位':>5} {'COM_FLTMODE':>12} {'期望 custom_mode':>28} {'实得':>28}")
    n_ok = 0
    for idx, param, want, got, ok in results:
        n_ok += ok
        print(f"{idx:>5} {param:>12} {px4_modes.describe(want):>28} "
              f"{px4_modes.describe(got) if got is not None else '没回读':>28}"
              f"   {'✔' if ok else '✘'}")
    print(f"\n模式档 {n_ok}/{len(results)} 对得上。"
          + ("   全对 = 模式档这条链（override -> input_rc -> rc_update 开关解码 -> "
             "ManualControl.evaluateModeSlot -> commander 切模式 -> HEARTBEAT 回读）整条通。"
             if n_ok == len(results) else
             "   ✘ 的那些要么没切过去，要么被 commander 拒了（多为该模式未解锁不允许）。"))
    if kill_result is not None:
        engaged, disengaged, texts = kill_result
        print(f"\n急停开关 ch7: "
              f"{'✔ 出现 Kill-switch engaged' if engaged else '✘ 拉高后没有任何 Kill-switch 消息'}"
              f" / {'✔ 松开后 disengaged' if disengaged else '松开后没看到 disengaged'}")
        for s in texts:
            print(f"    STATUSTEXT: {s}")

    print(f"收到飞控回读 RC_CHANNELS {link._n_rc} 帧。")
    print("\n注：全程未尝试解锁，ch6/ch7 钉住。要验证解锁开关得另做（且需确认桨已拆）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
