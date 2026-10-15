#!/usr/bin/env python3
"""按键码扫描器 —— 只为核对 gamepad_rc.py 里的 KEY_NAME 表。

    python3 btn_scan.py                 # 扫 20 秒
    python3 btn_scan.py --secs 60
    python3 btn_scan.py --device BEITONG

它把【每一个】按键事件的原始 evdev 码都打出来，**包括 KEY_NAME 表里没有的码**。
gamepad_rc.py 的 Gamepad.poll() 会把不认识的码直接丢掉（`if name is None: continue`），
所以用 --monitor 是查不出"这个键发的是什么码"的 —— 那个工具只告诉你"有没有生效"。

输出格式：
    0x134  BTN_WEST  [表里叫 Y]  按下
    0x132  BTN_C     【表里没有！】  按下

`[表里叫 …]` 是 gamepad_rc.py 当前的名字；`【表里没有！】` 表示这个码
既不在 KEY_NAME 里，也就永远不会触发任何动作。
"""

import argparse
import json
import os
import struct
import sys
import time

import gamepad_rc as g

# 内核 input-event-codes.h 里的 BTN_* 符号 + 它对应的【物理位置】+ 丝印。
# ⚠ 0x133/0x134 的内核小名（BTN_X / BTN_Y）跟丝印是反的 —— 那是按
#   【任天堂】丝印起的（任天堂上面那个印 X），而这只手柄丝印是 Xbox 那套。
#   位置（NORTH=上 / WEST=左）才是可靠的，实测印证过。
BTN_SYM = {
    0x130: ("BTN_SOUTH / BTN_A", "下 · 丝印 A"),
    0x131: ("BTN_EAST / BTN_B", "右 · 丝印 B"),
    0x132: ("BTN_C", "？(少见)"),
    0x133: ("BTN_NORTH / BTN_X", "上 · 丝印 Y"),
    0x134: ("BTN_WEST / BTN_Y", "左 · 丝印 X"),
    0x135: ("BTN_Z", "ZR 附近的 Z"),
    0x136: ("BTN_TL", "L1"),
    0x137: ("BTN_TR", "R1"),
    0x138: ("BTN_TL2", "L2"),
    0x139: ("BTN_TR2", "R2"),
    0x13A: ("BTN_SELECT", "Select(-)"),
    0x13B: ("BTN_START", "Start(+)"),
    0x13C: ("BTN_MODE", "Home"),
    0x13D: ("BTN_THUMBL", "L3 背键（非摇杆按下）"),
    0x13E: ("BTN_THUMBR", "R3 背键（非摇杆按下）"),
}


def main():
    ap = argparse.ArgumentParser(description="按键码扫描器")
    ap.add_argument("--secs", type=float, default=20.0, help="扫多久，默认 20 秒")
    ap.add_argument("--device", help="手柄名字子串，如 BEITONG")
    ap.add_argument("--calib", default=os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "gamepad_calib.json"))
    args = ap.parse_args()

    calib = dict(g.DEFAULT_CALIB)
    try:
        with open(args.calib) as f:
            calib.update(json.load(f))
    except Exception:
        pass

    print("[scan] 等手柄（顺便等 USB 模式震荡结束，可能要十几秒）…")
    print("[scan] 手柄要是睡着了，先随便按一下把它唤醒。")
    dev = g.wait_stable(calib, args.device)
    if dev is None:
        print("[scan] 没找到手柄。先跑 `python3 gamepad_rc.py --list` 看看。",
              file=sys.stderr)
        return 1

    fd = os.open(dev["path"], os.O_RDONLY | os.O_NONBLOCK)
    print(f"[scan] 开始扫 {args.secs:.0f} 秒，现在开始按键。\n")

    seen = {}          # code -> 按了几次
    t0 = time.time()
    last_beat = t0
    try:
        while True:
            now = time.time()
            if now - t0 >= args.secs:
                break
            try:
                data = os.read(fd, g.SZ * 64)
            except BlockingIOError:
                data = b""
            except OSError as e:
                print(f"[scan] 设备出错：{e}", file=sys.stderr)
                break
            for off in range(0, len(data) - g.SZ + 1, g.SZ):
                _, _, typ, code, val = struct.unpack(g.FMT, data[off:off + g.SZ])
                if typ != g.EV_KEY or val != 1:      # 只看按下沿
                    continue
                seen[code] = seen.get(code, 0) + 1
                sym, pos = BTN_SYM.get(code, ("?", "?"))
                kb = g.KEY_NAME.get(code)
                tag = f"[表里叫 {kb}]" if kb else "【表里没有！】"
                print(f"  0x{code:03X}  {sym:22} {pos:14} {tag:16} "
                      f"第 {seen[code]} 次")
            # 每 5 秒给个心跳，免得用户以为卡死了
            if now - last_beat >= 5.0:
                last_beat = now
                print(f"  …还剩 {args.secs - (now - t0):.0f} 秒", flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[scan] 手动中止。")
    finally:
        os.close(fd)

    print("\n=== 汇总（按了多少次）===")
    if not seen:
        print("  一个按键事件都没收到。")
        print("  → 手柄在睡觉？先按一下任意键唤醒，或者跑 `--secs 60` 给多点时间。")
    for code in sorted(seen):
        sym, pos = BTN_SYM.get(code, ("?", "?"))
        kb = g.KEY_NAME.get(code)
        tag = f"[表里叫 {kb}]" if kb else "【表里没有！】"
        print(f"  0x{code:03X}  {sym:22} {pos:14} {tag:16} x{seen[code]}")

    unknown = [c for c in seen if c not in g.KEY_NAME]
    if unknown:
        print("\n  ↑ 上面标【表里没有！】的码不会触发任何动作，"
              "要的话得先加进 KEY_NAME。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
