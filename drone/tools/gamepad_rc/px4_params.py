#!/usr/bin/env python3
"""用 MAVLink 读写 PX4 参数 —— 这机器上没装 QGC，所以参数得自己下发。

    # 1. 先看飞控现在是什么样（只读，不改任何东西）
    python3 px4_params.py --link serial:/dev/ttyACM0 --check

    # 2. 顺手把飞控当前的全部参数存档（改之前留个底）
    python3 px4_params.py --link serial:/dev/ttyACM0 --dump before.txt

    # 3. 下发参数表里不一致的项，并逐项读回确认
    python3 px4_params.py --link serial:/dev/ttyACM0 --apply

    # 4. 重启飞控让参数生效（可选，多数参数是热生效的）
    python3 px4_params.py --link serial:/dev/ttyACM0 --reboot

`--link` 写法：serial:/dev/ttyACM0 / serial:/dev/ttyUSB1:57600 / udpin:127.0.0.1:14551

参数表格式就是 PX4/QGC 那种：`NAME VALUE`，`#` 后面是注释。
默认读 ros2_ws/src/uav_bringup/config/px4_gamepad_rc_params.txt。

⚠ 两个踩过的坑：

 1. **飞控在收到主机 ≥3 字节前是完全静默的**（`SYS_USB_AUTO=1` 自动探测，
    见 mavlink_link.wake_and_wait_heartbeat）。被动等心跳会一直等到超时。

 2. **非 REAL32 的参数，PARAM_VALUE 里那个 float 字段装的是整数的位模式**，
    不是浮点值。直接 `float(msg.param_value)` 会读出 2.24e-44 这种非规格化数。
    必须按 param_type 重新解释位模式（见 decode_value / encode_value）。
"""

import argparse
import os
import struct
import sys
import time

from pymavlink import mavutil

import mavlink_link

DEFAULT_PARAMS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "ros2_ws", "src", "uav_bringup", "config",
    "px4_gamepad_rc_params.txt"))

REAL32 = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
REBOOT = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

# MAV_PARAM_TYPE: 1 UINT8 2 INT8 3 UINT16 4 INT16 5 UINT32 6 INT32
#                 7 UINT64 8 INT64 9 REAL32 10 REAL64
UNSIGNED = {1, 3, 5}
SIGNED = {2, 4, 6}

TYPE_NAME = {1: "UINT8", 2: "INT8", 3: "UINT16", 4: "INT16", 5: "UINT32",
             6: "INT32", 7: "UINT64", 8: "INT64", 9: "REAL32", 10: "REAL64"}


# ----------------------------------------------------------- 值的编解码
def decode_value(raw_float, ptype):
    """PARAM_VALUE 的 float 字段 -> 真实数值。

    MAVLink 规定：param_type 不是 REAL32 时，param_value 这个 float 字段里装的是
    那个整数的**位模式**。PX4 的 INT32 参数（RC_MAP_* 、COM_FLTMODE* 等全是）
    直接当浮点读会得到 1.4e-45 这一类非规格化数。
    """
    if ptype == REAL32 or ptype not in (UNSIGNED | SIGNED):
        return float(raw_float)
    b = struct.pack("<f", raw_float)
    if ptype in UNSIGNED:
        return float(struct.unpack("<I", b)[0])
    return float(struct.unpack("<i", b)[0])


def encode_value(value, ptype):
    """真实数值 -> (要填进 param_value 的 float, param_type)。"""
    if ptype == REAL32 or ptype not in (UNSIGNED | SIGNED):
        return float(value), REAL32
    fmt = "<I" if ptype in UNSIGNED else "<i"
    b = struct.pack(fmt, int(round(value)))
    return struct.unpack("<f", b)[0], ptype


def fmt(v):
    """整数就别显示成 2.0，看着别扭。"""
    return f"{v:g}"


# --------------------------------------------------------------- 参数表解析
def parse_params_file(path):
    """读 PX4 参数表 -> ({name: float}, {name: 行号})。行尾 # 注释会被剥掉。"""
    spec, lineno_of = {}, {}
    with open(path) as f:
        for n, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                print(f"[params] {path}:{n} 这行看不懂，跳过：{raw.strip()!r}",
                      file=sys.stderr)
                continue
            name, value = parts[0], parts[1]
            try:
                spec[name] = float(value)
            except ValueError:
                print(f"[params] {path}:{n} 值不是数字，跳过：{raw.strip()!r}",
                      file=sys.stderr)
                continue
            lineno_of[name] = n
    return spec, lineno_of


# --------------------------------------------------------------- MAVLink 侧
def connect(url, timeout=25.0):
    print(f"[mav] 连接 {url} （{mavlink_link.describe(url)}）…")
    mav = mavlink_link.open_link(url, source_system=250)
    hb = mavlink_link.wake_and_wait_heartbeat(mav, timeout=timeout)
    if hb is None:
        raise SystemExit(
            f"[mav] {timeout:.0f} 秒没等到心跳。检查：线插了没 / 口对不对 / "
            f"飞控上电没 / 这个口是不是被别的进程占着。")
    print(f"[mav] 心跳 OK  sysid={mav.target_system} compid={mav.target_component}  "
          f"type={hb.type} autopilot={hb.autopilot}")
    return mav


def fetch_all(mav, timeout=90.0):
    """拉全部参数 -> {name: (真实值, param_type)}。

    ⚠ 别用 `param_index + 1 >= param_count` 判断收完 —— PX4 会额外发一条
      `_HASH_CHECK`，它的 `param_index` 是 **65535**，那条一到就满足条件，
      于是只收到 1 个参数就退出了（踩过）。改用「按名字去重后的个数」比
      `param_count`。
    """
    print("[mav] 拉取飞控全部参数 …")
    mav.param_fetch_all()
    got, expected = {}, None
    t0 = last = time.time()
    while time.time() - t0 < timeout:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            if got and time.time() - last > 3.0:
                break                      # 3 秒没新参数，就当发完了
            continue
        last = time.time()
        ptype = int(msg.param_type)
        got[str(msg.param_id).rstrip("\x00")] = (
            decode_value(float(msg.param_value), ptype), ptype)
        if msg.param_count:
            expected = int(msg.param_count)
        if expected and len(got) >= expected:
            break
    print(f"[mav] 收到 {len(got)} 个参数"
          + (f"（飞控声明 {expected} 个）" if expected else ""))
    return got


def set_param(mav, name, value, ptype, timeout=3.0):
    """写一个参数，等飞控回 PARAM_VALUE 确认。ptype 用飞控报的类型。

    ⚠ pymavlink 的 `mavfile.param_set_send` 是个**包装**，签名只有
      `(parm_name, parm_value, parm_type=None)`，它自己会补 sysid/compid
      （见 mavutil.py）。千万别按原始消息那套 6 参数写
      `(sysid, compid, name, value, type)`，会报
      "takes from 3 to 4 positional arguments but 6 were given"（踩过）。
    """
    enc, t = encode_value(value, ptype)
    mav.param_set_send(name, enc, t)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if str(msg.param_id).rstrip("\x00") == name:
            return decode_value(float(msg.param_value), int(msg.param_type))
    return None


def do_reboot(mav):
    print("[mav] 发重启命令（MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN param1=1）…")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component, REBOOT, 0,
        1, 0, 0, 0, 0, 0, 0)
    print("[mav] 已发。飞控断开重连是正常的，等它起来再跑 --check 确认。")


# ------------------------------------------------------------------- 动作
def cmd_dump(mav, path):
    got = fetch_all(mav)
    with open(path, "w") as f:
        f.write("# 飞控参数存档  %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        f.write("# 格式: NAME VALUE TYPE\n")
        for name in sorted(got):
            v, t = got[name]
            f.write(f"{name:16} {fmt(v):>12}  {TYPE_NAME.get(t, t)}\n")
    print(f"[dump] 已写入 {path}（{len(got)} 个参数）")


def cmd_check(mav, spec, lineno_of, apply_it=False):
    got = fetch_all(mav)
    missing, mismatch, ok = [], [], []
    for name, want in spec.items():
        if name not in got:
            missing.append(name)
        elif abs(got[name][0] - want) > 1e-6:
            mismatch.append((name, want, got[name][0], got[name][1]))
        else:
            ok.append(name)

    print(f"\n=== 参数核对：一致 {len(ok)} / 不一致 {len(mismatch)} / "
          f"飞控没有 {len(missing)} ===")
    if mismatch:
        print(f"\n{'参数':18} {'表里要的':>10} {'飞控现在':>10}  类型      出处")
        for name, want, have, t in mismatch:
            print(f"{name:18} {fmt(want):>10} {fmt(have):>10}  "
                  f"{TYPE_NAME.get(t, str(t)):8}  参数表第 {lineno_of.get(name,'?')} 行")
    if missing:
        print("\n飞控上找不到这些参数（名字拼错？固件版本不对？）：")
        for name in missing:
            print(f"  {name}   参数表第 {lineno_of.get(name,'?')} 行")

    if not apply_it:
        if mismatch or missing:
            print("\n（只读检查。要下发就加 --apply）")
        else:
            print("\n参数全对，不用下发。")
        return 0 if not (mismatch or missing) else 2

    if not mismatch:
        print("\n没有需要下发的。")
        return 0 if not missing else 2

    print(f"\n[mav] 下发 {len(mismatch)} 个参数 …")
    bad = []
    for name, want, have, t in mismatch:
        back = set_param(mav, name, want, t)
        if back is not None and abs(back - want) <= 1e-6:
            print(f"  {name:18} {fmt(have):>10} -> {fmt(want):<10}  "
                  f"读回 {fmt(back)}  OK")
        else:
            print(f"  {name:18} {fmt(have):>10} -> {fmt(want):<10}  "
                  f"读回 {fmt(back) if back is not None else '没回应'}  !! 没成")
            bad.append(name)
    if bad:
        print(f"\n!! 这些没写成功：{', '.join(bad)}")
        return 1
    print("\n全部下发并读回确认。多数参数热生效；求稳可以再 --reboot。")
    return 0


def cmd_info(mav):
    """把 21 个待配参数在飞控上的原始值和类型都打出来（解码正确性的体检）。"""
    print("\n[mav] 请求 AUTOPILOT_VERSION …")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 4:
        msg = mav.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=0.5)
        if msg is not None:
            v = int(msg.flight_sw_version)
            print(f"       flight_sw_version = 0x{v:08X}  ->  "
                  f"{v >> 24}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF}  "
                  f"type={v & 0xFF}")
            print(f"       os_sw_version     = 0x{int(msg.os_sw_version):08X}")
            print(f"       board_version={msg.board_version} "
                  f"vendor_id={msg.vendor_id} product_id={msg.product_id}")
            break


def main():
    ap = argparse.ArgumentParser(description="MAVLink 读写 PX4 参数")
    ap.add_argument("--link", required=True,
                    help="serial:/dev/ttyACM0 或 udpin:127.0.0.1:14551")
    ap.add_argument("--file", default=DEFAULT_PARAMS, help="参数表路径")
    ap.add_argument("--check", action="store_true", help="只读核对（默认动作）")
    ap.add_argument("--apply", action="store_true",
                    help="把不一致的参数下发下去并读回确认")
    ap.add_argument("--dump", metavar="PATH",
                    help="把飞控当前全部参数存到文件（改之前留底）")
    ap.add_argument("--reboot", action="store_true", help="重启飞控")
    args = ap.parse_args()

    mav = connect(args.link)
    try:
        if args.dump:
            return cmd_dump(mav, args.dump)
        if args.reboot:
            do_reboot(mav)
            return 0
        cmd_info(mav)
        spec, lineno_of = parse_params_file(args.file)
        print(f"[params] {args.file} 里读到 {len(spec)} 个参数")
        return cmd_check(mav, spec, lineno_of, apply_it=args.apply)
    finally:
        mav.close()


if __name__ == "__main__":
    sys.exit(main())
