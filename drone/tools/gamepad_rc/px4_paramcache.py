#!/usr/bin/env python3
"""离线读 QGC 的参数缓存 —— 不用碰串口，QGC 开着也能看参数。

**为什么需要它**：QGC 打开 `/dev/ttyACM0` 是**独占**的（连只读 open 都报 EBUSY），
所以 QGC 一开着，本目录所有走 MAVLink 的脚本（`px4_params.py` / `px4_shell.py` /
`gamepad_rc.py`）全都连不上。但只要 QGC 连过一次飞控，它就会把**飞控的全部参数**
落盘成缓存文件，那份数据是可以直接读的：

    ~/.config/QGroundControl/QGroundControl/ParamCache/<sysid>_<compid>.v2

（`1_1.v2` = system 1 / component 1。文件在 QGC 每次连上/刷新参数时会重写，
所以 `stat -c %y` 的时间戳就是这份快照的时间。）

**文件格式**（2026-10-12 从 926 条的真实文件反推，整文件恰好读完可自证）：

    头部   uint32 BE  参数条数
    每条   uint32 BE  名字字节数 n（UTF-16 的字节数，= 2 × 字符数）
           名字       UTF-16 BE
           uint32 BE  字段 A（double 是 8，int32 是 5 —— 不是纯粹的长度，别当长度用）
           uint32 BE  类型码（38 = double，2 = int32）
           1 字节     固定 0
           值         大端：A >= 8 取 8 字节 double，否则 4 字节 int32

**用法**

    python3 px4_paramcache.py                      # 全量 dump
    python3 px4_paramcache.py --grep '^EKF2_'      # 正则筛
    python3 px4_paramcache.py EKF2_HGT_REF NAV_DLL_ACT   # 按名字查

⚠ 和 `px4_params.py` 的区别：这个是**离线快照**，时间点取决于 QGC 什么时候刷的参数，
  而且**看不到你刚用脚本改完还没让 QGC 重读的参数**。要权威值还是得直连串口。
"""

import argparse
import re
import struct
import sys
from pathlib import Path

CACHE = Path.home() / ".config/QGroundControl/QGroundControl/ParamCache"
TYPE_DOUBLE, TYPE_INT32 = 38, 2


def parse(path):
    """-> [(name, value, 类型码)]，解析不完整就直接抛，别静默给半份数据。"""
    d = Path(path).read_bytes()
    count = int.from_bytes(d[:4], "big")
    out, i = [], 4
    while i + 8 < len(d):
        n = int.from_bytes(d[i:i + 4], "big")
        if n < 2 or n > 80 or n % 2 or i + 4 + n + 9 > len(d):
            raise ValueError(f"偏移 {i} 处名字长度 {n} 不合理，格式对不上")
        name = d[i + 4:i + 4 + n].decode("utf-16-be")
        j = i + 4 + n
        a = int.from_bytes(d[j:j + 4], "big")
        t = int.from_bytes(d[j + 4:j + 8], "big")
        # 用【类型码】决定值宽度，别用 A —— A 对 double 是 8、对 int32 是 5，
        # 看着像是长度，但它跟宽度不是一一对应（有 A >= 8 却是 int32 的记录，
        # 拿 A 当判据会在偏移 928 处就崩）。
        if t == TYPE_DOUBLE:
            size = 8
        elif t == TYPE_INT32:
            size = 4
        else:
            raise ValueError(f"{name} 的类型码 {t} 没见过（A={a}）")
        raw = d[j + 9:j + 9 + size]
        if len(raw) < size:
            raise ValueError(f"{name} 的值被截断了")
        if size == 8:
            val = struct.unpack(">d", raw)[0]
        else:
            val = float(struct.unpack(">i", raw)[0])
        out.append((name, val, t))
        i = j + 9 + size
    if i != len(d):
        raise ValueError(f"只解析到 {i}，文件 {len(d)} 字节 —— 尾巴没对完")
    if len(out) != count:
        raise ValueError(f"解析出 {len(out)} 条，头部声明 {count} 条")
    return out


def fmt(v):
    return f"{v:g}" if v == int(v) else f"{v:.6g}"


def main():
    ap = argparse.ArgumentParser(description="离线读 QGC 的 PX4 参数缓存")
    ap.add_argument("names", nargs="*", help="按名字精确查（可多个）")
    ap.add_argument("--grep", metavar="RE", help="按正则筛名字")
    ap.add_argument("--file", default=None, help=f"默认 {CACHE}/1_1.v2")
    args = ap.parse_args()

    path = Path(args.file) if args.file else sorted(CACHE.glob("*.v2"))[-1]
    import datetime
    mt = datetime.datetime.fromtimestamp(path.stat().st_mtime)
    print(f"# {path}")
    print(f"# 快照时间 {mt:%Y-%m-%d %H:%M:%S}（QGC 上次刷参数的时刻）")

    recs = parse(path)
    print(f"# 共 {len(recs)} 条\n")

    if args.names:
        m = dict((n, v) for n, v, _ in recs)
        for n in args.names:
            print(f"{n:24} = {fmt(m[n]) if n in m else '（缓存里没有）'}")
        return 0

    pat = re.compile(args.grep) if args.grep else None
    for n, v, t in recs:
        if pat is None or pat.search(n):
            print(f"{n:24} = {fmt(v):>14}   {'double' if t == TYPE_DOUBLE else 'int32'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
