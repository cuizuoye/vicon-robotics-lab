#!/usr/bin/env python3
"""TRS 无线链路打通探测 —— 在地面口主动发 GCS 心跳，看飞控是否从空中回话。

**链路长这样**（TRS = 遥数一体，数传 + 遥控共用一条 2.4G 空中链路）：

    PC --USB/CP2102--> TRS 高频头 ~~~2.4G~~~ TRS 接收机 --UART--> 飞控
                       (地面侧)                 (机载侧)      Telemetry Tx/RX/5V/GND
                                                             RC  SBUS/NC/5V/GND

高频头的工作模式决定**两侧**的串口波特率：远距离/标准 = 57600，高速 = 115200。
串口波特率和模式绑定，不用单独设；接收机对频后会自动匹配。

**为什么要主动发**：2026-10-12 那次检查是纯被动接收，57600 / 115200 各听 6 秒
都是 0 字节。但 0 字节**证明不了链路坏** —— PX4 只在收到 GCS 心跳之后才开始往
那条 MAVLink 实例推流；没有心跳，接收机空中就没东西可转，地面口当然是安静的。
所以必须按 1 Hz 主动发 GCS 心跳，飞控才会回心跳和遥测。

**这个脚本对每个候选波特率做三步**：

  1. 静默听 `--passive` 秒   —— 对面本来就在说话吗（有数据且能解出 MAVLink = 直接通）
  2. 按 1 Hz 发 GCS 心跳 `--secs` 秒 —— 飞控收到后才会回话
  3. 报原始字节数、MAVLink 消息数、HEARTBEAT 的来源 sysid/comp

**判读**：

| 现象 | 结论 |
| --- | --- |
| 收到 sysid=1 的 HEARTBEAT | 链路通了，而且就是这个波特率 |
| 有字节但一条 MAVLink 都解不出（乱码） | 波特率不对：听得见对面，但频率错 |
| 0 字节 | 对面没说话：没对频 / 接收机没供电 / 接线错 / 高频头没上电 |

用法：

    python3 trs_link_probe.py                       # 扫 57600 + 115200
    python3 trs_link_probe.py --bauds 115200 --secs 20
    python3 trs_link_probe.py --port /dev/ttyUSB0 --passive 5

⚠ 跑之前先关掉 QGroundControl：QGC 会独占 `/dev/ttyUSB0`（连只读 open 都 EBUSY）。
⚠ 而且 QGC 自己会持续发 GCS 心跳，做链路判断时它在不在场会污染结论。
"""

import argparse
import sys
import time

import serial

# 用 **v2 方言**，不是 `mavutil.mavlink`（那个默认 WIRE_PROTOCOL_VERSION='1.0'，
# 打包出来是 fe 帧头）。飞控 `MAV_PROTO_VER=0` 是"默认 v1，收到 v2 才切 v2"，
# 发 v1 它就一直回 v1。QGC / MAVROS 走的都是 v2，这里保持一致。
# v2 的解析器向下兼容，对面回 v1 帧照样能解。
from pymavlink.dialects.v20 import common as mavlink2


class Sink:
    """冒充 MAVLink 编码器的输出口，把打包好的字节截下来自己发。"""

    def __init__(self):
        self.buf = bytearray()

    def write(self, b):
        self.buf += b
        return len(b)

    def take(self):
        b = bytes(self.buf)
        self.buf.clear()
        return b


def hexdump(data, n=48):
    h = data[:n].hex(" ")
    return h + (" …" if len(data) > n else "")


def is_real(name):
    """pymavlink 在 robust_parsing 下会把**坏帧和没见过的 msgid 也变成消息**：

        BAD_DATA        CRC 没过的帧
        UNKNOWN_<id>    帧头长得对、但 msgid 不在方言里

    用纯乱码灌它，能吐出几十上百条这种占位消息。拿它们当"解出了 MAVLink"
    会把"波特率不对"误判成"链路通了"，所以必须滤掉。
    """
    return name != "BAD_DATA" and not name.startswith("UNKNOWN_")


def probe(port, baud, passive_s, active_s):
    """对一个波特率跑完整三步。-> dict"""
    res = {"baud": baud, "passive_bytes": 0, "active_bytes": 0,
           "msgs": {}, "heartbeats": [], "raw_head": b"", "err": None}

    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except Exception as e:
        res["err"] = f"{type(e).__name__}: {e}"
        return res

    try:
        sink = Sink()
        mav = mavlink2.MAVLink(sink, srcSystem=255, srcComponent=190)
        mav.robust_parsing = True          # 乱码直接跳过，别抛异常中断探测

        def drain(duration, send):
            t_end = time.time() + duration
            next_hb = 0.0
            total = 0
            while time.time() < t_end:
                if send and time.time() >= next_hb:
                    next_hb = time.time() + 1.0
                    sink.take()
                    mav.heartbeat_send(mavlink2.MAV_TYPE_GCS,
                                       mavlink2.MAV_AUTOPILOT_INVALID,
                                       0, 0, 0)
                    ser.write(sink.take())
                chunk = ser.read(4096)
                if not chunk:
                    continue
                total += len(chunk)
                if not res["raw_head"]:
                    res["raw_head"] = chunk
                for m in (mav.parse_buffer(chunk) or []):
                    name = m.get_type()
                    res["msgs"][name] = res["msgs"].get(name, 0) + 1
                    if name == "HEARTBEAT":
                        key = (m.get_srcSystem(), m.get_srcComponent())
                        res["heartbeats"].append(key)
            return total

        res["passive_bytes"] = drain(passive_s, send=False)
        res["active_bytes"] = drain(active_s, send=True)
        ser.close()
    except Exception as e:
        res["err"] = f"{type(e).__name__}: {e}"
        try:
            ser.close()
        except Exception:
            pass
    return res


def report(res):
    b = res["baud"]
    print(f"\n{'=' * 62}\n[波特率 {b}]")
    if res["err"]:
        print(f"  打开/通信失败：{res['err']}")
        return False

    print(f"  静默 {res['passive_bytes']} 字节 / 主动后共 {res['active_bytes']} 字节")
    if res["raw_head"]:
        print(f"  原始头部： {hexdump(res['raw_head'])}")

    if not res["msgs"]:
        if res["passive_bytes"] or res["active_bytes"]:
            print("  ✗ 有字节但解不出任何 MAVLink —— **波特率不对**（能听见对面）")
        else:
            print("  ✗ 一个字节都没有 —— 对面没在说话")
            print("    （没对频 / 接收机没供电 / 接线错 / 高频头没上电）")
        return False

    real = {k: v for k, v in res["msgs"].items() if is_real(k)}
    junk = sum(v for k, v in res["msgs"].items() if not is_real(k))
    if not real:
        print(f"  ✗ 一个有效帧都没有，只有 {junk} 条坏帧/未知 msgid —— **波特率不对**")
        print("    （能听见对面，但频率对不上，解出来全是乱码）")
        return False

    top = sorted(real.items(), key=lambda kv: -kv[1])[:8]
    print(f"  ✓ 解出 {sum(real.values())} 条 MAVLink：" +
          ", ".join(f"{k}×{v}" for k, v in top))
    if junk:
        print(f"    （另有 {junk} 条坏帧/未知 msgid，正常现象，忽略）")
    hbs = sorted(set(res["heartbeats"]))
    if hbs:
        print(f"  心跳来源 (sysid, compid)： {hbs}")
    # 认飞控：PX4 是 sysid=1 / compid=1
    fc = [h for h in hbs if h[0] == 1]
    if fc:
        print(f"  ★ **链路已打通**：收到飞控 (sysid=1) 的心跳 -> 波特率 {b} 正确")
        return True
    if hbs:
        print("  ⚠ 收到心跳但不是飞控（sysid≠1）—— 可能是别的设备或另一个地面站")
    return False


def main():
    ap = argparse.ArgumentParser(description="TRS 无线链路主动探测")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="地面侧串口，默认 /dev/ttyUSB0")
    ap.add_argument("--bauds", default="57600,115200",
                    help="候选波特率，逗号分隔。57600=标准/远距离，115200=高速")
    ap.add_argument("--passive", type=float, default=3.0, help="每个波特率先静默听几秒")
    ap.add_argument("--secs", type=float, default=12.0, help="每个波特率主动发几秒心跳")
    args = ap.parse_args()

    bauds = [int(x) for x in args.bauds.split(",") if x.strip()]
    print(f"TRS 链路探测  端口 {args.port}")
    print(f"候选波特率 {bauds}；每个先静默 {args.passive:.0f}s，再发 {args.secs:.0f}s 心跳")

    ok = None
    for b in bauds:
        r = probe(args.port, b, args.passive, args.secs)
        if report(r):
            ok = b
            break

    print("\n" + "=" * 62)
    if ok:
        print(f"结论：链路通了 —— 用 {ok} 连。")
        print(f"  地面侧：{args.port} @ {ok}")
        print(f"  飞控侧：接收机 Telemetry 接在哪路 UART，那路的 SER_TELx_BAUD 就得是 {ok}")
    else:
        print("结论：候选波特率都没通。下一步按这个顺序查：")
        print("  1) 接收机指示灯 —— 常亮=已对频已连接；慢闪=没对上；快闪=在对频态")
        print("  2) 接收机有没有 5V（它靠飞控的 5V 供电，飞控必须有电）")
        print("  3) 接线：接收机 Telemetry Tx->飞控 RX、Rx->飞控 TX，GND 共地")
        print("  4) 高频头对频时有没有真的上电（对频要高频头在发射）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
