#!/usr/bin/env python3
"""pymavlink 连接字符串的统一解析 —— 只因为 pymavlink 自己那个坑。

pymavlink **不认** `serial:` 前缀，而且 mavutil.py:2089 有这么一条：

    if device.find(':') != -1 and not suffix in logsuffixes:
        return mavudp(device, ...)

也就是说**凡是带冒号的字符串一律当 UDP**。于是
`serial:/dev/ttyACM0:2000000` 会被丢进 mavudp，报
"UDP ports must be specified as host:port" —— 跟串口一点关系都没有。

这里补上 `serial:` 前缀，并允许直接写裸路径。两种写法都支持：

    serial:/dev/ttyACM0            -> 串口，用默认波特率
    serial:/dev/ttyUSB1:115200     -> 串口，指定波特率
    /dev/ttyACM0                   -> 同上（裸路径）
    udpin:127.0.0.1:14551          -> 原样交给 pymavlink
    udpout:192.168.1.5:14550
    tcp:127.0.0.1:5760

注意串口设备名里不含冒号，所以 `serial:` 后面按【最后一个】冒号切波特率是安全的。
"""

UDP_PREFIXES = ("udpin:", "udpout:", "udp:", "udpbcast:", "mcast:",
                "tcp:", "tcpin:", "ws:", "wss:", "wsserver:", "csv:")


def parse(url, default_baud=115200):
    """-> (device, baud)；baud 为 None 表示不是串口，别传 baud 给 pymavlink。"""
    if url.startswith("serial:"):
        rest = url[len("serial:"):]
        if ":" in rest:
            dev, _, b = rest.rpartition(":")
            if b.isdigit():
                return dev, int(b)
        return rest, default_baud
    if url.startswith(UDP_PREFIXES):
        return url, None
    return url, default_baud            # 裸路径 = 串口


def open_link(url, default_baud=115200, source_system=250):
    """按 url 建连接。串口会带上 baud；UDP/TCP 原样传。"""
    from pymavlink import mavutil
    dev, baud = parse(url, default_baud)
    if baud is None:
        return mavutil.mavlink_connection(dev, source_system=source_system)
    return mavutil.mavlink_connection(dev, baud=baud, source_system=source_system)


class KeepAlive:
    """会话期间按 1 Hz 持续补 GCS 心跳。**不是可选项，漏了会静默。**

    实测（2026-10-10，MicoAir743v2 / PX4 v1.14.3）：只在连接阶段补心跳、
    之后不补（`wake_and_wait_heartbeat` 就是补满 7 秒就停），链路会在几秒后
    被飞控从"地面站"降级 —— 之后 **STATUSTEXT 一个都不再下发**。

    表现极具迷惑性，排查时被坑了很久：
      - RC 覆盖照常生效、RC_CHANNELS 回读照常回来、模式档照常切、
        急停也照常执行（`actuator_armed.manual_lockdown` 确实在翻转）；
      - 但飞控一句解释都不说了。
    于是同一个急停脉冲，"刚连上就打"有反应、"跑过十几秒再打"就没反应 ——
    看日志完全像急停功能坏了，实际是消息不再下发。本项目没有遥控器，
    预解锁失败原因 / 解锁被拒 / 急停消息全靠 STATUSTEXT，静默就等于瞎。

    `tick()` 自节流，随便在循环里高频调用。
    """

    def __init__(self, mav, period=1.0):
        import time as _time
        from pymavlink import mavutil
        self._mav = mav
        self._time = _time
        self._mavutil = mavutil
        self._period = period
        self._next = 0.0
        self.n_sent = 0

    def tick(self):
        now = self._time.time()
        if now < self._next:
            return
        self._next = now + self._period
        try:
            self._mav.mav.heartbeat_send(self._mavutil.mavlink.MAV_TYPE_GCS,
                                         self._mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                         0, 0, 0)
            self.n_sent += 1
        except Exception:
            pass                          # 网络口还没对端，发不出去很正常


def wake_and_wait_heartbeat(mav, timeout=20.0, send=True,
                            send_interval=0.5, max_sends=14):
    """等心跳。**串口上必须主动说话**，不能干等。

    实测（2026-10-10，这块 MicoAir743v2）：飞控的 `SYS_USB_AUTO = 1`（自动探测）。
    cdcacm_autostart.cpp:237-242 的逻辑是 —— 只有当主机先发来 ≥3 字节，
    它才会决定在 USB 口上跑 MAVLink 还是 NSH。在那之前**端口完全静默**，
    一个字节都不吐。被动 `wait_heartbeat()` 会一直等到超时，看起来像"飞控没反应"，
    其实是它在等你先开口。

    所以这里边等边发 GCS 心跳（顺带也是地面站该做的事）。发不出去也不报错
    （UDP 监听口在收到对端之前没有目的地，发不了很正常）。返回心跳消息或 None。

    想彻底免掉这一步：连上后把 `SYS_USB_AUTO` 设成 2（MAVLink），重启即生效。

    ⚠ 这里只补到连上为止（`max_sends` 发满就停）。**连上之后必须换成
    `KeepAlive` 继续按 1 Hz 补**，否则几秒后 STATUSTEXT 就全没了，见 `KeepAlive`。
    """
    import time
    from pymavlink import mavutil
    t0 = time.time()
    next_send = 0.0
    n_sent = 0
    while True:
        now = time.time()
        if now - t0 >= timeout:
            return None
        if send and n_sent < max_sends and now >= next_send:
            try:
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                       0, 0, 0)
                n_sent += 1
            except Exception:
                pass                      # 网络口还没对端，正常
            next_send = now + send_interval
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.3)
        if msg is not None:
            mav.target_system = msg.get_srcSystem()
            mav.target_component = msg.get_srcComponent()
            return msg


def describe(url, default_baud=115200):
    dev, baud = parse(url, default_baud)
    return f"串口 {dev} @ {baud}" if baud is not None else f"网络 {dev}"
