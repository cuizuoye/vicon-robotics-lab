#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手柄 -> PX4 虚拟遥控器（RC_CHANNELS_OVERRIDE）

历史原型，不是已验收的真机飞行控制器。当前HardFault事件守卫禁止控制发送。
本原型HOME按住请求急停、松开撤销，不符合当前“松开不解除”的锁存要求；
摇杆回中对应油门通道1500，不能把回中误认为零油门。
以下配置说明为历史设计，不能与QGC的MANUAL_CONTROL参数混用。

把 USB 手柄（北通 BTP-KP20，Switch Pro 协议）映射成 8 通道虚拟遥控器，
以固定频率通过 MAVLink 发 RC_CHANNELS_OVERRIDE (#70) 给 PX4。

----------------------------------------------------------------------
为什么走 RC_CHANNELS_OVERRIDE 而不是 MANUAL_CONTROL (#69)
----------------------------------------------------------------------
1) 开关可用性。RC 覆盖经 rc_update 出去时 data_source = SOURCE_RC
   (rc_update.cpp:685)，会走完整的 RC_MAP_* 映射，于是解锁开关/模式开关/
   急停开关/Offboard 开关全都能用。MANUAL_CONTROL 出去是 SOURCE_MAVLINK_0
   (mavlink_receiver.cpp:2135)，且它的 buttons 字段对多旋翼无效——只有
   uuv_pos_control 会读 (uuv_pos_control.cpp:168)，等于没有任何物理急停手段。
   本机没有备份遥控器，急停开关是唯一的手动兜底，所以这一条就定了。

2) COM_RC_IN_MODE 必须配 0 而不是 1。ManualControlSelector.cpp:76-78 里
   mode 1(MAVLink only) 要求 isMavlink(source)，而 SOURCE_RC 不是
   (ManualControlSelector.cpp:122-135)。所以 mode 1 会把本节点发的东西全部丢弃。

代价：RC 覆盖这条路要求 rc_update 认为"已标定"，即 RC_CHAN_CNT > 0
(rc_update.cpp:152, 681)，且解锁前 rcCalibrationCheck 会遍历全部 18 个通道的
RCn_MIN/TRIM/MAX (rcCalibrationCheck.cpp:66-90)——默认值 1000/1500/2000 合法，
别去动它。

----------------------------------------------------------------------
用法
----------------------------------------------------------------------
  gamepad_rc.py --list                       # 列出手柄候选设备
  gamepad_rc.py --calibrate                  # 交互式校准，写 gamepad_calib.json
  gamepad_rc.py --monitor                    # 只打印，不发 MAVLink（先跑这个！）
  gamepad_rc.py --dry-run                    # 连上 PX4 但不发 RC 覆盖
  gamepad_rc.py --link udpin:127.0.0.1:14551 # 正常跑

  --link 支持网络写法 udpin:/udpout:/tcp:，以及串口写法
         serial:/dev/ttyACM0 或 serial:/dev/ttyUSB1:115200（裸路径也行）。
         解析在 mavlink_link.py —— pymavlink 自己【不认】serial: 前缀，
         而且会把任何带冒号的字符串一律当 UDP，所以不能直接透传给它。

先决条件：飞控侧参数见 ros2_ws/src/uav_bringup/config/px4_gamepad_rc_params.txt
设计说明见 docs/GAMEPAD_RC.md
"""

import argparse
import collections
import fcntl
import json
import os
import re
import select
import struct
import sys
import time

import mavlink_link       # 自己的模块；里面才 import pymavlink（保持惰性）
import px4_modes          # HEARTBEAT.custom_mode 的解码（三层编号，见模块 docstring）
from incident_guard import require_active_bench_clear

# ---------------------------------------------------------------- evdev 常量
FMT = "llHHi"                      # struct input_event, x86_64 = 24 字节
SZ = struct.calcsize(FMT)
EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03

ABS_X, ABS_Y, ABS_RX, ABS_RY = 0, 1, 3, 4
ABS_HAT0X, ABS_HAT0Y = 16, 17

ABS_NAME = {ABS_X: "ABS_X", ABS_Y: "ABS_Y",
            ABS_RX: "ABS_RX", ABS_RY: "ABS_RY",
            ABS_HAT0X: "ABS_HAT0X", ABS_HAT0Y: "ABS_HAT0Y"}

# EVIOCGABS(abs) = _IOR('E', 0x40 + abs, sizeof(struct input_absinfo))
#   _IOC_READ=2 -> (2<<30) | (24<<16) | (ord('E')<<8) | (0x40+abs)
# struct input_absinfo = 6 x __s32 = 24 字节: value,min,max,fuzz,flat,resolution
ABS_IOCTL = 0x80184540
ABSINFO_FMT = "6i"


def evdev_live_axis(fd, code):
    """向内核查该轴的【当前】值。

    必须要有这个：HID 设备只在数值变化时才发事件，
    所以"松手别碰"这种步骤一个事件都收不到，光靠读事件拿不到回中值。
    内核一直维护着 ABS 当前状态，ioctl 能直接取出来。
    """
    try:
        buf = fcntl.ioctl(fd, ABS_IOCTL + code,
                          b"\x00" * struct.calcsize(ABSINFO_FMT))
    except OSError:
        return None
    return struct.unpack(ABSINFO_FMT, buf)[0]

# 北通 BTP-KP20 在 NS 模式下的按键码。
#
# ⚠ 面键的 X / Y 跟内核的 BTN_X / BTN_Y 是【反的】。这不是笔误，实测过
#   （2026-10-10，用 btn_scan.py 逐个按出来的）：
#
#     物理位置   手柄丝印   实际发出的码
#     下方       A          0x130  BTN_SOUTH
#     右方       B          0x131  BTN_EAST
#     上方       Y          0x133  BTN_NORTH   ← 内核管它叫 BTN_X
#     左方       X          0x134  BTN_WEST    ← 内核管它叫 BTN_Y
#
#   位置本身没错（上=NORTH、左=WEST 都对），错的是"字母"：hid_nintendo 是按
#   【任天堂的丝印】分配的 —— 任天堂手柄上面那个印 X、左边那个印 Y；
#   而这只手柄的丝印是 Xbox 那一套（上 Y / 下 A / 左 X / 右 B）。
#   两边一错位，物理 Y 就拿到了内核名叫 BTN_X 的码。
#
#   所以下面一律按【手柄丝印】写，也就是按你手上看到的字写。
#   千万别照着内核名字（BTN_X/BTN_Y）去理解这两个码。
KEY_NAME = {
    0x130: "A", 0x131: "B", 0x133: "Y", 0x134: "X",
    0x136: "L1", 0x137: "R1", 0x138: "L2", 0x139: "R2",
    0x13A: "MINUS", 0x13B: "PLUS", 0x13C: "HOME",
    # 这只手柄的 L3 / R3 是【背键】，不是摇杆按下（实测 L3 = 0x13D）。
    0x13D: "L3", 0x13E: "R3",
}

# ---------------------------------------------------------------- 默认配置
# 按手柄在 NS 模式下的实测数据给的初值；强烈建议先跑 --calibrate 覆盖。
# min/center/max 是原始 evdev 值，映射到 [-1, 1]。
DEFAULT_CALIB = {
    "device": {"name": "BEITONG", "vid": "057e", "pid": "2009"},
    "axes": {
        # ！！这台手柄的回中值【不是 0】！！
        # NS 模式下 hid_nintendo 报出来的静止值是 ±16379，正好是它声明的
        # min/max(±32767) 的一半。ABS_X 停 -16379、ABS_Y 停 +16379，符号相反
        # 数值对称 —— 这是驱动对这个兼容手柄的中心校准错了，不是杆真的偏了。
        # 所以 center 必须用实测值。invert 一律 False：方向已经由
        # min/center/max 的先后关系编码了（见 axis_to_unit）。
        # 下面是 2026-10-10 实测的粗略值，跑一次 --calibrate 会覆盖。
        "ABS_X":  {"min": -32542, "center": -16379, "max": -120, "invert": False},
        "ABS_Y":  {"min": 120, "center": 16379, "max": 32543, "invert": False},
        "ABS_RX": {"min": -32510, "center": -16379, "max": -120, "invert": False},
        "ABS_RY": {"min": 120, "center": 16379, "max": 32638, "invert": False},
    },
    "stick": {"deadband": 0.03, "expo": 0.25},
    # 功能 -> 轴。改这里就能换手（比如换成美国手）
    "map": {"roll": "ABS_RX", "pitch": "ABS_RY", "throttle": "ABS_Y", "yaw": "ABS_X"},
    # 功能 -> 按键
    "buttons": {
        "mode_next": "L1", "mode_prev": "R1",
        "arm_toggle": "PLUS", "kill_hold": "HOME", "offboard_toggle": "Y",
        "goto_position": "A", "goto_land": "B",
    },
}

# ---------------------------------------------------------------- 通道分配
# ch1..ch8 -> RC_MAP_ROLL/PITCH/THROTTLE/YAW/FLTMODE/ARM_SW/KILL_SW/OFFB_SW
CH_ROLL, CH_PITCH, CH_THR, CH_YAW = 1, 2, 3, 4
CH_FLTMODE, CH_ARM, CH_KILL, CH_OFFB = 5, 6, 7, 8
NUM_CH = 8

PWM_MIN, PWM_MID, PWM_MAX = 1000, 1500, 2000
MODE_SLOTS = 6                     # manual_control_switches_s::MODE_SLOT_NUM

# 档位顺序。要和 COM_FLTMODE1..6 一致（见 config/px4_gamepad_rc_params.txt）。
# 注意 COM_FLTMODE* 的取值不是 nav_state 枚举值：
#   Offboard=7, Land=11 (ManualControl.cpp:576-595 navStateFromParam)
#   1=Position(2) 2=Altitude(1) 3=Stabilized(8) 4=Offboard(7) 5=Loiter(4) 6=Land(11)
DEFAULT_SLOT = 0                   # 开机停在 slot1 = Position

# 本项目 COM_FLTMODE1..6 里填的值，要和 config/px4_gamepad_rc_params.txt 第 3 节一致。
# ⚠ 这是【参数值】那一层。飞控回读的 HEARTBEAT.custom_mode 是另一套编号（px4_custom_mode
#   的打包结构），换算全在 px4_modes.py 里 —— 别在这里再抄一份表，抄岔过一次。
SLOT_PARAM = [2, 1, 8, 7, 4, 11]

# 手柄模式震荡：上电后会在 XInput / NS 两种 USB 模式间重枚举约 10 秒，
# 必须等它稳定再绑定，否则拿到的是马上要消失的那个节点。
STABLE_SEC = 4.0


# ===================================================================== 设备发现
def _sysfs_attr(path, *parts):
    try:
        with open(os.path.join(path, *parts)) as f:
            return f.read().strip()
    except OSError:
        return None


def list_candidates():
    """扫描所有 event 设备，返回 [{path,name,vid,pid,has_abs,has_btn}]。

    刻意同时读 sysfs 和 /proc/bus/input/devices：hid-nintendo 会把设备注册成
    两个 input 设备（本体 + IMU），我们用按键能力位把它们区分开。
    """
    out = []
    for ev in sorted(os.listdir("/sys/class/input")):
        if not re.fullmatch(r"event\d+", ev):
            continue
        dev = f"/sys/class/input/{ev}/device"
        name = _sysfs_attr(dev, "name")
        if not name:
            continue
        vid = _sysfs_attr(dev, "id", "vendor")
        pid = _sysfs_attr(dev, "id", "product")
        out.append({
            "path": f"/dev/input/{ev}", "event": ev, "name": name,
            "vid": (vid or "").lower(), "pid": (pid or "").lower(),
            "has_abs": os.path.exists(os.path.join(dev, "capabilities", "abs")),
            "has_btn": _has_gamepad_button(ev),
        })
    return out


def _has_gamepad_button(event):
    """该 event 设备是否声明了手柄按键（BTN_SOUTH/EAST/THUMBL）。

    用能力位区分"手柄本体"和它顺带暴露的 IMU 子设备——后者没有按键。
    """
    try:
        with open(f"/sys/class/input/{event}/device/capabilities/key") as f:
            words = f.read().split()
    except OSError:
        return False
    if not words:
        return False
    # sysfs 的位图是高位字在前
    for bit in (0x130, 0x131, 0x13D):
        idx = len(words) - 1 - bit // 64
        if 0 <= idx < len(words):
            try:
                if (int(words[idx], 16) >> (bit % 64)) & 1:
                    return True
            except ValueError:
                pass
    return False


def find_device(calib, pattern=None, verbose=True):
    """按名字 / VID:PID 找手柄。绝不按 eventN 写死。"""
    want = calib.get("device", {})
    cands = [c for c in list_candidates() if c["has_abs"] and c["has_btn"]]
    if pattern:
        cands = [c for c in cands if pattern.lower() in c["name"].lower()]
    if want.get("vid") and want.get("pid"):
        exact = [c for c in cands
                 if c["vid"] == want["vid"].lower() and c["pid"] == want["pid"].lower()]
        if exact:
            return exact[0]
    if want.get("name"):
        named = [c for c in cands if want["name"].lower() in c["name"].lower()]
        if named:
            return named[0]
    if verbose and not cands:
        print("[dev] 没找到手柄。当前所有输入设备：")
        for c in list_candidates():
            tag = []
            if c["has_abs"]:
                tag.append("有轴")
            if c["has_btn"]:
                tag.append("有手柄键")
            print(f"      {c['name']:40} {c['path']:20} {'/'.join(tag)}")
        print("[dev] 手柄没插 / 在重枚举 / 睡着了？")
    return cands[0] if cands else None


def wait_stable(calib, pattern=None):
    """等 USB 模式震荡结束：同一个设备路径连续 STABLE_SEC 秒不变才算稳。"""
    t0 = time.time()
    last = None
    stable_since = None
    announced = False
    while True:
        cur = find_device(calib, pattern, verbose=False)
        cur_path = cur["path"] if cur else None
        now = time.time()
        if cur_path != last:
            if cur_path and not announced:
                print(f"[dev] 看到 {cur['name']} @ {cur['path']}，等 USB 模式稳定…")
                announced = True
            last = cur_path
            stable_since = now
        elif cur_path and stable_since and (now - stable_since) >= STABLE_SEC:
            print(f"[dev] 稳定：{cur['name']} @ {cur['path']} "
                  f"(VID:PID {cur['vid']}:{cur['pid']})")
            return cur
        if now - t0 > 60:
            print("[dev] 等 60 秒还没稳定，放弃。", file=sys.stderr)
            return None
        time.sleep(0.25)


# ===================================================================== 手柄状态
class Gamepad:
    """打开 evdev 设备，维护轴/键的当前状态。"""

    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = {}          # abs_code -> raw value
        self.raw_buttons = set()
        self.key_edges = []     # 本次 poll 内按下的按键名
        # 开机就把各轴填成当前真实位置，而不是等第一个事件。
        # 否则节点第一帧只能拿"0=假设回中"，稳杆在手的时候就是错的。
        for code, val in self.read_live_axes().items():
            self.axes.setdefault(code, val)

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def read_live_axes(self):
        """读四个摇杆轴的当前值（ioctl，不依赖事件）。"""
        out = {}
        for code in CODE_OF.values():
            v = evdev_live_axis(self.fd, code)
            if v is not None:
                out[code] = v
        return out

    def poll(self, timeout):
        """读事件，返回 False 表示设备掉了。"""
        self.key_edges = []
        try:
            r, _, _ = select.select([self.fd], [], [], timeout)
        except OSError:
            return False
        if not r:
            return True
        try:
            data = os.read(self.fd, SZ * 64)
        except BlockingIOError:
            return True          # 只是这一轮没数据，不是掉线
        except OSError:
            return False         # ENODEV / EIO -> 设备真的掉了
        if not data:
            return False
        for off in range(0, len(data) - SZ + 1, SZ):
            _, _, typ, code, val = struct.unpack(FMT, data[off:off + SZ])
            if typ == EV_ABS:
                self.axes[code] = val
            elif typ == EV_KEY:
                name = KEY_NAME.get(code)
                if name is None:
                    continue
                if val == 1:
                    self.raw_buttons.add(code)
                    self.key_edges.append(name)     # 只在按下沿记一次
                elif val == 0:
                    self.raw_buttons.discard(code)
        return True

    def held(self, name):
        for code, n in KEY_NAME.items():
            if n == name:
                return code in self.raw_buttons
        return False


# ===================================================================== 映射
def axis_to_unit(raw, cfg):
    """原始值 -> [-1, 1]，按 min/center/max 分段线性。"""
    lo, mid, hi = cfg["min"], cfg["center"], cfg["max"]
    if hi == lo:
        return 0.0
    if raw >= mid:
        v = (raw - mid) / (hi - mid) if hi != mid else 0.0
    else:
        v = (raw - mid) / (mid - lo) if mid != lo else 0.0
    v = max(-1.0, min(1.0, v))
    return -v if cfg.get("invert") else v


def shape(v, deadband, expo):
    """死区 + 指数曲线。死区外重新归一化到 [0,1]，避免跳变。"""
    a = abs(v)
    if a <= deadband:
        return 0.0
    a = (a - deadband) / (1.0 - deadband)
    a = (1.0 - expo) * a + expo * a * a * a
    return a if v > 0 else -a


def unit_to_pwm(u):
    u = max(-1.0, min(1.0, u))
    return int(round(PWM_MID + u * 500.0))


class Mapper:
    """手柄状态 -> 8 通道 PWM。开关状态是节点自己维护的有状态量。"""

    def __init__(self, calib):
        self.calib = calib
        self.slot = DEFAULT_SLOT
        self.armed = False          # 开关位置，不是飞控真实状态
        self.kill = False
        self.offboard = False
        self._prev = {}

    def edge(self, pad, name):
        """只在按下沿触发一次（避免按键卡住导致反复翻转）。"""
        now = pad.held(name)
        was = self._prev.get(name, False)
        self._prev[name] = now
        return now and not was

    def update_switches(self, pad):
        b = self.calib["buttons"]
        if self.edge(pad, b.get("mode_next")):
            self.slot = (self.slot + 1) % MODE_SLOTS
            print(f"[rc] 模式档 -> slot {self.slot + 1}")
        if self.edge(pad, b.get("mode_prev")):
            self.slot = (self.slot - 1) % MODE_SLOTS
            print(f"[rc] 模式档 -> slot {self.slot + 1}")
        if self.edge(pad, b.get("arm_toggle")):
            self.armed = not self.armed
            print(f"[rc] 解锁开关 -> {'ARM' if self.armed else 'DISARM'}")
        if self.edge(pad, b.get("offboard_toggle")):
            self.offboard = not self.offboard
            print(f"[rc] Offboard 开关 -> {'ON' if self.offboard else 'OFF'}")
        if self.edge(pad, b.get("goto_position")):
            self.slot = 0
            print("[rc] 强制切到 slot 1 (Position)")
        if self.edge(pad, b.get("goto_land")):
            self.slot = MODE_SLOTS - 1
            print("[rc] 强制切到 slot 6 (Land)")

        # 历史原型行为：松开撤销请求，不满足当前锁存要求，不能直接用于真机飞行。
        kill_name = self.calib["buttons"].get("kill_hold")
        self.kill = pad.held(kill_name) if kill_name else False

    def channels(self, pad):
        self.update_switches(pad)
        ax = self.calib["axes"]
        mp = self.calib["map"]
        db = self.calib["stick"]["deadband"]
        ex = self.calib["stick"]["expo"]

        def stick(fn):
            code = mp[fn]
            code_num = {"ABS_X": ABS_X, "ABS_Y": ABS_Y,
                        "ABS_RX": ABS_RX, "ABS_RY": ABS_RY}[code]
            raw = pad.axes.get(code_num)
            if raw is None:
                return 0.0
            return shape(axis_to_unit(raw, ax[code]), db, ex)

        # 6 档：把 [1000,2000] 等分，取每档中点
        slot_pwm = int(round(PWM_MIN + (PWM_MAX - PWM_MIN) * (self.slot + 0.5) / MODE_SLOTS))

        chans = [PWM_MID] * NUM_CH
        chans[CH_ROLL - 1] = unit_to_pwm(stick("roll"))
        chans[CH_PITCH - 1] = unit_to_pwm(stick("pitch"))
        chans[CH_THR - 1] = unit_to_pwm(stick("throttle"))
        chans[CH_YAW - 1] = unit_to_pwm(stick("yaw"))
        chans[CH_FLTMODE - 1] = slot_pwm
        chans[CH_ARM - 1] = PWM_MAX if self.armed else PWM_MIN
        chans[CH_KILL - 1] = PWM_MAX if self.kill else PWM_MIN
        chans[CH_OFFB - 1] = PWM_MAX if self.offboard else PWM_MIN
        return chans


# ===================================================================== MAVLink
class Link:
    def __init__(self, url, dry_run=False):
        self.dry_run = dry_run
        self.url = url
        self.mav = None
        self.target_system = 1
        self.target_component = 1
        self._last_sent = None      # 最近一次发出去的 8 路，用来和回读值对比
        # 最近发过的若干帧。回读（飞控 5Hz）比发送（我们 20~50Hz）慢好几拍，
        # 所以"回读 != 刚发的那帧"通常只是延迟，不是链路错。要拿整条历史去比。
        self._sent_history = collections.deque(maxlen=32)
        self._n_rc = 0              # 收到多少帧 RC_CHANNELS 回读
        self._last_mode = None      # 上一次看到的 custom_mode
        self._last_armed = None     # 上一次看到的 SAFETY_ARMED
        self._texts = []            # 收到的 STATUSTEXT 原文（给台架脚本断言用）
        if dry_run:
            print(f"[mav] dry-run：不连 {url}")
            return
        from pymavlink import mavutil
        self.mavutil = mavutil
        print(f"[mav] 连接 {url} （{mavlink_link.describe(url)}）…")
        self.mav = mavlink_link.open_link(url, source_system=250)
        hb = mavlink_link.wake_and_wait_heartbeat(self.mav, timeout=20)
        if hb is None:
            raise SystemExit("[mav] 20 秒没收到心跳。检查链路 / 波特率 / mavlink-router。")
        self.target_system = self.mav.target_system
        self.target_component = self.mav.target_component
        print(f"[mav] 心跳 OK  sysid={self.target_system} "
              f"compid={self.target_component}")
        # 连上之后必须一直补 GCS 心跳，否则几秒后 STATUSTEXT 全停（见 KeepAlive）
        self._keepalive = mavlink_link.KeepAlive(self.mav)
        # 让飞控按固定频率回读 RC_CHANNELS（#65）—— drain() 全靠它。
        # MAV_CMD_SET_MESSAGE_INTERVAL (511)，interval 单位微秒。5Hz 足够，别贪。
        self.mav.mav.command_long_send(
            self.target_system, self.target_component,
            self.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            self.mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 200000, 0, 0, 0, 0, 0)

    def send_rc(self, chans):
        """发一帧 RC_CHANNELS_OVERRIDE (#70)，把手柄状态灌进飞控的 RC 链路。

        三个点，都对着 PX4 **v1.14.3**（实机固件）源码核过：

        1. **必须走 `self.mav.mav.xxx_send()`**。pymavlink 的 mavfile 对象
           （mavserial / mavudp / mavtcp…）**没有** `rc_channels_override_send`
           这个包装方法 —— 它只为 `param_set_send` 等少数几个消息生成包装。
           写成 `self.mav.rc_channels_override_send(...)` 会
           `AttributeError: 'mavserial' object has no attribute ...`（踩过）。

        2. **只发 8 路，别去凑 18 路**。RC_CHANNELS_OVERRIDE 的基本字段就是
           chan1~chan8，chan9~chan18 是 **MAVLink v2 extension 字段**；而且这个
           pymavlink build (2.4.49) 的 dialect 压根不认识它们（fieldnames 只有
           8 个）。发短的完全没问题：CRC_EXTRA 只按基本字段算，接收端把缺的
           extension 解成 0。PX4 侧 mavlink_receiver.cpp:1981-1991 的规则是
               ignore_max  = (value == UINT16_MAX)     # 对所有路都成立
               ignore_zero = (i > 7) && (value == 0)   # 只对 chan9~chan18
           即 chan9~chan18 收到 0 就算"没给"，channel_count 正算成 8。
           硬塞 18 个值会 TypeError（函数只收 8 个）。

        3. **ch1~ch8 不能用 0 占位**。0 不在忽略规则里，会被当成"杆打到最低"。
           用不到的路要么给真实值，要么给 65535（UINT16_MAX = 忽略）。

        只校验 target_system，不看 target_component（同文件 :1955-1958）。
        """
        if self.dry_run or self.mav is None:
            return
        self._keepalive.tick()
        payload = list(chans)
        if len(payload) < 8:
            payload += [0xFFFF] * (8 - len(payload))   # 65535 = 让 PX4 忽略这一路
        payload = payload[:8]
        self._last_sent = payload
        self._sent_history.append(tuple(payload))
        self.mav.mav.rc_channels_override_send(
            self.target_system, self.target_component, *payload)

    def drain(self, slot, echo=False):
        """非阻塞收干飞控这一拍回给我们的消息 —— **这才是 override 真到了的证据**。

        `send_rc()` 只说明"发出去了"。UDP 发出去没人收、串口对端不解析，都叫发出去
        了。真正的判据是**飞控回读**：`RC_CHANNELS` (#65) 的内容就是飞控侧的
        `input_rc`（mavlink_receiver.cpp:1953-1991 把 override 直接填进 input_rc），
        所以它应该逐路等于**我们发过的某一帧**。

        ⚠ 注意是"某一帧"不是"刚发的那帧"：回读是飞控按 5Hz 推的，我们却按
        20~50Hz 发，所以回读会滞后好几拍。只跟 `_last_sent` 比会平白报"对不上"。
        这里拿 `_sent_history` 整条历史去比，命中历史帧就算链路 OK。

        同一根串口只能被一个进程打开，所以没法"再开一个 rc_watch.py 边发边看"——
        回读必须在这个循环里做。

        顺带打印（这两条是加分项，没出现**不等于**链路坏）：
          - HEARTBEAT.custom_mode 变没变 -> 模式档（L1/R1）解码对不对。
            ⚠ 回读的是 nav_state 枚举值，不是 COM_FLTMODE* 参数值，见上面的表。
          - HEARTBEAT 的 SAFETY_ARMED 位 -> 解锁开关（PLUS）解码对不对。
            地面推桨、EKF 没有位置源时【根本解不开锁】，这是正常的；
            这时 STATUSTEXT 会给出被哪条 prearm 拦下 —— 那本身就证明开关解码对了。
          - STATUSTEXT 原样转发。

        `echo=True` 时每一帧回读都打（默认只打第一帧 + 所有变化），免得 5Hz 刷屏。
        """
        if self.dry_run or self.mav is None:
            return
        self._keepalive.tick()
        mavlink = self.mavutil.mavlink
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()

            if t == "RC_CHANNELS":
                self._n_rc += 1
                vals = [getattr(msg, f"chan{i}_raw") for i in range(1, 9)]
                if echo or self._n_rc == 1:
                    sent = self._last_sent
                    hist = self._sent_history
                    if sent is None or not hist:
                        tail = ""
                    elif tuple(vals) == hist[-1]:
                        tail = "   ✔ 与发出的一致"
                    elif tuple(vals) in hist:
                        # 回读比发送慢几拍（我们 20~50Hz，飞控 5Hz），
                        # 回读到的是前面某一帧，链路是好的。
                        tail = "   ✔ 与之前某帧一致（回读滞后）"
                    else:
                        tail = ("\n      咱发的 " + " ".join(f"{v:>4}" for v in sent)
                                + "   <-- 对不上！")
                    print("[rc←] 飞控回读 "
                          + " ".join(f"{v:>4}" for v in vals) + tail, flush=True)

            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.target_system:
                if msg.custom_mode != self._last_mode:
                    self._last_mode = msg.custom_mode
                    exp = (px4_modes.custom_mode_from_param(SLOT_PARAM[slot])
                           if 0 <= slot < len(SLOT_PARAM) else None)
                    mark = ("   == 当前档位期望值" if exp == msg.custom_mode
                            else "   (当前档位期望 %s)" % px4_modes.describe(exp)
                            ) if exp is not None else ""
                    print("[fc ] 模式 -> %s%s" % (px4_modes.describe(msg.custom_mode), mark),
                          flush=True)
                armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if self._last_armed is None:
                    self._last_armed = armed
                    print(f"[fc ] 当前{'已解锁 ARMED' if armed else '未解锁 disarmed'}",
                          flush=True)
                elif armed != self._last_armed:
                    self._last_armed = armed
                    print(f"[fc ] >>> {'已解锁 ARMED' if armed else '已上锁 DISARMED'}",
                          flush=True)

            elif t == "STATUSTEXT":
                txt = str(msg.text).rstrip("\x00").strip()
                if txt:
                    self._texts.append(txt)
                    print(f"[fc!] {txt}", flush=True)


# ===================================================================== 校准
CALIB_STEPS = [
    ("rest",   "把两个摇杆都松手，什么都别碰", 3.0),
    ("ABS_Y_up",   "左杆推到【最上】（油门最大方向），保持住", 3.0),
    ("ABS_Y_dn",   "左杆推到【最下】（油门最小方向），保持住", 3.0),
    ("ABS_X_rt",   "左杆推到【最右】（偏航右），保持住", 3.0),
    ("ABS_X_lt",   "左杆推到【最左】（偏航左），保持住", 3.0),
    ("ABS_RY_up",  "右杆推到【最上】（俯仰向前），保持住", 3.0),
    ("ABS_RY_dn",  "右杆推到【最下】（俯仰向后），保持住", 3.0),
    ("ABS_RX_rt",  "右杆推到【最右】（横滚右），保持住", 3.0),
    ("ABS_RX_lt",  "右杆推到【最左】（横滚左），保持住", 3.0),
]

CODE_OF = {"ABS_X": ABS_X, "ABS_Y": ABS_Y, "ABS_RX": ABS_RX, "ABS_RY": ABS_RY}


def do_calibrate(calib, pad, path):
    print("\n=== 校准 ===  每一步都保持住，按提示做即可。")
    rec = {}
    for key, prompt, dur in CALIB_STEPS:
        print(f"\n>>> {prompt}")
        for n in (3, 2, 1):
            print(f"    {n}…", end="", flush=True)
            time.sleep(0.7)
        print("  采集！", flush=True)
        samples, t0 = {}, time.time()
        while time.time() - t0 < dur:
            if not pad.poll(0.2):
                raise SystemExit("[calib] 手柄断开，校准中止。")
            # 每轮都额外读一次"当前值"。事件只在变化时来，
            # 光靠事件的话「松手别碰」那一步会一个样本都收不到。
            for code, val in pad.read_live_axes().items():
                samples.setdefault(code, []).append(val)
        med = {c: sorted(v)[len(v) // 2] for c, v in samples.items() if v}
        rec[key] = med
        print("    采到: " + "  ".join(
            f"{ABS_NAME[c]}={med[c]:+d}" for c in sorted(med)))

    center = rec.get("rest", {})
    axes = {}
    for name, (lo_key, hi_key) in {
            "ABS_X":  ("ABS_X_lt", "ABS_X_rt"),      # 左杆左右 -> 偏航
            "ABS_Y":  ("ABS_Y_dn", "ABS_Y_up"),      # 左杆上下 -> 油门
            "ABS_RX": ("ABS_RX_lt", "ABS_RX_rt"),    # 右杆左右 -> 横滚
            "ABS_RY": ("ABS_RY_dn", "ABS_RY_up"),    # 右杆上下 -> 俯仰
    }.items():
        code = CODE_OF[name]
        lo = rec[lo_key].get(code)
        hi = rec[hi_key].get(code)
        mid = center.get(code)
        if lo is None or hi is None or mid is None:
            print(f"[calib] {name} 数据缺失，保留默认值", file=sys.stderr)
            continue
        # 注意：这里**不能**按大小交换 lo/hi。
        # axis_to_unit() 的分段线性对 lo>hi（轴反向）本身就是对的：
        # raw 落在 hi 那一侧会得到负值。交换反而会让极性整体翻转。
        axes[name] = {"min": int(lo), "center": int(mid), "max": int(hi),
                      "invert": False}
        print(f"[calib] {name:7} min={int(lo):7} center={int(mid):7} "
              f"max={int(hi):7}" + ("   (轴反向)" if lo > hi else ""))

    out = dict(DEFAULT_CALIB)
    out["device"] = calib.get("device", DEFAULT_CALIB["device"])
    if axes:
        out["axes"] = axes
    with open(path, "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"\n[calib] 已写入 {path}")
    return out


# ===================================================================== 主流程
def main():
    ap = argparse.ArgumentParser(
        description="手柄 -> PX4 RC_CHANNELS_OVERRIDE 桥",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="udpin:127.0.0.1:14551",
                    help="pymavlink 连接串 (默认 udpin:127.0.0.1:14551)")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="RC 覆盖发送频率 Hz (默认 50；TRS 链路紧张时降到 20)")
    ap.add_argument("--device", help="设备名字串，如 BEITONG 或 X-Box")
    ap.add_argument("--calib", default=os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "gamepad_calib.json"),
        help="校准文件路径")
    ap.add_argument("--calibrate", action="store_true", help="交互式校准后退出")
    ap.add_argument("--list", action="store_true", help="列出手柄候选设备")
    ap.add_argument("--monitor", action="store_true", help="只打印，不发 MAVLink")
    ap.add_argument("--dry-run", action="store_true", help="连 PX4 但不发 RC 覆盖")
    ap.add_argument("--no-wait-stable", action="store_true",
                    help="不等待 USB 模式震荡结束（不推荐）")
    ap.add_argument("--echo", action="store_true",
                    help="每一帧都打印飞控回读的 RC_CHANNELS 并和发出值对比"
                         "（默认只打首帧和所有变化）")
    args = ap.parse_args()

    # 已知飞控HardFault期间保留本机输入检查，禁止旧完整遥控器打开控制链路。
    # --monitor/--dry-run完全不连接飞控；不能把它们当作飞行验证。
    if not (args.list or args.calibrate or args.monitor or args.dry_run):
        require_active_bench_clear(ap)

    if args.list:
        cands = [c for c in list_candidates() if c["has_abs"] and c["has_btn"]]
        if not cands:
            print("没找到手柄候选（要求：有轴 + 有手柄键）。当前所有输入设备：")
            for c in list_candidates():
                print(f"  {c['name']:40} {c['path']:20} {c['vid']}:{c['pid']}")
            return 1
        print("手柄候选（有轴 + 有手柄键）：")
        for c in cands:
            print(f"  {c['name']:40} {c['path']:20} {c['vid']}:{c['pid']}")
        return 0

    calib = dict(DEFAULT_CALIB)
    if os.path.exists(args.calib):
        try:
            with open(args.calib) as f:
                calib.update(json.load(f))
            print(f"[cfg] 读入校准 {args.calib}")
        except (OSError, ValueError) as e:
            print(f"[cfg] {args.calib} 读不了（{e}），用默认值", file=sys.stderr)

    # ---- 等设备稳定 ----
    if args.no_wait_stable:
        dev = find_device(calib, args.device)
        if not dev:
            return 1
    else:
        dev = wait_stable(calib, args.device)
        if not dev:
            return 1

    pad = Gamepad(dev["path"])

    if args.calibrate:
        do_calibrate(calib, pad, args.calib)
        pad.close()
        return 0

    mapper = Mapper(calib)
    link = Link(args.link, dry_run=args.monitor or args.dry_run)

    # 开关初值刻意不跟随按键：卡住的按键不会在启动瞬间触发一次模式/解锁跳变。
    print("\n[rc] 通道分配: ch1 roll ch2 pitch ch3 throttle ch4 yaw "
          "ch5 fltmode ch6 arm ch7 kill ch8 offboard")
    print("[rc] 开关初值: slot1 / 未解锁 / kill 释放 / offboard 关")
    print("[rc] 开始。Ctrl-C 退出。\n")

    period = 1.0 / args.rate
    next_tx = time.time()
    n_tx = n_lost = 0
    last_print = 0.0

    try:
        while True:
            now = time.time()
            if not pad.poll(max(0.0, min(0.05, next_tx - now))):
                print("\n[rc] !! 手柄断开 —— 停止发送。"
                      "飞控会在 COM_RC_LOSS_T 后进入 RC 丢失失效保护。", flush=True)
                n_lost += 1
                pad.close()
                time.sleep(1.0)
                dev = wait_stable(calib, args.device)
                if not dev:
                    continue
                pad = Gamepad(dev["path"])
                mapper = Mapper(calib)
                print("[rc] 手柄恢复，开关回到安全初值。", flush=True)
                continue

            now = time.time()
            if now >= next_tx:
                chans = mapper.channels(pad)
                link.send_rc(chans)
                # 收飞控的回读。必须在同一个循环里 —— 一根串口只能被一个进程打开。
                link.drain(mapper.slot, echo=args.echo)
                n_tx += 1
                # monitor 时按 ~5 Hz 打印（人眼够看），正常跑时每秒一行
                if (now - last_print) >= (0.2 if args.monitor else 1.0):
                    last_print = now
                    state = "ARM" if mapper.armed else "dis"
                    kill = " KILL!" if mapper.kill else ""
                    offb = " offboard" if mapper.offboard else ""
                    print(f"[{n_tx:6d}] ch={chans[:8]}  slot{mapper.slot+1} "
                          f"{state}{kill}{offb}", flush=True)
                next_tx += period
                if next_tx < now - 0.5:      # 落后太多就重新对齐，别追赶
                    next_tx = now + period
    except KeyboardInterrupt:
        # --monitor 走的是 dry_run 链路，一个字都没发出去，别写成"发过"。
        verb = "演练过" if (args.monitor or args.dry_run) else "发过"
        print(f"\n[rc] 退出。{verb} {n_tx} 帧，手柄掉线 {n_lost} 次。")
        if not (args.monitor or args.dry_run):
            got = link._n_rc
            if got:
                print(f"[rc] 期间收到飞控回读 RC_CHANNELS {got} 帧"
                      f" —— override 确实进了飞控的 input_rc。")
            else:
                print("[rc] ⚠ 一帧 RC_CHANNELS 回读都没收到，override 可能没被接受。"
                      "\n     查：COM_RC_IN_MODE 是不是 0 / target_system 对不对 / "
                      "这根串口是不是单向的。")
    finally:
        pad.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
