#!/usr/bin/env python3
"""PX4 飞行模式的**三层编号** —— 这是本项目最容易读错的地方，所以单独成模块。

同一句"把模式档放到 Position"，在这条链路上经过了三个互不相等的编号：

  ① COM_FLTMODE* 参数里填的**参数值**（QGC 下拉框那一套，也是 navStateFromParam 的入参）
        0 Manual   1 Altitude 2 Position  3 Mission 4 Loiter  5 RTL    6 Acro
        7 Offboard 8 Stabilized 9 PositionSlow 10 Takeoff 11 Land
        12 Follow  13 PrecLand 14 Orbit   15 VTOL-Takeoff
     出处 src/modules/manual_control/ManualControl.cpp navStateFromParam()
          v1.14.3 :523-543，v1.17.0 同（刻度两版本一致）

  ② **nav_state 枚举**（msg/VehicleStatus.msg）—— ① 经 navStateFromParam() 映射后的
     内部编号。v1.14.3 里 Offboard=14（不是 7）、Land=18（不是 11）。
     ⚠ 这一层【从不】出现在 MAVLink 上，只看 PX4 日志 / uORB 才用得到。

  ③ **HEARTBEAT.custom_mode 里那个 uint32** —— 飞控真正报出来的。它是
     px4_custom_mode 的**打包结构**，不是上面任何一层的裸值：
            data = main_mode << 16 | sub_mode << 24
     出处 src/modules/commander/px4_custom_mode.h:85-97  union px4_custom_mode
          src/modules/commander/px4_custom_mode.h:102-237 get_px4_custom_mode()
          src/modules/mavlink/streams/HEARTBEAT.hpp:104,130  custom_mode.data 发出去
     v1.14.3 与 v1.17.0 的 main/sub 常量完全一致（见文件末尾注释）。

把 ① 当 ③ 读是错的 —— 本项目第一版 rc_watch.py 就是这么错的，把 Offboard(7)
报成 14 那档、把 Land(11) 报成 18 那档。所以现在只留这一个模块，别的文件 import 它。

实机验证（2026-10-10，MicoAir743v2 / PX4 v1.14.3）：
    飞控启动初值  HEARTBEAT.custom_mode = 50593792 = 0x03040000 -> Auto/Loiter
    我们 slot1 生效后                     = 196608   = 0x00030000 -> Position
两者都被下面的表解对了。
"""

# ------------------------------------------------------------------ ③ 的解码表
# px4_custom_mode.h:44-56  enum PX4_CUSTOM_MAIN_MODE
MAIN_MODE = {1: "Manual", 2: "Altitude", 3: "Position", 4: "Auto", 5: "Acro",
             6: "Offboard", 7: "Stabilized", 8: "Rattitude", 9: "Simple",
             10: "Termination", 11: "AltitudeCruise"}

# px4_custom_mode.h:58-77  enum PX4_CUSTOM_SUB_MODE_AUTO（只在 main=AUTO 时有意义）
SUB_MODE_AUTO = {1: "Ready", 2: "Takeoff", 3: "Loiter", 4: "Mission", 5: "RTL",
                 6: "Land", 8: "Follow", 9: "PrecLand", 10: "VTOL-Takeoff"}

# px4_custom_mode.h:79-83  enum PX4_CUSTOM_SUB_MODE_POSCTL
SUB_MODE_POSCTL = {0: "", 1: "Orbit", 2: "Slow"}


# --------------------------------------------------- ① -> ③ 的映射（我们的 6 档）
# 由 ① 经 navStateFromParam() 走到 ②，再由 get_px4_custom_mode() 打成 ③。
# 表里的 (main, sub) 就是 px4_custom_mode.h:107-193 的分支结果。
PARAM_TO_MODE = {
    0:  (1, 0),    # Manual
    1:  (2, 0),    # Altitude
    2:  (3, 0),    # Position
    3:  (4, 4),    # Mission     -> AUTO / MISSION
    4:  (4, 3),    # Loiter      -> AUTO / LOITER
    5:  (4, 5),    # RTL         -> AUTO / RTL
    6:  (5, 0),    # Acro
    7:  (6, 0),    # Offboard
    8:  (7, 0),    # Stabilized
    10: (4, 2),    # Takeoff     -> AUTO / TAKEOFF
    11: (4, 6),    # Land        -> AUTO / LAND
    12: (4, 8),    # Follow      -> AUTO / FOLLOW_TARGET
    13: (4, 9),    # PrecLand    -> AUTO / PRECLAND
    14: (3, 1),    # Orbit       -> POSCTL / ORBIT
    15: (4, 10),   # VTOL Takeoff-> AUTO / VTOL_TAKEOFF
}


def pack(main_mode, sub_mode):
    """(main_mode, sub_mode) -> HEARTBEAT.custom_mode 里那个 uint32。"""
    return ((main_mode & 0xFF) << 16) | ((sub_mode & 0xFF) << 24)


def custom_mode_from_param(param_value):
    """COM_FLTMODE* 里填的值 -> 那时飞控应报出的 custom_mode。未分配返回 None。"""
    pair = PARAM_TO_MODE.get(int(param_value))
    return pack(*pair) if pair else None


def decode(custom_mode):
    """HEARTBEAT.custom_mode -> (可读名字, main_mode, sub_mode)。"""
    data = int(custom_mode) & 0xFFFFFFFF
    main = (data >> 16) & 0xFF
    sub = (data >> 24) & 0xFF
    if main == 4:
        name = SUB_MODE_AUTO.get(sub, "Auto/%d" % sub)
    elif main == 3:
        s = SUB_MODE_POSCTL.get(sub, str(sub))
        name = "PosCtl" + ("/" + s if s else "")
    else:
        name = MAIN_MODE.get(main, "main%d" % main)
    return name, main, sub


def describe(custom_mode):
    """给日志用的一行：`0x00030000 = main 3 / sub 0 (Position)`。"""
    name, main, sub = decode(custom_mode)
    return "0x%08X = main %d / sub %d (%s)" % (int(custom_mode) & 0xFFFFFFFF,
                                               main, sub, name)


# ---------------------------------------------------------------------------
# 两个固件版本的差异（2026-10-10 比对 v1.14.3 与 v1.17.0 的 px4_custom_mode.h）：
#   main_mode  : 我们用到 1..7 两版本完全一致。1.17.0 多了 10 Termination
#                (1.14.3 也有) 和 11 AltitudeCruise（1.14.3 无）。
#   sub_mode   : READY=1 TAKEOFF=2 LOITER=3 MISSION=4 RTL=5 LAND=6
#                FOLLOW_TARGET=8 PRECLAND=9 VTOL_TAKEOFF=10 两版本一致；
#                1.17.0 多了 EXTERNAL1..8（11..18），1.14.3 无。
#   => 本项目 COM_FLTMODE1..6 只用到位 1..8 / 3 / 4 / 6，两边都安全。
# ---------------------------------------------------------------------------
