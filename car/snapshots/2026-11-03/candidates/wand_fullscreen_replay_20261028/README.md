# 普通窗口 / 全屏只读回放诊断

这是独立的离线诊断入口，不是小车演示入口。不修改原操作台、矩形程序、固件、网络、驱动、内核或显示设置。

## 隔离范围

- 仅读取 2026-10-28 上午两次异常会话的 `observation.jsonl`；记录原文件 SHA-256。
- 提取车/杆位姿、监督器和门控的显示字段；不回放指令、租约、许可、设备地址或串口报文。
- 原速、同序列回放约 163 秒，先 1280×720 普通窗口，再使用窗口管理器全屏；不修改桌面分辨率。
- 直接复用已用操作台的地图、轨迹、文字和大字体绘制代码；轨迹仍限制为 500 点，计时仍 `after(20)` / 绘图间隔门限 33ms。
- 保留每周期的本机 X11 `XQueryKeymap` 只读查询，不合成按键。不运行原操作台的控制状态机。
- ROS 导入仅为加载原绘图代码；禁止 ROS 初始化、节点/执行器构造、Python socket、子进程和串口打开。没有 DDS 节点、硬件端口或控制输出。
- 所有控制按钮禁用，空格无效；Esc / 关闭窗口仅关闭回放。全过程小车应断电。
- 独立进程每 0.5 秒记录资源，每 0.1 秒检查界面文件心跳。界面无心跳超过 0.5 秒时请求 Python 栈，超过 2 秒时仅终止本次回放子进程。系统/内核级阻塞时不能保证监督进程仍能运行。

## 执行

仅在本机 X11 桌面运行，目录名必须为新的、尚不存在的路径：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_fullscreen_replay_20261028
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v test_replay.py
PYTHONDONTWRITEBYTECODE=1 python3 replay_diagnostic.py compare --output /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fullscreen_readonly_2026-10-28/new_run
```

## 记录

- `dataset.json`：输入哈希、时长、事件数量，以及断电日志末尾未写入的 NUL 字节。只排除终端 NUL/空白后缀；中间损坏、损坏 JSON 直接失败。
- `*_ui.jsonl`：每周期间隔、X11 查询 / 数据分发 / 绘图耗时、画布和窗口尺寸、轨迹/图元数量、已分发事件数量。
- `*_x11_probe.jsonl`：另一进程的 X11 往返耗时。
- `*_system.jsonl`：界面、Xorg、GNOME 的 CPU 计数、RSS、状态、等待点；系统内存、Swap、CPU 和压力；可读的温度、Intel GPU 频率/节流状态。
- `*_watchdog.jsonl` / `*_python_stacks.log`：独立看门狗与卡顿栈。
- `xorg_increment.log`、`kernel_journal.json`、`gnome_journal.json`：本轮新增的相关日志，不收集其他应用的日志或完整环境。
- `protected_before.json` / `protected_after.json`：原代码保护清单。
- `summary.json`：汇总指标、子进程结局、完成度和隔离状态。

正式结果见 `/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fullscreen_readonly_2026-10-28/DIAGNOSIS.md` 和 `run_03`。`run_02` 使用了会重复注册空按钮回调的第一版诊断包装，不用其内存增量或约101ms的短卡顿来推断原程序问题。当前版本只注册一次空回调，并记录回调数量、绘图线程CPU及地图细分耗时；不改原操作台。

## 结论边界

此测试隔离的是原绘图/X11负载，不重现真实 ROS/DDS 消息调度、Tenda、ESP32 或 STM32 负载。回放出现的历史 FAULT 是记录内容，不代表当前硬件状态。CPU 百分比以一个逻辑核心为 100%；绘图耗时包含等待及被系统抢占的墙钟时间。未复现不能排除驱动/合成器问题，也不能据此宣称实车演示稳定性已经修复。
