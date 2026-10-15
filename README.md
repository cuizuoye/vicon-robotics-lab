# Vicon Robotics Lab

ROS 2 / Vicon 麦克纳姆小车与 MicoAir 无人机的跨电脑接力开发基线。

这是**脱敏后的公开源码快照**，不是原电脑目录的完整镜像，也不是硬件起飞/行驶许可。
原工作区保持原样；本仓库的代码迁移改动只发生在独立副本。

## 从另一台 Linux 开始

参考环境：Ubuntu 24.04 x86_64、Python 3.12、ROS 2 Jazzy。
没有 ROS 也可先做无人机纯 Python 策略测试；实时 Vicon/ROS 及完整小车回归需要 Jazzy。

```bash
git clone https://github.com/cuizuoye/vicon-robotics-lab.git
cd vicon-robotics-lab
git switch -c dev/laptop/first-task
bash scripts/doctor.sh
bash scripts/bootstrap_python.sh       # 仅创建本仓库 .local/venv，联网安装固定 Python 依赖
bash scripts/test_offline.sh
```

然后在这个目录中开始接力，先阅读 `AGENTS.md` 和 `handoff/CURRENT_STATE.json`。
不复制原电脑的 Codex 登录信息；新电脑独立登录。不要依靠旧对话推断当前安全状态。

## 目录

| 目录 | 内容 |
| --- | --- |
| `car/` | ROS 小车包、STM32 各阶段固件、ESP-IDF 完整 SoftAP 桥与隔离诊断例程 |
| `drone/` | MicoAir TRS/HOME/定位策略、ROS 仿真、PX4 1.14.3 修补差异 |
| `shared/vicon_receiver_ws/` | 带 marker/遮挡过滤的 Vicon ROS 接收器源码，不含专有 SDK |
| `handoff/` | 当前状态、依赖版本和迁移核验清单 |
| `scripts/` | 环境自检、离线测试、公开上传扫描及可选构建入口 |

## 当前进度与限制

- 小车：四轮已做过悬空方向/编码器/初始 PI 验证。完整 SoftAP → UART → STM32
  只读链路、断流和重连检查通过。流式运动候选版已构建，但**未烧录、未落地**。
- 小车真实 Vicon：前移、左转与遮挡恢复已核对，模型及实际轨迹可以显示；
  +135°仍是临时显示偏移，车体中心和精确航向没有正式标定。
- 无人机：真机是自定义 PX4 **1.14.3**，不是 SITL 的 1.17。已完成固件修补、
  TRS/HOME若干未解锁验证及重建刚体的一次短时静止融合；**未完成自主起飞验收**。
- 无人机物理 hold、历史故障守卫和刚体版本守卫保留。一次例外通过不能放开全局守卫。
  HOME依赖电脑/TRS，不等于独立于电脑的遥控急停；常驻链路和电池独立融合仍须验收。

详见 [小车说明](car/README.md)、[无人机说明](drone/README.md) 和机器可读交接文件。

## 不影响主电脑的接力方法

1. `main`只作为人工验收后的基线；新电脑在 `dev/<machine>/<topic>` 分支提交和推送。
2. 分支推送不等于主电脑更新，也不授权自动烧录或给硬件上电。
3. 主电脑先克隆本仓库到一个**新目录**，查看差异、运行测试，再挑选需要的提交。
4. 若方向错误，可以另建一个稳定版本目录，不删除当前成果：

```bash
git worktree add --detach ../vicon-baseline baseline-2026-10-15
```

标签只是代码/文档基线，不是“已验证可飞/可行驶”的标签。不要用 `reset --hard` 或强推作为默认回滚。

## 私密和机器专属资料

本仓库不含真实 Wi-Fi 密码、完整 Flash/NVS、PX4 参数全备份、一次性许可或有效校准文件。
设备标识已替换为合成值，因此公开版本默认无法匹配原真机；这是有意的安全边界。
历史实机脚本中的原电脑路径已换成`/home/PRIMARY_USER`占位符，不属于快速开始入口；
它们需要单独做本地配置与审查，不能直接全目录批量执行。
需要继续实机工作时，由设备所有者通过受控方式提供本地 `.local/` 配置，并重新核验
UID、串口、固件哈希、桨/电池状态和 marker 版本；不能自动套用旧许可。

Vicon DataStream SDK 是专有依赖，须在确认相应许可后从官方获取并按共享接收器
README的目录约定本地安装。它不影响纯 Python 策略与小车离线算法测试。

## 许可与发布范围

没有为整个混合仓库新授予统一许可证。各组件保留原许可证和版权头：Vicon接收器GPL-3.0、
PX4差异基于BSD-3-Clause源码、STM32 HAL/CMSIS带各自许可证。
厂商二进制、原始实验记录和未明确授权分发的资料不公开上传。
