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
环境前提和双机工作方法见 [接力与回退](handoff/WORKFLOW.md)，
本次迁移验证结果见 [VALIDATION.json](handoff/VALIDATION.json)。
小车最新交付见 [2026-11-03状态](handoff/CAR_STATE_2026-11-03.json)、
[更新技术说明](car/reports/2026-11-03/TECHNICAL_REPORT.md) 和
[简历项目表述](car/reports/2026-11-03/RESUME.md)。
最新源码在 [独立快照](car/snapshots/2026-11-03/README.md)，新增验证见
[CAR_VALIDATION](handoff/CAR_VALIDATION_2026-11-03.json)。
[10月28日Word报告](car/reports/2026-10-28/Vicon标定杆遥操作ROS全向小车项目技术说明.docx)
保留为历史版本；`VALIDATION.json` 是10月15日历史收据，均不能替代最新交接状态。

## 目录

| 目录 | 内容 |
| --- | --- |
| `car/` | ROS 小车包、STM32 各阶段固件、ESP-IDF 完整 SoftAP 桥与隔离诊断例程 |
| `drone/` | MicoAir TRS/HOME/定位策略、ROS 仿真、PX4 1.14.3 修补差异 |
| `shared/vicon_receiver_ws/` | 带 marker/遮挡过滤的 Vicon ROS 接收器源码，不含专有 SDK |
| `shared/vicon_receiver_snapshot_20261028/` | 最新接收器开发源码及小车依赖桥接，不代表已安装二进制等价或硬件验收 |
| `handoff/` | 当前状态、依赖版本和迁移核验清单 |
| `scripts/` | 环境自检、离线测试、公开上传扫描及可选构建入口 |

## 当前进度与限制

- 小车：已完成真实Sign驱动实车的1:1平移及相对旋转跟随；4WSC档6进行了
  约60秒正常跟随和定时停车的操作者验收，配置上限0.70m/s、2.00m/s²，非测速认证。
  历史独立矩形方案完成循环和受扰回轨；不会被新版候选自动替换。
- Vicon同时提供Sign输入和car01实测位姿，构成整车位置外环；STM32编码器闭合
  轮速内环。采用速度前馈、误差反馈和XY/yaw耦合限幅，取消中间位置参考点。
- 最新部署记录为4WSC STOPFIX_V1，协议v8、208字节诊断、MCU630秒独立截止。
  这是历史读回核对记录，本次上传没有连接芯片、刷写或启动小车。
- 浏览器Canvas界面与后台会话解耦，允许已启动会话在切换应用/最小化时继续；
  关闭页面、断连、定位失效、控制超时和操作停止仍撤权。Windows远程实机未验收。
- 最新静止数据源修复完成600.003秒实际ROS节点＋原生C回环软件耐久，
  包含120ms输入处理停顿；**不是实车600秒验收**，新候选完整资格仍未完成。
  真实Tracker持续性、实车长时运行、停车包络和电气温升仍需现场验证；不具备避障能力。
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

## 新版小车离线复现

```bash
bash scripts/test_offline.sh           # 小车通用回归 + 原无人机回归
bash scripts/test_car_release.sh       # 导出的4WSA C控制器和78项主机测试
bash scripts/test_car_update_20261103.sh # 4WSC七组C测试＋27项数据源单元测试
python3 scripts/public_scan.py
```

当前工作区代码和选定运行时源码已收录；历史层叠生成器仍有原机路径占位符及未发布
的私有依赖，不能作为新电脑的一键实机部署入口。运行时中的现场确认均已撤销，
原始构建/刷写许可未转移。移植适配和重新建立本地硬件准入必须单独完成。

## 许可与发布范围

没有为整个混合仓库新授予统一许可证。各组件保留原许可证和版权头：Vicon接收器GPL-3.0、
PX4差异基于BSD-3-Clause源码、STM32 HAL/CMSIS带各自许可证。
厂商二进制、原始实验记录和未明确授权分发的资料不公开上传。
