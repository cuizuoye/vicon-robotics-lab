# 整车速度接口与麦轮安装整改

后续状态：三向规划已接入独立PI；旧5秒镜像首次left50未启动并保护全停，之后已完整备份、更新独立`drive_pi_start8_v1`，唯一一次10秒悬空left50轮向/停机通过。用户确认断动力，最终fault=0/used=1/en=0，不自动复位或连跑；见[当前启动观察记录](DRIVE_STARTUP_OBSERVE_2026-10-10.md)。下文保留原离线规划阶段的实现/未烧录边界，不代表当前板内状态，仍不证明落地侧移或旋转。

日期：2026-10-10。本阶段保持动力断开，只修改软件、构建及离线测试；未通过串口读写开发板、未烧录/复位、未转动电机，未改Wi-Fi、代理或虚拟网卡。

## 实车参数与安装

用户报告轮心轴距L约122mm、轮距W约170mm，采用标称60mm轮径；运动学半尺寸为61mm/85mm，旋转力臂L/2+W/2=146mm。尺寸是现场近似值，不是精密测量或Vicon标定结果，RViz示意尺寸没有被当作实车参数。

最初报告的俯视滚子轴线为FL反斜线/FR反斜线/RL斜线/RR斜线，与本工程X模型不匹配。按建议整改右侧两轮后，用户确认手转顺畅、尺寸不变，排列如下（车头朝上；符号是顶部小滚子的轴线，不是轮毂侧面纹路）：

```text
       车头 ↑
左前  \     /  右前
左后  /     \  右后
```

安装确认和原排列均保留在[几何配置](../src/mecanum_vicon_control/config/chassis_geometry.json)。此前四轮悬空前进试验只证明四路电机/编码器速度环工作，发生在此次安装整改之前，不作为整改后侧移、旋转或落地验收。

## 新增接口的作用

`vx/vy/wz`分别表示车体向前速度(m/s)、向左速度(m/s)、逆时针角速度(rad/s)，与ROS控制器输出约定一致。顺序固定FL/FR/RL/RR；每轮“前进”为归一化正方向。

目标处理：车体速度 → X型逆运动学 → 四轮轮缘速度 → 各自CPR换算 → 四路归一化PI目标。四轮共同比例限速，避免逐轮裁剪改变原运动方向。当前规划上限为每轮50mm/s、每轮300cps，未提升任何已验证台架输出上限。

保留近似CPR：FL1059.67/FR1049/RL1055.67/RR1061。电机/编码器符号仍为+1/−1/+1/−1；PI目标不提前乘电机符号，最终PWM方向才应用motor_sign。报告中的motor_direction_reference仅作方向诊断，不是PWM占空比或动作指令。

反向接口将四个原始编码器cps按encoder_sign归一化后做正运动学，得到编码器模型车体速度；它不是Vicon真值、融合里程计或滑移补偿。正式外环仍使用Vicon刚体位姿，不以单个marker坐标控制车体。

尺寸、X安装或CPR未确认，非法数值、错误轮序/直径/符号、放宽上限均拒绝。C新接口失败时输出全归零；配置中的滚子排列若与X确认标记矛盾，Python加载器拒绝。`planning_ready=true`也不代表运动授权：离线命令永远报告`motion_authorized=false`。

## 代码与边界

- STM32：`Inc/Src/mecanum_drive_plan.*`，新增SI正/逆运动学在`mecanum_kinematics.*`；旧整数接口保持原行为。
- PC：`mecanum_vicon_control/drive_plan.py`和`drive_plan_dryrun`安装入口。
- CMake只新增静态库`mecanum_drive_planning`，未链接到当前前进固件；没有新三向可烧录镜像。
- 本次没有接入MC-Link会话、UART或PWM，没有动态修改四轮PI状态；组控制的使能、时序、故障锁存、5秒时限和10%输出保护须在下一阶段集成中继续保留。
- 上一次前进BIN仍与原独立读回一致，SHA256为`da22f0a097c418cccf2adba9ed4f9876c8292b2d6f3d3a9145e3729446fb0931`。板内未更新，本阶段也未重新读取实机状态；最近保存的试验终态为fault=0/used=1，不能据此再次直接运行。

## 离线结果

以下是轮缘目标，不是实际运动记录：

| 车体命令 | FL/FR/RL/RR轮缘目标(mm/s) |
| --- | --- |
| 前进vx=0.05m/s | +50 / +50 / +50 / +50 |
| 左移vy=0.05m/s | −50 / +50 / +50 / −50 |
| 逆时针wz=0.1rad/s | −14.6 / +14.6 / −14.6 / +14.6 |

组合命令vx=0.05、vy=0.03、wz=0.1，统一缩放系数约0.52854，轮缘目标约2.854/50/34.567/18.288mm/s。前进50mm/s时四轮cps各约281.085/278.256/280.024/281.439，而非复制同一个电机的计数目标。

验证：

- 新C接口6组ASan/UBSan检查通过，包含441组模型正逆换算；GCC静态分析通过。
- 2000组固定随机种子的C/Python目标与反馈模型一致性检查通过，包含不同CPR触发计数限速的情况。
- ROS控制包85项测试通过，colcon报告0错误/0失败/0跳过；构建及安装后的离线入口通过。
- 原台架工具194项测试通过，旧整数运动学回归通过；ARM静态库构建通过。

这些不证明侧移/旋转实机性能、负载或真实无线控制。完整模型结果见[离线记录](drive_plan_offline_20261010.json)，测试源码分别为`tests/test_mecanum_drive_plan.c`与`test/test_drive_plan.py`。

## 自己查看目标，不会转轮

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run mecanum_vicon_control drive_plan_dryrun \
  --geometry install/mecanum_vicon_control/share/mecanum_vicon_control/config/chassis_geometry.json \
  --wheel-profile docs/four_speed_bench_adopted_profile_20261009.json \
  --vx 0 --vy 0.05 --wz 0
```

该工具只有文件读取和数学计算，不创建ROS节点，不打开串口或网络，不发送速度/心跳/使能。

## 下一步

将新目标生成层接入独立、受限的三向四轮组控制状态机，补测零目标轮、固定方向、整组故障停机和通信时序；不要把目标数组直接写成PWM。先离线/ARM验证，再在断动力确认后备份烧录、只读检查，最后另行确认悬空上电，做有限的整体侧移/旋转观察。随后才做低速落地、MC-Link无线运动与Vicon位姿外环；不追加重复逐轮PI整定。
