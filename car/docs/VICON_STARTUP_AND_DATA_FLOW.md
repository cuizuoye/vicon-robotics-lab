# Vicon Tracker 4 + ROS 2 Jazzy 启动与数据传输流程

本文根据现有 `Vicon_ROS2_启动指令汇总.txt` 整理，并加入麦克纳姆小车闭环所需的
数据层、录包和安全检查。示例刚体仍使用 `T2/T2`；小车正式刚体建立后可替换为
`car01/base`。

## 1. 数据层与完整链路

控制使用的是 Vicon Tracker 解算后的**刚体中心位姿**，不是 T21～T25 任意一个
物理 marker 的坐标。

```text
Vicon 相机
  -> Windows Tracker 4（刚体解算）
  -> DataStream，TCP/UDP 801（或实验室配置端口）
  -> Linux vicon_client
  -> /vicon/T2/T2 (PoseStamped) 与 TF
  -> car_state_bridge
  -> /car/pose
  -> pose_controller
  -> /car/cmd_vel_raw
  -> safety_watchdog
  -> /car/cmd_vel
  -> 后续 Wi-Fi/ESP32-S3/STM32F407
```

辅助观测数据：

- `/vicon/T2/markers/T21`～`T25`：物理 marker 的世界坐标，仅用于遮挡和刚体质量检查。
- `/vicon/frame_number`：Vicon SDK 原始帧号，用于丢帧和跨传感器对齐。
- `/vicon/latency`：Vicon 报告的曝光到客户端延迟，单位秒。
- `/tf`、`/tf_static`：`map -> vicon -> T2_T2` 等坐标关系。

## 2. 网络准备

推荐 Windows Tracker 与 Linux ROS 2 电脑使用独立有线网络。

原有配置：

```text
Windows Tracker：10.10.10.2
Linux 有线口：   10.10.10.1
```

两端子网掩码必须一致。原记录使用 `255.0.0.0`；如果实验室没有固定配置，独立
点对点网络通常可使用更收敛的 `255.255.255.0`，但不要在未确认时单方面修改现有
Tracker 网络。

Linux 验证：

```bash
ping 10.10.10.2
```

Windows 打开 Tracker 4，并在 `View -> Connections` 查看客户端连接。Tracker 可处于
Live 或 Review 模式，但做实时闭环必须使用有效的 Live 数据。

## 3. 环境与构建关系

每个新终端按以下顺序加载：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
```

最后一行是小车 overlay；未构建前不会存在。

重新构建 Vicon 接收器：

```bash
cd /home/PRIMARY_USER/vicon_receiver_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select vicon_receiver --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

重新构建小车工作区：

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
colcon build --symlink-install
```

## 4. 可选：先用 DataStream SDK 自检

在 Vicon DataStream SDK 的 Linux64 目录运行：

```bash
./ViconDataStreamSDK_CPPTest 10.10.10.2
```

若实验室使用显式端口：

```bash
./ViconDataStreamSDK_CPPTest 10.10.10.2:804
```

如果程序没有执行权限：

```bash
chmod +x ViconDataStreamSDK_CPPTest
```

## 5. 启动 Vicon ROS 2 接收器

终端 A：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash

ros2 launch vicon_receiver client.launch.py \
  hostname:=10.10.10.2 \
  topic_namespace:=vicon \
  buffer_size:=200 \
  world_frame:=map \
  vicon_frame:=vicon \
  map_xyz:='[0.0, 0.0, 0.0]' \
  map_rpy:='[0.0, 0.0, 0.0]' \
  map_rpy_in_degrees:=false \
  stream_mode:=ServerPush \
  publish_latency:=true
```

闭环优先使用 `ServerPush`。marker ray、debug 和 device data 保持关闭，除非正在诊断
对应问题，避免增加无关数据量和处理延迟。

## 6. 启动后的必检项

终端 B：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash

ros2 node list
ros2 topic list
ros2 topic info /vicon/T2/T2
ros2 topic echo /vicon/T2/T2 --once
ros2 topic hz /vicon/T2/T2
ros2 topic hz /vicon/frame_number
ros2 topic echo /vicon/latency --once
ros2 run tf2_ros tf2_echo map T2_T2
```

预期类型：

```text
/vicon/T2/T2                  geometry_msgs/msg/PoseStamped
/vicon/T2/markers/T21         geometry_msgs/msg/PointStamped
/vicon/frame_number           std_msgs/msg/Int64
/vicon/latency                std_msgs/msg/Float64
```

marker 话题在收到首帧后动态创建。marker 遮挡或 SDK 返回失败时该点本帧不发布，不能
把 `[0, 0, 0]` 当作真实坐标。刚体位姿的遮挡、跳变和恢复仍需单独验证。

## 7. 录制一份可重复开发的数据

没有小车时，可把多个 marker 固定在临时板上建立刚体，依次完成静止、平移、旋转、
短暂遮挡和重新出现，然后录制：

```bash
mkdir -p /home/PRIMARY_USER/vicon_mecanum_ws/bags
cd /home/PRIMARY_USER/vicon_mecanum_ws/bags

ros2 bag record \
  /vicon/T2/T2 \
  /vicon/T2/markers/T21 \
  /vicon/T2/markers/T22 \
  /vicon/T2/markers/T23 \
  /vicon/T2/markers/T24 \
  /vicon/T2/markers/T25 \
  /vicon/frame_number \
  /vicon/latency \
  /tf \
  /tf_static
```

回放：

```bash
ros2 bag play <bag目录>
```

## 8. 启动小车软件在环

不需要 Vicon 或电机：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash

ros2 launch mecanum_vicon_control software_in_loop.launch.py
```

软件在环数据流：

```text
demo_goal -> /car/goal_pose
simulated_vicon_car -> /vicon/car01/base
car_state_bridge -> /car/pose + /car/odometry + TF
pose_controller -> /car/cmd_vel_raw
safety_watchdog -> /car/cmd_vel
/car/cmd_vel -> simulated_vicon_car
```

## 9. 用真实 Vicon 驱动控制器

保持终端 A 的 Vicon 接收器运行，在终端 C 执行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash

ros2 launch mecanum_vicon_control vicon_control.launch.py \
  vicon_pose_topic:=/vicon/T2/T2
```

此时 `/car/cmd_vel` 只在本机产生，还没有通过无线发送。真实模式默认禁用，检查坐标
方向、跳变阈值和目标位置后再执行：

```bash
ros2 topic pub --once /car/enable std_msgs/msg/Bool "{data: true}"
```

任一位姿或控制命令超时，安全看门狗会撤销使能并输出零速度；恢复数据后不会自动
重新使能。

## 10. 可视化

PlotJuggler：

```bash
ros2 run plotjuggler plotjuggler
```

重点曲线：

- `/car/pose/pose/position/x`、`y`
- `/car/cmd_vel/linear/x`、`linear/y`
- `/car/cmd_vel/angular/z`
- `/vicon/frame_number/data`
- `/vicon/latency/data`

RViz2：

```bash
rviz2
```

Fixed Frame 设为 `map`，添加 TF、Pose、Odometry。物理 marker 更适合用自定义
MarkerArray 转换节点显示；RViz2 默认显示插件不一定能直接按话题批量展示 PointStamped。

## 11. 闭环上车前门槛

- 确认控制输入为刚体中心位姿，不是单个 marker。
- 确认前进时 `x` 增大、左移时 `y` 增大、逆时针时 yaw 增大。
- 确认 Vicon 丢失、Wi-Fi 丢失、ESP32 丢包和 STM32 命令超时都会停车。
- 确认恢复通信不会自动继续旧目标。
- 确认实体急停独立于 Linux、Wi-Fi 和 ESP32。
- 先以 `0.2 m/s` 以下速度验证，再逐步提高。

