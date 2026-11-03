# 不使用ESP32：先打通Vicon刚体与RViz纯观测

后续更新：`car01`已建立，真实刚体话题`/vicon/car01/car01`和`/car/pose`均约330Hz，
新接收器已按用户确认重启，RViz模型/箭头已显示。详见
[真实接入结果与下一步](CAR01_LIVE_OBSERVE_2026-10-12.md)。下方保留最初的准备流程，
其中“未建立/未重启/未打开窗口”是此前快照；不要重复启动当前已运行的接收器和观测入口。

## 当前路线与边界

用户确认Tracker可操作、小车刚体尚未建立。ESP32出现异常发热，暂时完全退出系统，
不再尝试通过USB、COM或另一块驱动板给它供电。保留已有STM32固件、轮序、CPR和PI成果。
本阶段整车可以完全断电：电池拔掉、两块驱动板关闭，电机插头不用恢复。
**不烧录、不复位、不启动电机、不修改热点、代理、虚拟网卡或IP配置。**

当前数据链路只做观测：

```text
手动移动车架 → Vicon相机/Tracker刚体解算 → Linux vicon_receiver
            → 刚体PoseStamped → car_state_bridge → /car/pose、/car/odometry、TF
            → RViz小车模型和 /car/path_actual 实际轨迹
```

这不需要ESP32或STM32参与，不代表无线控制或实车轨迹跟踪已完成。

## 1. 在Tracker建立小车刚体（现在先做这项）

1. 将反光点固定在刚性车架/刚性支架上，不能固定到轮子、小滚子或会相对车架活动的部件。
   建议用5个以上、分布开且不对称的点，避免全部排成直线和易混淆的对称布局。
   **Tracker 4创建对象至少选择4个marker，官方建议5个以上**，不是此前简化说明中的3个。
2. 把断电小车放进已标定的动捕区域，保持静止。在Tracker的3D视图确认这些点都可见。
   若看不到未标记点，检查View Filters中的Unlabeled Markers与Processing中的
   Enable Unlabeled Reconstructions；不重新标定实验室相机。
3. 在Tracker 4中按Ctrl逐个选择小车的点（或Alt框选，避免选到其它物体），
   在Tracking面板Object名称栏填写`car01`，点击Create，随后保存对象。
   不同版本的布局可能略有区别，以当前版本界面为准。
4. 缓慢平移并转动车架，确认同一个刚体持续被识别，点没有相对车架移动。
   记录实际主体/segment名称。原点位置和车头轴不一定自动正确，后续单独核对；
   **不要为对齐小车而修改实验室全局原点或相机标定**。

依据：[Vicon Tracker 4.0官方用户手册，Create objects章节](https://help.vicon.com/download/attachments/14189359/Vicon%20Tracker%20User%20Guide.pdf)。
少量marker暂时遮挡时能否稳定跟踪，要在实际布置中验证，不能由点数直接保证。

## 2. Tracker到Linux的连接（最新只读检查已通）

本轮早期检查有线口尚未连接，后续只读复查发现已有连接：Linux有线口UP，地址
`10.10.10.1/8`；`10.10.10.2`两次ping均成功。手机热点与TUN仍在，未修改任何网络配置。
本机已有接收进程使用`hostname:=10.10.10.2`，`/vicon/frame_number`约329.6Hz更新，
说明Vicon帧流已经到达Linux；还读到帧号131490及SDK总延迟约3.90ms（单样本，不是完整时延统计）。
发现`/vicon/MicoAir/MicoAir`话题，但两次位姿读取（含BEST_EFFORT订阅）均超时，
不能把已有话题当作有效刚体位姿，更不能把它当小车的位姿。

现有接收进程未被停止或重启；其运行映像已被本次构建替换，须在确认不影响其它实验后，
协调重启一次才能确保新的遮挡过滤生效。**不要再并行启动第二个同配置接收器。**
不切换手机热点，不调整代理/TUN；后续若必须改变网络配置，先说明具体改动再确认。

接收器与纯观测入口的终端使用相同ROS域。不要同时启动软件在环、控制launch或硬件桥，
不要把隔离域72中的合成测试数据当作真实Vicon。每个终端先加载：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
```

以下仅供接收器未运行或已协调停止旧实例时启动；本次现有进程保持不动：

```bash
VICON_HOST='10.10.10.2'  # 本次已核实可达；后续每次接入仍核对实际地址
ros2 launch vicon_receiver client.launch.py \
  hostname:="$VICON_HOST" topic_namespace:=vicon \
  world_frame:=map vicon_frame:=vicon stream_mode:=ServerPush publish_latency:=true
```

终端B使用`ros2 topic list -t`找到小车实际的`geometry_msgs/msg/PoseStamped`话题。
主体可能为`car01`，segment不假设叫`base`。单个marker的`PointStamped`不能替代刚体位姿。

```bash
CAR_POSE_TOPIC='/vicon/car01/填写实际segment'
ros2 topic type "$CAR_POSE_TOPIC"
ros2 topic echo "$CAR_POSE_TOPIC" --once
ros2 topic hz "$CAR_POSE_TOPIC"
```

确认首帧`frame_id: map`、数值有限、四元数非零、时间持续更新。
接收器已做mm→m及map←vicon变换，不重复除1000、不仅改frame_id伪装坐标变换。
若实际实验室坐标轴需要变换，先核实，再配置接收器的map_xyz/map_rpy。
时间戳是Linux接收/处理时刻，不是相机曝光时刻；SDK延迟另看`/vicon/latency`。

## 3. 启动真正的纯观测入口

终端B停止频率检查后，使用刚才确认的实际话题：

```bash
ros2 launch mecanum_vicon_control vicon_observe.launch.py \
  vicon_pose_topic:="$CAR_POSE_TOPIC"
```

本入口只启动模型、位姿桥、轨迹显示和RViz，不加载控制器、看门狗控制链、目标发生器、
UDP/串口桥或使能节点；RViz也没有发目标的工具。模型使用60mm轮径、122mm轴距、170mm轮距，
属于参数化示意模型，不是精确机械CAD。TF为`map → base_footprint → base_link`，避免重复父节点。
这一步轮子模型不会反映真实编码器转速。

观测输出：

- `/car/pose`：车体平面位置与航向；`/car/odometry`：位姿及差分速度。
- `/car/path_actual`：最近的实际轨迹；不是预先规划的矩形。
- `/car/vicon_valid`：有效且新鲜的位姿标志。超过200ms无有效数据时变为false。

可在第三终端查看`ros2 topic echo /car/vicon_valid`。
手动慢移车架约0.3–0.5m、慢转约90°，核对模型、箭头和实际轨迹是否对应。
再短暂遮挡/移出覆盖区，核对有效标志变false、恢复后重新接受新鲜数据。
RViz可能保留最后的模型/轨迹，**显示留在原处不代表仍在跟踪**。
新构建的接收器在源SDK报告失败或刚体遮挡时跳过本帧，不给无效值换新时间戳继续发布；
当前已有进程尚未重启，不能假定它已经加载本次新构建。

车体原点/航向偏差需测量后才设置base_offset_x_m、base_offset_y_m、base_offset_yaw_rad，
偏移x/y按刚体局部坐标解释。不要为让图形看起来正确而猜值，或改全局实验室坐标。

## 软件验证与尚未完成部分

- 164项ROS包单元测试通过；两个小车ROS包及vicon_receiver构建通过。
- 隔离ROS域72、DDS仅本机的合成运行通过9项检查：有效位姿、四类坏数据拒收、
  丢流失效、恢复、无控制发布者、TF单父节点。记录见`vicon_observe_synthetic_20261012_02.json`。
- 合成测试没有访问真实Vicon、串口或电机；没有开启实际RViz窗口进行视觉验收。
- 另行只读检查确认实际Vicon帧号约329.6Hz更新，未操作硬件或改变网络。
- **小车真实刚体创建、位姿频率/坐标/遮挡验证与RViz现场验收仍待进行。**

纯观测完成后，再选择恢复通信方式。若先用USB-TTL，只安排短距离台架维护，
当前`4WB1`仍是受限前进固件，不能靠插USB-TTL直接运行四向6m×5m矩形。
无线大范围运动需替换/修复通信硬件、通用受限固件及停机验收后再推进；不重复已完成的逐轮标定。
