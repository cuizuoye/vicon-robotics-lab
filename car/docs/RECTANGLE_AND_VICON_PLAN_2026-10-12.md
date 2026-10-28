# 6m×5m矩形与Vicon接入（2026-10-12）

最新补充：car01刚体已建立，Linux/RViz纯观测已接通。ESP32仍停用；用户改为先做
STM32脱离电脑的1m×1m单圈演示，使用板载KEY启动，编码器估算距离，Vicon仅记录真值。
独立新固件已编译及离线回归，并在用户确认无动力后完成ST-Link备份、烧录、读回和运行零输出检查；
随后KEY启动尝试已完成倒计时，但首段590ms触发RR无进度保护，未完成运动验收；
动力断开后已备份并换为首段最多5秒的启动记录版；记录版试转也未成功，四轮均未动，
800ms无反馈触发全停，启动过程记录已保存。暂不重复试转，先区分供电/公共地与启动曲线原因。
地面矩形尚未进行，见
[STM32独立矩形演示说明](STM32_KEY_RECTANGLE_DEMO_2026-10-12.md)。
以下“刚体未建立”、6m×5m和无线方案均为此前阶段记录。

后续状态更新：车载供电试验发现ESP32单独供电也异常发热，已停止使用。
Tracker可操作，但小车刚体尚未建立。暂不推进无线落地运动，先按
[无ESP32的Vicon纯观测流程](NO_ESP32_VICON_OBSERVE_2026-10-12.md)建立刚体、接收位姿和显示轨迹。
以下保留矩形软件成果与此前分阶段计划。

## 目标与已完成部分

保持车头朝向不变，按起始车体坐标走：前6m → 左5m → 后6m → 右5m。
这是麦轮平移演示，不是在每个角点转90°后继续向前。
相对起点的四角为`(0,0) → (6,0) → (6,5) → (0,5) → (0,0)`，一圈22m。
路径按首次有效起始位姿的前/左方向变换到map，不强迫起点位于Vicon世界原点。
运行期间路径固定，不随小车当前位置漂移；`loops=0`表示循环到操作者取消。

本轮完成纯软件实现与验证，**没有烧录、复位STM32或ESP32，没有发送UDP/电机运动指令**：

- 新增固定航向、起点对齐、等待新鲜位姿、循环航点；保留旧圆/矩形/8字的默认切线航向。
- 128项ROS包单元测试通过，两个ROS包构建通过。
- 第一次软件演示在启动阶段发生位姿/指令超时，模型被安全看门狗锁止，未完成轨迹。
  原记录保留为`docs/rectangle_6x5_ros_sim_20261012.json`。
- 新演示入口默认不启动控制/轨迹；待话题稳定后，在隔离域72中明确启动一次仿真。
  未放宽200ms软件看门狗或任何硬件保护，也不自动解除中途锁止。
- 真实ROS2软件在环完成一圈并进入下一圈：16105帧模拟位姿、8066帧指令被观察；
  四段速度方向正确，角速度始终0，运动阶段约96.88秒，回到起点时误差约0.01970m。
  该误差来自模拟模型和2cm到达阈值，**不是Vicon精度或实车误差**。
- 演示已停止，没有遗留仿真控制进程。

## 目前不能直接跑实车大矩形的原因

1. 用户确认STM32与ESP32都由电脑供电；必须先完成车载逻辑供电，不能带着电脑USB、
   USB-TTL或ST-Link拖线跑6m×5m。
2. STM32当前反馈为`4WB1`受限无线测试镜像，只支持一次`vx=50mm/s, vy=0, wz=0`，
   10秒上限、10%输出上限；不支持无线侧移/后退/连续矩形。
   不能把ROS2轨迹启动命令当成实车固件已经支持这些运动。
3. 今天5秒零速无线往返通过，但延长观察约5.23秒后触发150ms链路超时。
   根因尚未定位；暂不优化手机热点不等于跳过停机保护。
4. Vicon刚体尚未建立，Vicon PC尚未与Linux连接。没有绝对位姿就不能验证6m/5m距离，
   仅用“速度×时间”或麦轮编码器会受打滑影响。

## 软件演示入口（不会连接ESP32）

终端A，使用与实车不同的ROS域，DDS仅在本机发现，不改变网卡或代理：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
export ROS_DOMAIN_ID=72
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch mecanum_vicon_control rectangle_demo.launch.py
```

RViz显示模型、实际轨迹、目标和矩形计划路径；默认保持未启动。
新终端B加载同一环境和相同两个ROS环境变量，先观察位姿/控制流稳定更新：

```bash
ros2 topic hz /car/pose
# 观察几秒，Ctrl+C结束这个频率检查；再检查：
ros2 topic hz /car/cmd_vel_raw
```

随后只在这个隔离仿真域中明确启动：

```bash
ros2 topic pub --once /car/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /car/trajectory_enable std_msgs/msg/Bool '{data: true}'
```

停止模型与自动目标推进：

```bash
ros2 topic pub --once /car/enable std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /car/trajectory_enable std_msgs/msg/Bool '{data: false}'
```

`trajectory_enable=false`只取消继续发航点，不能单独作为紧急停车。
上述仿真启动指令不是实车操作授权。此launch不加载`cmd_vel_udp_bridge`。

## 接下来按顺序

### A. 脱离电脑的逻辑供电

用户已确认STM32开发板有3.3V和5V丝印；目前实际使用USB-TTL的3.3V给它供电。
3.3V电源入口已有使用记录，但仅有5V丝印不能确认其输入用途、稳压路径或USB防回灌。
本阶段仍保持动力断开；没有实施车载供电改线。

2026-10-12查阅厂商手册第3.3节及D157B原理图后确认：

- D157B允许用稳压输出给单片机供电，DCP3512不是必需组件。
- D157B的3.3V来自RT9013-33GB（芯片规格500mA），三个3.3V插针共用同一路电源，
  不是三路独立500mA。板内还有负载和热限制，不能把芯片标称电流当作外接负载保证值。
- D157B电源开关同时切断电机电源及其5V/3.3V稳压电路的输入。
- 用户明确选择不使用DCP3512，按D157B自带稳压输出完成车载逻辑供电；
  因而关闭第一块D157B时，STM32和ESP32也会断电，这一取舍已写入上电检查流程。

建议供电结构（待逐根接线确认，不是已完成状态）：

```text
3S电池配电点
  +-- 第一块D157B Vin（前轮板）
  |     +-- 3.3V稳压输出 -> STM32现用的3.3V电源入口
  |     +-- 5V稳压输出 -> ESP32-S3开发板的5V/5Vin电源入口
  +-- 第二块D157B Vin（后轮板）：不使用它的5V/3.3V给主控供电
所有电源负端/GND共地；UART、控制、编码器信号接线保持。
```

ESP32资料所示开发板有5V到3.3V的板载稳压电路；外接5V时先断开两个USB口。
D157B给STM32供电使用之前已有使用记录的3.3V入口，不使用尚未确认的STM32板5V入口；
ESP32用5V输入，不用D157B同一路3.3V同时承担STM32和ESP32的负载。
输出有余量和启动稳定性还须通过本次装机检查验证，不能把之前空载测量当作负载验证。

接线时所有USB/USB-TTL供电和电池先断开，USB-TTL与ST-Link暂时从STM32断开。
不并联两块D157B的5V/3.3V输出，也不再连接DCP3512。
**电池电压不能直接接任何GPIO、3V3或5V脚，稳压5V也不能接STM32的3.3V入口。**
第一次供电检查先拔下两块板的全部四个电机六针插头；第二板保持关闭，只开第一板给主控供电。
这时电机和编码器均物理断开，无需发使能或运动指令。
零输出和无线状态检查通过后，再安排完全断电下恢复电机插头、四轮悬空验收。
电机恢复接入后，先开第一板等待主控启动、零状态确认，再开第二板；停机先关第二板、再关第一板。
不得单独切断主控电源却让另一板的5V编码器信号仍接到未供电的STM32。
软件上电零输出不能保证初始化前没有瞬态，首次带电机上电仍须悬空、现场观察，异常立即断电。
供电稳定后，按B项进行持续零速链路和受限常规运动，不直接启动6m×5m实车轨迹。
具体接线与分阶段确认见`docs/DRIVER_ONLY_POWER_2026-10-12.md`。

核对依据（厂商本地资料）：

- `下载/【WHEELTEC】AT8236驱动模块附送资料/【WHEELTEC】AT8236驱动模块附送资料_V2.8_2026.07.25/1.用户手册与教程视频/电机驱动模块使用手册—AT8236-2025.08.27.pdf`
- 同资料的`5.原理图/2.AT8236稳压模块原理图（D157B）.pdf`
- 同资料的`6.芯片手册/RT9013-33GB.PDF`
- `下载/ESP32-S3 NXRX开发板资料/ESP32-S3 NXRX开发板资料/开发资料/原理图.pdf`

### B. 短距离常规无线落地运动

先完成持续零速通信与新通用四轮速度固件。复用现有轮序、CPR、方向和初始PI，
不重复逐轮标定；新增受限vx/vy/wz、斜坡、显式使能、失联停机、故障锁存和最长运行时限。
独立新目标必须离线回归、断动力备份/烧录、零输出检查，再悬空验收。
现有四轮电机供电和控制参数不因想跑大矩形就自动提高；负载启动不足须用数据评估。

落地先一次短直行、确认停止，再验证侧移/后退，预计位移仅20–30cm，现场可随时断动力。
短动作不用Vicon也可进行，但不能因此称6m×5m距离闭环已完成。
不通过反复重启MCU绕过现有单次额度；不在超时/故障后自动重试。

### C. 同步准备Vicon（不要等跑完大矩形才接入）

1. 将marker固定在刚性车架上，不装在车轮或能相对车架活动的部件上。
   Tracker 4创建对象至少选择4个marker，官方建议5个以上；使用分布开、不共线、
   不对称的固定点，实际遮挡表现仍需现场验证。
2. 在Tracker中创建并保存小车刚体，建议主体名`car01`，实际segment名字以Tracker与ROS话题为准，
   不假设一定叫`base`；记录原点到车体中心的平面偏移、车头轴方向。
3. 用有线连接Tracker电脑与Linux；先核实实际IP/子网再配置。
   本轮只发现Linux有线口尚未连接，没有修改它、Wi-Fi、代理或虚拟网卡。
   历史`10.10.10.2`仅为旧Tracker配置，不能未经核实当作当前IP。
4. 启动现有接收器，使用Live数据；下面的`<Tracker实际IP>`必须替换：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
ros2 launch vicon_receiver client.launch.py \
  hostname:=<Tracker实际IP> topic_namespace:=vicon \
  world_frame:=map vicon_frame:=vicon stream_mode:=ServerPush publish_latency:=true
```

在同一ROS域中用`ros2 topic list`找到小车的真实刚体PoseStamped话题，
再检查类型、频率、frame_id和yaw。控制使用刚体位姿，不使用任意单个marker坐标。
现有接收器已将mm转成m、经map←vicon变换后发布；不要再重复除1000或仅改frame_id。
接收器时间戳为本地接收/处理时刻，不等于Vicon曝光时间；延迟另看SDK latency。

保持动力断开，先手动平移、旋转和短暂遮挡小车：核对RViz轨迹、车头方向、
丢位姿时输出零速及不会自动恢复使能。还要沿计划四角检查实际动捕覆盖；
房间能放下矩形不等于Vicon覆盖整个矩形。

5. 当前优先使用`vicon_observe.launch.py`，完全不加载控制/目标/轨迹发生或硬件桥。
   下面较早的矩形预览入口会加载默认禁用的控制节点，**不是纯观测入口**：

```bash
ros2 launch mecanum_vicon_control rectangle_demo.launch.py \
  mode:=vicon_preview vicon_pose_topic:=<实际刚体PoseStamped话题>
```

这个终端必须与真实Vicon接收器使用相同ROS_DOMAIN_ID；**不要把隔离仿真域72的模型
与真实Vicon话题、硬件桥混在一起**。收到实际位姿后路径按该起点对齐。
首次闭环实车应设置`forward_m:=0.6 left_m:=0.5 loops:=1`，并由实车控制层设置运行时限；
完成误差/停机验收后才放大到6m×5m并开放多圈。

## 关键文件与证据

- `src/mecanum_vicon_control/launch/rectangle_demo.launch.py`
- `src/mecanum_vicon_control/mecanum_vicon_control/trajectory_generator.py`
- `tools/rectangle_ros_observe.py`：只订阅、记录，拒绝非隔离域运行。
- `docs/rectangle_6x5_ros_sim_manualstart_20261012.json`：本次完整一圈模拟记录。
- `docs/FOUR_LINK_RECONNECT_2026-10-12.md`：当前无线结果与固件限制。

本计划最初记录时尚未实施车载逻辑供电改线。后续供电试验已进行，但ESP32异常导致该路线暂停；
通用无线运动固件、实际落地运动、真实Vicon连接或实车矩形仍未实施。
