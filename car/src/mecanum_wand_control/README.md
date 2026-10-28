# Vicon 标定杆相对位移控制：离线版本

已确认的约定：杆收回时车也返回；水平位移比例 2；前后左右以每次启用时的车头方向为准；T形五点杆作为一个刚体输入。第一版不映射高度和杆朝向，车头保持启用时的朝向。

本包包含离线仿真与真实Vicon只读预览。没有ESP32/STM32通信入口，不加载任何硬件launch。仿真使用 /wand_sim 与隔离域97/98/99；真实预览在域0订阅Vicon，仅在 /wand_preview 发布图形与目标位姿。已有矩形程序和固件文件没有改动。

## 映射定义

启动时记录车位姿 c0、杆位姿 w0、车头角 yaw0。

    d = R(-yaw0) * (wand.xy - w0.xy)
    goal.xy = c0.xy + R(yaw0) * 2 * d
    goal.yaw = yaw0

其中 d 是固定起步车体系的前/左位移；不能随杆或车当前转角不断重定义。比例在两轴相同时，上式等于世界坐标中位移同向放大2倍；不能再把世界位移多旋转一次。杆回到 w0 时目标回到 c0。

杆位姿表示 Vicon 刚体中预先选定的参考点；不是五点各自独立下指令。实际杆的参考点原点和手握位置偏移，需在之后接真实 Vicon 的预览阶段确认。本包的五个绿色点只是示意布局，不代表真实杆尺寸。

## 构建与重复验证

在工作区执行：

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    colcon build --base-paths src --packages-select mecanum_wand_control --symlink-install
    source install/setup.bash
    ros2 run mecanum_wand_control wand_offline_suite --output /tmp/wand-results
    ros2 run mecanum_wand_control wand_ros_smoke --output /tmp/wand-ros-results

第一项运行虚拟时间测试，输出 JSON、CSV 和对照图；第二项自行启停隔离域98的真实 ROS 节点，验证消息链路和服务。无需启动硬件栈。

## 看96秒自动仿真演示

    ros2 launch mecanum_wand_control wand_sim.launch.py

演示从车头60度、非零世界位置启动：杆前移0.5m/车前移1m并返回，杆左移0.25m/车左移0.5m并返回，最后斜移并返回。绿色五点是模拟杆，橙色球是目标，蓝色车与轨迹是仿真响应。演示结束后自动暂停。

不打开 RViz 时追加 start_rviz:=false。只启动待机仿真、由外部离线输入驱动时追加 scripted:=false。

在另一个终端加载环境后，只能用以下仿真服务：

    ROS_DOMAIN_ID=97 ros2 service call /wand_sim/pause std_srvs/srv/Trigger '{}'
    ROS_DOMAIN_ID=97 ros2 service call /wand_sim/engage std_srvs/srv/Trigger '{}'
    ROS_DOMAIN_ID=97 ros2 service call /wand_sim/stop std_srvs/srv/Trigger '{}'

pause/stop立即置零并丢弃未完成目标；engage要求两路新鲜有效位姿，重新捕获两侧实测原点。对自动演示暂停后如需完整从头重播，请退出并重启 launch。

## 本版边界

- 水平合速度0.15m/s，合加速度0.30m/s²；不是对两个轴分别放宽到0.15。
- 输入位姿过期0.20s、非法姿态/坐标系、杆跳变超过0.25m、目标离锚点超过2m时锁存停车。
- 重复/倒序时间戳不续期，旧时间戳不能通过重新发布掩盖丢跟踪。
- 数据重新出现不会自行恢复。显式 engage 才重新对齐，避免追赶旧目标。
- 输入2mm死区、80ms一阶滤波；到位容差15mm。其余抖动参数仍需真实杆静止数据再标定。
- 仿真复用现有 simulated_vicon_car；虚拟时间模型使用同样的0.12s惯性和0.98/0.88平移增益。没有模拟轮地摩擦、质量、打滑、遮挡重建细节和无线链路。
- 本节离线结果不表示真实T形杆或实车遥操作已经验证。后续真实只读与软件车联调结果分别见下文；实车遥操作仍未迁移。

## 本次验证结果（2026-10-22）

- 新包47项测试通过，包括ROS消息序列化和四组闭环场景；原控制包275项测试通过。
- 虚拟时间四组场景：前后往返、90度起步横移往返、60度起步斜移往返、受扰后回位再返回；到位误差约11至14mm，均在25mm验收范围内。
- 实际ROS节点链路：90度起步，杆前移0.5m对应目标1m，小车到约0.99m；杆收回后回起点误差约12mm。速度上限0.15m/s。
- 完整96秒自动演示通过：状态STANDBY→ACTIVE→PAUSED，无故障，最终返回误差11.29mm。
- 已验证暂停重新对齐、杆数据中断、冻结源时间戳、非法四元数、恢复数据后不自动启动。
- 逐一比较已有控制包72个文件的SHA256，内容未改变。
- 可视化使用独立10Hz定时器，控制维持50Hz；无人订阅时不构造绘图点数组。源时间戳允许5ms以内的小幅时钟偏差并钳制到接收时刻，明显未来帧与过期帧仍拒绝。
- 详细数据在工作区 docs/wand_offline_2026-10-22 和 docs/wand_ros_2026-10-22；测试结果说明见 docs/WAND_OFFLINE_VALIDATION_2026-10-22.md。

参考逻辑：现有Vicon接收器的PoseStamped刚体接口，以及 [LeRobot的离合/重对齐机制](https://github.com/huggingface/lerobot/blob/5517a326eeb9b457cfe07d59b9bfdf291799c8d9/docs/source/isaac_teleop.mdx)。只借鉴设计，不引入机械臂或XR运行依赖。

## 真实Sign标定杆只读预览

小车保持断电。已有Vicon接收器须在ROS域0发布 /vicon/Sign/Sign、/vicon/car01/car01 和Sign1至Sign5的标记点。接收器已运行时，不要启动第二份。

    ros2 launch mecanum_wand_control wand_preview.launch.py

两路有效位姿持续1秒后自动记录杆和车原点。绿色五点来自真实杆，蓝色方块/箭头是断电小车实测位置与朝向，白色箭头是固定启动前向，橙色球与曲线是2倍映射的预期目标。这里没有模拟小车追踪，更没有实车动作。

车头修正沿用当前矩形入口的+135度参数，并非本轮重新做了方向精标。杆参考点是Vicon的Sign刚体原点，旋转杆可能使此点绕握持位置运动；不会把杆的角度直接变成车的转向。

重新记录当前位置为起点（只影响图形）：

    ROS_DOMAIN_ID=0 ros2 service call /wand_preview/capture_origin std_srvs/srv/Trigger '{}'

暂停图形目标更新：

    ROS_DOMAIN_ID=0 ros2 service call /wand_preview/pause_preview std_srvs/srv/Trigger '{}'

遮挡/无效/超过0.20秒无新鲜位姿或突然跳变时，橙色目标隐藏，状态变为RECAPTURE_REQUIRED；恢复跟踪后显式capture_origin才恢复目标更新。不得自动把重现的位置当成原来的指令。

记录目录在启动日志中给出，可用record_dir:=一个尚不存在的绝对目录指定。mapping.csv记录杆位移与目标，summary.json每5秒刷新频率、间隔、首10秒静态候选抖动及2倍映射核对。首10秒若发生移动，标准差只代表这一段运动分布，不应称为测量噪声。时间戳年龄只衡量本机接收器发布到回调的时间，不是完整Vicon端到端延迟。

2026-10-22真实手持运动已通过只读验收：15079组有效映射，拟合比例2.000000；杆最大水平位移1.140685m，对应目标2.281370m。未触发失效或跳变状态，采样中五点均可见，实车运动/使能发布者仍为0。手动返回后约5.9cm杆偏移对应约11.7cm目标偏移；这不是实车跟随误差。详细报告见工作区docs/SIGN_READONLY_PREVIEW_2026-10-22.md。

## 真实Sign驱动仿真车（实车继续断电）

    ros2 launch mecanum_wand_control sign_live_sim.launch.py

该入口通过两个独立ROS上下文从域0只订阅真实Sign与断电car01参考，再单向转发到域99。源时间戳不变；域99虚拟车按实体车参考初始化并自动启用一次。没有域0运动发布者或硬件通信节点。2倍映射、0.15m/s合速度上限、2m目标半径、暂停/遮挡后显式重新对齐均沿用已有仿真逻辑。

仿真暂停/恢复/停止服务位于ROS_DOMAIN_ID=99下的 /wand_sim/pause、/wand_sim/engage、/wand_sim/stop。切勿将这些仿真服务当成实车启动命令。默认启动后即可用真实杆控制；若只想等待，追加auto_engage:=false。

新增单向桥测试后包内共68项测试通过。真实输入、虚拟扰动和仿真侧输入阻断的自动验收通过，实测合速度0.15m/s、输入中断约0.226秒停车锁存。

真实手持和物理遮挡也已完成：非人工阻断情况下，杆帧年龄0.209秒触发FAULT和零指令；恢复可见后保持停止，显式engage重新对齐后无位置跳变。手持时虚拟车受限速约束会落后于快速移动的目标；静止目标自动测试到位误差约12mm。实车继续断电，域0七个运动/使能话题发布者均为0。完整记录与操作见工作区docs/SIGN_LIVE_SIMULATION_2026-10-22.md。

## 独立安全链路：离线监督器与末级门

新入口`wand_safe_sim.launch.py`只运行域96、`/wand_safe_sim`下的专用监督器、安全门和软件车，不改变上面的旧域99入口或现有矩形程序。

- 默认失能；显式启动、独立末级握手、新鲜持续使能才能运动。松开或失联停车，不自动恢复。
- 固定场地围栏检查目标和整个旋转车身；原始高度/倾斜、位姿跳变、双时基过期和模式/控制源冲突停车。
- 确认零输出与持续静止后才宣布停稳；缺失反馈保持STOP_UNCONFIRMED。故障恢复需确认并重新启动，真故障/急停不可自动清除。
- 末级执行使用每次launch随机密钥的签名、序号和有效期；普通Twist仅用于观察，不能直接驱动软件车。
- 初始配置2倍映射、合速度0.05m/s、加速度0.08m/s²、航向0.10rad/s、30秒会话上限。围栏和高度是仿真值，尚不是实车标定。

    ros2 run mecanum_wand_control wand_safe_suite --output /tmp/wand-safe-virtual-new
    ros2 run mecanum_wand_control wand_safe_ros_smoke --output /tmp/wand-safe-ros-new

输出目录须尚不存在。31组虚拟时间场景、真实ROS功能/伪造发布/进程暂停故障注入已通过；该阶段本包136项和原控制包275项测试共411项通过。详见工作区`docs/WAND_SAFETY_OFFLINE_2026-10-22.md`。当前没有真实Sign安全适配器，也没有硬件控制入口；实车继续断电。

## 不需要实物杆的安全操作台

新增鼠标虚拟杆与持续键盘使能操作台，沿用上节的独立监督器、末级门和软件车。它不订阅Vicon，也不会伪造域0的真实Sign数据。

    cd /home/PRIMARY_USER/vicon_mecanum_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch mecanum_wand_control wand_safe_console.launch.py

不要同时运行另一份域96安全仿真或操作端；控制源冲突会触发停车。

1. 如果提示“先轻按并松开”，先在本窗口轻按并松开一次空格以确认松键；输入解锁后点击“准备（不运动）”，等待顶部提示“准备就绪”。
2. 按住空格并拖动右侧虚拟杆：蓝色仿真车追踪橙色目标，位移2倍，合速度上限0.05m/s。
3. 松开空格、切出窗口或按暂停/停止，都会释放使能。等待PAUSED与停车确认；再次运动须重新准备，再按住空格。
4. 可恢复故障须先消除原因、松开空格、等待停车确认、点击“确认可恢复故障”，再重新准备。仿真急停锁存，需退出重启此独立仿真。

每次重新启用均捕获当时的杆和车位置；不是继续追赶上一次旧目标。虚拟杆回中不是停车键。输入最大半径0.245m，活动会话内也限制相对捕获点的移动范围；30秒会话上限沿用安全配置，到时会停止而非无限运行。窗口关闭时先释放使能和请求停止，再结束本次仿真。

界面阻塞超过0.10秒会锁止本地使能；后端仍有独立使能租约超时。新鲜反馈不可用时不把未知状态显示成“已停稳”。界面按钮的请求已接收不等于已停车。

2026-10-22后续修复：输入锁止时禁用准备按钮；顶部新增持续更新的当前步骤，区分“等待握手”“松键解锁”“准备就绪”“等待停车”。如果在握手前提前按空格，保持ARMED但不使能；松开后重新按住即可，不需要再点击已禁用的准备按钮。终端`UI_STATE`记录状态/锁止/焦点变化，便于复查没有运动的原因。回归新增启动焦点切换和实际ttk鼠标绑定，不再只通过直接调用按钮回调验收。

新增12项输入/隔离测试后，共423项回归通过。临时独立X显示器内的Tk键鼠事件注入与真实ROS软件节点联调通过，覆盖早按、按键重复、释放、失焦、界面冻结和关闭中停车；不是实车或人工键盘验收。最终记录、截图和边界见工作区`docs/WAND_OPERATOR_CONSOLE_2026-10-22.md`。

真实Sign和实车迁移入口另置于`mecanum_wand_hardware`，默认仍为真实Sign驱动软件车。旧离线包不导入该实机包或UDP桥；默认仿真域96/保护不变。新的显式验收入口、现场配置、回环MC-Link验证和待验收项见`docs/WAND_HARDWARE_PREPARATION_2026-10-22.md`及独立包README，不要将旧虚拟杆操作台当成实车入口。
