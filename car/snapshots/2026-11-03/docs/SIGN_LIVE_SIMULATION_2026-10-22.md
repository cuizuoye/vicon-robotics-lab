# 真实Sign标定杆驱动仿真小车

本阶段仅驱动二维软件小车，实车保持断电。真实Vicon接收器与原只读预览继续运行，旧矩形程序、硬件通信和固件未修改。

## 数据链路与映射

域0的sign_sim_source只订阅 /vicon/Sign/Sign 与 /vicon/car01/car01，不发布任何运动目标或控制指令。源位姿保持原时间戳，经单向桥送入域99的 /wand_sim/wand_pose；不会用重复旧位姿更新时间来掩盖跟踪中断。

域99只运行仿真车、已有位置控制器、桥接节点和RViz。仿真车按断电实体车的当前位置与朝向初始化，沿用矩形入口+135度刚体轴修正。仿真世界与map明确使用相同原点/坐标轴，属于显式恒等变换。

每次启用时记录杆与仿真车当前位置，保持起步朝向。杆的水平相对位移放大2倍，杆收回则目标收回。位置滤波80ms、杆死区2mm、到位容差15mm继续沿用；杆高度与姿态不直接指挥车移动或转向。

## 边界与恢复

- 平移合速度上限0.15m/s，合加速度上限0.30m/s²。
- 目标半径2m，对应杆相对本次起点半径1m。超范围锁存停止，不暗中改变2倍比例。
- 任一路控制位姿超过0.20秒未更新，输出归零并进入FAULT。
- 源重新可见不会自动恢复。显式engage重新捕获当前杆与虚拟车原点，取消旧目标，避免追赶。
- pause/stop立即发出仿真零输出；模型仍按0.12s惯性逐渐停下，不能把指令归零等同于模型瞬间静止。
- 首次初始化可自动启用一次；后续故障/暂停均须显式恢复。

## 已完成自动验收

在真实Sign输入保持静止时，仅给仿真车注入位置扰动，并仅在仿真侧阻断输入，不改变真实Vicon或只读预览：

| 项目 | 结果 |
| --- | --- |
| 位移比例 | 2倍 |
| 实测最大合速度指令 | 0.150000m/s |
| 虚拟位置扰动后到目标误差 | 12.23mm |
| 暂停时输出 | 零 |
| 暂停后重新对齐漂移 | 0.0018mm |
| 仿真侧输入中断到停车锁存 | 0.226s |
| 重新收到数据 | 保持FAULT，不自动追赶 |
| 显式恢复位置漂移 | 0.026mm |
| 实际域七个运动/使能话题发布者 | 全部0 |

68项包测试通过。跨域桥只将实时杆输入按约100Hz处理、静止小车参考按20Hz处理，再以50Hz转发最新帧；控制输入队列仅保留1帧。保留0.20秒门限，并记录两路源年龄和最大控制周期，便于区分真实遮挡、仿真反馈与调度延迟。首次接入曾出现未细分输入超时，失败记录保留在sign_live_sim_2026-10-22，修正后的验收和实时记录位于sign_live_sim_2026-10-22_checked。

以上自动输入中断是仿真侧注入；下列人工测试使用真实遮挡，两者证据分别保留。

## 真实手持与物理遮挡验收

用户已完成真实Sign平移和遮挡，再重新露出杆。本次桥接记录中人工测试全程synthetic_occlusion=false，真实源出现多次失去新鲜帧，前两段持续约1.45秒和1.42秒。

| 项目 | 实测结果 |
| --- | --- |
| 真实杆驱动虚拟车 | 有响应；相对起点最大位移约0.567m |
| 手持运动期间最大合速度指令 | 0.150000m/s |
| 首次真实遮挡触发锁存时杆帧年龄 | 0.209163s |
| 同时虚拟车反馈年龄 | 0.010073s，并非车反馈超时 |
| 锁存期间速度指令 | 始终为0 |
| 真实跟踪恢复可见后 | 仍保持FAULT，没有自动追赶旧目标 |
| 显式恢复 | ACTIVE，当前杆与虚拟车重新对齐 |
| 本次恢复前后位置差/新目标误差 | 记录中均为0m，无跳变 |
| 实车域运动/使能话题发布者复查 | 七项全部0 |

本次实际杆移动较快，滤波后的最大相对水平位移约0.951m，对应约1.901m目标位移；虚拟车受0.15m/s限速约束，未在持续移动的杆后立即到位。因此本次验证真实输入响应、限速及遮挡恢复门控，不把瞬时追踪滞后当成静止到位精度结论。静止目标的到位误差12.23mm来自前面的自动验收。

约0.21秒指从最后新鲜杆位姿到控制器FAULT/速度指令归零，不代表有惯性的虚拟车或真实车辆在同一瞬间完全静止。后续可见性短暂恢复不解除锁存；人工恢复前约170秒内一直保持FAULT和零指令。

这次真实遮挡后已经显式engage，仿真恢复为ACTIVE；实车继续保持断电。恢复会把当前杆位置和虚拟车位置作为新原点，不延续遮挡前未完成的目标。

## 使用命令

已有域0 Vicon接收器时启动，勿重复运行域99仿真：

    cd /home/PRIMARY_USER/vicon_mecanum_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch mecanum_wand_control sign_live_sim.launch.py

窗口标有REAL Sign -> SIM CAR。蓝色是移动的仿真车，橙色是目标，半透明灰色是保持断电的实体车参考。绿色五点仅是杆的姿态示意，真实五点仍可在原只读预览中观察。

仅控制仿真：

    ROS_DOMAIN_ID=99 ros2 service call /wand_sim/pause std_srvs/srv/Trigger '{}'
    ROS_DOMAIN_ID=99 ros2 service call /wand_sim/engage std_srvs/srv/Trigger '{}'
    ROS_DOMAIN_ID=99 ros2 service call /wand_sim/stop std_srvs/srv/Trigger '{}'

专供自动验收的仿真侧输入阻断（不停止接收器、不修改Vicon）：

    ROS_DOMAIN_ID=99 ros2 service call /wand_sim/simulate_occlusion std_srvs/srv/SetBool '{data: true}'
    ROS_DOMAIN_ID=99 ros2 service call /wand_sim/simulate_occlusion std_srvs/srv/SetBool '{data: false}'

人工测试时不要使用该服务代替物理遮挡。实际遮挡恢复后，要执行engage才能恢复仿真跟随。原域0只读预览若进入RECAPTURE_REQUIRED，可单独capture_origin恢复图形，两者不是同一个启停门。

## 证据

- sign_live_sim_2026-10-22_checked/automatic_acceptance.json：真实输入与虚拟扰动/输入阻断验收。
- 同目录summary.json与live_simulation.csv：桥接与控制状态连续记录。
- 同目录physical_occlusion_acceptance.json：真实手持及物理遮挡分析。
- 同目录physical_resume_acceptance.json：显式恢复时的虚拟车/目标位姿订阅验收。
- 同目录physical_occlusion_validation.png：手持输入、虚拟车响应、限速与遮挡锁存曲线。橙色目标根据记录的滤波杆位移重建，不是独立订阅的目标位姿。
- 同目录live_observer：较晚启动的只订阅记录，覆盖锁存后的等待和显式恢复，不包含此前的手持及遮挡发生过程；发生过程以桥接CSV为准。
- 同目录hardware_isolation_final.json：本次结束前域0七个运动/使能话题发布者复查。

重复自动验收只作用于正在运行的仿真：

    /usr/bin/python3 tools/verify_sign_live_sim.py --output /tmp/sign-sim-acceptance.json

自动验收结束会把虚拟车放回初始参考，并保持ACTIVE等待杆输入。人工记录离线重分析不连接ROS，也不发命令：

    /usr/bin/python3 tools/analyze_sign_physical_occlusion.py docs/sign_live_sim_2026-10-22_checked

本阶段完成真实Vicon输入到软件车的验证，不是实车运动、无线通信或实车制动验收。原控制包72个源文件/配置/测试文件哈希不变，新增包68项测试再次通过。
