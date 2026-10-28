# Sign真实标定杆只读预览

2026-10-22接入真实Vicon，整车保持断电。此阶段只接收刚体位姿，显示2倍位移映射目标，不产生电机指令。

## 当前入口与可视化

- 真实杆刚体：/vicon/Sign/Sign，PoseStamped，世界坐标系map、单位m。
- 断电小车刚体：/vicon/car01/car01。
- 杆的五个真实标记点：/vicon/Sign/markers/Sign1 至 Sign5。
- 只读入口：mecanum_wand_control 的 wand_preview.launch.py，ROS域0。
- 图形与目标全部在 /wand_preview；没有速度、控制使能或故障清除发布接口，也没有调用实车服务的客户端。
- 绿色球为真实杆的五点；蓝色方块/箭头为断电小车；白色箭头为启动时固定前向；橙色球与线为2倍映射目标。

车头修正沿用现有矩形入口的+135度。本轮没有重新进行车体中心或朝向精标；用户观察方向仍需和实体车头核对。标定杆的角度不作为小车旋转指令，但杆绕手转动时刚体参考点的真实平移仍会反映到目标。

## 已验证的真实数据

初始10秒独立订阅：Sign与car01均约329.7Hz，四元数无效样本0，最大接收间隔约5.5ms。Vicon SDK报告平均延迟约4.29ms，这不是已测量的整条控制链延迟。

预览启动后的首10秒：Sign有效3269帧、约329.5Hz；静止XYZ标准差约0.0266/0.0238/0.0169mm，五点可见。这是当前静止条件的重复性，不代表绝对定位精度。运行中的最新频率和最大间隔以summary.json为准，接收器/预览调度可能丢弃被更新替代的旧消息。

初始独立订阅器用源时间戳匹配238组Sign原始位姿与预览目标，核对目标位移等于2倍杆位移：拟合比例1.9999999999992397，数值误差小于2e-16m。该数值只表明映射运算一致，不代表实车精度。随后已记录用户真实大范围手持运动，结果见下节。

运行图检查：/car/cmd_vel、/car/cmd_vel_raw、/car/enable、/car/control_enable、/car/run_lease、/car/clear_faults、/car/trajectory_enable 的发布者均为0。此前硬件程序72个文件SHA256仍与原清单一致。

新包累计61项测试通过，包括新增14项只读映射、失效后重新锚定、位姿校验、真实点图形序列化、发布接口及launch隔离契约测试。构建通过。

## 真实手持运动验收：通过

用户已完成运动，动作顺序与距离为自由手动，不要求严格沿预定顺序。分析快照截至预览开始后755.06秒，其中有效映射15079组；主要运动发生在约528.4至551.0秒。

| 指标 | 实测 |
| --- | --- |
| 独立按世界位移拟合比例 | 2.000000 |
| 杆最大水平位移模长 | 1.140685m |
| 目标最大水平位移模长 | 2.281370m |
| 杆前后方向范围（固定启动系） | -0.709018至1.044430m |
| 杆左右方向范围（固定启动系） | -0.841627至0.538154m |
| 五点均可见的20Hz有效预览样本比例 | 100% |
| 中断、跳变或重新锚定状态 | 未发生，只有启动WAITING后持续PREVIEW |
| 拟合数值最大残差 | 6.3e-16m，仅反映运算一致性 |

运动后的杆相对最初原点仍约为前0.04976m、左0.03119m，模长0.05873m；目标相应为前0.09952m、左0.06238m，模长0.11746m。这是手动收回没有严格回到最初记录点的剩余输入，符合映射预期，不能解释为小车回位误差。

两路姿态有效记录没有非法或倒序样本，最大应用层接收间隔约38.9ms，未达到0.20秒失效门限。20Hz记录可能看不到更短暂的单点遮挡，因此本轮不等于专门的遮挡失效测试；故意遮挡后的隐藏与重新锚定仍只在软件测试中覆盖。

动作后再次检查七个实车运动/使能话题，发布者仍全部为0。原点只捕获一次，未因动作或返回自动漂移。用户观察方向大致符合要求；尚未用外部量具进行距离或车头方向精标。

本次真实输入已超过1m，2倍目标也超过2m。当前只读预览允许完整显示；将来迁移至跟随控制时须沿用并核对已有2m目标范围限制，不能直接把自由挥动都当作可执行动作。本轮不修改限制、不启动实车。

结果图与数据：

- wand_sign_preview_2026-10-22/real_sign_movement.png：实际杆与2倍目标的轨迹、前后/左右时序。
- wand_sign_preview_2026-10-22/movement_acceptance.json：固定分析快照和通过条件。
- wand_sign_preview_2026-10-22/movement_snapshot.csv：与本次报告对应的有效记录快照。

复现分析：

    /usr/bin/python3 tools/analyze_sign_preview.py --record-dir docs/wand_sign_preview_2026-10-22

## 复现

已运行接收器时直接开启预览，不能重复启动接收器：

    cd /home/PRIMARY_USER/vicon_mecanum_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch mecanum_wand_control wand_preview.launch.py

如接收器尚未启动，在另一个终端执行：

    source /opt/ros/jazzy/setup.bash
    source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
    ROS_DOMAIN_ID=0 ros2 launch vicon_receiver client.launch.py hostname:=10.10.10.2 topic_namespace:=vicon buffer_size:=200 world_frame:=map vicon_frame:=vicon stream_mode:=ServerPush publish_latency:=true

两路位姿有效持续1秒后只自动锚定一次。手持杆进入合适起始姿势后，可以重新记录原点：

    ROS_DOMAIN_ID=0 ros2 service call /wand_preview/capture_origin std_srvs/srv/Trigger '{}'

此服务只影响图形，不产生运动。暂停预览：

    ROS_DOMAIN_ID=0 ros2 service call /wand_preview/pause_preview std_srvs/srv/Trigger '{}'

## 人工观察步骤

小车保持断电，杆先保持稳定姿态。沿实体车头方向移动约0.5m、停2秒、收回；再向左移动约0.25m、停2秒并收回。橙色目标应分别前移约1m、左移约0.5m并返回，蓝色小车仍留在实测原位。

如果拿起或遮挡杆导致跟踪中断超过0.20秒，目标会隐藏，显示RECAPTURE_REQUIRED。等待杆与车恢复可见后显式capture_origin，再从新原点继续；不会自动追赶丢失前的旧目标。

## 记录

本次运行目录：docs/wand_sign_preview_2026-10-22。

- mapping.csv：20Hz杆相对位移、目标及可见点数。
- summary.json：每5秒更新的有效帧、频率、最大间隔、初始静止段与状态变化。
- initial_acceptance.json：独立订阅与无硬件发布者的首次验收快照。
- movement_acceptance.json 与 real_sign_movement.png：后续真实手持运动的验收结果。

本阶段未测试实车动作，也未改变Vicon标定、网络、固件或原有矩形控制程序。
