# Vicon路径扰动恢复：离线软件准备记录（2026-10-21）

## 结论

独立的连续路径恢复控制器、确定性扰动仿真、Vicon断流锁止、RViz标记和量化工具已完成。
本阶段只使用隔离ROS域中的合成位姿，没有启动`cmd_vel_udp_bridge`、Vicon客户端或
`rectangle_run_guard`，没有发送真实电机命令，也没有修改STM32、ESP32或网络配置。

现有`pose_controller`、5m×4m硬件launch和4WG8固件入口均保持不变。新功能使用独立节点与
独立launch，后续迁移实车时可以回退到现有矩形控制而无需恢复源文件。

## 新数据流

```text
trajectory_generator -> /car/goal_pose + /car/trajectory_active
                                         |
simulated Vicon -> car_state_bridge -> /car/pose
                                         |
                                         v
                              path_recovery_controller
                                         |
                                  /car/cmd_vel_raw
                                         |
                                  safety_watchdog
                                         |
                                    /car/cmd_vel
                                         |
                               simulated_vicon_car only
```

控制器只跟踪轨迹发生器指定的当前活动线段。它不会搜索全局最近矩形边，因此受较大扰动后
不会错误跳到对面的平行边。

## 控制行为

- 将小车当前位置投影到当前有向线段，计算有符号横向误差和沿线进度。
- 正常状态下保持沿线速度，同时修正小横向误差。
- 横向误差达到`0.08m`时进入`RECOVERY`，将沿线速度限制为`0.03m/s`，优先侧移回线。
- 误差回到`0.03m`以内并持续`0.50s`后恢复`TRACKING`。
- 偏差超过`0.45m`进入锁止`HOLD`，不自行追赶远处轨迹。
- Vicon位姿超过`0.20s`未更新时进入锁止`VICON_LOST`；位姿恢复不会解除锁止。
- 总平移速度按矢量模长限幅，不是分别限制x/y后允许对角线超限。
- 轨迹末端使用沿线误差减速；超过终点时会反向回到终点。

## 确定性ROS仿真结果

配置：第一条直线、固定车头、最高`0.10m/s`，在沿线约`0.35m`处瞬时施加世界坐标
`+0.20m`横向扰动。

| 指标 | 结果 |
| --- | ---: |
| 峰值横向误差 | 0.2000 m |
| 返回`TRACKING`用时 | 3.0697 s |
| 稳定时横向误差 | 0.01513 m |
| 最大命令平移速度模长 | 0.1000 m/s |
| Vicon断流后控制器状态 | `VICON_LOST` |
| 位姿恢复是否自动继续 | 否 |
| 硬件桥是否加载 | 否 |
| 真实电机命令 | 否 |

随后从执行器中移除仿真位姿源。ROS安全看门狗在日志中测得位姿年龄约`0.211s`时锁止，
恢复仿真位姿后控制器和安全看门狗均保持零输出，符合实车失位姿后必须重新启动受控流程的原则。

证据：

- `docs/disturbance_recovery_offline_20261021.json`
- `docs/disturbance_recovery_offline_20261021.csv`
- `docs/disturbance_recovery_offline_20261021.png`

SHA-256：

```text
316574eac914a302ff39cb4655f0bba8a1625a041d3ecf7247204faf5a7e75b1  disturbance_recovery_offline_20261021.json
d02b99ab59ea3dffe2e8e5489642af48e762abb7816618e56d3214039444dd7a  disturbance_recovery_offline_20261021.csv
33e0f127a9da3c6fb4d2adffd63c50b1b7c6cc88db0f932008cc3223674f4191  disturbance_recovery_offline_20261021.png
```

## 构建与回归

- `mecanum_description`和`mecanum_vicon_control`构建成功。
- `mecanum_vicon_control`完整Python/ROS测试：`241 passed`。
- 新增专项测试覆盖投影符号、投影端点、退回超调终点、恢复滞回、总速度限幅、ROS跨话题
  首目标/使能乱序、可视化标记和硬件隔离。
- 独立launch实际启动全部9个软件节点并以单次SIGINT干净退出。

## 运行方式

自动验收，不打开GUI：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
cd /home/PRIMARY_USER/vicon_mecanum_ws
python3 tools/verify_disturbance_recovery.py \
  --ros-domain-id 96 \
  --output /tmp/vicon_disturbance_recovery.json
```

交互式仿真与RViz：

```bash
ROS_DOMAIN_ID=95 ros2 launch mecanum_vicon_control \
  disturbance_recovery_sil.launch.py start_rviz:=true
```

小车进入第一条直线后，从相同ROS域的另一个终端施加一次20cm仿真横向扰动：

```bash
ROS_DOMAIN_ID=95 ros2 topic pub --once /car/sim_disturbance \
  geometry_msgs/msg/Pose2D '{x: 0.0, y: 0.20, theta: 0.0}'
```

RViz中蓝色为完整计划路径、绿色为实际路径、蓝色球为当前投影点、黄色球为前视目标、
红色箭头和`RECOVERY`文字表示正在返回路径。

## 实车迁移入口

已新增`vicon_recovery_line_hardware.launch.py`作为首次实车迁移入口。它只定义1m直线，
默认限速`0.10m/s`，当前精确要求`4WG9`；ROS安全门、STM32运动门和轨迹发生器均从失能
状态启动。启动launch本身不会使车轮运动，仍必须单独运行一次性安全守卫才能进入
运动状态。

该入口复用已验收的Vicon桥接、Tenda/UDP桥接和安全看门狗。4WG8完成第一次实车直线后
暴露出同次上电不能再次运行的问题，因此当前入口已切换到独立4WG9安全重装填固件；
ESP32无需修改，STM32需在完整读回备份后刷写一次4WG9。

## 首次实车运动门槛

本记录不是硬件运行许可。首次运动仍必须按以下顺序通过：

1. 精确要求4WG9固件身份；刷写前完整读回并核对当前板内镜像。
2. 保留现有`rectangle_run_guard`的预检、一次运行和最终失能原则；4WG9在健康静止状态
   下先安全重装填额度。
3. 第一次只运行1m直线，以`0.10m/s`无扰动验证方向、停车和最终失能。
4. 第二次仍运行1m直线，人工轻推横向约`0.15m`。
5. 确认Vicon连续、故障位为0、没有错误换边、恢复后误差小于`0.05m`。
6. 通过低速验收后才允许`0.20m/s`矩形扰动演示。

2026-10-21只读现场预检时，Vicon `car01`位姿为约`329.6Hz`且正常更新；Tenda
`wlx000000000000`已被USB系统识别，但当时状态为`DOWN/NO-CARRIER`、无`192.168.50.x`
地址。因此未启动实车launch或安全守卫，更没有发送真实电机命令。

## 首次实车迁移结果

2026-10-21恢复已有Tenda专用连接后，`192.168.50.1`三次ping均成功；未修改密码、
静态地址或主Wi-Fi。启动失能态实车链路后，确认固件为`4WG8`、故障位为0，且
PWM、目标转速与车体速度命令均为0。

一次性安全守卫随后执行1m无扰动直线，运动约`11.154s`。Vicon计划起点为
`(0.64753,-1.07481)m`，终点为`(0.72472,-0.07780)m`，实际最终位姿为
`(0.72120,-0.09511)m`，终点平移误差约`0.01767m`。运动结束后两级门均失能，
四轮指令、占空比与比较值全部归零，实车launch已干净退出。详细数据见
`docs/vicon_recovery_hardware_run1_20261021.json`。

随后在不重新上电的情况下请求第二次运行，车轮动作前出现`0x00020000`。源码核对确认
这是4WG8正常结束后保留`motion_started/run_used`所触发的`FOUR_LINK_REJECTED`，不是
摩擦力不足、编码器异常或真实电机故障。4WG9离线候选及其安全边界见
`docs/FOUR_LINK_GROUND9_SAFE_REARM_OFFLINE_2026-10-21.md`；尚未刷写和实车验收。

## 当前限制

- 仿真模型是带方向增益的一阶速度模型，不代表真实轮胎、滚子、地面摩擦和1.4kg车体动力学。
- 控制器现在跟踪分段直线，角点仍由原航点发生器逐点推进；这正适合当前固定车头矩形。
- `0.08/0.03m`恢复阈值和增益是安全离线初值，必须由低速实车数据确认，不能直接提高。
- 故障后不自动重新使能是有意的安全设计，不作为待修复缺陷。
