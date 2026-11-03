# 3 m x 2 m Vicon连续矩形与任意边回轨

## 当前实现

- 硬件入口：`vicon_rectangle_loop_recovery_hardware.launch.py`
- 下一次刷写后的固件身份：`4WGB`（当前板上仍为已锁存故障的`4WGA`）
- 速度上限：`0.15 m/s`
- 路径：从启动时实测位姿锚定，前进3 m、左移2 m、后退3 m、右移2 m；车头保持不变。
- `loop_count=0`，轨迹不会因圈数自行结束。
- 路径只保留四个角点；每个角点保持0.80 s零速驻留，让轮组惯性和齿隙先稳定再换向。
- 每圈结束后，监督器关闭ROS速度门和STM32运动门，确认至少500 ms静止零输出，
  安全重装填STM32单次运行额度，再沿同一条已锚定矩形继续。不会逐圈重设原点。
- 当前边使用投影、前视和横向误差闭环。受到干扰时保持当前边，不会跳到矩形对边。
- 横向误差达到0.08 m进入`RECOVERY`，降至0.03 m以内并稳定0.50 s后恢复正常跟踪。
- 连续矩形专用的最大允许恢复偏差为0.65 m；超过该值进入`HOLD`并由监督器停车，不宣称可从任意大偏差恢复。

## 电脑端命令

先在一个终端启动默认失能的控制栈：

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mecanum_vicon_control vicon_rectangle_loop_recovery_hardware.launch.py
```

在另一个已加载相同环境的终端启动连续运动：

```bash
ros2 service call /car/start_loop std_srvs/srv/Trigger '{}'
```

正常停止命令：

```bash
ros2 service call /car/stop std_srvs/srv/Trigger '{}'
```

查询监督状态：

```bash
ros2 topic echo --once /car/loop_supervisor_status --field data --full-length
```

停止服务会同时请求：轨迹失能、ROS速度门失能、STM32运动门失能。返回
`success=True`只表示监督器已接收请求；随后应以`STANDBY`、`permitted=0`和四路零输出
作为最终停止证据。

## 仍然保留的自动停车条件

“连续运行”不绕过安全边界。以下任一情况仍会自动停车，并要求重新执行启动服务：

- Vicon位姿超过0.20 s没有新鲜有效数据；
- ESP32/STM32链路丢失、命令不推进或STM32报告故障；
- 横向偏差超过0.65 m，路径控制器进入`HOLD`；
- 单圈超过140 s，避免撞上STM32独立150 s运行截止；
- 连续监督器进程退出，20 Hz运行租约在0.30 s内过期；
- 操作者调用`/car/stop`、停止整个launch或切断动力电源。

STM32仍保留600 ms命令租约、150 s单次运行截止和真实故障不可自动清除等独立保护。

## 验证状态

- Python/ROS测试：274项通过。
- ROS包构建：通过。
- 4WGB的12套原生状态机测试、ARM交叉编译、刷写及独立读回：通过。
- 实机失能启动：确认服务存在、`STANDBY`、`4WGA`、故障位0、`permitted=0`、四路输出0。
- 首圈完整运行和圈间自动重装填已通过实车验证。
- 一次随机干扰已进入`RECOVERY`、回到原轨迹并继续运行。
- 4WGA历史测试随后在航点换段附近锁存
  `FOUR_LINK_REJECTED + FOUR_SPEED_FAULT_FEEDBACK`，故障轮为RR；主机因此增加0.80 s角点
  零速驻留。该真实故障已由本次断电刷写过程复位，且固件仍禁止由软件清除真实故障。
- 4WGB已完成备份、刷写、独立读回和上电静态验收；0.15 m/s三秒落地直线净位移
  0.3863 m，全程故障0并正常归零。新四角连续矩形及随机干扰仍待验证。
