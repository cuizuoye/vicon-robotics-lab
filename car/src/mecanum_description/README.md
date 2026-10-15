# mecanum_description

这个包只负责小车的“几何外观和坐标系”，不包含运动控制。

## 包内容

- `urdf/mecanum_car.urdf.xacro`：参数化小车模型，含底盘、四轮、Vicon刚体坐标系和 K230D 安装位占位。
- `launch/display.launch.py`：生成 `robot_description`，并启动 `robot_state_publisher`、`joint_state_publisher` 和 RViz2。
- `rviz/mecanum_car.rviz`：预置小车模型、TF、目标点、实际轨迹和目标参考线。

## 目前参数的可信度

- 已知：麦克纳姆轮直径 `0.060 m`，轮宽 `0.0306 m`。
- 占位：底盘长、宽、高，前后轮中心距，左右轮中心距，K230D 安装位置。
- 仅用于 RViz：当前质量和惯量不可用于 Gazebo/物理仿真。

硬件到达后可通过启动参数修改外形，无需重写模型：

```bash
ros2 launch mecanum_description display.launch.py \
  chassis_length:=0.24 chassis_width:=0.20 \
  wheelbase:=0.18 track_width:=0.16
```

