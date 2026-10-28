# 标定杆同步性：速度/加速度匹配，SIM ONLY

本目录只做纯软件、虚拟时钟对照。不会启动 ROS、Vicon SDK、操作台、串口、UDP、
固件烧录或实车。实验参数不写入任何已有实车/仿真操作台配置。

采用组合式 `SimOnlyLimits`：复用并校验原 `ServoSafetyConfig` 的哈希及全部安全/围栏参数，
仅在本进程的模型实验里覆盖枚举的速度和加速度。原配置构造器依然拒绝这些较高参数。
连续伺服、监督器、独立 FinalGate 直接复用原实现，不修改源码。
没有硬件模式、IP、节点/通信启动接口；产物不是硬件配置文件。

## 结论入口

[2026-10-23 对照报告](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_servo_matching_sim_2026-10-23/REPORT.md)

- 最小平顺手势档：0.50 m/s，1.20 m/s²。
- 扩展手势仿真推荐档：0.70 m/s，2.00 m/s²，覆盖本次峰值0.75 m/s的平顺往返挑战。
- 所有参数都是**平面合速度/合加速度**，不是每个方向各自允许这么大。
- 这不是实车速度、制动、电流、温升、通信链路或用户体感验收。

## 重现（不需要连接硬件或启动 ROS）

使用全新的输出目录，拒绝覆盖证据：

```bash
python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_servo_matching_sim/matching_sim.py \
  --output /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_servo_matching_sim_2026-10-23/comparison_repeat

python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_servo_matching_sim/analyze_matching.py \
  --comparison /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_servo_matching_sim_2026-10-23/comparison_repeat \
  --output /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_servo_matching_sim_2026-10-23/extended_review_repeat

python3 -m pytest -q /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_servo_matching_sim/test_matching_sim.py
```

需要 Python 3、pytest、matplotlib；不需要 `rclpy.init()` 或联网。2D图片生成时当前环境可能提示
无法导入 Axes3D，该可选3D扩展不参与本试验，2D结果已实际查看。

## 对照设计

25组参数：速度[0.20,0.35,0.50,0.70,1.00] m/s × 加速度[0.08,0.30,0.60,1.20,2.00] m/s²。
每组9条轨迹：4条平顺合成轨迹、3段真实Sign记录中心回放、2段原角落起点回放。
1.00 m/s仅作模型上界参照，不是对现有硬件许可或性能的判断。

回放使用原ACTIVE控制周期选择的真实位姿，保留周期、源时间戳和源年龄，
重复时间戳不刷新，不做插值、时间缩放、补帧或跨遮挡拼接。
只平移软件车的初始位置；原杆XY位移与朝向不缩放、不旋转，旧车反馈不参与新模型。
固定3m×3m目标区和四边50cm缓冲不移动；角落限幅单列，不作为同步改善。

误差是未做时间平移的同一时刻XY距离，同时报告全部样本、手移动样本、P95、峰值和换向延迟。
换向延迟为平顺往返目标速度过零后，模型车前向速度首次换号的时间差。
“passed”仅表示模型运行没有意外停机且保留输出/几何限制，**不表示该档同步性达标**。

松键段是每条轨迹结束后额外施加的“新鲜静止杆＋释放使能”合成夹具，
不冒充真实记录中的操作或遮挡恢复。故障恢复由独立注入测试验证，
此前现场尚未操作的手动恢复步骤仍然尚未现场验收。

保留原200ms位姿超时、使能租约、命令/反馈超时、固定整车足迹围栏、源/owner/epoch/序号、
末级速度/加速度/参考连续性检查、抬车/倾斜、会话超时、故障后不自动恢复等保护。
这里没有运行原ROS签名输出链，不把纯FinalGate测试冒充完整ROS/实车通信验收。

## 新增：真实Sign驱动0.70/2.00仿真（2026-10-23）

上述“没有运行ROS链路”针对原纯虚拟时钟扫描。后续独立入口现已准备：

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_wand_servo_matching_real_sign_sim.sh
```

只读接收域0的真实`/vicon/Sign/Sign`，独立域115、命名空间`/wand_servo_matching_sim`；
只允许软件车/软件反馈。中心起点(1.5,1.5)，固定围栏不动，1:1 XY，合速度0.70m/s、合加速度2.00m/s²。
新`matching_wire.py`使用独立SIM ONLY签名范围，新旧通道互拒，原0.20m/s校验器不改。
软件车使用与纯模型对照相同的0.12s滞后、0.98/0.88/0.95增益模型，未经硬件辨识。
UI绘制门限由100ms降低到33ms，真实周期仍取决于20ms事件循环和绘制耗时；不影响使能/失焦/超时保护。

初次打开不准备、不使能。必须由用户点击准备、等待就绪并持续按住空格。
松键/失焦停车，遮挡后不自动恢复；需释放空格、确认并重新准备、等待就绪、重新按住。
本入口与旧实时仿真互斥，不停止已有接收器/矩形进程；有冲突时拒绝启动。

[本次实时验收状态与证据](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_servo_matching_sim_2026-10-23/LIVE_ACCEPTANCE.md)

