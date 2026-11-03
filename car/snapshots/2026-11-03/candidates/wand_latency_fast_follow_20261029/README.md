# 延迟测量与快速手势跟随：独立离线候选

2026-10-29。仅新增本目录、独立文件核验脚本及报告。不修改原演示、矩形、Vicon接收器、固件或Tenda；没有ROS初始化、设备端口、使能/控制发布、刷写或实车启动入口。

## 复用与新增

- 既有实现：已保存4WS8连续伺服、1:1平移/相对旋转、无固定围栏策略、原监督器和独立末级门、400RPM轮速包络、既有一阶软件车模型。
- 新增离线工具：`measure.py`严格区分记录曲线的等效滞后与同画面可见起转测量；输出输入哈希、相关性、时钟边界、采样间断及视频帧区间。不跨定位丢失插值，不用反馈年龄当运动延迟。
- 新增离线算法：`FastCatchupSession`只改变平移速度目标生成。参考位置、20cm领先约束、源时间戳、独立安全门、正常速度/加速度/轮速限制原样复用。
- 本次没有硬件验收。两段旧实车输入驱动的新车体响应是模型预测，不是把旧车反馈重新标为新算法实测。

## 策略

1. 原有20cm参考领先保护触发时，仅在原始目标和受控参考方向一致、杆速度前馈新鲜且大于0.55m/s、杆正在沿该方向远离小车时，保留相应速度前馈，避免直接退回 `1.4/s × 20cm` 的慢追赶。
2. 杆已经稳定约120ms、新鲜样本持续到达、参考与原始目标方向一致时，可以采用有界位置追赶；进入10cm邻域后退出。速度目标仍受0.70m/s、位置距离及加速能力约束，不预测或外推标定杆位置。
3. 杆收回/换向、参考方向不一致、样本过旧时不使用上述前馈补偿；原有耦合SI加速度整形和400RPM包络仍只应用一次。

保持0.70m/s、0.80m/s²、0.10rad/s、0.40rad/s²。没有移植尚未刷写的4WSA高旋转参数，也没有借此修改120秒会话或任何超时。超过现有运动能力的手势仍不可能严格同步；优化不是取消保护或提高硬件许可。

`catchup.py`没有覆盖 `advance_reference`、原始映射、`observe`、监督器或末级门。安全与性能必须分别判定：模型保持ACTIVE/限制未超仅表示功能保护测试通过，不表示跟随误差已经满足实车展示要求。

## 文件核验

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/check_wand_latency_fast_follow.sh --check
```

默认也是文件核验；`--launch`明确拒绝，不会构造ROS节点或通信端口。此脚本不能启动小车或真实Sign仿真界面。真实Sign接入、实际ROS签名/源绑定链路及实车均待下一阶段验收。

## 重跑离线测试

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_latency_fast_follow_20261029
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v test_candidate.py
PYTHONDONTWRITEBYTECODE=1 python3 benchmark.py --output /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_latency_fast_follow_2026-10-29/new_comparison
```

输出目录必须不存在。16组轨迹×3种同条件软件车模型×基线/候选，共96次模型运行。模型滞后0.06/0.12/0.20秒是既有模型的不确定性扫描，不是根据110ms记录滞后反推出的实车动力学参数。控制器/末级门为20ms虚拟步长演练，不是完整ROS100Hz调度性能认证。

比较同一时刻位置误差，不平移时间来美化新算法误差。旧实录规范化杆/车原点位置，保留原世界方向的XY位移、相对转角、车的起始朝向、源间隔与源年龄；不把旧车反馈作为新模型的响应。Z不驱动，模型杆高固定1m，此回放不是原始六自由度/遮挡验收。报告中的“3cm到位时间”是软件模型指标，不是物理制动或实时性认证。

## 重算已有记录的滞后

```bash
PYTHONDONTWRITEBYTECODE=1 python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_latency_fast_follow_20261029/measure.py logs \
  --input /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_feedback_300ms_startup_2026-10-27/prepared/oneclick_session_jiqg5pw1/observation.jsonl \
  --seconds 60 \
  --output /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_latency_fast_follow_2026-10-29/new_timing
```

输入采用既有演示已经自动记录的文件，不新增ROS订阅或影响演示负载。发布端时间戳至控制候选只表示电脑侧年龄，不是相机曝光时刻。50/100/200ms同条件导数窗口结果单列；低相关性/搜索边界不能当固定延迟保证。

## 物理可见起转计时

现场步骤与30/60fps精度限制见[测量流程](/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_latency_fast_follow_20261029/VIDEO_PROCEDURE.md)。模板不包含任何实测结果，默认全部未确认。分析器只读取原视频哈希与人工帧标注，不播放、生成或插补视频帧。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_latency_fast_follow_20261029/measure.py video \
  --video /absolute/path/original_video.mp4 \
  --metadata /absolute/path/verified_metadata.json \
  --annotations /absolute/path/annotations.csv \
  --output /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_latency_fast_follow_2026-10-29/new_video_timing
```

这不是通电命令。真实试验前还需确认当次设备、定位、悬空固定/场地和立即断电条件。首次可见起转不是电气起转时刻；普通视频无法证明毫秒级电气延迟。原演示仍按原入口运行，本候选没有替换它。
