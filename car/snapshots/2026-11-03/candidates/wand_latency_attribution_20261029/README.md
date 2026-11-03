# 延迟归因工具（只读诊断，不是控制入口）

只读取已关闭的真实Sign→软件车会话。严格以源摘要/源时间/控制周期连接，验证原始包摘要，不跨100ms源间断插值；低相关、搜索边界或缺乏动作时不宣称固定延迟。JSON结果明确分开实测时间差、曲线等效滞后和既有模型常数。

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_latency_attribution_20261029/analyze.py \
  --session /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/real_sign_5gzsjthu \
  --output /absolute/new/output_directory
```

输出目录必须不存在。生成JSON和同时间轨迹图，不进行时间平移美化。图中的软件车不是实际car01运动。

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_latency_attribution_20261029
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest -v test_analysis.py
```

`sdk_probe.py`仅订阅既有唯一、已核验接收器的`/vicon/latency`和`/vicon/frame_number`10秒，不改参数、不重连SDK、不启动控制。SDK Total边界按API契约解释，不当作本项目完整端到端实测；当前采样不能与旧会话按不同时间窗口直接相加。

当前并未实现新的逐帧末级门/签名接收/UI绘制埋点，也没有新实车或仿真使能入口。缺少的测点列在[报告](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_latency_attribution_2026-10-29/REPORT.md)，不得假装已完成完整闭链计时。
