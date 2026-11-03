# 低延迟连续伺服候选（仅离线）

本目录不是 STM32 固件，不修改原矩形、已验收操作台或网络。
固定原起点/朝向的3×3米目标区、车体外各50厘米缓冲、额外20厘米余量和1:1 XY映射保持。
速度0.20m/s、加速度0.08m/s²、偏航0.10rad/s；没有借此提高实车动力许可。

`continuous_servo.py`在原跟随器上移除80ms位置低通和半加速度慢参考，
保留参考限速/20cm领先限制，最终速度命令只做一次正常运动加速度整形。
用40ms源时间戳回归估计速度；不预测位姿、不续期旧帧，60ms无新样本便撤掉速度前馈。
运动时弱位置纠偏、停稳时位置收敛，避免换向前过度追赶；参考被领先限制挡住时恢复正常追赶。
原监督器状态机和独立FinalGate保持，只有跟随器选择、源时间戳传入和翻译速度钩子改变。

`build_candidate.py`机械生成独立包；要求新输出目录，复制前后核对旧源哈希，核对末级门源码不变。
仅导出离线验证入口，禁止hardware/emulator模式；没有实车启动或刷写入口。
域112、合成输入域113，未启动这些域中的任何节点。

最终验证记录见：`/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/README.md`。

仅离线重跑：

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/run_wand_servo_offline_checks.sh
```

结果不能代替真实Vicon端到端延迟、车辆动力学、停车、电流或温升验收。
