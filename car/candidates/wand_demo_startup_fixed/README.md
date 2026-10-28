# 原实车平移演示：启动零输出时序修复

独立主机候选。只替换 `auto_prepare.py` 的启动等待判定及阶段显示，运动算法、
安全门/监督器、MC-Link、ESP32/STM32、网络配置均不修改。沿用0.70m/s、0.80m/s²、
1:1平移、前0.5/后1.5/左0.5/右0.4米活动区及四边50cm缓冲、120秒。
不是A2平移+旋转仿真的2m/s²或12rad/s²实机迁移。

实际故障日志：`ready_v4/oneclick_session_9rsmnc5n`。首个零输出安全门状态为
COMMAND_TIMEOUT；旧自动准备立即结束本轮，而4.38s后后台已ready_to_start。
新流程不清这个锁存，只在从未使能、无准备epoch、零速且无历史停车/硬故障时等待。
来源、定位、时钟、停稳、通信均新鲜，且传输层确认持续零基线/命令进展，连续0.3s后
请求一次准备。旧安全门仍须通过新disabled ARMED epoch自行核验并清除初始可恢复锁存。
等待最多30s；准备交接最多5s。其他故障立即取消，成功准备后再超时不得走启动等待。
从不请求recover/acknowledge/enable；必须新的真实空格单击才能运动。

旧脚本、旧快捷命令及全部已保存仿真清单互相固定引用，因此原字节不动，保留回退。
本次新统一入口：

```bash
小车演示
```

或 `bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_demo_startup_fixed.sh`。
旧 `小车单击` 与旧 `start_car_demo_120s_toggle_eventio.sh` 仍是旧版，不再用于本次演示。
1280×720；勾选现场安全条件后自动检查和准备，READY后单击空格开始；再次空格/Esc停车。
松键不停车，失焦/遮挡/实际故障/围栏/120s保护保留。修复不自动启动任何实车进程。
