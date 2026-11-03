# 独立高同步参数仿真

默认仅仿真：0.70 m/s、4.00 m/s²、400 RPM运动学包络，1:1位移映射。
真实Sign输入；car01只是只读参照；运动反馈来自签名软件车。
不含实车通信桥，不更改原矩形、本机演示、固件或Tenda配置。

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_wand_sync_tuning_sim.sh --check
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_wand_sync_tuning_sim.sh --launch
```

打开后未使能：拿稳Sign→准备→真正READY→单击空格。
再次空格或Esc停止；松键不停车；失去焦点、遮挡和异常仍撤权。
保护不放宽：位姿200ms，指令/验签/UI100ms，租约150ms，反馈300ms，最长120秒。

## 证据边界

- 参数模型扫描：880个名义模型对照完成；0.70/4.00是本动作集合的近平台候选，不是全局最优或硬件能力声明。
- 22项单元/签名/UI/IPC/静态结果复用/报告归因测试通过。
- 15项实际ROS故障注入回归、独立门定时器停摆注入通过。
- 最终代码60秒实际ROS合成往返＋真实Sign接收负载通过，原始/IPC拒绝均为0。
- 用户此前真实Sign试验在约12.51秒因门状态迟到中断；已保留失败记录，不能算验收通过。隔离与计算消重修复后，仍须新的真实Sign人工复验。
- 慢执行器模型部分未能在原1.50秒内确认停稳；现有硬件的加速度、制动、电流和温升仍未验收。

最新复盘：`/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_sync_tuning_2026-10-29/interruption_fix_01/REPORT.md`。
准备清单：`/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_sync_tuning_2026-10-29/PREPARED.json`。

## 时间与计算约束

域0只读接收在独立spawn进程中。IPC仅一个最新原始帧槽，不替代控制同源FIFO。
先有界接收、按实际接收时钟校验，再采Source时钟；不对原始帧重新打时间戳。
两个控制消费者分别验证新包；同一不可变包只复用几何解/digest，每次访问仍检查六路源时间和接收年龄。
10ms控制/独立门、25ms界面快照、持久Canvas；硬故障不自动清除或恢复。
所有时序是本机软件链路测量，不是相机曝光到真实车轮起转的物理延迟。
