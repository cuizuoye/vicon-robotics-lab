# 4WSC 档0落地初验交付

2026-11-02，独立快照 `prepared_v3` 已通过离线交付检查。当前允许的用途是**档0、单轮最多60秒的受控落地初验**，不是已完成停车、电气和温升审核的600秒演示。

## 最新实车观察

用户已完成一轮档0落地演示并反馈“演示正常”。日志实际ACTIVE约28.452秒，由 `CLIENT_STOP_BUTTON` 人工结束，末级输出归零并确认停稳；`completed:false` 表示未跑到60秒，不是软件故障，不修改原始结果。50ms窗口的Vicon观测峰值平移速度约0.217m/s，停止请求后的观测轨迹约4.0mm；这些观测不是高档性能、电气/温升或600ms失联停车包络证明。

用户随后要求接入高同步档并延长至600秒。现有4WSC固件和正式主机已经支持档8（0.70m/s、4.00m/s²、2.00rad/s、12.00rad/s²）与600秒，无需再刷写；本次没有将请求擅自解释为档9的1.20m/s，也没有篡改高档物理准入。高档本版固件的落地短程、停车及持续负载证据仍待取得，当前不提供可绕过准入的高档600秒启动路径。[本轮原始证据和晋档请求记录](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_ground_2026-11-02/STAGE0_REVIEW_AND_PROMOTION_REQUEST.json)。

## 当前结果

| 范围 | 结果 | 证据边界 |
| --- | --- | --- |
| 基础回归 | 21项通过 | 参数、准入、签名、限时和未知测量状态；不是实车动作 |
| 实际ROS＋独占UDP桥＋原版native C软件代理 | 6项通过，包括不复位的连续会话、遮挡、car01丢失、反馈丢失及控制卡顿 | 软件代理不是物理电机；反馈消失时保留停车未确认 |
| 浏览器协议对等端＋上述软件链 | 11项通过；正常ACTIVE约60.004秒，按 `GROUND60_TIME_COMPLETE` 退出并确认模型停稳 | 协议对等端不是实际浏览器UI，也不是实车60秒运行 |
| 真实ESP32→STM32失能零速握手 | 8秒通过；4WSC/权限8、时钟READY、故障位0、未选档/未使能、四轮输出0 | 未清故障、未选档、未使能、未转轮 |
| 新入口只读Vicon接收器生命周期 | 恢复Sign后8秒通过：2617个完整Sign新帧、2618个car01新帧；无来源冲突、几何拒绝或错误 | 只读，未发送运动；仅关闭本次自建接收器 |

第一次只读复核因Sign无新帧而失败，记录仍保留于 `live_receiver_l7gtzdff`；用户恢复Sign后复核通过于 `live_receiver_koom32ju`。没有靠重启小车、放宽保护或伪造数据取得通过。

## 当前启动指令

保持已确认的场地、接线和随时断电条件，Sign与car01持续实时可见时执行：

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_4wsc_ground_test.sh --launch --site-ready
```

程序重新做当前链路自检并打开浏览器，READY后需按一次并松开空格才运动；再次空格、Esc或停止按钮停止。松键不停车。仅档0：0.20m/s、0.08m/s²，旋转0.20rad/s、0.80rad/s²。最多60秒，不自动升档、恢复或续跑。先小幅前移/返回及左移/返回，异常立即Esc，必要时断电。现场条件改变时不要沿用旧确认启动。

低档用于真实动作和正常停车观察，不能以本轮低档跟随感受判断最高档同步性。无绿色中间点、无固定围栏不意味着没有障碍风险；Vicon不是避障系统。

## 保留与仍待完成

- 原矩形、原本机/浏览器演示入口及正式600秒准入不变；未再次刷固件、未改ESP32/Tenda/联网路由。档0已由用户启动并完成上述真实观察，本轮晋档审查没有再发送硬件指令。
- 实际电机负载表现、停车包络、电池电流及电机/驱动板温度仍待测；未测项目保持未知。正常Esc停车观察不等于已验证600ms失联回退停车包络。
- 首次真实浏览器准备与档0动作已经用户操作并观察正常；先前失能预检本身仍只证明握手和只读Vicon，不能把预检单独当作真实动作证据。
- 高档、600秒和无级调速均未因本轮离线通过而获得物理合格状态。

## 文件与证据

- [操作步骤与保护参数](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_ground_2026-11-02/OPERATIONS.md)
- [离线验收](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_ground_2026-11-02/tests_gol84qax/QUALIFICATION.json)
- [实际链路预检索引](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_ground_2026-11-02/LIVE_PREFLIGHT.json)
- [真实失能零速握手](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_browser_2026-11-02/idle_probe_20261102_111812/RESULT.json)
- [恢复Sign后的只读复核](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_ground_2026-11-02/live_receiver_koom32ju/RESULT.json)

只检查交付文件、不初始化ROS或硬件：`bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_4wsc_ground_test.sh --check`。
