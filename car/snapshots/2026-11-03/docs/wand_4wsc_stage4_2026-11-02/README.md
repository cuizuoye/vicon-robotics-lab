# 4WSC 档4独立落地短验

2026-11-02，唯一离线发布快照为 `prepared_v2`；`prepared_v1` 是保留的未发布草稿。复用已有直接1:1平移＋相对旋转、ROS监督器/签名末级门、浏览器和原版native C固件，不新增绿色参考目标，不改固件或网络。

## 实验边界

固定档4：平移0.70m/s、0.80m/s²；旋转1.50rad/s、6.00rad/s²；PWM≤80%、每轮目标≤400RPM，平移旋转仍受共同轮速约束。单次最多60秒，不升档、不自动恢复、不开放600秒。参数是上限，不能据此声称实际轮速、负载或制动性能已通过。

前置为本版4WSC档0的真实动作和正常人工停车观察，原结果与用户“演示正常”反馈都绑定哈希。该记录仍保留未测电流、温度和完整失联停车包络为未知，不充当档4/正式600秒合格凭据。

## 已完成的软件验证

| 范围 | 结果 | 不代表 |
| --- | --- | --- |
| 基础/权限/原始证据回归 | 25项通过，包括只允许档4、拒绝混合或更高参数、保持原安全配置、拒绝600秒、保留未知测量、不同档位/入口签名隔离 | 已测过高档物理性能 |
| 实际ROS、独占UDP桥与原版native C软件代理 | 6项通过，包括同一代理不复位连续两轮、Sign/car01丢失、反馈丢失及150ms控制卡顿 | 软件代理等于物理车或其负载 |
| Node浏览器协议对等端和上述ROS/C链 | 11项通过，约59.999秒后按独立60秒截止自动停车；另含不开始、Esc、失焦、证明超时、控制/画面断连、第二次空格、重放、越权目标、遮挡 | 协议对等端是实际浏览器UI或600秒实车验收 |

故障场景要求撤权且恢复后不自动续走；反馈断开时正确保留停车未确认，不制造停止证据。档0、原演示和正式600秒准入保持原样，文件哈希复核通过。没有再次刷写STM32、修改ESP32或改Tenda/默认联网路由。

## 当前实车窗口

操作者已重新确认落地、朝向、定位、接线、空旷和立即断电条件。真实失能零速握手通过：4WSC、时钟READY、故障位0、未选档/未使能/全零输出；随后只读Vicon和真实ROS失能建基准通过，浏览器已打开。日志记录到一次空格按下/松开后开始本轮档4实车实验，未由工具发送开始动作。

**本轮真实后台验收已完成**：实际ACTIVE约60.0066秒，按 `STAGE4_GROUND60_TIME_COMPLETE` 自动结束并确认停稳、末级输出归零，没有提前异常退出；控制器及本轮自建接收器均已退出。Vicon的50ms窗口观测峰值为平移约0.570m/s、旋转约1.446rad/s；页面回应最大间隔27ms，最大控制循环间隔约41.7ms。本轮不等于证明达到配置极限或具备持续负载能力。

计时结束时已是小指令/近静止，停止请求后的约2.1mm观测位移**不是高速紧急停车距离**。用户随后确认“高速档演示通过，操作正常，可以继续做下一步了”，已另存[操作员反馈](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/OPERATOR_REVIEW.json)，原始日志及此前后台汇总不改写。电流、温度及600ms失联停车包络仍未测，不据此自动开放600秒或4.00m/s²。

- [操作与保护说明](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/OPERATIONS.md)
- [离线验收结果](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/tests_35j7ryux/QUALIFICATION.json)
- [本轮真实失能预检](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/launch_8dnaw_xd/DISABLED_ZERO/RESULT.json)
- [本轮真实合同](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/prepared_v2/sessions/fast_hardware_ur3wTC/CONTRACT.json)
- [本轮真实测试汇总](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/LIVE_TRIAL_RESULT.json)

## 再次启动时使用

当前页面已经运行时不要执行第二次命令。后续新一轮在现场条件仍满足、旧页面/控制进程退出后才执行：

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_4wsc_stage4_test.sh --launch --site-ready
```

自检并自动准备后，新的单击空格才开始；再次空格/Esc/停止按钮结束，松键不停。失焦、页面或反馈中断、姿态及底层故障保护全部保留。初次仅先小幅往返，正常后逐渐放大，不追求旋转极限。只检查文件使用 `--check`，无ROS或硬件IO。
