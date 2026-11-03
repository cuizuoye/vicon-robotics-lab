# 4WSC 档6独立落地短验

2026-11-02，已发布软件快照 `prepared_v2`。前置是本版4WSC档4已完成60秒真实演示、停稳确认及用户“高速档演示通过，操作正常”的反馈。独立入口仅用于受控60秒下一档实验，不构成正式600秒、高速急停包络或电气/温升合格证明。

## 本轮参数和复用范围

固定档6：平移≤0.70m/s、加速度≤2.00m/s²；旋转≤2.00rad/s、角加速度≤12.00rad/s²；每轮目标≤400RPM、PWM≤80%，叠加运动仍服从共享轮速限制。最多60秒，不自动升档、恢复或续跑。

复用已经通过的直接1:1平移＋相对旋转算法、ROS监督器、签名末级门、浏览器和原版native C固件；没有新增绿色参考点或固定围栏，不混入独立无级调速实验。未再次刷写STM32、修改ESP32/Tenda或默认联网路由，也未覆盖档4、档0、原矩形/演示源文件。

## 离线验证

- 25项基础/权限/配置回归通过：只允许精确档6、拒绝混搭和600秒，保留非运动安全配置、前置真实证据及未知测量状态，签名与其他档位隔离。
- 6项实际ROS＋独占UDP桥＋原版native C软件代理场景通过：同一代理不复位的两次连续运行、Sign/car01丢失、反馈中断及150ms控制卡顿。撤权后恢复数据也不自动续走；缺反馈时仍保留停车未确认。
- 11项浏览器协议对等端场景通过：完整约60.0137秒运行并自动停稳、不开始、Esc、失焦、证明超时、控制/画面断连、第二次空格、随机数重放、越权目标及遮挡。
- 这些协议对等端不是实际浏览器UI，native C软件编码器也不是实车负载；不能把上述数据当作物理动作、加速度或停车测量。

`prepared_v1` 因manifest档位标记仍为4，被文件检查拒绝；已保留失败说明并在新快照修正，未向硬件发送该草稿。唯一发布版本是v2。原档4、档0及正式主机入口的文件校验仍通过。

## 本轮真实接入

用户已再次确认现场条件，允许打开未使能页面。真实8秒失能零速握手通过：4WSC、时钟READY、故障位0、未选档/未使能/全零输出；真实ROS/Vicon建基准通过后打开浏览器。记录到操作者的一次新空格按下/松开后进入ACTIVE，工具没有发送开始动作。

**本轮真实后台结果已通过**：ACTIVE约60.0082秒，按 `STAGE6_GROUND60_TIME_COMPLETE` 自动结束、末级输出归零并确认停稳，全程观测硬件故障位为0，无提前异常退出；控制器和本轮自建接收器已退出。Vicon的50ms窗口观测峰值为平移约0.457m/s、旋转约0.989rad/s；浏览器回应最大间隔29ms，控制循环最大间隔约16.4ms。

本轮手势未与档4严格一致，不能用较低的峰值声称同步性变差，也不能声称已达到0.70m/s或2.00m/s²的所有物理极限。停止发生在计时结束，**未记录运动中的主动Esc停车**；停止请求后约1.7mm观测位移不是高速制动距离。用户随后确认演示正常、2.00m/s²已足够，选择保留此档并延长到600秒，[操作员反馈](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/OPERATOR_REVIEW.json)已单独保存；原始结果不改写。600秒软件候选另行准备，运动中停车、完整600ms失联包络和电流/温升仍未测，不自动标为物理合格。

- [操作与保护参数](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/OPERATIONS.md)
- [离线结果](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/tests_847hq92h/QUALIFICATION.json)
- [真实失能预检](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/launch_nxmk8q9f/DISABLED_ZERO/RESULT.json)
- [本轮合同](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/prepared_v2/sessions/fast_hardware_VfJ2Bc/CONTRACT.json)
- [本轮真实测试汇总](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/LIVE_TRIAL_RESULT.json)
- [前置档4操作员反馈](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/OPERATOR_REVIEW.json)

## 后续新会话的启动命令

当前页面运行时不要另开第二个控制器。仅在旧控制器退出、现场条件仍满足时执行：

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_4wsc_stage6_test.sh --launch --site-ready
```

READY后单击空格才开始，再次空格/Esc/停止按钮停车，松键不停。浏览器/输入100ms、命令100ms、定位200ms、反馈300ms、租约150ms、底层600ms看门狗及其他原安全保护均保留。建议先小幅往返，正常后逐渐放大；本轮在车仍平移且前方空旷时按Esc，观察正常停车，不必等满60秒。

数据缺失、停车未确认以及未测电流/温度都如实保留；正常停稳不等于已验证高速失联停车包络或持续600秒负载。真实ROS域128、回环测试域195、硬件端口独占，退出时只关闭自建接收器。只检查交付文件可用 `--check`，不会初始化ROS或硬件。
