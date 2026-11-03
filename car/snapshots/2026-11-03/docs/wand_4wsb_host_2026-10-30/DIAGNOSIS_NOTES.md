# 开发过程中的失败记录与边界

## 第一轮：控制间隔超100ms，不是整机死机复现

`prepared/ui_600s/normal/normal_stop_RESULT.json`：活动约511.42秒后，控制/门
间隔分别达到113.85/112.71ms，原100ms保护正确触发。UI最大间隔40.37ms，
独立X11最大往返约2.81ms，未出现“鼠标不能移动”的整机冻结。

测试器控制进程保留了51113条逐帧嵌套字典。仅在另一个无ROS进程读取相同
数据后执行全量GC，五次耗时46.69、52.24、48.07、57.42、49.49ms。
这表明观测器存在显著开销，**没有证据证明它独自解释全部114ms，也不能据此
解释此前原桌面死机**。

修正：测试器改用512条有界队列交给独立进程落盘，内存只保留最后200条；
主机SourceRelay对原始内容完全相同的帧不重复几何解算，每次仍检查六路原始
source/received时间。新增独立GC耗时观测，不禁用GC，不延长看门狗。

## 第二轮：软件执行器错误读取中间态

`prepared_v2/ui_600s/normal_chain.log`：约43秒发生
`C oracle failed to produce telemetry`。测试器在发送一条极小速度后、下一个
控制tick之前立即读取状态，碰到了四轮归零更新尚未完成的中间态。

已用`test_native_snapshot_follows_production_isr_request_tick_snapshot_order`
复现：发送0.002m/s目标后，中途取样失败；完成下一次10ms控制更新后，严格
报文解码通过、PWM全零且无C故障位。

生产STM32的`HAL_TIM_PeriodElapsedCallback`顺序是：处理待收命令→控制更新→
应用CCR→缓存遥测；发送线程只发布缓存。仅修正软件执行器为相同顺序，
**未改动任何固件源码、镜像或保护校验**。

最终长测目录为`prepared_v2/ui_600s_correct_isr`；是否通过以最终RESULT为准。
所有失败目录保留，没有改写原日志。

## 另外两项测试判定修正

- `TransportPolicy.receive()`在合法、验签通过的ESTOP后也会返回false，表示
  权限已硬撤销；这不等于HMAC签名错误。增加专用单元测试区分。
- 控制定时器被故障注入暂停后，core可能保留旧ACTIVE文字；应检查独立门和
  C实际permit/PWM，而不是将旧文字当成自动恢复。恢复定时器后仍验证硬锁存。

这些修改均限于新候选/软件验收，旧演示和原矩形保持不变。
