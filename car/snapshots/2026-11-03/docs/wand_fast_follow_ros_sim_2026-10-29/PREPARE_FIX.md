# 仿真操作台“准备不了”修复

2026-10-29。只修改独立仿真候选`wand_fast_follow_ros_sim_20261029`；原演示、矩形、固件和网络配置不变。实车保持断电，不构造硬件桥或发送电机命令。

## 本轮故障证据

会话`real_sign_2go5zddr`没有进入ACTIVE。三次准备请求都发生在`WAND_POSE_TIMEOUT`之后；最初Sign有效的时段没有记录到准备请求，后续输入丢失时仍允许点击准备，最终只显示泛化的英文稳定/停稳超时。停止动作也不能区分真正的Esc和窗口失焦。

此外，源码中存在确认的交互缺陷：`request_start()`仅使监督器进入ARMED，旧界面便显示READY，但独立末级门可能尚未完成握手。于是底部说READY，实际开始请求又被拒绝。缺少按钮状态和清晰原因使这种状态看上去像操作无效。

当前进程已经关闭并不是上述历史故障的替代解释；定位依据是当时保存的源数据、状态和操作时序。

## 改动

1. READY只在原有健康、停稳及独立末级门条件都通过后出现；等待期显示1/2、2/2，不提前宣称准备成功。
2. Sign丢失/安全输入未就绪/软件车未停稳时禁止准备，显示具体中文原因；不会先清空会话再等待五秒。开始按钮只在真正READY或运行中可用。
3. 重复准备请求不续期、不重抓原点、不重新消费会话；准备中定位丢失即取消，恢复后仍需人工新请求。
4. 失焦保留保护：准备/READY/跟随时取消授权；空闲时不制造新的STOPPING。失焦与Esc分别记录，重复停止不重置已经确认的停稳窗口。
5. 保留100ms操作年龄保护，过期/未来操作明确拒绝并记录，不补执行；停止与失焦撤销不因消息过期被忽略。
6. 正常关闭等待软件车零输出/停稳确认，不以固定0.5秒截断日志。

## 验证证据与边界

最终15组真实ROS用例、12项单元/界面回归全部通过；238个原文件哈希未变。已发布的`PREPARED.json`固定本次实现及对应全量证据，原演示入口未覆盖。

- [针对性真实ROS＋Tk流程](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/readiness_fix_targeted_02/RESULT.json)：按钮调用→同一个操作分发器→真实ROS握手→软件车签名输出，包含准备中丢失后的取消及恢复不自动开始。
- 首次组合回归的原13组已通过，但新增Tk用例因为构造Tk时暂停了同线程测试调度，停稳窗口按保护重新建立；测试一度在窗口重建完成前点击了已禁用的准备按钮，故该次整体失败。后续测试等待真实`can_prepare`条件，没有缩短停稳窗口或放宽时序保护。
- 最终全量证据由[readiness_fix_release](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/readiness_fix_release/RESULT.json)记录，发布前同时检查实现哈希、原238个受保护文件及新增单元测试；未通过时入口仍拒绝启动。
- [真实Sign零速准备](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/readiness_fix_real_sign_zero/PREPARE_RESULT.json)通过：READY保持约0.315秒，339条签名输出均为零；没有发送开始动作，没有进入ACTIVE。正常关闭后的模型停稳确认也保留在同目录会话结果中。

新开的未使能窗口会话：[real_sign_3nya_nat](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/real_sign_3nya_nat)。需要操作员在同一窗口连续完成准备、开始、动作、Esc停车后再切回聊天；ARMED/ACTIVE时切出窗口按设计取消授权，不把失焦取消误认为自动准备失败。

以上不等于真人快速手势、换向、停车、真实遮挡和手动恢复已经完成，也不等于实车通过。
