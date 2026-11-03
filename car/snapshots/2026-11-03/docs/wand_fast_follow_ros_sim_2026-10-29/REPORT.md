# 真实Sign软件车与ROS输出链：验证记录

2026-10-29。本轮范围：真实Sign只读输入、软件车和真实ROS节点；实车保持断电。不刷固件、不构造硬件桥、不发电机UDP命令、不改Tenda或原演示/矩形。

后续操作员报告“准备不了”已单独修复，当前版本为15组真实ROS、12项新增回归通过，且真实Sign零速准备已通过；详细原因、修复及最新窗口见[准备流程修复](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/PREPARE_FIX.md)。下文保留首次交付的历史证据，不把首次窗口未验收写成已完成。

## 已完成的自动验证

1. 复用上一轮哈希固定的快速追赶控制器，未改变1:1映射、0.70m/s、0.80m/s²、0.10rad/s、0.40rad/s²、120秒和安全超时，也未恢复固定围栏。
2. 真实ROS链路在私有域189、本机发现、随机命名空间中运行：同帧Sign包→监督器→精确来源摘要/源时间/模拟车时间戳匹配→独立末级门→签名输出→软件车。软件车不接受裸Twist。
3. 独立仿真复用了之前离线准备的有界参考帧历史/FIFO修复，没有写入原演示。重新准备只经人工确认创建新仿真会话，保留模型位置与余速、保持逻辑命令主和递增序号，拒绝旧队列。
4. [最终真实ROS回归](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/ros_checks_release/RESULT.json)共13组通过：快速往返/换向/停止、遮挡停车与手动恢复、错误Sign摘要、错误Sign时间、错误车体时间、未知FIFO头不能跳过、末级加速度拒绝、签名篡改、裸Twist不可驱动、反馈丢失、模型定位丢失、UI心跳丢失、重复ROS来源。
5. 8项新增单元/界面测试通过：真实六路包同帧、重复不续龄、错几何/未来帧、Z不映射、签名重放/过期/异主/超速/裸报文、真实Tk按键绑定/自动重复与1280×720布局。上一轮25项测试和96组模型比较的交付核验仍通过。
6. 238个原演示、矩形、固件和接收器受保护文件哈希未变。

合成ROS测试中，操作员停止到软件车收到签名零速约5.2ms；停止输入Sign样本后约216ms收到零速，符合保留的200ms定位超时加调度周期。这些是一次本机软件测试时序，不是Vicon曝光到电机起转/物理停车时间，不是时延上界保证。

## 已完成的真实输入检查

- 确认有线Vicon服务器可达，启动已核验版本的只读接收进程，使用原服务器10.10.10.2、map/vicon与零变换；未修改网络配置。首次启动缺少SDK库路径，随后仅为该接收子进程提供既有库目录，连接成功；没有重编译接收器。
- [8秒真实Sign只读检查](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/live_probe_01/RESULT.json)：Sign＋五点来源身份和同帧几何通过，无拒绝记录。
- [10秒未准备、未使能启动检查](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/live_startup_01/STARTUP_RESULT.json)：443个采样点Sign有效，1412条签名输出均为零；没有ARMED/ACTIVE，没有真实电机输出。
- 原始Sign发布端时间是该接收实现的本机处理时间，不等于相机曝光时间。仿真车位姿及反馈来自软件模型，不使用实际car01位姿冒充新控制器响应。

## 真人动作验收状态：尚待完成

当前真人会话：[real_sign_l_dw6joa](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/real_sign_l_dw6joa)。窗口已打开、未由本代理准备/开始。当前没有ACTIVE会话，输出保持零。按键由操作员触发，日志不自动宣称验收完成。

启动只读检查后，Sign输出暂时消失。独立5秒只读消息计数收到1280条`/vicon/car01/car01`，Sign刚体及五个标记点均为0条；TCP连接仍建立。可确认并非整条Vicon连接断开，但本机记录不能区分Tracker暂停输出该对象、对象未启用或杆不可见的具体现场原因。已请操作员确认Tracker中Sign启用、五点无遮挡，恢复后再验证。没有为此放宽定位保护或重写源时间戳。

需分别取得日志和人工观察：

- 快速手势/连续换向：平移25～30cm，用0.6～1秒，反复收回、前移；检查方向、换向拖后、是否冲过目标。
- 停车：运动中Esc或再次空格，输出归零且模型停稳；松键不停车属于此单击模式的定义。
- 真实遮挡：全部五点遮挡约3秒，软件车停；露出后保持停止，不自动恢复。
- 手动恢复：消除原因、模型停稳，点击“确认并重新准备”；READY时不按空格仍静止，新空格动作后可跟随，再停车。

真实Sign暂不可见时不会准备。恢复数据不会使模型自动运动。遮挡合成用例已通过不等于真人遮挡已完成，真实输入的未使能检查也不等于快速动作通过。

## 独立入口

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_fast_follow_ros_sim.sh --check
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_fast_follow_ros_sim.sh --launch
```

必须维持实车断电；这是软件车入口，不是实车启动命令。文件固定和测试证据见[PREPARED.json](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_fast_follow_ros_sim_2026-10-29/PREPARED.json)。原本机演示指令未替换。本轮尚未测出新的物理起转时延，也没有将模型误差改善宣称为实车性能提升。
