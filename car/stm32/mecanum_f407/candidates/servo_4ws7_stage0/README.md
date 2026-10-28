# 4WS7 v3首档实机入口（悬空动作待验收）

## 2026-10-23电脑端离线修复

首档镜像已经完整备份、刷写并独立读回核对。本次只修电脑端，不重新构建或刷写ARM固件。
第一次悬空尝试在准备阶段被拦截，使能与非零速度均为0；不是一次完成的5秒动作验收。

- HELLO_ACK严格要求RESPONSE；周期WHEEL_TELEMETRY/LINK_STATUS按真实publish()接受FAULT/ENABLED，
  不携带RESPONSE。CRC、会话、序号、时间戳、权限v3/档0、RPM/PWM及故障检查保留。
- 原生C报文模型的标志表达式与生产publish()逐字归一比对；默认100ms实际发布周期。
  另回放944帧未经改写的真实遥测，仅验证摄入，不冒充成功选档或运动。
- 下拉框焦点使用原始Tcl路径，不经Python控件字典解析；弹窗不授予运动权限。
  关闭弹窗、保持按住空格不会自动使能；未知UI异常撤能且继续轮询。
- 私有窗口回归使用真实Tcl下拉弹窗、替身物理键和本机C模型；测试过程禁止Python网络socket，
  不启动小车、Vicon或真实ROS控制节点。覆盖准备零输出、按住、松键、5秒结束、失焦、GUI卡顿、窗口关闭与手动再准备。

修复后的独立运行时与证据：
`docs/wand_4ws7_stage0_2026-10-23/host_fix_20261023/verified_v2/`。
保留旧ready_v2、失败现场与source_before，不覆盖其manifest；新的启动脚本只在整套回归通过后切换。
底层镜像沿用SHA256 `25f235a38100a8ab851f4e18d813f5040399139a9c8034c68a6108712eabe53e`，
原实际读回记录仍需由新入口按文件核验；文件核验不打开ST-Link、不代表本轮重新读取了硬件。

离线复现（输出目录必须全新）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
python3 /home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws7_stage0/run_prepare.py \
  --output /tmp/4ws7-host-fix-new \
  --reuse-firmware-manifest /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4ws7_stage0_2026-10-23/ready_v2/build_manifest.json
```

以下为首版准备说明；其中“本轮”指最初离线准备，不替代上面的最新状态。

日期：2026-10-23。本目录是独立首档候选，不修改原矩形、4W20、4WGD、原4WS7锁定候选、ESP32或Tenda。
本轮允许的是软件准备；所有实机批准默认缺失。与硬件的真实连接、刷写、悬空转轮均未执行。

## 固定首档与两个入口

- 专用STM32目标`mecanum_f407_4ws7_stage0_NOT_FLASHED`，只支持档0：
  合速度≤0.20m/s、向量加速度≤0.08m/s²、航向≤0.10rad/s、PWM≤35%、起动≤30%。
  每轮目标≤100RPM、原始反馈>120RPM故障；高档1～6在本目标的策略函数中全部拒绝。
  源自4WS7的SI协调斜坡、实测中立/有界重新追赶保留，未恢复旧800cps/s固定慢斜坡。
- `4WS7`身份、**184字节输出权限v3**。v2离线锁定包保持原样，不能混用；新解码器不接受旧固件身份/版本。
  `hardware_released=1`只表示这份代码具备首档试验能力，不代表已刷写或已取得现场/电气验收。
- Sign入口：独立ROS域118、`/wand_4ws7_stage0`；沿用成熟操作台、物理空格键位图检查、监督器、末级门控、
  HMAC来源验证及同会话准备流程。1:1 XY、原固定起点/朝向、3m×3m和四边50cm缓冲不变，Z不映射成车体运动。
  电脑端正常命令限制和MCU同等SI上限作为两道独立边界，未叠加更慢的轮级800cps/s限制。
  未来更高档仍需单独迁移，不在此入口提档。
- 悬空入口：独立Tk窗口＋独立安全线程，不启动Vicon/ROS位置外环。八个固定方向，单次≤5秒、无自动循环。
  按住空格之前只准备、不转轮；方向选择不直接运动。PC和MCU各自执行5秒上限，后续包不能续长MCU计时。

悬空模式通过独立`CONTROL_ENABLE.reserved=0x7080`选择，Sign首档用`0x7000`；选模式本身不使能。
MCU先要求失能零租约和实测停稳，再确认模式、单独使能、确认许可后才接受非零速度。
切换模式需要新的人工准备流程，不允许边转轮边切换。悬空测试结束后旧使能速度包不会重启。

## 保留的保护

松键、失焦、GUI卡顿/150ms操作租约失效、控制线程间隔>100ms、200ms反馈/命令进展超时均撤能。
准备/使能/模式确认超时不自动重试；停止会丢弃尚未发出的旧使能请求。
真实MCU故障不能从“重新准备”清掉；正常停止后需反馈确认停稳、松键、人工解锁和重新准备。
硬件端仍保留600ms断令、150s绝对预算、堵转、超速、方向/CCR校验、30ms实测中立及750ms换向期限。
普通停车是00滑行，零PWM不等于已停稳；停车未确认需人工准备断电。

两个入口共享`/tmp/wand_mc_link_46001.lock`和固定遥测端口，禁止并行占用；Sign保留ROS来源/图审查。
不更改Tenda连接、热点、路由或全局键盘设置；硬件UDP仅允许绑定已确认的Tenda接口。
读回证明与当次现场确认先于Node/socket创建；启动命令默认只是文件检查，不会连硬件。

## 本轮离线验证与复现

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 stm32/mecanum_f407/candidates/servo_4ws7_stage0/run_prepare.py \
  --output /tmp/4ws7-stage0-new
```

输出必须全新。原生C测试和C动态库使用同一首档权限代码，无测试专用放行宏；Python对接原生C协议/轮控，
反馈为PWM驱动的合成编码器，不是实车。窗口回归使用私有Xephyr、物理键状态替身和本机C假IO，
禁止Python网络socket，没有操作用户真实键盘，没有启动Vicon或真实ROS控制节点。
生成/构建两个私有ROS包不等于执行其入口；独立FinalGate类与旧版本文本一致。

首版输出（保留归档）：[ready_v2](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4ws7_stage0_2026-10-23/ready_v2/)。
首轮`ready/`保留失败记录：C专用预包含头误用于汇编启动文件导致ARM编译失败，
已仅在新CMake目标中限制为C语言；旧源码和安全阈值未改。

## 现在可以执行的检查命令（不连硬件）

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_4ws7_stage0_console.sh --check
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_4ws7_stage0_suspended.sh --check
```

未来才使用`--launch POST_FLASH_RECORD.json`。没有真实刷后完整读回记录会在连接前被阻止，
还需人在终端确认当次支撑/场地及立即断电条件；准备工具不会生成任何批准的刷写记录或现场配置。
本轮不提供已确认的启动许可，不需要现在上电或插ST-Link。

后续刷写须先断电、完整备份原Flash、写入首档镜像、独立完整读回并逐字节核对，再生成实际证据记录。
悬空先看供电和编码器响应，之后才另行安排Sign落地入口；替换同一MCU仍需兼容验收，不能承诺旧矩形命令自动适配新身份。
