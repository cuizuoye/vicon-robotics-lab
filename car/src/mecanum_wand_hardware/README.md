# 真实Sign到实车：显式验收入口

默认`shadow`，只用真实Sign驱动软件车；没有实车UDP。`emulator`只用域93模拟定位和127.0.0.1:54600/54601协议仿真器。只有显式`hardware`、完整审核参数、匹配参数哈希、显式现场确认，才可构造真实MC-Link桥接器；启动仍默认失能。

此包独立于旧矩形控制器和原离线杆包。旧矩形文件和原MCU固件源文件不改动；4WGC使用独立候选覆盖层，复用既有MC-Link会话/零速预置/固件身份/反馈/限时逻辑。

## 最新电脑端修复入口：大目标引导与显式重新准备（2026-10-23）

同日补充：已修正重开时的MCU时间基准误差引发的新轮速包误丢弃。198项检查及完整关闭后两次失能重开准备通过；没有放宽20ms时钟不确定度/200ms反馈期限，也没有刷固件或改网络。见工作区`docs/wand_pc_governed_recovery_2026-10-23/reopen_filter_trace/README.md`。真实链路间断仍会停车，动作验收与失能准入分开。

当前4WGC不需要再次刷写。独立配置`config/hardware_acceptance.corner_2x2_4wgc_governed_2026-10-23.json`显式启用`goal_reference_policy=governed`；其余物理、空间、速度、时限及通信参数与下节历史4WGC配置一致。旧配置保留为direct策略，不会自动切换。修复说明与证据：`docs/wand_pc_governed_recovery_2026-10-23/`（工作区根目录）。

- 2倍XY映射仍指向最终绝对目标，杆收回则最终目标返回；橙圈显示滤波/边界限幅后的最终目标，青圈显示≤0.05m/s逐步移动的即时跟踪点。正常超前量约20cm，参考点不追随被推走的车重新锚定。
- 保留65cm异常跟踪偏差保护，并在独立末级门重复检查；末级门还校验即时参考点的位移速度。新增有平移指令而3秒内无至少1cm位置进展的停车保护，避免参考点限幅掩盖不动的车。
- 松键、定位/反馈超时、抬车、硬故障等仍停车；200ms反馈期限未延长。普通失能不再删除真实历史反馈，缓存时间不续期，缓存permit=1绝不算停稳。
- 数据回来不自动续跑。松键、恢复新鲜反馈、实测停稳后，点击“确认并重新准备（不运动）”；重新累计稳定窗口并建立新会话。新故障/提前按键/5秒等待超时将取消请求，硬故障不能用此按钮解除。再次显示就绪后仍须新按住空格，按下瞬间捕获新杆/车原点，旧未完成目标丢弃。
- 194项单元/静态检查、195个原试车记录时刻回放和双包构建通过。记录回放不是新实车动作验收，也没有消除物理链路间断根因。实机全程松键准备检查通过，约0.638秒完成门控准备；89帧4WGC诊断，故障及输出为0，随后停车确认并退出。

在现场条件已确认、没有另一个运动入口运行时启动（默认仍失能）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
ros2 launch mecanum_wand_hardware wand_sign_acceptance.launch.py \
  mode:=hardware \
  profile:=/home/PRIMARY_USER/vicon_mecanum_ws/src/mecanum_wand_hardware/config/hardware_acceptance.corner_2x2_4wgc_governed_2026-10-23.json \
  approved_sha256:=7ad78970f707bc98d71b9028d6a1b20edeb690cf7d1102ca611801e2c89733ff \
  hardware_consent:=CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY
```

## 保留的4WGC初次迁移入口（2026-10-23，direct策略）

用户已明确授权新候选适配和断电后备份刷写。专用配置为`config/hardware_acceptance.corner_2x2_4wgc_2026-10-23.json`，SHA256为`513914a3c5877e8809421b987603956a081e15454081c8d343d0597b32ab1843`。空间/物理尺寸/0.05m/s/120秒/3m目标半径与下文2×2m配置相同，严格要求**4WGC、156字节、输出权限版本1**，不接受旧离线132字节诊断或伪装为4WGB。

独立`continuous_bridge`仅复制旧桥构造器和接收分支（原源码哈希及结构一致性有检查），其余握手、发送、超时、失能和急停安全方法继承原实现；不修改原桥/原解码器或它们的全局白名单。正常换向的旧方向减速按“当前有效目标＋已授予方向＋实际PWM/CCR”校验，不按未来请求方向误报，也不撤销极性保护。

77项无节点单元检查、实际C静态序列化/ABI/CRC检查、ARM与专用ROS包构建通过；按用户要求没有追加动态模型仿真或真实ROS故障注入。新镜像46928字节，SHA256为`6b67c3e59cd1374f24702b199fa6dabd26e7d8e1085d11bd0a76600a2d3a32a1`，路径为`docs/wand_4wgc_migration_2026-10-23/arm/mecanum_f407_continuous_4wgc_OFFLINE.bin`。刷写/独立读回及实机验收状态以该目录记录为准。

只有完成断电刷写、独立读回一致、操作员恢复电源和确认场地后，才执行纯失能准入，再进入操作台。不得直接把编译通过当作这些步骤已经完成。以下命令是后续使用的入口，启动仍失能，不自动开始运动：

```bash
ros2 launch mecanum_wand_hardware wand_sign_acceptance.launch.py \
  mode:=hardware \
  profile:=/home/PRIMARY_USER/vicon_mecanum_ws/src/mecanum_wand_hardware/config/hardware_acceptance.corner_2x2_4wgc_2026-10-23.json \
  approved_sha256:=513914a3c5877e8809421b987603956a081e15454081c8d343d0597b32ab1843 \
  hardware_consent:=CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY
```

旧矩形软件和已验收4WGB镜像保留，但旧矩形入口有意不接受4WGC：刷入4WGC后旧4WGB命令将拒绝身份，不会悄悄用未适配的矩形入口控制新固件。需跑旧矩形时应另行适配独立入口或明确授权恢复备份，不能伪造身份或绕过门控。Tenda/ESP32设置不改。

## 保留的4WGB配置：旧矩形起点一角，前2m、左2m（2026-10-23）

用户再次确认车体完整长宽为190×210mm，起点由人工放回原矩形起点，向前2m、向左2m一侧有充分Vicon覆盖和空旷余量，并明确接受将单次会话由30秒延长至120秒。新的独立配置为`config/hardware_acceptance.corner_2x2_2026-10-23.json`；下文10月22日配置保留用于历史复现，不是新区域的启动参数。

- 真正半长/半宽为0.095/0.105m，不再用完整长宽充当半尺寸。独立0.20m边界余量不减少。
- 以10月21日最后4WGB矩形记录第一次使能前的位姿为固定参考：x=0.64370348m、y=-1.05286854m、车头yaw=1.52626041rad。它是历史位置，不是本次实测，也不会替代实车新鲜定位。
- 名义车中心区为沿该朝向前0～2m、左0～2m。外围使用世界坐标轴对齐包围框，包含每侧0.10m摆放余量，再外扩物理车体的外接圆半径与0.20m安全余量；因此完整2×2m车中心区不会被车体检查再缩小。外围框约2.979×2.979m，不应把它误读成车中心可平移2.979m。由于采用外包矩形，允许区会略大于名义区域。
- 目标半径3.0m，足够覆盖2×2m对角点的2.828m距离。保持2倍水平位移、边界限幅、不自动重设固定围栏；杆参考框每侧仍比外围框多0.50m，不是实测Vicon边界或人车防撞距离。
- 速度仍为0.05m/s，会话120秒；底层150秒预算、600ms断令、所有短时效/来源校验、松键、遮挡、抬车、跟踪偏差和堵转保护不变。120秒到期仍安全停止，不自动续期；快速把目标移到车前0.65m以外仍可能触发跟踪偏差保护，扩大空间不是取消此保护。
- 本次只做配置/几何审核，不运行动态仿真、ROS节点或实车。`test_corner_workspace.py`的12项纯配置/几何及提示检查通过，旧配置准入相关15项检查也通过；原矩形及固件源文件的既有哈希清单核对一致。新配置不是实车动作验收记录。操作台按绿色中心可用区提示限幅并显示真实配置时限，不再把角点处很小的全方向内切半径误显示成主要杆活动范围；超时说明不再写死30秒。
- 新配置仍显式要求原实机入口的4WGB身份，不增加4WGC白名单；4WGC候选与实机入口的兼容迁移仍是独立待办，不能刷写后继续冒用4WGB启动参数。

新配置SHA256：`2cd114d5f7d183ca509b93fa3e2849fe81db86e232c8c76822c23eb09ecf407c`。下一次显式启动时需选择这个配置和相应审核哈希；已经打开的控制台不会热更新，继续用旧配置/旧哈希仍是旧范围。纯文件审核命令（不创建节点或打开套接字）：

```bash
ros2 run mecanum_wand_hardware wand_acceptance_review \
  /home/PRIMARY_USER/vicon_mecanum_ws/src/mecanum_wand_hardware/config/hardware_acceptance.corner_2x2_2026-10-23.json
```

实车每次仍人工放回原起点和原朝向；现场朝向、Vicon原点或场地改变需重新审核固定边界。`preview_start_xy_m`只为软件预览提供起始位置，实车来源始终为实时car01，准备/暂停不会搬动围栏。此次没有刷写、启动小车、修改原矩形或更改网络设置。

## 历史配置：大范围手持杆＋车目标边界限幅（2026-10-22）

用户已明确选择“范围内2倍、车目标到边界后限幅”，杆参考区在车围栏每侧至少多0.50m。新配置为`config/hardware_acceptance.wide_wand_2026-10-22.json`，不要与下文保留的旧严格越界配置混用。

- 不增加杆位置硬围栏：在Vicon可有效跟踪的范围活动；原始时间戳、失踪、位置跳变和有限数检查保持生效。
- 原车固定1.4×1.4m围栏、保守轮廓、0.20m余量不扩大。紫色参考框为2.4×2.4m，每侧多0.50m；不是经实测确认的Vicon覆盖边界，也不是人车防撞距离。
- 原始2倍目标先投影到车体/余量允许的固定矩形，再保留本会话0.50m目标半径。目标在限制内时保持2倍；到边缘后杆可继续活动，会话不因目标越界锁死。不会悄悄重设杆原点。回收至可映射区域后目标重新跟随，收回原点仍对应本轮车起点。
- 原始目标限幅发生在滤波之前，避免大幅越界输入在滤波器中积累；独立末级门仍拒绝任何越界的实际目标。实车本身越界、定位丢失、松键、失焦和超时仍停车。Z不参与平面映射。
- 初验0.05m/s、30秒、4WGB/600ms均保持；不刷固件、不改原矩形或网卡设置。

仅软件审核已通过，新配置SHA256：`a6bb21d836190fb77a3e89a877c4af25a586538e9e956889db1037b2c4bac629`。519项pytest、宽范围真实ROS/本机MC-Link协议仿真以及真实Sign/私有X11键盘GUI回归通过。协议仿真中杆移动0.8m（原始目标1.6m），实际目标dx约0.280m，收回误差约0.0248m；运动中松键到仿真MCU失能约0.062s。它们不是实车制动/无线验收。

实车上电、当前空旷场地和物理断电措施确认后，使用本节新配置及哈希进入`mode:=hardware`。启动保持失能，不是开始运动。先检查新鲜4WGB/600ms身份、零故障、定位和真实静止反馈；不得因本节软件通过而略过实车准入。

```bash
ros2 launch mecanum_wand_hardware wand_sign_acceptance.launch.py \
  mode:=hardware \
  profile:=/home/PRIMARY_USER/vicon_mecanum_ws/src/mecanum_wand_hardware/config/hardware_acceptance.wide_wand_2026-10-22.json \
  approved_sha256:=a6bb21d836190fb77a3e89a877c4af25a586538e9e956889db1037b2c4bac629 \
  hardware_consent:=CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY
```

最终记录：`docs/wand_wide_workspace_2026-10-22_protocol/summary.json`、`docs/wand_wide_workspace_2026-10-22_gui/summary.json`及只读定位`docs/wand_wide_workspace_2026-10-22_preflight.json`。当前尚无实车动作验收结果。

### 首次上电链路准入与停稳估计修复

原专用Tenda连接已恢复，SSID/BSSID均未改，主网络默认路由/DNS未改。真实MC-Link握手与4WGB诊断已收到；这不是读取了板内完整Flash。首次无运动准入因Vicon逐帧微小角度噪声误判速度而未通过；原瞬时角速度中位约0.022rad/s，半数超过停稳0.02rad/s阈值。

仅杆安全核心改为按原始消息时间戳、原配置0.50s窗口内的XY/角度全包络估计停稳运动量，避免回调批处理和逐帧噪声放大；中间点也参与，不能用首尾抵消往返运动。原0.005m/s、0.02rad/s阈值、连续0.50s停稳、0.60s松键就绪和真实MCU零输出/零轮速反馈仍保留；原始抬起、倾斜、跳变、过期和边界检查仍逐帧生效。代价是冷启动就绪需要更完整的历史，仿真Harness初始等候增至1.8s，不改实车保护参数。

实际静止录制数据：`docs/wand_hardware_2026-10-22_stationary_car_samples.json`。0.5s窗口重放后静止段最大线速度估计约0.00111m/s、角速度约0.0113rad/s；叠加0.02m/s平移或0.04rad/s旋转的离线反例全部被判为非静止。528项pytest通过；最终协议隔离复测`docs/wand_hardware_2026-10-22_stationary_isolated_protocol/summary.json`通过。前一轮出现FEEDBACK_TIMEOUT并保护停车的仿真记录保留，不计通过，不放宽超时。

实车无运动最终准入：`docs/wand_hardware_2026-10-22_idle_final/summary.json`。12秒只发送HELLO/失能零速，收80个实时4WGB诊断，故障0、permitted=0、PWM/目标/反馈轮速0，MCU时钟进展，STANDBY、stop_confirmed和ready_to_start为true。没有调用准备/使能/清故障。电池电压遥测为空，不能把它当成实测电池电压。最初两轮静止误判记录及0.20s窗口仍偶发重置的记录均保留，不能冒充最终通过。

实车GUI现在明确使用“实车工作区”文字；硬故障提示不再建议只重启仿真来清实车急停。实车动作、制动和实际方向仍须独立首验。

### 按住空格即反馈超时：准备阶段握手修复

首轮实车记录`docs/wand_hardware_2026-10-22_first_motion_live/events.jsonl`显示：按键被正确接受，进入ACTIVE约0.12s后因FEEDBACK_TIMEOUT保护停车，尚未发出任何非零速度。原因是旧适配器在ACTIVE边沿才调用继承的重新HELLO/清诊断缓存/600ms零速预置流程，与200ms活动反馈期限不匹配；不是操作员松键，也不是要求重启电源。

仅专用`HardwareBridge`更改：在ARMED期间完成连续600ms真实、无故障、失能零输出且命令消费进展的基线，完成前不报告gate_ready。健康使用预算也只在ARMED/静止时单次重置。人按住空格后沿用该epoch对应的同一MCU会话，不清真实反馈缓存、不重新HELLO；继承的发送器仍必须收到新鲜MCU许可后才能发非零速度。新会话、过期、故障或未准备状态拒绝使能；1s未确认许可也停车。原200ms反馈期限、600ms MCU断令租约和所有速度限制未放宽，原矩形桥接器和固件文件未修改。

新MCU采样比初次网络映射早到几毫秒时，只对该次确实前进的采样把映射时间上限限制为实际接收时刻；重复、乱序、冻结和过期包仍拒绝，缓存不能更新时效。未通过的两个准备计时回归记录保留在`...prepared_handshake_10hz`和`...prepared_handshake_diagnostics`，不计为通过。

验证结果：543项pytest及两个包定向构建通过。`docs/wand_hardware_2026-10-22_prepared_handshake_clock_fixed/summary.json`在真实ROS节点/10Hz回环MCU下连续三轮准备、运动、松键再准备均通过，只发送1次初始HELLO，使用同一会话，健康预算重置2次；反馈采样最大间隔约101ms，模拟端松键撤销许可约44–61ms。`docs/wand_hardware_2026-10-22_prepared_full_protocol/summary.json`的11项完整功能/故障注入也通过，包括冻结MCU采样不能续命、伪造来源急停、Sign/操作租约丢失、抬车、旧控制器冲突与桥进程SIGSTOP后独立600ms仿真端断令；这些不是实车制动距离测试。

修复后宽范围映射回归`docs/wand_hardware_2026-10-22_prepared_wide_protocol/summary.json`通过：0.8m杆输入的原始目标1.6m、实际目标dx约0.280m，到边界继续持杆不锁死；收回、重新准备、Sign丢失/恢复均保持既定行为。

实车只准备不使能复核：`docs/wand_hardware_2026-10-22_real_prepared_zero/summary.json`通过。保持按键租约始终释放，约0.644s进入ARMED就绪，随后取消并确认PAUSED。61个真实4WGB诊断中许可/PWM/目标/反馈轮速均零、故障零，未进入ACTIVE。未刷固件、未改网络或原矩形；实际按键后运动、方向和松键制动仍待操作员复测。

## 杆充电后先做此项（实车仍断电）

已有Vicon接收器继续在域0发布`/vicon/Sign/Sign`，不要重复启动接收器。

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mecanum_wand_hardware wand_sign_acceptance.launch.py \
  mode:=shadow profile:=/home/PRIMARY_USER/vicon_mecanum_ws/src/mecanum_wand_hardware/config/hardware_acceptance.site_2026-10-22.json
```

窗口标题必须为“真实Sign → 安全仿真车（实车断电）”。鼠标不能替代杆输入。看顶部步骤：必要时轻按并松开空格解锁 → 准备 → 等待准备就绪 → 持续按住空格并缓慢移动真实杆。首验仅杆约10cm/目标约20cm；杆收回返回本次起点。松开、失焦、断流后不会自动继续。每次重新准备/使能重新捕获杆和车原点，但固定围栏不随之移动。

真实Sign缺失时会明确显示等待跟踪且无法使能；不会生成假Sign或重新打时间戳维持旧位姿。

## 实车阶段：须另行确认上电及场地后执行

配置文件`hardware_acceptance.site_2026-10-22.json`已按用户输入准备；这不是实车验收通过记录。`hardware_acceptance.draft.json`则故意保留空值/false，不能用于实车。

首验保留当前Vicon起点，约x=0.61701m、y=-1.05145m。以此为中心的固定1.4m×1.4m世界坐标围栏不自动重定心。基准高度约0.10791m；车体外廓190×210mm、刚体原点大致居中。为覆盖原点不精确，首验采用完整长宽作为保守半尺寸0.19/0.21m，再另加0.20m余量，因此可移动范围比图上的外框小。

首验只保持0.05m/s、30秒会话、约10cm杆位移。沿用+135度车头修正和4WGB/600ms协议；运行时必须用新鲜MCU诊断确认匹配，不以离线文件冒充当前固件。身份不符、故障或无静止反馈时拒绝启动；不自动刷写、回滚或清除真实故障。

确认实车上电/场地安全、真实Sign软件阶段已通过，并退出旧矩形/UDP桥后，先作纯文件审核：

```bash
ros2 run mecanum_wand_hardware wand_acceptance_review \
  src/mecanum_wand_hardware/config/hardware_acceptance.site_2026-10-22.json
```

该命令不打开硬件UDP，只输出应逐字采用的`approved_sha256`。审核通过后才运行下面的**实车入口**：

```bash
ros2 launch mecanum_wand_hardware wand_sign_acceptance.launch.py \
  mode:=hardware \
  profile:=/home/PRIMARY_USER/vicon_mecanum_ws/src/mecanum_wand_hardware/config/hardware_acceptance.site_2026-10-22.json \
  approved_sha256:=b36058e3efa4ffbec1c260bda7f3551573bcd8dfd975a8cc3e870bf186d1a10f \
  hardware_consent:=CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY
```

标题必须为“真实Sign → 实车低速验收”。启动、准备均不运动；持续按住空格并移动杆后可能运动。每次新会话只在健康、失能零输出、持续静止的4WGB状态重置使用预算；活动会话中失去MCU许可会停车锁止，不能自动重置或续跑。恢复仍需显式确认/准备/新按键边沿。软件急停会发送MCU ESTOP，并且不能代替旁边人员随时物理断电。

旧矩形的`/car/start_loop`、`/car/stop`不是本入口的启停服务，不要混用。此入口优先松开空格或点击停止；另一个终端也可向新入口请求停止：

```bash
ROS_DOMAIN_ID=94 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 service call /wand_acceptance/stop std_srvs/srv/Trigger '{}'
```

服务返回只表示请求已接收，仍须观察停车反馈。`/wand_acceptance/estop`是此域的软件急停服务；真实故障/急停不通过普通确认按钮解除。

保留已有600ms MCU断令保护。电脑/桥接进程冻结时，末端可能在约0.6秒后才归零，之后还有实际车身制动；本轮未测实车停车距离。不要误认为软件100ms签名有效期把现有固件保护缩短了。首次上电后必须实测停车、遮挡/松键和重新准备，未通过前不提高速度/扩大范围。

Tenda继续使用原接口`wlx000000000000`，软件仅按原方式绑定自己的UDP套接字，不改热点、网卡、电源管理、路由或其他联网配置。小车断电时无载波/无MCU反馈属预期，不能绕过。

## 不需要杆的回归

```bash
ros2 run mecanum_wand_hardware wand_acceptance_smoke --output /tmp/wand-mc-emulator-new
ros2 run mecanum_wand_hardware wand_acceptance_preflight --seconds 5 --output /tmp/wand-readonly-new.json
```

第一项会创建本机回环MC-Link协议仿真器和域93/94节点，不触及真实硬件；第二项只读实际Vicon/图，不创建通信桥。输出目标须尚不存在。

签名仅保护本机安全门到桥接器的权限、命令、序号和时效，不改变无线/MCU上的既有CRC协议，也不是对恶意同用户进程或攻击者的完整安全认证。MCU采样时钟进展与会话同时校验，冻结/重放/缓存数据不能作为新停车证据。MCU重启、异常时钟或硬故障后不自动恢复；先停止并检查，再重启本次验收入口。

历史准备记录见工作区`docs/WAND_HARDWARE_PREPARATION_2026-10-22.md`；当前真实Sign已接入，实车零输出及ARMED准备检查通过。实车运动验收尚未通过，不能以软件或协议仿真代替。
