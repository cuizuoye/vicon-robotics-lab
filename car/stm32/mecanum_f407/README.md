# STM32F407 D157B 分阶段台架验证工程

## 2026-10-21 当前最新：4WG7实车通过，4WG8仅离线完成

板内4WG7已完成0.30m x 0.20m、0.03m/s真实Vicon地面矩形；没有在本轮重新读回或刷写。
新的独立`mecanum_f407_four_link_ground8`把明确的线速度/轮缘上限提高到0.20m/s、
目标轮速上限提高到1200cps，并为18m路线使用150s绝对时限；35% PWM、30%受限启动、
150ms命令租约及全部故障停机保持不变。离线原生测试、ARM构建和ROS侧精确4WG8门禁通过。
本轮没有连接、复位、刷写或启动实机，详见
[4WG8离线记录](../../docs/FOUR_LINK_GROUND8_200MMPS_OFFLINE_2026-10-21.md)。

## 2026-10-21 较早状态：板内4WG5故障锁存，4WG7仅离线完成（历史）

板内仍是上次已验证写入的`4WG5`；Vicon矩形试跑后锁存`STALL (0x20)`，当前没有再次
刷写或发运动命令。未刷写的4WG6只解决小于20cps抵消项，仍保留固定30%启动模型，已停止
作为下一版候选。新的`mecanum_f407_four_link_ground7`采用每轮`kS+kV+PI`、低速积分抑制、
渐升破静摩擦、首个编码器进展立即交接、目标与输出双门槛失速；ROS门禁要求精确标签
`4WG7`。离线设计、参数边界、测试证据和硬件未验证边界见
[4WG7离线记录](../../docs/FOUR_LINK_GROUND7_OFFLINE_2026-10-21.md)。

## 2026-10-15 优先状态：板内只读，流式运动版仅离线构建

板内实际读回为26988字节无PWM只读版，SHA256：
`12d4224f5337b42d3bdc3eef30f4142e957d65d900d61b64d8b8fb22033e9aa6`。
本轮没有烧录、复位或动力测试。ESP32使用SoftAP完整桥接，连接地址192.168.50.1。
下方4WB1等“当前”描述均为历史，不应直接作为板内镜像判断。

新的显式目标 `mecanum_f407_four_link_motion`（诊断标签4WM1）排除在默认构建外：

```bash
# 在本目录，先把已安装的arm-none-eabi工具链加入PATH。
bash tests/run_motion_tests.sh
cmake -S . -B build/MotionOffline20261015 -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/MotionOffline20261015 --target mecanum_f407_four_link_motion
```

产物只是候选固件，不能据此称实车通过，更不能直接使用旧F50测试工具自动烧录。
默认读回/原bench行为保持不变；已有轮向、每轮CPR、几何和PI不重新标定。
新目标有流式速度、同组限幅、同向变速、零目标立即归零；反向先全停，收到持续新鲜指令
且实测静止500ms后才重新建立方向。串口/反馈/采样故障、150ms指令断流全部锁停。
首次运动起120s绝对期限跨换向连续计时；正常失能也消耗一次额度，需维护复位重新确认。
10%输出、50mm/s轮缘上限、800cps/s和原8%/最多200ms启动补偿保留；很低目标仍可能触发
原800/500ms无进展保护，尚未证明带载低速跟踪性能，不能直接用于无限矩形循环。

ROS使用新增显式 `bounded_motion_mode` 才支持4WM1诊断与零速基线准备。
新HELLO后至少600ms禁用零速及新鲜静止诊断，再发CONTROL_ENABLE；另需安全使能和新鲜命令。
`config/bounded_motion.yaml` 为主机显式参数覆盖文件，ESP地址默认空，未修改现有launch默认值。
原生Python/真实C内存联调覆盖此流程及断流后不可自动恢复；未触碰真机。

2026-10-12当前反馈标签为`4WB1`，对应`mecanum_f407_four_link_bench`：921600二进制MC-Link，只有显式门控后的单次F50运动（10秒、10%上限），不是下方历史通信只读镜像。无动力工具为`tools/four_link_bench_probe.py --mode no-power`，需先确认动力完全断开；不能用旧4RO1或115200台架工具。今日5秒零速检查通过、延长检查链路超时，未启用无线电机运动。本轮只改ROS2轨迹/文档，未烧录或复位。[当前记录](../../docs/FOUR_LINK_RECONNECT_2026-10-12.md)、[矩形与Vicon计划](../../docs/RECTANGLE_AND_VICON_PLAN_2026-10-12.md)。以下“最新/当前”均为历史。

2026-10-10最新板内状态：`mecanum_f407_four_link_readonly`，921600二进制MC-Link；电机脚GPIO低、不配置PWM。断动力烧录及独立读回通过，ESP32无线四轮回包/ROS2话题通过；150ms只读会话过期/新握手恢复通过，未发运动使能。当前USB-TTL仅供电，ESP32 GPIO17→PD6、GPIO18←PD5，共地；不要使用下面115200旧文本工具。当前入口`tools/four_link_ro_probe.py`/`tools/ros_four_link_ro_observe.py`，仍需保持动力断开。[完整记录](../../docs/FOUR_LINK_READONLY_2026-10-10.md)。下方板内“当前”均为历史。

2026-10-10最新状态：已备份并烧录独立`rr_diag_328_v1`（115200文本），唯一一次RR右后轮2倍目标328cps/约9.7秒悬空对照完成，971条100Hz记录；实测后段约327.13cps/18.50rpm，是上一轮的1.98倍，现场卡顿减轻或消失。10%上限/PI/其它轮零输出/停机保护不变；四轮自行停止、用户已断动力，60帧复核100Hz、8CCR=0、fault/en/wdg_reset=0、used=1。235项回归及烧录独立读回通过，原五个BIN不变。未做3倍或落地/无线/Vicon控制；当前只读用`rr_speed2_probe.py --mode post-run --confirm-battery-disconnected`，不自动复位/重试；下方旧“当前”均为历史。[本次记录](../../docs/RR_SPEED2_2026-10-10.md)。

2026-10-10当前状态更新：`rr_diag_pwm01_v1`、115200文本，唯一一次RR164/9.7秒悬空对照及971条100Hz记录已完成；程序检查通过，但用户报告卡顿目视基本相同并已断动力，**物理平顺性未通过**。断动力60帧复核通过：100Hz、四轮计数稳定、8CCR=0、fault/en/wdg_reset=0、used=1。本轮没有再复位/重试/烧录，当前只读用`tools/rr_pwm_fine_probe.py --mode post-run --confirm-battery-disconnected`；后续恒定PWM对照尚未实施。[最新对照与下一步](../../docs/RR_PWM_FINE_2026-10-10.md)。下方used=0/未转轮为历史，不能据此再启动。

2026-10-10当前板内：独立`rr_diag_pwm01_v1`、115200文本；仅RR的PWM步进从1%细化为0.1%，目标164cps/原PI/10kHz频率配置/10%上限/停机保护不变。完整备份、烧录验证、独立读回和60帧无动力检查通过：实际TIM12 ARR=8399/PSC=0、100Hz、8CCR=0、fault/en/used=0。**旧基线卡顿仍在，新版尚未转轮，不算修复验收。** 当前只读用`tools/rr_pwm_fine_probe.py --mode no-power --confirm-battery-disconnected`，旧RR/startup/drive工具不适用。230项主机回归及新旧C安全/量化验证通过，四个旧BIN不变；未调整网络。[当前结果与下一步](../../docs/RR_PWM_FINE_2026-10-10.md)。下方保留历史，不能据此启动旧动作。

2026-10-10最新实机集成：已完整备份并烧录独立RR诊断 `rr_diag_start8_v1`，编程验证/独立读回均一致；60帧无动力stop/status检查通过、100Hz、fault=0/en=0/used=0、8个实际CCR均零。**没有上动力或发hb/run，未验证右后轮平顺性。** 用户切换电脑USB口后旧串口复测通过，首次乱码原因仍未确定；未调整网络。当前只读用 `tools/rr_diagnostic_probe.py --mode no-power --confirm-battery-disconnected`，旧startup/drive工具不再用于当前镜像。下一次需新确认上电、悬空静止，才做唯一一次RR164试转，不自动重试。[集成/备份/当前入口](../../docs/RR_DIAGNOSTIC_INTEGRATION_2026-10-10.md)。下方为历史阶段。

2026-10-10最新软件准备：新增独立RR单轮诊断 `rr_diag_start8_v1`，复用原接线/CPR/PI/PWM保护，只允许RR正向164cps、约10秒一次动作，其它三轮零输出；100Hz记录停机后导出。224项主机测试、RR C安全/记录及旧启动安全回归、ARM构建通过，三个旧BIN校验值完全不变。**尚未烧录/复位/发串口命令/转轮，板内仍是下方旧start8记录。** 新工具不能用于当前旧镜像；先确认断动力再备份、烧录及无动力检查。[准备说明](../../docs/RR_DIAGNOSTIC_PREP_2026-10-10.md)。

2026-10-10当前阶段：板内仍为`drive_pi_start8_v1`，115200文本。断动力复位准备后，只执行一次yaw200约10秒；软件检查通过、最大采样输出8%，现场报告右后轮全程轻微速度脉动，平顺性待排查，不进入落地。用户确认全部停轮断动力，随后60帧只读稳定、fault=0/used=1/en=0、8个CCR为零；额度已用，不自动复位/重试。仅主机增加无损日志`--summary-only --include-trace`，213项回归通过，未更改固件/PI/PWM。[当前记录](../../docs/YAW200_BENCH_2026-10-10.md)。当前只读为startup工具post-run；下方均为历史快照。

2026-10-10上一阶段（左移历史）：板内为独立`drive_pi_start8_v1` / `mecanum_f407_drive_startup_observe`、115200文本。旧故障镜像完整备份可恢复，新BIN烧录校验和独立读回一致；8%/最多200ms启动补偿、10秒观察、总输出仍不超过10%，首次新镜像悬空left50通过，采样最大9%。现场确认FL/RR朝车尾、FR/RL朝车头，约10秒后自行停止，无明显异常并已断动力。随后只发stop/status，60帧计数稳定、100Hz、`fault=0/used=1/en=0`、8个CCR为零；没有第二次动作、复位或清故障。当前只读入口为`tools/drive_startup_probe.py --mode post-run --confirm-battery-disconnected --summary-only`。211项Python、新8组/旧9组C安全回归通过。[当前完整记录](../../docs/DRIVE_STARTUP_OBSERVE_2026-10-10.md)、[历史失败](../../docs/DRIVE_PI_LEFT50_FAILURE_2026-10-10.md)。仍未验证落地侧移/旋转、无线运动、Vicon外环；下方旧drive/forward/speed工具及“当前”描述均属历史，不用于新镜像。

2026-10-10较早软件阶段（历史）：新增`mecanum_drive_plan.h/.c`与SI正/逆运动学，用实测近似轴距122mm/轮距170mm及确认的X型安装生成各轮归一化cps。只新增`mecanum_drive_planning`静态库，未链接当前前进镜像，未烧录、未重新读取板内状态、未转轮；最近一次保存的前进终态仍为used=1。6组C安全检查/441模型回环、2000组C/Python一致性、ARM库和ROS85项测试通过，原台架工具194项回归通过。这不是侧移/旋转实机验收；[完整记录](../../docs/MECANUM_DRIVE_PLAN_2026-10-10.md)。下方当前固件描述属于上一次实机验证记录。

2026-10-10较早前进版状态（历史）：板内为独立四轮前进速度环`forward_pi_v1`，115200文本协议；已备份、烧录、读回一致并完成唯一一次50mm/s目标、约4.7秒悬空联动，最大输出8%，现场确认四轮朝前、自行停止、无明显异常且已断动力。最终100Hz采样、8个CCR为零、`fault=0/used=1`，不能再次启动。新增目标`mecanum_f407_forward_bench`，仅使用`tools/forward_pi_probe.py`；禁止用历史控制台或MC-Link桥直接发动作。共用初始PI、保留各轮CPR/符号，不重复逐轮整定；这不是负载/落地/侧移旋转/无线/Vicon运动验收。任一轮故障全组停止且锁存，不自动重试/清故障/复位。[集成与首次联动证据](../../docs/FOUR_FORWARD_PI_INTEGRATION_2026-10-10.md)。

历史前进版断动力只读入口（不适用于当前start8镜像；新入口见顶部及新记录）：

```bash
python3 -B tools/forward_pi_probe.py /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --mode post-run --confirm-battery-disconnected
```

历史前进版当时额度已使用；不要再次上电试转。`no-power`/`powered-stationary`/`forward50`仅适用于以后经计划并在断动力状态准备好的未使用镜像状态，不适用于当前`used=1`。正常运行后断动力检查用上述post-run模式；有故障时保留记录，不复位重试。烧录/复位始终先断动力。下方均为历史阶段，旧“当前”描述只适用于各段所标日期。

2026-10-09后续最新状态：板内仍是受限`speed_pi_v2`。左前轮首次悬空300cps PI、UART断流后在主机stop之前自主全归零/故障锁存及现场停轮反馈通过；当前动力拔掉，`fault=4/used=1`锁存。此时仅用`tools/speed_pi_link_loss_probe.py --mode latched --confirm-battery-disconnected`做stop/status检查，普通post-run工具会拒绝这一故障。下次动作须先在断动力状态复位/检查，再确认上电；不要在线清故障。已按用户要求共用初始PI增益、保留各轮CPR/符号，不再重复逐轮整定。新增`four_forward_group.h/.c`只完成离线C/ARM目标文件编译，未加入CMake固件目标、未烧录、未验证四轮速度联动；[最新记录与下一步](../../docs/SPEED_PI_COMMAND_LOSS_2026-10-09.md)。下方是历史阶段，不代表当前板内状态；勿用旧协议发动作。

2026-10-09较早PI阶段（历史）：首次左前轮300cps后段滤波平均307.94cps，全程遥测最大PWM7%，未做负载或充分整定；当时停机后fault=0/used=1。[首次完整记录](../../docs/SPEED_PI_FL_FIRST_TRIAL_2026-10-09.md)。

2026-10-09较早只读阶段（历史）：当时板内为独立`speed_ro_v1`速度采样只读版（115200文本），不能启动电机。旧`four_bench_v1`已完整备份；只读BIN烧录及读回一致。断动力和上电静止5秒检查均通过：采样/测速100Hz、周期10ms、GPIO低、8个实际CCR为0、无故障。记录见[只读集成验收](../../docs/SPEED_READONLY_INTEGRATION_2026-10-09.md)。下面旧固件命令只属历史记录，不可用于当前板内程序。

新增`four_wheel_speed.h/.c`速度换算/独立PI算法、100Hz TIM6采样及非阻塞UART。有独立目标`mecanum_f407_speed_readonly`和`mecanum_f407_speed_bench`；后者已构建但未烧录，实机速度闭环尚未验证。用户采用FL1059.67/FR1049/RL1055.67/RR1061作为台架近似CPR，保留测量与重复性未认证事实；只在独立台架初始化中采用，算法库默认CPR=0/未确认/零增益仍锁定。原四轮台架BIN未变；[软件与集成边界](../../docs/FOUR_WHEEL_SPEED_SOFTWARE_2026-10-09.md)。

历史只读镜像的观察入口（不适用于当前PI版；当前使用speed_pi_probe.py的post-run模式）：

```bash
python3 -B tools/speed_readonly_probe.py /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --confirm-battery-disconnected
```

现场明确确认两板上电、四轮悬空且静止后，可改用`--confirm-powered-readonly`做只读静止观察；两种确认选项互斥。仍不接受任何启动命令。烧录或复位前必须先断动力。

2026-10-08交付更新：用户暂停继续硬件推进，已准备独立`mecanum_f407_front_demo30`展示目标与
`tools/front_wheels_demo30.py`，两前轮A+/B-固定10%、单次30秒请求后全停。
仅完成构建和离线测试，未烧录、未做30秒实机验证；当前板内仍是下述dual_v2。
原台架镜像重建后与已烧录读回逐字节一致，时限未放宽。使用边界见[30秒演示说明](../../docs/FRONT_WHEELS_DEMO30.md)。

2026-10-08最新：单A正向与编码器原始增量已实测（+1720），CPR尚未标定。
目前板内已切换为独立`mecanum_f407_dual_bench`、115200的`dual_v2`协议，
烧录/读回与A/B零输出检查通过；B独立及A/B同时+10%限时请求串口通过。
用户确认共同车头参考下左右轮向相反，并已停转断电；主机新增`both-forward`候选组合A+100/B-100。
用户希望延长方向观察，另授权断电后备份更新；118项Python与12组C离线检查通过。
专用`both-forward-observe`固定A+100/B-100、约5秒，保留500ms心跳与5.2秒软件上限；普通测试仍约2秒。
新固件已烧录、读回一致，无动力检查通过；用户重新确认上电静止后，仅执行一次约5.009秒A+/B-请求，串口通过。
接线不变，用户已确认两轮朝车头且自行停止；当前前进符号FL/A正、FR/B负。
本轮最终断电已由用户确认；仅A反向前的无动力检查通过。
用户另确认上电静止并许可后，已仅执行一次A反向10%、约2.008秒、B保持0的请求，串口通过、A计数增量-1541。
用户确认“确定只有左前轮向后。流程正常结束”；实际A后退、B未转动已反馈。
用户随后确认关闭开关、拔掉电池、两轮停止；本轮1→2→3台架顺序结束，未再发动作。
FL编码器正反符号已验证；FR编码器、CPR、后轮、速度闭环及落地控制仍未验证。
当前固件只使用`tools/dual_motor_bench_probe.py`，不要用下面历史单A控制台/协议直接发送动作。
详情见[双通道台架记录](../../docs/DUAL_MOTOR_BENCH_2026-10-08.md)。

下方先介绍仍保留的原单A目标，再介绍其它独立目标；新台架目标不会覆盖原单A源码与镜像。

这是麦克纳姆小车的第一阶段固件，只验证一块D157B的Motor_A：

- `PE5/TIM9_CH1 -> AIN1`
- `PE6/TIM9_CH2 -> AIN2`
- `PB4/TIM3_CH1 <- E1A`
- `PB5/TIM3_CH2 <- E1B`
- `PD5/USART2_TX -> USB-UART RX`
- `PD6/USART2_RX <- USB-UART TX`
- STM32、D157B和USB-UART必须共地。

电机插在D157B的Motor_A白色六针口。STM32由调试器/USB供电时，不要把D157B的5V与开发板5V并联。

2026-10-08 已烧录本工程的 `115200` 文本镜像，并完成驱动板电池拔掉时的
启动及串口复测：两轮共20次状态查询全部回应，遥测10 Hz，输出始终未使能。
这不代表电机或编码器已验证；备份、读回校验及范围详见
[本次验证记录](../../docs/SINGLE_MOTOR_UART_VALIDATION_2026-10-08.md)。
当前板内文本镜像不能直接与原 `921600` 二进制 MC-Link ESP32 桥接固件配合。
下一步接线、电平检查及手动计数顺序见
[Motor_A 编码器检查清单](../../docs/MOTOR_A_ENCODER_BRINGUP.md)。

## 安全边界

- 上电、未使能、急停和通信超时均为`AIN1=0, AIN2=0`高阻停机。
- 正常零速可使用`AIN1=1, AIN2=1`动态制动；驱动采用10 kHz单极性慢衰减。
- 调试占空比被硬限制在`+-200 permille`，即`+-20%`。
- 必须先持续发送`hb`，再发送`arm A5`才能输出。
- 心跳超过500 ms、串口接收溢出或`estop`都会关闭PWM并锁存故障。
- 正反转切换强制经过5 ms零输出死区。
- 原单A已完成限时正向与计数试转，速度PI及负载能力未验证；每次台架测试仍须架空固定车架。

## 编译

```bash
export PATH=/home/PRIMARY_USER/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH
cmake --preset Debug
cmake --build --preset Debug
```

产物位于`build/Debug/`。烧录前应保留当前固件备份。

主机命令解析测试：

```bash
cc -std=c11 -Wall -Wextra -Werror -IInc tests/test_command.c Src/command.c -o /tmp/mecanum_command_test
/tmp/mecanum_command_test
```

## 串口命令

USART2使用`115200 8N1`。命令以换行结束：

```text
hb            刷新500 ms安全心跳
arm A5        无故障且心跳有效时使能
duty 100      正向10%
duty -100     反向10%
speed 1000    闭环目标+1000编码器计数/秒（初始PI参数须实车整定）
brake         正常制动；仅在已使能状态可用
stop          输出归零并解除使能
estop         输出归零并锁存急停故障
clear 5A      停机状态下清除故障
zero          编码器累计值清零
status        立即输出状态
help          输出命令提示
```

遥测每100 ms输出一次：

```text
TEL,ms=<时间>,en=<使能>,hiz=<高阻>,mode=<duty/speed>,target_cps=<目标>,duty=<千分比>,enc=<累计计数>,cps=<计数/秒>,fault=<故障位>
```

可使用`tools/motor_a_console.py /dev/ttyUSB0`自动发送心跳并交互控制。

仅验证通信且不使能时，优先使用 `tools/motor_a_safe_probe.py`：它只发送
`stop/status`，不会自动发送心跳、使能或速度指令。保持电机动力断开后运行：

```bash
python3 tools/motor_a_safe_probe.py \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --duration 20 --queries 10
```

## ESP32-S3到STM32安全链路测试

工程同时生成独立的`mecanum_f407_link_test`镜像。这个镜像只初始化：

- `PD5/USART2_TX`
- `PD6/USART2_RX`
- `PB2`状态灯

它不会初始化TIM3、TIM9或任何电机PWM。串口使用`921600 8N1`，接收
`COBS(MC-Link帧) + 0x00`；只有收到CRC32正确、无载荷且`session_id`非零的
`HELLO`，才返回对应的`HELLO_ACK`并翻转一次状态灯。

接线：

```text
ESP32 GPIO17/TX -> STM32 PD6/USART2_RX
ESP32 GPIO18/RX <- STM32 PD5/USART2_TX
ESP32 GND       -- STM32 GND
```

两端均为3.3 V逻辑。本测试不要连接两块开发板的5 V引脚，也不要连接电机
驱动器。烧录产物为：

```text
build/Debug/mecanum_f407_link_test.bin
build/Debug/mecanum_f407_link_test.hex
```

ESP32启用UART后不再本地生成`HELLO_ACK`。因此PC端收到合法响应，就证明数据已
完整经过`PC -> ESP32 -> STM32 -> ESP32 -> PC`。

主机侧协议向量测试：

```bash
cc -std=c11 -Wall -Wextra -Werror \
  -IInc -I../../protocol tests/test_mc_protocol.c Src/mc_protocol.c \
  -o /tmp/mc_protocol_test
/tmp/mc_protocol_test
```

## MC-Link单电机正式链路镜像

`mecanum_f407_motor_link`是独立的第三个镜像，保留前两个镜像不变：

| 镜像 | 串口 | 电机外设 | 用途 |
|---|---:|---|---|
| `mecanum_f407` | 115200文本 | TIM3/TIM9 | 架空后的人工单电机整定 |
| `mecanum_f407_link_test` | 921600二进制 | 不初始化 | 已验证的通信回退镜像 |
| `mecanum_f407_motor_link` | 921600二进制 | TIM3/TIM9 | MC-Link状态机、速度环与遥测 |
| `mecanum_f407_dual_bench` | 115200独立dual_v2文本 | TIM3/TIM9/TIM1 | 显式A/B/AB固定10%台架；普通2秒；5秒A+/B-串口、用户共同前进/自行停止确认通过 |
| `mecanum_f407_front_demo30` | 115200独立demo30_v1文本 | TIM3/TIM9/TIM1 | 独立A+/B-10%单次30秒展示，构建/离线测试通过，尚未烧录 |

新镜像支持`HELLO`、`CMD_VEL`、`CONTROL_ENABLE`、`ESTOP`和10 Hz
`WHEEL_TELEMETRY`，并检查CRC、会话、序号和每个速度帧携带的超时。超时、急停、
串口溢出、无效配置均使AIN1/AIN2进入高阻。

当前`Inc/app_config.h`中的
`MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV`有意保持为`0`。这表示尚未实测输出轴每圈
编码器计数；固件会设置`FAULT_CONFIG_UNCALIBRATED`并拒绝所有带`ENABLED`的运动
命令，但HELLO和遥测仍可正常验证。不要用商品页推测值替代实测值。

构建产物：

```text
build/Debug/mecanum_f407_motor_link.bin
build/Debug/mecanum_f407_motor_link.hex
```

状态机主机测试使用非零的测试专用计数值，不会改变正式固件配置：

```bash
cc -std=c11 -Wall -Wextra -Werror \
  -DMOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV=900U \
  -IInc -I../../protocol \
  tests/test_motor_link_control.c Src/motor_link_control.c Src/mc_protocol.c \
  -o /tmp/motor_link_control_test
/tmp/motor_link_control_test
```

四轮 X 型麦克纳姆逆运动学使用固定轮序 `FL, FR, RL, RR`，车体坐标遵循
ROS REP-103：`+x` 向前、`+y` 向左、`+yaw` 逆时针。它先输出各轮轮缘线速度，
再由后续四轮固件结合实测编码器每圈计数换算为 CPS；任何单轮超限时四轮使用
同一比例缩放，保持期望运动方向。主机测试：

```bash
cc -std=c11 -Wall -Wextra -Werror -IInc \
  tests/test_mecanum_kinematics.c Src/mecanum_kinematics.c \
  -o /tmp/mecanum_kinematics_test
/tmp/mecanum_kinematics_test
```
