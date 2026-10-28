# 交接文档：Vicon 麦克纳姆小车 —— 下一步单电机台架与编码器标定

生成时间：2026-09-30
适用对象：接手本项目的 AI agent（以下简称"你"）

> 2026-10-08 状态更新（优先于下文 09-30 的运行状态）：STM32 现在运行
> `mecanum_f407` 的 115200 文本调试镜像，不是二进制 MC-Link。
> 电池已拔掉，USB-TTL ↔ USART2 串口复测通过；编码器电平、每圈计数和电机转动
> 尚未验证。文本镜像没有未标定使能锁，禁止依据下文旧锁止状态直接上动力。
> 当前证据见 [串口验证记录](SINGLE_MOTOR_UART_VALIDATION_2026-10-08.md)，
> 下一步见 [编码器检查清单](MOTOR_A_ENCODER_BRINGUP.md)。不要修改用户代理或网络连接。

---

## 0. 你的任务

在一个已完成约 70% 的项目上，**完成单电机台架测试并实测编码器每圈计数**，解开当前唯一的硬阻塞。

**不要**重新勘察项目全貌、不要重做已完成的部分、不要先修网络问题。本文档已经替你完成勘察，
第 9 节有文件地图，第 6 节是你要执行的步骤。

---

## 1. 工作区与铁律

- 工作区：`/home/PRIMARY_USER/vicon_mecanum_ws`
- ROS 2 **Jazzy**，Python 3.12
- **该目录不是 git 仓库**（无版本控制）。动手改代码前建议先 `git init` + 首次提交，
  `.gitignore` 至少包含 `build/ install/ log/ **/__pycache__/ .pytest_cache/`；
  ESP32 侧已忽略 `sdkconfig`（含 Wi-Fi 凭据，不要提交）。
- 本工作区是 `/home/PRIMARY_USER/vicon_receiver_ws` 的 **overlay**。**不要修改 `vicon_receiver`**，
  也不要复用 PX4/Crazyflie 的节点。
- 涉及真实电机、烧写固件、改动硬件接线的操作，**先向人类确认**再执行。
- 主控板已确认为 **立创·天空星 LCKFB-LSPI-SkyStar-STM32F407VGT6**，人类已长期使用该板并
  完成过其它项目，**板子本身是就绪的，不要再提焊接、排针、上电准备之类的基础事项**。
  原理图备查：`wiki.lckfb.com`（路径 `zh-hans/tkx/tkx-stm32f407vxt6/`）与立创商城商品页。

---

## 2. 项目目标与架构

用 Vicon 提供全局位姿，在 PC 的 ROS 2 里做目标生成、轨迹跟踪和安全监督；
PC 通过 Wi-Fi 下发车体速度，经 ESP32-S3 转发给 STM32F407；
STM32 负责麦克纳姆逆运动学、四轮编码器速度闭环和最终安全停机。

```text
Vicon -> vicon_receiver -> /car/pose -> ROS2 位姿控制器 -> /car/cmd_vel
Linux PC --UDP/MC-Link--> ESP32-S3 --UART/MC-Link--> STM32F407
                                        -> 2 x D157B/AT8236 -> 4 x MG310 -> 60 mm 麦轮
```

MC-Link 协议版本 `MC_PROTOCOL_VERSION 1`（`protocol/mecanum_protocol_v1.h`）。

---

## 3. 当前状态（含证据）

| 层 | 状态 | 证据 |
|---|---|---|
| ROS 2 (PC) | ✅ 功能完整 | 位姿桥接、位姿控制器、安全看门狗、RViz 显示、圆形/矩形/8字轨迹、软件在环、MC-Link 编解码、`cmd_vel_udp_bridge` UDP 节点均已实现 |
| ROS 2 测试 | ✅ 41 个 pytest 用例全通过 | `.pytest_cache/v/cache/nodeids` 41 条，`lastfailed` 为 `{}`，最后一次运行 09-30 16:35 |
| ESP32-S3 | ✅ 已实板烧录 | Wi-Fi STA、UDP 46000/46001、UART1 921600、会话/序号/CRC 校验、突发折叠、750 ms 会话看门狗 |
| STM32 通信镜像 | ✅ 已烧录验证 | `mecanum_f407_link_test`：只做 HELLO/ACK，不初始化 PWM |
| STM32 单电机镜像 | ⚠️ 已烧录并做过无电池在线验证 | `mecanum_f407_motor_link`：MC-Link 状态机 + 编码器 PI + 10 Hz 遥测；运动输出被未标定故障锁止 |
| STM32 文本镜像 | ✅ 已编译，可直接用于台架 | `mecanum_f407`：115200 文本命令，**无未标定锁**，可开环给占空比 |
| STM32 四轮正式固件 | ❌ 未实现 | 仅有 `mecanum_kinematics.c` 的逆运动学（主机测试通过）和引脚规划文档 |
| 硬件实物 | ⚠️ 已到手，可开工 | 电机、D157B、3S 电池、车架已确认可用，车架可架空；但从未带载验证，编码器供电电平未测 |
| 主控板 | ✅ 已就绪 | 天空星 SkyStar-STM32F407VGT6，人类已长期使用并完成过其它项目；引脚与固件的对应关系接线前对照一次即可（见 Step 0b） |
| 三线 UART | ✅ 已物理连接 | `docs/NO_BATTERY_SAFETY_VALIDATION_2026-09-30.md` 记录了实板在线往返数据 |

**最后活动时间：2026-09-30 16:58。**

---

## 4. 唯一的关键阻塞

```c
/* stm32/mecanum_f407/Inc/app_config.h:15 */
#define MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV 0U   /* 有意为 0：尚未实测 */
```

`motor_link_control_init()` 在收到 `calibrated == false` 时置位
`FAULT_CONFIG_UNCALIBRATED`（`Src/motor_link_control.c:20`），**拒绝所有带 `ENABLED` 的运动命令**。
`clear_faults` 用 `control->fault_bits &= FAULT_CONFIG_UNCALIBRATED;`（`Src/motor_link_control.c:106`）
只清除可恢复位、保留该位 —— 这与 09-30 实测的 `fault_bits 0x0E --clear--> 0x08` 完全吻合。

**后果：在测出这个数字之前，四轮固件、轮速闭环、Vicon 外环一步都动不了。**

⚠️ **该锁只存在于 `motor_link` 镜像。** 文本镜像 `mecanum_f407` 不做此检查，
所以台架开环测试**不需要**先标定即可进行 —— 这正是标定手段。

---

## 5. 为什么现在**不**先修 UDP 突发问题

09-30 的无电池验证发现手机热点 UDP 突发导致 STM32 侧 UART 溢出（故障位 `0x0E`）。
**不要把它当作当前的首要任务**，理由：

1. **它没有下游消费者。** 即使把链路修到零丢包，第 4 节的锁仍然让任何运动都无法使能。
2. **它与台架测试天然互斥。** PD5/PD6 上的 USART2 在"115200 文本控制台"和"921600 MC-Link"
   之间是**同一个物理引脚**，二者互斥。做台架必须拔掉 ESP32，所以热点突发**不可能**干扰台架测试。
3. **它的正解可能是环境而非代码。** 该问题源于手机热点。换一台独立 2.4 GHz 路由器
   （或 ESP32 开 SoftAP + PC 固定 IP）大概率直接消灭这一整类问题，成本远低于继续在
   桥接节点里堆补偿逻辑。

什么时候回来做它：**第 6 节 Step 6 之后**，即重新接上 ESP32 做无电池在线复测时。

---

## 6. 你的任务：分步执行

前置：STM32 + 一块 D157B + 一个 MG310 电机 + USB-UART 转接器。**不需要底盘其它部件**，桌面即可。

### Step 0 — 开工前的两项确认

**0a. 备份当前固件。** 当前板上应为 `mecanum_f407_motor_link`（09-30 在线验证用到遥测），
但 **16:55 重建的版本没有对应验证记录，板上实际版本未确认** —— 先确认再动手。

**0b. 接线前对照一次引脚。** 开发板本身是就绪的，不需要任何准备工作。只需把固件写死的
引脚与实物排针丝印对一遍，确认接线接的是同一组信号：

| 用途 | 引脚 | 外设 |
|---|---|---|
| PWM | `PE5`, `PE6` | 单电机 A（TIM9_CH1/CH2） |
| 编码器 | `PB4`, `PB5` | 编码器 A/B（TIM3_CH1/CH2） |
| 串口 | `PD5`, `PD6` | USART2（台架时接 USB-UART，Step 6 时接 ESP32） |

同时确认这 6 个脚没有被板载外设占用。人类熟悉这块板，直接问即可，不用查原理图。

若发现某个脚已被占用，**不要自行飞线或改板**：停下来报告，由人类决定换脚还是换板。
改引脚涉及 `stm32f4xx_hal_msp.c` 和全部定时器配置，是独立改动，不属于本次交接范围。

> 顺带一提（不急，四轮阶段再处理）：`docs/STM32_FOUR_WHEEL_PIN_PLAN.md` 的四轮方案需要
> **18 个引脚**（PE5/6/9/11/13/14、PB14/15、PB4/5/6/7、PA15、PB3、PC6/7、PD5/6），
> 而这块板只引出约 40 个 GPIO。届时可能需要重新选脚，值得提前和人类确认一次。

### Step 1 — 不接电机，验证 PWM 波形
烧写文本镜像 `mecanum_f407`。不接驱动板和电机，用逻辑分析仪/示波器看 `PE5`/`PE6`：
- 10 kHz；
- 复位期间保持低电平；
- 上电后未 `arm` 时双向均为低（高阻停机）。

### Step 2 — 测编码器供电电平（**接线前必做**）
用万用表测 D157B 的 `E1A`/`E1B` 高电平：
- ≈3.3 V → 可直连并共地；
- ≈5 V → 必须先确认所用 STM32 引脚具 5 V 容限且 MCU 已上电，否则加分压/电平转换；
- 绝对不要把 5 V 接到 MCU 电源脚。

依据：`docs/STM32_FOUR_WHEEL_PIN_PLAN.md:49`。

### Step 3 — 接电机，架空，低速确认转向
接线：`PE5/TIM9_CH1 -> AIN1`、`PE6/TIM9_CH2 -> AIN2`、`PB4/TIM3_CH1 <- E1A`、`PB5/TIM3_CH2 <- E1B`，
STM32 / D157B / USB-UART **共地**。

3S 电池从 **±5% 占空比**起步（固件硬限 ±20%）。用 `tools/motor_a_console.py` 自动发心跳：
```bash
python3 tools/motor_a_console.py /dev/ttyUSB0   # 端口按实际改
```
发送 `hb` → `arm A5` → `duty 50` / `duty -50`，确认：
- 正占空比 → `cps` 为正；反向亦然。**记录是否需要 `encoder_sign` 取反**（不要改固定轮序和运动学公式）。
- 反转时经过 5 ms 零输出死区（`MOTOR_DIRECTION_DEADTIME_MS`）。

### Step 4 — 实测编码器每圈计数（**核心交付物**）
注意是 **1:20 减速后的输出轴**，不是电机轴。
```
zero           # 清零累计值
# 用手把输出轴精确转整 1 圈（或 10 圈以提高精度）
status         # 读 enc=<累计计数>
```
建议转 10 圈再除以 10，降低手转误差。**不要用商品页推测值替代实测值。**

### Step 5 — 固化标定值并解锁
1. 把实测值写入 `Inc/app_config.h` 的 `MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV`；
2. 重建 `mecanum_f407_motor_link`：
   ```bash
   export PATH=/home/PRIMARY_USER/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:$PATH
   cmake --preset Debug && cmake --build --preset Debug
   ```
3. 烧录，确认 `FAULT_CONFIG_UNCALIBRATED`（`0x08`）消失，且带 `ENABLED` 的命令不再被拒。
4. 架空做阶跃测试，整定 `SPEED_KP_*` / `SPEED_KI_*`（当前是保守初值，见 `app_config.h`）。

### Step 6 — 重新接上 ESP32，复测链路（此处才处理第 5 节的问题）
接回三线 UART，复位两板，ESP32 USB 日志应从 `WAITING` 变 `PASS`。
重做无电池安全验证，目标是**零 UART 溢出**。

**已确认现场没有独立路由器**，只有手机热点，因此第 5 节的抖动问题在 Step 6 会真实出现。
按成本从低到高，先试前两个再考虑改代码：

1. **改用手机 USB 网络共享（tethering）代替 Wi-Fi 热点。** 零成本，通常比热点稳定得多。
2. **把 ESP32 改成 SoftAP 模式，PC 直连 ESP32 自建的热点。** 零成本，且彻底绕开手机热点
   和它的省电/排队策略——链路里只剩 ESP32 一个 AP，抖动来源最少。**这是首选方案。**
3. 若前两者仍溢出，再考虑降低周期帧发送率、调整 ESP32 接收缓存与会话看门狗。
   STM32 的 150 ms 独立停机边界**不要为了迁就网络而放宽**。

---

## 7. 硬性安全约束（违反会烧硬件或伤人）

1. **首次任何电机上电，车轮必须架空。**
2. **本车架目前没有可触及的急停开关，断电靠手动关停驱动板。**
   这意味着物理断电是**最慢**的停止手段。台架阶段的快速停止顺序应该是：
   先发 `estop` 命令 → 拔掉 USB-UART（触发 500 ms 心跳超时）→ 最后才去关驱动板电源。
   架空状态下车轮空转风险低，但**任何带载落地测试之前必须先加装可触及的总开关/急停**。
3. 未确认方向前，占空比从 ±5% 起，不要直接给大值。
4. 上电、未使能、急停、通信超时的默认状态必须是 `AIN1=0, AIN2=0` 高阻。
   `1/1` 动态制动只能在悬空单轮确认电流温升后使用。
5. 电池、两块驱动、STM32、ESP32 **必须共地**；电机动力线与逻辑降压支路从配电点分开。
6. ESP32 与 STM32 之间**只接 TX/RX/GND**，不要互接 5 V/3.3 V 电源。
7. STM32 侧的 150 ms 独立停机边界是最后一道安全层，**不要放宽**它。
8. 不要在 STM32Cube 工程目录里执行 ESP-IDF 的 Build/Full Clean（会破坏 CMake 目录）。

---

## 8. ⚠️ 文档矛盾警告（先读，否则会被误导）

现有文档**互相矛盾**，以本表为准：

| 位置 | 过时说法 | 实际 |
|---|---|---|
| `README.md:178` | "ESP32与STM32尚未物理连接" | **已连接**，09-30 文档记录了实板在线往返数据 |
| `README.md:54`（路线图） | "ROS/Python 离线测试 25 passed" | 实际 **41 个用例**（参数化后），全通过 |
| `README.md:194` | "单电机固件…尚未烧录" | `motor_link` 已烧录并完成无电池在线验证 |
| `docs/项目现状与路线图_2026-09-28.md:124` | "拔掉任一链路后进入安全状态" | 已实现，且已修过"遥测存在但命令序号不推进"的假在线误判 |

权威顺序：本文档 > `docs/NO_BATTERY_SAFETY_VALIDATION_2026-09-30.md`（最新实测） >
`docs/项目现状与路线图_2026-09-28.md` > `README.md`（最易过时）。

---

## 9. 关键文件地图

```text
protocol/mecanum_protocol_v1.h              # 双方共享的线格式定义（权威）

src/mecanum_vicon_control/                  # ROS 2 包
  mecanum_vicon_control/cmd_vel_udp_bridge.py   # PC<->ESP32 UDP 桥（含会话恢复逻辑）
  mecanum_vicon_control/mc_link_session.py      # 会话/序号状态
  mecanum_vicon_control/transport_protocol.py   # COBS/CRC32/帧编解码
  mecanum_vicon_control/mecanum_kinematics.py   # 麦轮运动学（Python 版）
  launch/{software_in_loop,hardware_bridge,vicon_control}.launch.py
  config/control.yaml                           # 所有可调参数集中在此

stm32/mecanum_f407/
  Inc/app_config.h                        # ← 标定值写在这里（第 4 节）
  Src/{motor_a,encoder_a,command,speed_controller}.c   # 文本镜像（台架用）
  Src/{motor_link_control,motor_link_main}.c           # MC-Link 镜像
  Src/mecanum_kinematics.c                # 四轮逆运动学（已测，未接四轮固件）
  tools/motor_a_console.py                # 台架主机控制台
  tests/                                  # 6 个主机侧单元测试

esp32/mecanum_wifi_bridge/main/app_main.c   # 桥接主逻辑
docs/                                       # 所有设计文档
```

三个 STM32 镜像互不干扰，同时构建：

| 镜像 | 串口 | 电机外设 | 用途 |
|---|---:|---|---|
| `mecanum_f407` | 115200 文本 | TIM3/TIM9 | **台架人工整定（你用这个）** |
| `mecanum_f407_link_test` | 921600 二进制 | 不初始化 | 通信回退镜像 |
| `mecanum_f407_motor_link` | 921600 二进制 | TIM3/TIM9 | MC-Link + 速度环 + 遥测 |

---

## 10. 验证命令

```bash
# ROS 侧单元测试（41 用例）
cd /home/PRIMARY_USER/vicon_mecanum_ws/src/mecanum_vicon_control && python3 -m pytest test/ -v

# STM32 主机侧测试（不需要硬件）
cd /home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407
cc -std=c11 -Wall -Wextra -Werror -IInc tests/test_command.c Src/command.c -o /tmp/t && /tmp/t
cc -std=c11 -Wall -Wextra -Werror -IInc -I../../protocol \
   tests/test_mc_protocol.c Src/mc_protocol.c -o /tmp/t && /tmp/t
cc -std=c11 -Wall -Wextra -Werror -IInc \
   tests/test_mecanum_kinematics.c Src/mecanum_kinematics.c -o /tmp/t && /tmp/t
cc -std=c11 -Wall -Wextra -Werror -DMOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV=900U -IInc -I../../protocol \
   tests/test_motor_link_control.c Src/motor_link_control.c Src/mc_protocol.c -o /tmp/t && /tmp/t

# 无硬件软件在环（可选，确认 ROS 侧没坏）
cd /home/PRIMARY_USER/vicon_mecanum_ws && source install/setup.bash
ros2 launch mecanum_vicon_control software_in_loop.launch.py start_rviz:=false
```

---

## 11. 完成定义

本次交接**完成**的标志（缺一不可）：

0. **Step 0b 的引脚对照结论已记录在案**（接线接的是固件期望的那组信号，无板载外设占用）；
   若发现冲突，此项以"已向人类报告并给出可行替代方案"为准；
1. 实测出输出轴每圈编码器计数，数值已写入 `Inc/app_config.h` 并有记录说明测量方法；
2. `mecanum_f407_motor_link` 重建并烧录，`FAULT_CONFIG_UNCALIBRATED` 位消失；
3. 架空阶跃测试完成，PI 参数有整定后的值；
4. 一份新文档记录上述结果（沿用 `docs/NO_BATTERY_SAFETY_VALIDATION_2026-09-30.md` 的格式）；
5. 第 8 节的过时说法已在新文档中更正，或直接修掉源文档。

**不属于本次范围**（不要顺手做）：四轮固件、Vicon 外环闭环、K230D 视觉、UDP 突发治理
（除非已走到 Step 6）。

---

## 12. 已确认的前提条件（人类已回答）

| 问题 | 回答 |
|---|---|
| 电机、D157B、3S 电池是否到手可上电？ | ✅ 全部可用，台架路径成立 |
| 车架能否架空？ | ✅ 可以架空 |
| 主控板型号与原理图？ | 立创·天空星 **LCKFB-LSPI-SkyStar-STM32F407VGT6**，官方 wiki 与商城页有原理图 |
| 是否有独立路由器？ | ❌ **没有**，只有手机热点 —— Step 6 的抖动问题会真实出现，优先用 SoftAP 方案 |
| 断电手段？ | ⚠️ 仅手动关停驱动板，无可触及急停 —— 见第 7 节第 2 条 |

### 仍需在开工前确认的

1. **四轮阶段的引脚重选**（不急）：18 个引脚 vs 约 40 个可用 GPIO，见 Step 0b 末尾。
2. **`PE5/PE6/PB4/PB5/PD5/PD6` 有无被板载外设占用**：人类熟悉这块板，一句话即可确认。
