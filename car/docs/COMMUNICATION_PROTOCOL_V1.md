# MC-Link v1 通信协议

2026-10-12当前实机边界：反馈为`4WB1`受限无线前进镜像，基础WHEEL40字节不变，LINK_STATUS为100字节台架诊断。只允许显式使能后的单次F50，10%输出/10秒硬限/150ms失联保护仍在，不支持无线四向矩形。5秒零速往返通过、延长检查约5.23秒后超时，未启动电机。当前使用`four_link_bench_probe.py`，下面4RO1/115200等“当前”段落均为历史，完整协议定义不等于所有消息已可在现固件执行。[本轮证据](FOUR_LINK_RECONNECT_2026-10-12.md)、[矩形软件准备](RECTANGLE_AND_VICON_PLAN_2026-10-12.md)。

2026-10-10最新实现边界：已烧录四轮通信只读镜像，921600二进制UART与ESP32/ROS2真实回传通过；基础40字节轮速布局不变，新增60字节`4RO1`只读诊断扩展。默认禁动作、拒绝使能/非零速度、8电机脚GPIO低；150ms只读会话过期验证通过，**不是带动力断流停轮或无线运动验收**。当前不接动力，不使用下方历史115200工具。[实机记录](FOUR_LINK_READONLY_2026-10-10.md)。

2026-10-10最新台架边界：当前`rr_diag_328_v1`、115200文本，仅RR固定328cps对照已完成，实测约2倍、现场卡顿改善；已停止断动力，used=1，四轮输出为零。当前只读用`rr_speed2_probe.py --mode post-run`，旧164工具与921600二进制桥均不适用，不自动复位/重试，不启用无线运动。[当前记录](RR_SPEED2_2026-10-10.md)。下方其它固件的“当前”为历史。

2026-10-10当前台架边界更新：`rr_diag_pwm01_v1`、115200文本，唯一一次RR164/9.7秒对照及100Hz记录已完成；用户报告卡顿基本相同、已断动力。最终只读复核100Hz、8CCR=0、fault/en=0、used=1，当前仅`rr_pwm_fine_probe.py --mode post-run`只读，不自动复位/重试。平顺性未通过，恒定PWM后续对照未实施，不接921600桥、不启用无线运动。[最新记录](RR_PWM_FINE_2026-10-10.md)。下方used=0/未转轮属于历史，不代表当前状态。

2026-10-10当前台架边界：板内已更新独立`rr_diag_pwm01_v1`、115200文本。完整备份、烧录验证/独立读回、60帧无动力检查通过；实际TIM12 ARR=8399/PSC=0、100Hz、8CCR=0、fault/en/used=0。只改RR试验PWM步进0.1%，新版本未转轮、卡顿尚未修复验收。当前只读工具为`rr_pwm_fine_probe.py --mode no-power`，旧RR/startup/drive工具不适用；仍未整合921600二进制MC-Link，不启用无线运动。[当前集成与对照步骤](RR_PWM_FINE_2026-10-10.md)。下方仅为历史台架边界和正式目标协议。

2026-10-10最新台架边界：当前已备份并烧录独立 `rr_diag_start8_v1`、115200文本；编程验证/独立读回及60帧无动力检查通过，en=0/fault=0/used=0、8个CCR为零。**未上动力或发动作，RR平顺性仍待单次诊断。** 只读使用 `rr_diagnostic_probe.py --mode no-power`；旧startup/drive工具不适用于当前镜像。正式921600二进制MC-Link仍未整合，不连接桥启用无线运动。[当前集成记录](RR_DIAGNOSTIC_INTEGRATION_2026-10-10.md)。下面的旧台架边界保留为历史，不代表当前板内程序。

2026-10-10当前边界：`drive_pi_start8_v1`、115200文本。一次悬空yaw200程序检查通过，但右后轮全程轻微速度脉动待排查；已停轮断动力、fault=0/used=1/en=0，当前只读、不自动重试。完整遥测可用startup工具`--summary-only --include-trace`保存，未改板内固件或整合二进制MC-Link，不能连接921600桥启用无线运动。[当前旋转记录](YAW200_BENCH_2026-10-10.md)。

2026-10-10上一阶段边界（左移历史）：板内为独立`drive_pi_start8_v1`、115200文本。受限8%启动补偿后，唯一一次10秒悬空left50轮向/停机通过，用户确认断动力；最终fault=0/used=1/en=0，当前只读、不自动复位。未整合二进制MC-Link，仍不能接921600桥或启用无线运动；落地侧移/旋转与Vicon实车控制未验收。[当前启动观察记录](DRIVE_STARTUP_OBSERVE_2026-10-10.md)、[保留的旧失败](DRIVE_PI_LEFT50_FAILURE_2026-10-10.md)。下段与后续协议定义保留历史/目标状态。

2026-10-10实现边界：下面是正式无线控制的目标协议。已验收板内台架使用`forward_pi_v1`、115200文本，不能直接连接921600二进制桥；新三向速度目标规划层只完成离线及ARM静态库验证，尚未接入MC-Link/PWM。不要因协议定义了CMD_VEL就启用实车无线运动。[当前接口状态](MECANUM_DRIVE_PLAN_2026-10-10.md)。

## 1. 范围与职责

```text
Linux PC / ROS 2
  目标、轨迹、Vicon外环、可视化、操作者使能
             | UDP unicast
             v
ESP32-S3
  Wi-Fi STA、UDP/UART转发、序号/CRC检查、链路超时
             | UART 921600 8N1, COBS
             v
STM32F407
  麦轮逆运动学、四轮编码器PID、使能/急停锁存、最终PWM清零
```

ESP32-S3 不执行轨迹规划或电机 PID。STM32 不得依赖 Wi-Fi 实现最终停车：
UART 数据超时后必须独立将四路目标轮速和 PWM 清零。

## 2. 传输参数

| 链路 | 参数 |
|---|---|
| PC → ESP32-S3 | UDP 单播，命令端口 `46000` |
| ESP32-S3 → PC | UDP 单播，遥测端口 `46001` |
| ESP32-S3 ↔ STM32F407 | UART `921600 8N1`，3.3 V TTL，共地 |
| UDP 数据 | 一个 UDP datagram 对应一个原始 MC-Link 帧 |
| UART 数据 | `COBS(原始 MC-Link 帧) + 0x00` |
| 多字节整数 | 小端序 |
| 最大 payload | `256 bytes` |

Wi-Fi 使用 STA 模式接入实验室局域网。开发阶段可使用 DHCP，正式实验建议在路由器上为
ESP32-S3 做 DHCP 地址保留。发现可使用广播，但速度命令只使用单播。

## 3. 原始帧

CRC32 保护范围为从 `magic` 到 payload 的全部字节，多项式与 zlib/IEEE CRC32 一致。

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | magic | ASCII `MC` |
| 2 | 1 | version | 当前为 `1` |
| 3 | 1 | message_type | 消息类型 |
| 4 | 2 | flags | ACK、响应、故障、使能位 |
| 6 | 4 | sequence | 每个发送端单调递增，`uint32` 回绕 |
| 10 | 4 | session_id | PC 进程启动时生成的非零随机数 |
| 14 | 4 | sender_time_ms | 发送端单调时钟，仅用于诊断 |
| 18 | 2 | payload_length | `0..256` |
| 20 | N | payload | 按消息类型解释 |
| 20+N | 4 | crc32 | 小端序 CRC32 |

UDP 已经保留报文边界，因此不再包 COBS。UART 是字节流，使用 COBS 确保 `0x00`
只出现在帧结束分隔符中。

## 4. 消息类型

| 类型 | 值 | 方向 | 用途 |
|---|---:|---|---|
| `HELLO` | `0x01` | PC → ESP/STM | 建立新 `session_id`、交换能力 |
| `HELLO_ACK` | `0x02` | ESP/STM → PC | 报告协议版本、启动编号和能力 |
| `HEARTBEAT` | `0x03` | 双向 | 链路活性，不代替速度超时 |
| `CMD_VEL` | `0x10` | PC → STM | 车体坐标系速度目标 |
| `CONTROL_ENABLE` | `0x11` | PC → STM | 请求使能/禁用，可选清除允许清除的锁存故障 |
| `ESTOP` | `0x12` | PC/ESP → STM | 立即停车并在 STM32 锁存 |
| `ACK` | `0x20` | 反向 | 确认低频状态命令，不对每个 `CMD_VEL` 强制 ACK |
| `WHEEL_TELEMETRY` | `0x30` | STM → PC | 四轮转速、编码器、电压、故障、最后命令序号 |
| `LINK_STATUS` | `0x31` | ESP → PC/STM | Wi-Fi RSSI、丢包、CRC 错误和链路状态 |

### `CMD_VEL` payload（10 bytes）

| 字段 | 类型 | 单位 |
|---|---|---|
| `vx_mm_s` | `int16` | 车体 x 方向 mm/s |
| `vy_mm_s` | `int16` | 车体 y 方向 mm/s |
| `wz_mrad_s` | `int16` | 绕 z 轴 mrad/s |
| `timeout_ms` | `uint16` | STM 本地命令超时，建议 `150 ms` |
| `control_mode` | `uint8` | `0` = 车体速度 |
| `reserved` | `uint8` | 必须为 0 |

使用定点整数而不是传输 C `float`，避免编译器 ABI 和对齐差异，同时便于拨号日志。

### `CONTROL_ENABLE` payload（4 bytes）

| 字段 | 类型 | 说明 |
|---|---|---|
| `enabled` | `uint8` | `0`=失能，`1`=请求控制授权 |
| `clear_latched_faults` | `uint8` | `1`=清除允许恢复的锁存故障 |
| `reserved` | `uint16` | 必须为 0 |

清故障必须使用 `enabled=0, clear_latched_faults=1`。控制授权不会绕过
ROS 安全看门狗、STM32 本地超时、编码器标定检查或其它硬件故障。

ROS 2 映射为 `/car/estop`、`/car/control_enable` 和 `/car/clear_faults`
三个 `std_msgs/Bool` 话题，只有 `true` 触发急停和清故障动作；发布 `false`
到 `/car/control_enable` 会立即在 ROS 侧同步失能。

### `WHEEL_TELEMETRY` payload（40 bytes）

- STM32 采样时刻 `sample_time_ms`；
- 四轮 `rpm * 10`，顺序固定为 FL、FR、RL、RR；
- 四轮累计编码器计数 `int32`；
- `battery_mv`、`state_flags`、`fault_bits`；
- `last_command_sequence`，用于 PC 判断命令是否已到达 STM32。

## 5. 时序与安全状态

1. PC 生成新 `session_id`，发送 `HELLO`。
2. ESP32 只接受已配置 PC IP 或通过 HELLO 确立的单一端点，然后转发到 STM32。
3. `HELLO_ACK` 完成前，STM32 保持禁用。
4. PC 以 `50 Hz` 重复发送最新 `CMD_VEL`，新序号覆盖旧序号。
5. ESP32 连续 `200 ms` 未收到有效 UDP 命令时，用当前会话的下一序号向 STM32 注入一次零速，并锁住过期会话。
6. 超时后 PC 必须使用新的非零 `session_id` 重新发送 `HELLO`；ESP32 不接受旧会话恢复后的速度包。
7. STM32 超过 `CMD_VEL.timeout_ms`未收到新命令时，立即将目标轮速/PWM 清零并退出使能。
8. `ESTOP`、驱动故障、编码器异常等需在 STM32 锁存；重新收到速度不得自动清除。
9. PC 同时监视遥测中的 `last_command_sequence`。若遥测仍到达但该序号停止推进，
   说明 ESP32 可能已因 UDP 看门狗锁住旧会话，PC 必须换新 `session_id` 重握手。
10. `ESTOP`、清故障、控制授权等低频状态命令先建立新会话，再以递增序号重复
    发送并暂时压住周期 `CMD_VEL`，避免 UDP 重排使状态命令变成旧包。
9. 重新使能前要求速度为零、链路新鲜、无不可清除故障。

PC、ESP32 和 STM32 的单调时钟不做绝对同步假设。`sender_time_ms` 只用于日志；是否超时以
接收端本地时钟为准。

## 6. 源码对应

- Python 参考编解码：`mecanum_vicon_control/transport_protocol.py`
- ESP32/STM32 共享 C 字段定义：`protocol/mecanum_protocol_v1.h`
- 两侧必须用文档中的固定十六进制测试向量做互操作测试，不能只各自做往返测试。

固定测试向量的输入为：

```text
type=CMD_VEL, flags=ACK_REQUIRED|ENABLED
sequence=42, session_id=0x12345678, sender_time_ms=987654
vx=0.250 m/s, vy=-0.125 m/s, wz=0.800 rad/s, timeout=150 ms
```

原始 UDP 帧必须为：

```text
4d43011009002a0000007856341206120f000a00fa0083ff200396000000835a4e9f
```

UART COBS 帧（含末尾 `00`）必须为：

```text
064d43011009022a0101087856341206120f020a02fa0683ff200396010105835a4e9f00
```

ESP-IDF 侧使用官方支持的 BSD Sockets API 实现 UDP，UART 使用 ESP-IDF UART driver。
ESP32-S3 的 UART TX/RX 引脚必须等核心板到货后根据实际原理图确定，本文档不预先猜测引脚。
已建立的 ESP-IDF 工程位于 `esp32/mecanum_wifi_bridge`，需作为独立 VS Code 文件夹打开。

## 7. 硬件前可完成的测试

- Python 帧编解码、CRC32、COBS、序号回绕测试；
- Linux UDP loopback 丢包、乱序、重复包和超时测试；
- ESP-IDF 主机单元测试或 ESP32 开发板 UDP echo；
- ESP32 与 STM32 之间的 UART 回环、断线和 CRC 错误注入；
- 最后再进行抬轮测试、低速落地测试和 Vicon 闭环测试。
