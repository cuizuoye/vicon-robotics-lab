# 单电机 A：115200 无动力串口验证（2026-10-08）

## 结论与范围

经用户确认驱动板电池已拔掉后，将 STM32 的旧 MC-Link 镜像替换为
`stm32/mecanum_f407/build/Debug/mecanum_f407.bin` 文本调试镜像。
烧录校验、独立 Flash 读回比对、复位启动以及两次串口短时测试均通过。
本轮没有发送 `hb`、`arm`、`duty`、`speed` 或任何带使能标志的运动指令。
没有修改代理、Wi-Fi、路由、虚拟网卡或 ESP32 固件。

本结果仅验证 PC ↔ USB-TTL ↔ STM32 USART2，以及 MCU 停机状态。
不代表驱动板带电、电机转动、编码器每圈计数、四轮底盘、无线链路或 Vicon 闭环已验证。

## 设备与镜像

- ST-Link/V2：`0483:3748`，序列号 `E1007200D0D2139393740544`。
- 芯片报告：F405/F407 系列、Device ID `0x413`、1 MiB Flash；工程目标为 F407VGT6。
- USB-TTL：CH340 `1a86:7522`，本次为 `/dev/ttyUSB0`。
- 稳定设备路径：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
- 串口接线：USB-TTL TXD → PD6，RXD ← PD5，GND 共地。
- 供电边界：驱动板电池拔掉，STM32 保持单一电源供电；不并接不同板的电源输出。
- 本次镜像：`mecanum_f407`，`115200 8N1`，换行分隔文本命令。
- 镜像大小：53224 字节；不是 `mecanum_f407_motor_link` 或通信专用镜像。

## 备份与烧录证据

烧录前完整备份：

```text
stm32/mecanum_f407/backups/pre_115200_20261008_tvj3d4Zk/flash_1MiB.bin
size: 1048576 bytes
sha256: f4cbc2ec83164a36f173be8132e13ca235a3d742bc63fb86c30720514c4562fb
```

该备份与本轮较早保存的 `/tmp/mecanum_stm32_readback_zCk1PxUv/flash.bin`
校验值相同。它保存的是 Flash 内容，不包含运行中 RAM；本轮没有改写 Option Bytes。

烧录地址 `0x08000000`，烧录器自动擦除覆盖范围内的扇区 0..3，
随后报告 `Download verified successfully`。原 Flash 内容已由上述备份保留。
烧录后执行软件复位与运行，并捕获到 `READY,motor_a_safe_off`。

独立读回文件：

```text
stm32/mecanum_f407/backups/pre_115200_20261008_tvj3d4Zk/new_app_readback.bin
size: 53224 bytes
sha256: 6e551891a137d0192fa1fc0920d656d0ac9e0dd5f44621f6e91cda992a2f55fd
```

读回文件与待烧录 `.bin` 的 SHA-256 相同，`cmp` 也逐字节通过。
备份恢复会再次擦写 Flash，需确认安全供电与用户授权，不自动执行。

## 在线结果

测试只发送 `stop` 与 `status`，没有用逐字节延时规避接收问题。

| 项目 | 复位启动后首次测试 | 关闭后重新打开串口 |
|---|---:|---:|
| 监听时间 | 20 s | 12 s |
| 状态查询回应 | 10/10 | 10/10 |
| 收到遥测 | 188 条 | 120 条 |
| 遥测采样频率 | 10.0 Hz | 10.0 Hz |
| 最大遥测采样间隔 | 100 ms | 100 ms |
| 无法解析行 / ERR 行 | 0 / 0 | 0 / 0 |
| 脚本结果 | passed=true | passed=true |

两轮全部已解析状态均为：

```text
en=0, hiz=1, mode=duty, target_cps=0, duty=0, fault=0x00000000
```

本次编码器计数与速度字段均为 0；由于编码器尚未完成供电、电平和实测检查，
不能由这些零值推断编码器已正常工作。

ST-Link 只读检查另确认：

- 新启动向量与文本镜像一致，内核在运行。
- USART2 BRR 为 `0x016C`，串口控制寄存器保持收发及接收中断开启。
- TIM9 CCR1、CCR2 均为 `0`。
- GPIOE IDR 为 `0xFF9F`，PE5/PE6 读回均为低电平。

这些是 MCU 侧证据，不代替驱动板在供电后的输出测量。
本轮也没有验证使能后的心跳丢失停机或电机真实停止过程。

### 接线准备前追加复测

同日再次打开串口，监听 10 s，仅发送 stop/status：5/5 次状态查询回应、
100 条遥测、10 Hz、最大采样间隔 100 ms，解析错误、ERR、异常行均为 0。
全部状态仍为 `en=0,hiz=1,duty=0,target_cps=0,fault=0`，脚本 `passed=true`。
本次没有复位、烧录、使能或改动网络；编码器仍未供电验证。

接下来的实际接线及电平确认使用
[Motor_A 编码器检查清单](MOTOR_A_ENCODER_BRINGUP.md)，此处先暂停等待实物核对。

## 与之前 921600 问题的关系

此前旧 MC-Link 镜像在 `921600` 连续发送 HELLO 时出现接收帧缺失字节。
在最后一组对照中，整帧发送 0/3 次成功，字节间隔 10 ms 发送 3/3 次成功，
且成功时读出的接收缓冲区与发送帧一致。

本轮同时更换了波特率、文本/二进制格式与固件接收路径。因此可以确认
当前 `115200` 文本调试组合在两个短时窗口中稳定，不能宣称隔离模块是唯一根因，
也不能宣称旧 MC-Link 的高速接收问题已永久修复。
每字节 10 ms 延迟只适合诊断，不是 150 ms 超时控制链路的解决方案。

## 安全复测方法

在此目录执行（保持驱动板电池拔掉）：

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407
python3 tools/motor_a_safe_probe.py \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --duration 20 --queries 10
```

脚本不会复位、烧录或使能。它等待 READY 或有效遥测后才发送 stop/status；
异常使能/非零输出状态会请求 stop 并退出。需要捕获启动 READY 时，应先开启
监听，再在安全断开电机动力的条件下复位。普通重新打开串口没有 READY 行是正常的。

## 下一步边界

1. 先核对单块 D157B 的 Motor_A 接线与编码器输出电压，再设计安全的编码器供电测试。
2. 在电机仍未使能时，手动转动输出轴，验证方向与累计计数，测量每圈计数。
3. 完成前两项、车轮架空并明确授权后，才测试低占空比转动与故障停机。
4. 恢复无线控制前先统一两端协议和波特率。当前文本调试镜像不能直接与原
   `921600` 二进制 MC-Link ESP32 桥接固件配合使用。
