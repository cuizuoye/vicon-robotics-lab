# 四轮MC-Link无线只读联调记录（2026-10-10）

## 当前验收结果

动力电池完全拔掉、两驱动板开关关闭；STM32独立供电。已烧录独立目标
`mecanum_f407_four_link_readonly`，UART921600；不初始化PWM，8个电机脚GPIO低，
拒绝任何使能/非零速度请求。尚未完成无线驱动电机，不代表落地或Vicon实车控制。

- 完整备份旧RR328固件并核对前70908字节一致；新26504字节BIN烧录验证和独立读回一致。
- C原生ASan/UBSan测试通过：CRC、序号/会话、时间回绕、150ms过期、运动拒绝、四轮遥测与零寄存器检查。
- 主机工具237项测试通过；ROS包96项测试通过并构建完成。六个旧BIN校验值未改变。
- 初次CH340直连921600没有ACK并出现UART故障，原因未确定，不宣称已修复。
- 拆除USB-TTL数据线、换ESP32通路、逻辑供电重新插入后，无线完整回传通过。
- UDP阶段：5秒50组轮速帧、49组诊断；命令序号推进，fault=0，四轮计数/速度均零，8CCR=0。
- 停发命令0.45秒：会话过期（state=125、fault=1）；新session恢复state=109/fault=0。
  这是只读会话过期验证，不是带动力断流停轮实验。
- ROS2真实订阅10秒：100组轮速、计数、故障、状态、命令序号与诊断；connected消息500条。
  主机时间估计采样100.18Hz、诊断10.018Hz；MCU周期dt_min=dt_max=10ms、IRQ最大61us，
  ticks=updates、8CCR=0、gpio_low=1、watchdog_reset=0。

## 当前连接与网络

ESP32 COM口连接PC；ESP32 GPIO17→STM32 PD6、GPIO18←PD5、GND共地。
USB-TTL只保留3.3V/GND给STM32供电，其TX/RX已撤下；ST-Link仅CLK/DIO/GND，
驱动板5V/3.3V不与逻辑电源并联。

现有手机热点：PC WiFi `wlp114s0`/10.249.81.61，ESP32 10.249.81.215。
UDP命令46000、回传46001，串口921600 8N1、COBS/CRC32。
没有修改系统代理、路由、虚拟网卡或WiFi连接；工具仅绑定自己的socket到WiFi接口。
这些IP是本次DHCP记录，下次运行需重新确认。

ESP32启动自检记录`unexpected STM32 response`及等待PC前的队列满警告；
其自检只检查队列第一帧，现固件还会主动发送轮速/诊断帧，告警原因仍需进一步验证。
后续UDP实际收到STM32 ACK和四轮诊断，不能把启动告警当作整条链路失败或宣称已修复。

`WHEEL_TELEMETRY`保持40字节；`LINK_STATUS`新增60字节`4RO1`扩展。
0mV字段表示未接入电池ADC，ROS显示NaN/未知，不是实测电池电压。
动力断开时编码器未供电，零计数/零RPM不能证明带动力测速。

## 可复现入口（只读、保持动力断开）

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407
python3 tools/four_link_ro_probe.py --esp-ip 10.249.81.215 \
  --bind-ip 10.249.81.61 --interface wlp114s0 --seconds 5 \
  --check-timeout --confirm-battery-disconnected --output docs/four_link_ro_udp_new.json

cd /home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 stm32/mecanum_f407/tools/ros_four_link_ro_observe.py \
  --esp-ip 10.249.81.215 --interface wlp114s0 --seconds 10 \
  --confirm-battery-disconnected --output stm32/mecanum_f407/docs/four_link_ro_ros_new.json
```

两个工具不能同时占用UDP46001。不要再用115200旧文本工具检查当前镜像。
ROS桥`readonly_mode=true`独立拒绝使能/清故障，非零Twist也仅下发未使能零速度。

## 证据与恢复

- 原始UDP：`stm32/mecanum_f407/docs/four_link_ro_udp_20261010.json`
- 原始ROS：`stm32/mecanum_f407/docs/four_link_ro_ros_20261010.json`
- 旧固件完整备份：`stm32/mecanum_f407/backups/pre_four_link_ro_20261010_idodVbcb/flash_before_1MiB.bin`
- 备份SHA256：`9acd6f586b2a3117cd8423376c4858ba44f8fba588542724e6bd947ca5088c8a`
- 新BIN及独立读回SHA256：`d8bb5467522ddb3717a619283537d6a2b6b16a4555dbe0d319f638dbb52c5113`

后续：独立受限无线四轮PI固件→断动力烧录/零输出验证→新确认上电悬空一次前进→现场停轮断动力。
然后才考虑落地低速、Vicon刚体Pose/坐标转换、外环与失位姿停机。
用户已取消17:30的时间限制，按实际验收推进。
