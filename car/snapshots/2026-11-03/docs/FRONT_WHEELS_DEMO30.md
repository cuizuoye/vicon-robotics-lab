# 两前轮共同前进30秒展示

日期：2026-10-08。最初按用户要求暂停继续硬件推进，只准备软件。
随后用户明确请求运行一次Demo：原`dual_v2`已完整备份，独立`demo30_v1`已烧录并读回校验通过；无动力检查通过后，按本轮现场确认执行了一次30秒请求。
软件运行结果通过，用户独立确认持续向前、结束后自行停止、动力已断开且无明显异常；不推进四轮、落地或Vicon闭环。
实机证据见[本次记录](FRONT_DEMO30_HARDWARE_RUN_2026-10-08.md)。下文构建与离线验证保留最初软件准备阶段的事实。

## 演示内容

- 固定第一板Motor_A/左前轮FL与Motor_B/右前轮FR，第二板不供电，四轮悬空且车架固定。
- 沿用已经实测的共同前进组合：A=+100、B=-100，即10%开环请求；不改接线或编码器相位。
- 主机连续维持一次30秒请求，然后向两路发stop；没有循环播放、在线换向或升幅。
- 这是轮向与控制流程展示，不是小车落地前进、等速控制或Vicon闭环展示。
  每次展示的持续转动、实际停转与温升都需现场观察，本次确认见实机记录；零指令时可能惯性滑行。

## 独立固件和保护

新目标为`mecanum_f407_front_demo30`，编译开关`BENCH_FRONT_DEMO30=1`仅对该目标生效。
复用已验证的启动低输出、PWM映射、串口与致命异常全停路径，原台架2.2/5.2秒窗口不变。

- 版本为`demo30_v1`，启动标识`READY,front_demo30_safe_off_v1`，USART2仍115200 8N1。
- 只接受`arm AB A5 30s`与`duty AB 100 -100`这一组使能/运动命令。
- 默认上电不转，arm也保持零输出；每次启动只允许一次实际演示，stop不恢复运行资格。
- 主机约30秒发stop，固件主循环另检查30.2秒上限；心跳不能延长该上限。
- 主机每100ms心跳，固件心跳超过500ms全停并锁存故障；主机暂停过久、遥测失联、异常状态也退出并请求全停。
- 不接受旧单A、普通dual_v2动作、其它方向/时长/占空比、重复动作或清故障命令。
- Ctrl+C会进入主机finally全停；通信故障最终仍需现场关闭开关并拔掉电池。
- 时间与心跳限制是主循环软件保护，不是独立硬件安全限时器，也不保证机械轮子在精确30秒时静止。

要再次演示，必须先关闭驱动开关、拔掉动力电池，再安全复位MCU、无动力检查，并重新确认上电静止。
不能在5V编码器信号仍带电时让MCU先掉电、复位或烧录。

## 构建与离线验证

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407
cmake --preset Debug
cmake --build --preset Debug --target mecanum_f407_front_demo30
```

- Debug：Flash 51916B，RAM 4576B；生成`build/Debug/mecanum_f407_front_demo30.bin`与`.hex`。
- Release：Flash 39888B，RAM 4560B；位于`build/dual_bench_release_20261008/`。
- Debug SHA-256：`a377989584af8c141f441f1a788a005f351cdb31031fa11dd0a4f4d7b2c52727`。
- Release SHA-256：`f4d1676dde06cc815964fe584bc2774921c838c3f4e30870b8ac5760471ad366`。
- Python127项纯模拟回归通过，含9项新演示测试及故障子场景。
- C原台架12组＋演示8组场景通过，ASan/UBSan通过。
- 重建后的原台架Debug镜像与已烧录读回文件逐字节一致；Debug与Release SHA-256均保持原值。
- 本轮没有连接真实串口、访问ST-Link、烧录、复位或调整代理/热点/虚拟网卡。

构建与C验证日志：[离线验证记录](front_demo30_offline_validation_20261008.log)。
四天计划已保存到桌面，日期为10月9日至12日，提供Markdown与可打印HTML；这只是建议排期，不自动恢复硬件推进。

## 后续使用流程

原`dual_v2`不能直接运行30秒；不能通过重复5秒动作拼接绕过它的时限。
若需切换或重新烧录，需另获硬件操作许可，先在动力断开条件下备份、烧录独立demo镜像并读回校验，然后做无动力检查。
首次30秒展示前复核已验证5秒动作的状态、电池压降、驱动/电机温升，保留现场随时断电条件。
各项条件不满足时不展示，不把离线通过当成30秒带载验证。

演示固件烧录后的无动力检查命令，默认不使能：

```bash
python3 -B tools/front_wheels_demo30.py \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --confirm-battery-disconnected
```

通过检查后，接线保持PE5→AIN1、PE6→AIN2、PE9→BIN1、PE11→BIN2与公共GND；
A编码器PB4/PB5保持，B编码器仍不接MCU。STM32保持独立供电不复位，不并接驱动5V/3.3V。
开关关闭时接入电池，再开开关确认两轮静止，取得本次30秒动作许可后才运行：

```bash
python3 -B tools/front_wheels_demo30.py \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --run --confirm-suspended --confirm-two-motors \
  --confirm-channel-isolation --confirm-stationary --confirm-30s
```

终端每约5秒显示进度，结束输出原始遥测JSON。只记录A原始计数，不伪造B轮速、CPR或实际停转。
观察两轮顶部轮面是否均朝车头运动；异常立即断动力，不用手阻挡。
结束后确认实际停止，再关闭驱动开关、拔掉电池；勿带电调整接线或让MCU先断电。

由于协议是独立115200文本，不能直接与现有921600二进制MC-Link ESP32固件混用。
