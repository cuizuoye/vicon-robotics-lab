# ESP32-S3 MC-Link Wi-Fi Bridge

This ESP-IDF project is the transport bridge in the Vicon mecanum control loop:

```text
Linux PC / ROS 2 -- UDP MC-Link --> ESP32-S3 -- UART MC-Link --> STM32F407
Linux PC / ROS 2 <-- UDP telemetry -- ESP32-S3 <-- UART telemetry -- STM32F407
```

The ESP32-S3 validates the MC-Link header, CRC32, session and sequence, but it does
not run trajectory tracking, mecanum kinematics or wheel PID. Those responsibilities
remain on the Linux PC and STM32F407 respectively.

## Open it in VS Code

Open this directory as a **separate VS Code window**:

```text
/home/PRIMARY_USER/vicon_mecanum_ws/esp32/mecanum_wifi_bridge
```

Do not select the STM32Cube project directory as the ESP-IDF project. The message
`build doesn't seem to be a CMake build directory` occurs when ESP-IDF's Full Clean
command is run against the STM32 build folder; it is not an ESP-IDF installation
failure.

Then use the ESP-IDF extension:

1. `Select Current ESP-IDF Version` -> local ESP-IDF v6.1.
2. `Set Espressif Device Target` -> `esp32s3`.
3. `SDK Configuration Editor` -> `MC-Link Bridge`.
4. Set `Wi-Fi SSID` and `Wi-Fi password`.
5. For the verified YD-ESP32-S3-COREBOARD V1.4, use TX=`GPIO17`, RX=`GPIO18`.
6. Run `Build Project`.

The hardware baseline is now ESP32-S3-N16R8. The project uses 16 MB flash.
The replacement board's verified build uses UART0 through the **COM** connector
as its primary log console and USB Serial/JTAG as the secondary console.
The 8 MB PSRAM is intentionally not
enabled because the bridge does not need it; this also keeps the first hardware
bring-up independent of the module's PSRAM timing.

For the first flash, keep the Wi-Fi fields empty and UART GPIOs at `-1`. A
successful boot should only run the protocol self-test, initialize NVS and print
that the Wi-Fi SSID must be configured. It cannot send motor commands in this
state.

`sdkconfig` is ignored by Git so Wi-Fi credentials are not stored in source code.

## Safety behaviour

- Only a valid `HELLO` establishes the controlling PC address and `session_id`.
- Duplicate, out-of-order, foreign-address and bad-CRC packets are dropped.
- Consecutive `CMD_VEL` datagrams arriving in one Wi-Fi burst are collapsed to
  the newest command before UART transmission. State/safety commands are sent
  immediately and supersede older pending velocity commands.
- A 750 ms ESP32 command-link timeout injects one **disabled** zero `CMD_VEL`,
  latches the expired session, and requires a new `session_id`. This longer
  session timeout tolerates mobile-hotspot jitter; the STM32 still independently
  disables motor output after 150 ms without a fresh enabled command.
- The STM32 must still implement its own UART/CMD timeout and clear all motor PWM.
  Wi-Fi or ESP32 code is not the final safety layer.
- Wi-Fi power saving is disabled to reduce command latency and jitter.

## Transport

Normal bridge boot now defaults to UART idle: no unsolicited HELLO or motor
command is sent. The controlling PC must initiate the session. For bench-only
diagnosis, enable `MC_UART_STARTUP_PROBE` to send one HELLO and wait up to 500 ms
for a matching ACK. `MC_UART_DIAGNOSTIC_ONLY` performs that probe independently
and never starts Wi-Fi/UDP. A probe ACK proves transport, not fault-free control.

UART HELLO packets carry an extra leading empty COBS delimiter, allowing a
receiver recovering from a damaged partial frame to synchronize before the
HELLO body. Other packet types keep the original encoding. This delimiter is
not a command, does not create a session by itself and does not clear faults.

- PC -> ESP32-S3: UDP unicast port `46000`, one raw MC-Link frame per datagram.
- ESP32-S3 -> PC: UDP unicast port `46001`.
- ESP32-S3 <-> STM32F407: UART `921600 8N1`, `COBS(raw frame) + 0x00`.
- The verified UART mapping is ESP32 `GPIO17/TX` and `GPIO18/RX`.
- With UART disabled, ESP32 returns a local `HELLO_ACK` for Wi-Fi-only tests.
- With UART enabled, ESP32 suppresses that local response. A received
  `HELLO_ACK` must therefore have traversed STM32 and proves the full link.

Wire only 3.3 V logic and ground:

```text
ESP32 GPIO17/TX -> STM32 PD6/USART2_RX
ESP32 GPIO18/RX <- STM32 PD5/USART2_TX
ESP32 GND       -- STM32 GND
```

Do not connect the two boards' 5 V pins together for this test.

After Wi-Fi is configured and the board has joined the same 2.4 GHz LAN as the
PC, run the non-actuating discovery and round-trip test:

```bash
python3 tools/udp_link_test.py
```

The script broadcasts only `HELLO` frames to port `46000`, listens for matching
`HELLO_ACK` frames on port `46001`, validates CRC/session/sequence, and reports
the responder IP and round-trip time. Pass an IP address as the first argument
to use unicast instead of discovery.

The shared wire definition is located at
`../../protocol/mecanum_protocol_v1.h` in the workspace.

## Current validation boundary

Ordered software checks on 2026-10-14 reproduced receive gaps with IDF v6.1,
IDF v5.5.5 and ESP-only b/g mode; neither SDK downgrade nor b/g was adopted.
The low-rate lwIP observations did not report UDP/IP/ARP memory errors or
exhausted echo/tcpip stacks. Private Wi-Fi RX-buffer occupancy remains unknown;
disabled MEMP statistics must not be read as zero allocation errors.
See `../../docs/esp_ordered_software_checks_20261014.json` for complete evidence.

`MC_WIFI_LEGACY_BG_DIAGNOSTIC` and `MC_WIFI_RESOURCE_DIAGNOSTICS` default off
and require the isolated echo-only bench. Wi-Fi protocol settings can persist
in NVS: turning a build switch off alone does not establish the runtime protocol.
Echo-only builds now explicitly select b/g/n (bitmap 7), or b/g (bitmap 3) for
the compatibility test, and report the actual bitmap. These experiments require
same-device NVS backup/restoration, not NVS copied from another board.
They never initialize or forward the motor-control UART.

On 2026-10-14 the replacement ESP32 and STM32 were wired together and the
Wi-Fi/UDP/UART round trip was verified with the STM32 **four-link read-only**
image, motor battery disconnected, ESP32 powered through COM USB and STM32
powered only by ST-Link. The UDP run received 50 wheel and 50 diagnostic frames
in 5 seconds; ROS 2 received 100 of each in 10 seconds. The measured MCU
sampling interval was 10 ms and all eight PWM compares were zero. These are
no-motor communication results, not car battery supply or driving acceptance.
See `../../docs/replacement_wifi_readonly_20261014.json` for evidence.

ESP reset has been observed to latch an STM32 UART framing fault. Arbitrary
cold-start ordering and automatic recovery are **not validated**. The bench
sequence remains: ESP UART/Wi-Fi ready, explicitly reset the verified read-only
STM32 with motor power disconnected, then start a fresh PC session. Do not
automatically clear UART faults or weaken the independent motor watchdog.

## Standalone preparation without a hotspot

Compilation, flash preparation, native tests and USB logs do not require the PC
to join the phone hotspot. Wireless round-trip testing requires the ESP32 and
PC to be on mutually reachable networks; the usual setup is the same 2.4 GHz
hotspot. No code here changes PC Wi-Fi, proxies, routes or virtual interfaces.

The bridge starts and reconnects to its saved Wi-Fi configuration without a
serial monitor. **Removing the monitor is not the same as proving car-powered
operation.** Power wiring and loaded supply stability still require validation
after the correctly charged battery becomes available.

The UART consumer gate added on 2026-10-14 admits valid frames only for a
nonzero current reader session: the one-shot boot probe, diagnostic-only reader,
or active PC HELLO session. After an optional normal boot probe finishes, or the UDP
socket stops on disconnect/error, the gate closes and its queue is cleared.
Thus an absent hotspot/PC does not endlessly fill the telemetry queue. A PC
session's telemetry remains readable after command timeout so fault reports are
not hidden. HELLO ACKs still come from the STM32 when UART is enabled; no motor
commands or fault-clear commands are added. An active-reader overload may still
drop frames; this change is not a general queue-overload fix.

Native regression tests (run from workspace root):

```bash
gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I esp32/mecanum_wifi_bridge/main -I protocol \
  esp32/mecanum_wifi_bridge/tests/test_uart_consumer.c \
  -o /tmp/mc_uart_consumer_test
/tmp/mc_uart_consumer_test
```

The test covers idle discard, boot/PC session changes, ACK/telemetry admission
and disconnect discard. The normal bridge and corrected STM32 read-only image
were subsequently flashed and independently read back on 2026-10-14. A 15-second
normal boot had no spontaneous HELLO or queue-full warnings. A temporary
UART-only probe confirmed a real FE after ESP reset, zero rearm failures,
matching ACK and progressing telemetry, while UART/PI faults stayed latched and
all PWM outputs stayed disabled. Temporary diagnostic images were then removed
and the final read-only MCU was explicitly reset and checked fault-free.
See `../../docs/standalone_boot_verified_20261014.json`.

The phone hotspot remained available and the PC was on a different subnet in
this run; this proves no-PC-session startup and UART recovery, **not**
absent-hotspot reconnect or the updated wireless round trip. Car battery power
and arbitrary cold-start order are still unverified. The current devices remain
PC USB/ST-Link powered, with motor power disconnected.

The STM32 UART receiver also now checks ErrorCode before admitting a received
byte or rearming from RxCplt. Its ErrorCallback aborts the receive transfer,
discards pre-error queued frames, and rearms once, then discards the damaged
frame tail until a COBS delimiter. This fixes the HAL FE callback double-rearm
path; **the application UART fault remains latched**, so this is not automatic
control recovery. Native tests simulate both RxCplt-before-ErrorCallback and
error-with-receive-still-BUSY cases. No motion firmware is flashed by these tests.

## 2026-10-15：Tenda直连SoftAP完整只读链路通过

`CONFIG_MC_WIFI_SOFTAP`默认关闭；开启后是单客户端WPA2、HT20、信道6（可配）、
192.168.50.1/24的AP-only热点，没有STA上游或NAPT。配置使用RAM，NVS异常停止而
不擦除，密码不合法时在初始化控制UART前拒绝启动，不降级开放热点。STA默认路径保留。
原二进制帧、UDP46000/46001、UART921600、150ms MCU/750ms ESP保护不变。
断开epoch防止快速离开/重连恢复旧socket会话，后续需重新HELLO，不自动使能或清故障。
断开分支有离线检查；本阶段未另行实机注入Tenda断线，不冒充失联运动验收。

证据：`../../docs/tenda_softap_full_link_20261015.json`；原始记录及私有配置：
`../../tmp/softap_full_link_20261015_Shf4zmaQ/`（勿公开其中的Wi-Fi配置/Flash）。

- 初始SWD确认实际无PWM镜像26988字节哈希一致、PWM=0、GPIO低；但有既存UART
  故障，保留在`initial/`。ESP烧录/回读/COM打开及明确复位完成且TX稳定后，才对
  已核对的无PWM STM32做一次维护复位；未加入生产自动恢复，不用于运动固件。
- 只写ESP应用分区810448字节，独立回读哈希一致：
  `d834b7368e83540228ae63404582eb8bbe61069e84e33de51edf9509ef2b499c`。
  Bootloader/分区表/NVS/PHY未变，STM32没有重烧。
- 10秒UDP收到真实STM32 HELLO_ACK、100个轮反馈及99个诊断。
- 两轮ROS各约120秒，各1199组轮反馈/诊断，采样约100Hz、回报约10Hz；运行窗口
  各5998条命令均未使能零速、门限150ms，故障0、状态109，未启用QoS提权。
- 10Hz反馈仍有最长约200ms/180ms主机接收间隔；回传间隔不是命令间隔。
  正常固件未记录ESP逐包命令时间，不能提供其最大值或声称所有延迟消失。
- 停发后及关闭COM后SWD均确认预期link_fault=1、uart_fault=false、pi_fault=0，
  PWM=0、GPIO低、定时器关闭。没有清除预期租约超时来制造健康收尾。
- 482项Python/ROS回归及9个native C ASan/UBSan程序通过；AP/STA两种构建通过，
  原默认sdkconfig仍保持哈希
  `e4b7eeb37046e231dd0153d264fbaa158df09960491f21abbcb0fae92a421c8c`。

当前ESP留置完整SoftAP MC-Link桥，不再是SAP1隔离程序；STM32仍是禁止PWM只读镜像，
不能因通信通过就驱动小车。Tenda已连接，原内置Wi-Fi、Vicon、默认路由、DNS和代理/TUN
未修改，HTTPS200；测试发送器和COM监听已退出。

在本ESP项目目录复现私有AP构建：

```bash
source /home/PRIMARY_USER/.espressif/tools/activate_idf_v6.1.sh
idf.py -B /home/PRIMARY_USER/vicon_mecanum_ws/tmp/softap_full_link_20261015_Shf4zmaQ/build \
  -D SDKCONFIG=/home/PRIMARY_USER/vicon_mecanum_ws/tmp/softap_full_link_20261015_Shf4zmaQ/sdkconfig build
```

**VSCode默认sdkconfig仍为STA，不要误刷默认build覆盖实测热点版本。**
原Flash和SAP1应用均已备份；断动力核对MAC后可只恢复应用，无需擦NVS或改原网络。
下一步准备受限运动固件，再做架空、落地及车载供电验收，不重复已有轮向/CPR标定，
也不放宽失联保护。
