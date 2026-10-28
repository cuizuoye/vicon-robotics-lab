# 第1天：双前轮编码器准备与标定

日期：2026-10-09。软件准备、只读版烧录、无动力通信/寄存器检查，以及右前轮编码器接线后的双前轮上电静止观察已完成。本日尚未启动电机。用户要求减少重复电表测量、优先争取完成四轮验证；后续方案见[四轮台架准备](FOUR_WHEEL_BENCH_2026-10-09.md)。

## 今日起始检查

用户确认动力电池完全拔掉、驱动开关关闭、两轮停止，STM32由USB-TTL独立供电，ST-Link仅CLK/DIO/GND连接，不并接3.3V；原A/B控制线和左前轮编码器不变，右前编码器暂不新增。

- 已识别USB-TTL `1a86:7522` 和ST-Link `0483:3748`；稳定串口仍为`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
- 串口115200 8N1，仅读取当前`demo30_v1`并发送`stop/status`，无`arm/duty/hb`。
- 首条完整遥测`ms=731710`回报两路禁用、零指令、无故障；随后第一次stop事务收到`ERR,bad_command`。
- 最终stop/status确认两路仍禁用、零指令，但`fault=0x80`锁存，因此本次检查`passed=false`，不作为无故障验收。
- 未发现已知串口工具占用进程。尚无证据证明异常来自接线、旧接收片段或UART时序；不猜测根因、不发清故障命令、不启动电机。
- 原始结果：[起始无动力检查](front_encoder_preflight_demo_status_20261009_ms731782.json)。

## 独立双编码器只读固件

目标：`mecanum_f407_front_encoder_readonly`，版本：`front_enc_ro_v1`。

- PE5/PE6/PE9/PE11始终为低电平GPIO；不初始化电机PWM、不切换这些脚到定时器复用。
- 仅支持`stop/status/estop`；`arm/duty/hb/clear/zero/speed`等命令均拒绝并锁存协议故障，不能使能。
- 左前A：TIM3，PB4/PB5；右前B：TIM4，PB6/PB7；均为AF2、NOPULL。
- 每10ms采样，100ms发布`TEL3`；`STATUS3`为查询回包；分别报告`enc_a/cps_a/enc_b/cps_b`原始有符号计数。
- 标定前不换算RPM，不改变已验证前进映射，不启动速度PI，不假定两轮编码器符号相同。
- 双编码器极值遥测超过160B，仅新目标printf缓冲增至224B；原目标保持160B。
- 致命异常仍将四根AIN/BIN控制线拉低。GPIO低不代替上电现场静止与万用表检查。
- 只读固件不是串口异常根因修复；需烧录后重新验证无动力连续查询。若异常重复，先排查串口，不越级上电。

软件输出：

| 构建 | Flash | RAM | SHA-256 |
|---|---:|---:|---|
| Debug | 49004B | 4520B | `07e35a461cbf672a2d9cf44b71cb415eb672444f97b04ab812551f128428e723` |
| Release | 38308B | 4504B | `789a278b592ca6a4d9b951d942282504a8ed22601d881e07ca7b00e1a352b453` |

Debug镜像：`../stm32/mecanum_f407/build/Debug/mecanum_f407_front_encoder_readonly.bin`。
Release镜像：`../stm32/mecanum_f407/build/dual_bench_release_20261008/mecanum_f407_front_encoder_readonly.bin`。

## 软件验证证据

- 140项Python离线测试通过（原127项、新13项）；无真实串口参与。
- C只读命令拒绝/故障锁存测试及A/B独立计数、正反向、16位计数回绕、毫秒时间回绕测试通过，ASan/UBSan通过。
- 原双通道12组、Demo30的8组C保护场景仍通过。
- 原Demo和双通道Debug/Release镜像SHA-256均与修改前一致，未放宽原运动白名单或时限。
- `nm`核对新Debug镜像包含TIM4/B计数接口，不包含`HAL_TIM_PWM_Start`或`MX_PWM_Init`。
- 新工具帮助页正常；旧运动工具不能解析`TEL3/front_enc_ro_v1`，新只读工具拒绝旧固件/非零输出。

## 只读观察工具

`../stm32/mecanum_f407/tools/front_encoder_observe.py`只发送stop/status，不使能、复位、清故障或写CPR，不调整网络。

已取得烧录许可。用户明确不需要新增板内Demo备份，因此本次直接用只读版替换板内Demo；本地Demo源码、既有备份和实验日志保留。烧录及读回校验均在动力断开时进行，右前编码器暂不新增。无动力版本与输出检查命令：

```bash
cd /home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407
python3 -B tools/front_encoder_observe.py \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 --confirm-battery-disconnected
```

只有无动力通信验收通过，并复核PB6/PB7板级可达性和5V输入适配后，才提示新增E2A→PB6、E2B→PB7；不互接5V/3.3V电源。后续每次动线、复位或烧录先断动力。

只读固件已确认、车架固定四轮悬空、编码器供电与上电静止确认后，可做两轮静止观察；命令需要新的现场就绪确认，以下不是自动执行授权：

```bash
python3 -B tools/front_encoder_observe.py \
  /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --observe --seconds 10 --expect-stationary \
  --confirm-readonly-firmware --confirm-suspended --confirm-stationary
```

手转A或B时分别记录计数，核对只有被转动的轮子计数变化。CPR标定建议标记轮面、单轮输出轴完整转5圈，正反向各重复至少3次；每次观察窗口最多60秒。`--wheel A --turns 5`或`--wheel B --turns 5`仅说明操作者声明圈数，估计值为`abs(Δcount)/5`，不是自动识别圈数，也不会自动写入控制参数。

每次完整圈数、物理方向、是否误碰另一轮、结束实际静止都需现场反馈；无变化、非选中轮计数变化、遥测故障/超时均不生成有效CPR估计。
短时电动正反转必须后续另备运动固件并取得单次许可，不能在本只读固件中绕过限制。

## 实机烧录与无动力验收

- ST-Link：`E1007200D0D2139393740544`；设备ID `0x413`，Flash 1MiB，Vref约3.20V。
- 首次擦除失败，随后单独停核、读取Flash状态及保护配置；未修改选项字节、未执行全片擦除。再次烧录成功，首次失败根因尚未确认。
- 烧录后校验通过，另行读回49004B与Debug镜像逐字节一致，SHA-256为 `07e35a461cbf672a2d9cf44b71cb415eb672444f97b04ab812551f128428e723`。
- 动力断开时复位并运行，连续5轮串口检查全部 `passed=true`、`errors=[]`；每轮仅发送 `stop/status/status/stop/status`，均识别为 `front_enc_ro_v1`。
- 各轮均为禁用、零指令、A/B原始计数0、速度0、故障0；这只证明无动力通信与只读状态，编码器未供电、B信号未接入，不作为编码器工作或CPR证据。
- 原Demo起始检查的 `ERR,bad_command` 仍保留为未定位异常；新固件5轮通过不能证明已修复其根因。
- 烧录和读回日志：[部署目录](../stm32/mecanum_f407/backups/front_encoder_ro_20261009_ptfy6tIU/)。
- 原始串口结果：[5轮无动力检查](front_encoder_readonly_nopower_20261009.json)。

随后以SWD HOTPLUG只读寄存器，不复位、不停核、不写寄存器：

| 检查项 | 实际值 | 解读 |
|---|---|---|
| GPIOE MODER / PUPDR | `0x00441400` / `0x00882800` | PE5/6/9/11为GPIO输出、下拉，不是PWM复用 |
| GPIOE ODR / IDR | `0x00000000` / `0x0000F59F` | 控制脚输出锁存及输入采样均低；掩码 `0xA60` 均为0 |
| GPIOB MODER / PUPDR / AFRL | `0x0000AA90` / `0x00000000` / `0x22220000` | PB4/5/6/7为AF2、无上下拉 |
| RCC APB1ENR / APB2ENR | `0x10020006` / `0x00004000` | TIM3、TIM4时钟开启；电机TIM1、TIM9时钟关闭 |
| TIM3、TIM4 CR1 / SMCR / CCMR1 / CCER | 各为 `0x1` / `0x3` / `0x6161` / `0x11` | 两编码器计数器运行，TI12编码器模式 |
| TIM3、TIM4 PSC / ARR / CNT | 各为 `0` / `0xFFFF` / `0` | 16位计数范围，当前计数0 |

记录：[寄存器检查日志](../stm32/mecanum_f407/backups/front_encoder_ro_20261009_ptfy6tIU/readonly_gpio_timer_check.log)。寄存器读数不能替代外部线束、引脚电压和实际轮子静止确认。

## 右前轮接入准备：PB6/PB7已确认可用

用户确认PB6、PB7排针均可用，未连接其它外设。
复核ST官方DS8626 Rev 12（2026年3月）：Table 7第60页明确PB6/PB7为FT脚，Table 9第65页对应AF2 TIM4_CH1/CH2；Table 14第82页规定VDD为2～3.6V时FT输入上限5.5V，高于VDD+0.3V时必须禁用内部上下拉。FT不适用于模拟模式。
本项目当前数字编码器输入AF2/NOPULL与这些条件相符；MCU必须先独立供电并维持运行，不在编码器带电时复位/烧录/断开MCU电源。5V容限不是允许把5V接到MCU的3V3电源脚。

- [ST官方数据手册](https://www.st.com/resource/en/datasheet/dm00037051.pdf)。Firecrawl仅解析前65页，输入电气条件另按官方PDF第82页核对；不声称缓存涵盖全部206页。
- [官方引脚资料缓存](../.firecrawl/st-f407-pin-electrical-20261009.json)。
- [开发板官方原理图页面](https://wiki.lckfb.com/zh-hans/tkx/hardware/schematic.html)的已缓存排针图显示PB6/PB7从P2引出；接线按实物丝印，不凭图中左右方向定位。
- 新一轮无动力stop/status检查通过，`front_enc_ro_v1`，末条STATUS3 `ms=488842`，两路禁用/零指令、故障0；未复位、烧录或发送运动指令。

下一项仍先测右前Motor_B的E2A/E2B电压，信号暂不接MCU。保留原A/B控制、公共GND和完整六芯插头，车架固定、四轮悬空，第二块驱动板不供电。保持MCU独立供电、不复位。
万用表直流电压档，黑笔固定已验证6×GND，红笔固定Output_IO/E2A；在驱动开关关闭、动力电池拔掉时布置和固定引线。确认零输出、接线和悬空条件后，接入本板动力并开开关，仅静态读取E2A，不触碰轮子。
关开关、拔电池后再将红笔换到E2B，以同样步骤读取；结束关开关、拔电池。保持E2A/E2B不接PB6/PB7，读数和实际两轮静止由用户反馈；未收到反馈前不视为已完成。
因为完整六芯线仍连接，此测量不是电机动力物理隔离、不是“只给编码器供电”。任何自转、明显异常电压或接触不稳立即断动力。
后续确认兼容后的目标接线为E2A→PB6、E2B→PB7；不连接驱动板5V/3V3到MCU电源，不交换已验证控制线。当前尚未执行新增接线或上电。

## 双前轮接线与静止实机结果

- 用户报告E2A、E2B静态电压均约0V，两前轮始终静止。这是一个停点的低态，不证明完整正交波形；不再反复测高/低态，后续通过计数变化判断。
- 用户明确确认驱动开关关闭、电池拔掉后，新增E2A→PB6、E2B→PB7，撤下测量线；原控制线、左前编码器和公共GND不变。
- 接线后无动力stop/status通过，末条STATUS3 `ms=1083442`，原始A=1/B=2、cps均0、故障0。接线时少量计数变化不作为运动证据，不清零。
- 用户确认第一块板上电、两前轮静止、第二板不供电、四轮悬空后，仅读取5秒，50个样本。起始 `ms=1152452`，结束 `ms=1157473`；A=2/B=2保持不变、cps均0、输出禁用/零指令、故障0。
- 最终stop/status `ms=1157534`、零指令、故障0。用户随后明确确认关开关、拔掉动力电池。
- 原始结果：[双前轮静止记录](front_pair_encoder_stationary_20261009_ms1157534.json)。右前动态计数、方向、CPR尚未验证；本次不是四轮实机验收。
- 用户确认后轮7个普通排针可用，PB3在普通排针未找到；随后确认调试口SWO存在且空闲，作为PB3候选引出点，尚未接后轮线。

## 阶段状态

1. 软件准备：完成；起始Demo无动力检查发现协议故障，已保留异常证据。
2. 烧录只读版及无动力连续通信、GPIO检查：完成；用户免除新增Demo备份，板内Demo已替换，本地资料保留。
3. 右前轮电平/引脚确认与新增编码器接线：完成；双前轮上电静止观察通过，用户已再次断动力。
4. 双编码器动态/符号与CPR重复标定：待验证；独立四轮`four_bench_v1`已在用户确认断动力及许可后备份、烧录、回读一致，4次无动力串口与GPIO/定时器检查通过。后轮仍未接入或上电，未发转动指令；详见四轮验证记录。

本轮未修改代理、热点、路由、网卡或虚拟网卡。每一步未验收不推进下一步。
