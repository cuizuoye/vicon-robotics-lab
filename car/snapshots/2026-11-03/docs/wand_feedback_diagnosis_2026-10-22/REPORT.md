# 2026-10-22 Sign实车停机：只读诊断与原代码离线反例

## 条件与边界

用户确认：实际小车移动，无人推动或碰线；断电检查左后轮无卡住，接线无松动。本轮保持断电，没有打开硬件桥、清故障、刷写或修改固件、原矩形、Tenda配置。分析的是上一轮记录，不冒充当前板内Flash读回。

数据：`docs/wand_hardware_2026-10-22_prepared_retry_live/events.jsonl`、`/home/PRIMARY_USER/.ros/log/python3_89555_1792662670462.log`。时间用记录内单调时钟，日志墙钟为2026-10-22 17:52附近。

## 已确认的停机链条

第二次ACTIVE从30675.927820起，到30684.888909转STOPPING，共约8.96秒。Vicon实测车位置约从(0.62125,-1.04575)m到(0.76979,-0.77098)m，净移动约0.312m。

30684.887131首次收到带故障的轮反馈：`fault_bits=131088=0x20010`；随后30684.897031诊断`fault_wheel=3`（FL/FR/RL/RR的一基编号，对应左后轮）、permitted=0，所有输出为0。

- `0x10`：`FOUR_SPEED_FAULT_FEEDBACK`，单轮反馈保护。
- `0x20000`：`FOUR_LINK_REJECTED`，已有故障后后续命令被拒绝。离线原代码可复现先0x10再合并为0x20010。
- PC先锁停，继而独立末级门收到执行器故障，发送ESTOP并锁存硬故障。这不是先发生签名、按键、Vicon或UDP超时。
- 故障前MCU持续有新诊断/时钟进展，故障轮标志0；不是用旧缓存或电脑界面文字推测硬件故障。

## 故障前左后轮边界状态

| 接收单调时间 | 左后目标 count/s | 左后滤波反馈 count/s | 左后占空permille | 车体命令 mm/s,mm/s,mrad/s |
|---|---:|---:|---:|---|
| 30684.49971 | -39 | -289 | -30 | 32,-39,0 |
| 30684.59962 | 0 | -75 | 0 | 36,-34,0 |
| 30684.69751 | 0 | -12 | 0 | 37,-34,0 |
| 30684.79877 | 0 | -1 | 0 | 34,-37,0 |
| 30684.89703 | 0 | 0 | 0 | 0,0,0；fault_wheel=3 |

当x与-y近似抵消时，左后目标进入固件20count/s死区。其他轮仍在输出；随后PC目标再次变为负向非零。诊断约100ms一次，而单轮控制10ms一次，因此表中是故障前后快照，不是触发样本的精确目标/原始delta。不能把滤波值当作那一帧原始计数。

原`four_wheel_speed.c:294–300`把数值/几何无效、超过10000count/s、与run_direction相反的任意非零计数、非选定轮变化都合并为0x10。其中方向判据只要求requested_cps非零，没有过零重启/低速被动滚动的分情形判断；一帧一个反向计数也会报错。四轮组会把该轮编号记录为3并全部停止。

## 原版4WGB C算法离线对照（不是实车原始计数回放）

本目录`reproduce.c`直接链接未改动的原C源文件，使用原4WGB编译宏、实有轮参数、原MC-Link消息处理和四轮组。对照均人工合成10ms编码器计数，不声称重建了故障原始样本。

起始车体命令(32,-39)mm/s，左后目标-39.203count/s；再给(36,-34)，左后目标进入零速死区；零目标期间+1被动计数被接受且左后PWM=0；再给(31,-39)，左后目标-44.804count/s：

1. 正常-1计数：无故障。
2. +1反向计数：原代码立即报0x10、fault_wheel=3，下一正常控制包后合并为0x20010，与实车故障签名一致。
3. -101同向计数/10ms（超过10000count/s）：也报同样签名，证明现有故障码无法唯一确定触发分支。

三个对照断言全部通过。无固件修复实现。固件94个既有源文件及矩形72个既有控制文件的既有SHA256清单复核全部一致。

复现命令（工作区根目录，输出是PC本机诊断程序，不是板卡固件）：

```bash
diagnostic_dir=$(mktemp -d /tmp/wand-feedback-diagnosis-XXXXXX)
cc -std=c11 -O0 -g -Wall -Wextra -Werror -ffunction-sections -fdata-sections \
 -DDRIVE_STARTUP_OBSERVE=1 -DFOUR_LINK_MOTION=1 -DFOUR_LINK_GROUNDB=1 \
 -I stm32/mecanum_f407/Inc -I protocol \
 docs/wand_feedback_diagnosis_2026-10-22/reproduce.c \
 stm32/mecanum_f407/Src/four_link_motion.c \
 stm32/mecanum_f407/Src/drive_bench_control.c \
 stm32/mecanum_f407/Src/four_drive_group.c \
 stm32/mecanum_f407/Src/four_wheel_speed.c \
 stm32/mecanum_f407/Src/mecanum_drive_plan.c \
 stm32/mecanum_f407/Src/mecanum_kinematics.c \
 stm32/mecanum_f407/Src/mc_protocol.c \
 -Wl,--gc-sections -lm -o "$diagnostic_dir/reproduce"
"$diagnostic_dir/reproduce"
```

## 结论与未闭合证据

已定位为STM32单轮反馈保护，不是旧的ACTIVE握手超时，不能据此认定电机损坏或接线问题。当前最符合上下文的候选是近零轮重新参与控制时的单采样反向计数判定；它已可用原代码离线复现，但实际故障仍缺触发帧的raw delta、dt和具体分支，不能排除一次计数毛刺/异常高速样本。

此前Python MC-Link仿真直接由command生成wheel speed，未运行原MCU逐轮反馈判定，也无近零轮被动滚动/反向一计数模型。协议仿真通过不能证明这类实车工况通过，这是本次查明的覆盖缺口。

建议下一阶段先离线：细分首故障原因并保存触发快照；针对零目标轮被动滚动、同向重启和真正持续反向/超速分别做原生C故障注入；仅对真实需容许的过渡制定有界策略。不要整体关闭方向/超速保护，不靠降低PC速度或重复上电碰运气。任何候选应独立于已验收矩形/4WGB基线；刷写仍需另行明确授权及板内备份。
