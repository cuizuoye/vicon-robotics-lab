# 四轮输出轴CPR初测：手动整圈，只读计数

当前工具不使能电机、不写标定参数、不自动运行PI。CPR估计需要真实手转圈数，任意5秒试转计数不是CPR。

## 操作顺序

1. 车架固定，四轮悬空；旧异常六芯线停用，现有接线不动。STM32先独立供电，保持不复位；动力开关关闭时接电池，再开开关，确认四轮静止。
2. 选择每个轮面/轮毂的明显固定特征作为起点。贴小标记只是辅助，也可用已有螺丝。数的是整个输出轴/轮子转动，不是麦轮小滚子。先不转轮，读取基线。
3. 基线工具：`four_encoder_stationary_observe.py PORT --confirm-powered-stationary --confirm-suspended`。固定5秒，只有stop/status；四路未使能、CCR=0/0、cps=0和计数稳定才能开始手转。
4. 保持供电和固件/接线不变，不发arm/duty/hb；收到“基线已保存，可以手转”的提示后，按FL、FR、RL、RR顺序各轻转整轮5圈，统一前进参考为轮子顶部轮面朝车头。每轮起点/终点特征对齐；遇到异常阻力不强拧，停止并说明。转完保持不动，回报每个轮实际圈数和方向；不要先断动力，避免断电信号边沿混入计数。
5. 程序再次用相同只读工具记录5秒静止数据。末条原始计数减基线原始计数，再除实际圈数，得到每个轮CPR初测估计。测量区间未连续采样，不能从端点判断期间每一时刻或任何往返抵消；圈数和无额外移动靠现场记录。
6. 保存端点原始数据和现场圈数。初测不宣称重复性完成；换方向或同方向再测一致后才生成运行参数候选。标定实测输出轴CPR已包含解码倍频和减速比，不再重复除这些因子。

## 离线换算器

`stm32/mecanum_f407/tools/four_wheel_cpr_evaluate.py`只读取文件并输出JSON，没有串口打开或参数写入。以下5圈是用法示例，运行时必须替换为现场实际圈数，不允许默认当作已完成：

```bash
python3 -B tools/four_wheel_cpr_evaluate.py BEFORE.json AFTER.json \
  --turns FL=5 FR=5 RL=5 RR=5 \
  --encoder-signs FL=1 FR=-1 RL=1 RR=-1 --direction forward \
  --confirm-manual-only --confirm-no-reset-or-power-change \
  --confirm-integer-output-turns
```

可以只测一个轮，但其它轮在两端点之间必须无计数变化。两端点须来自成功的`four_wheel_powered_stationary_observation`，命令只有stop/status；发生故障、计数/CPS不稳定、错误符号、零增量、MCU时间异常或圈数不可靠则不生成估计。

单次初测估计的输出永远标记`repeatability_verified=false`、`runtime_parameters_written=false`。圈数不确定就重做测量，不能调整圈数凑出期望CPR。全部端点读取完，再关闭两板开关、拔动力电池。

## 圈数容易混淆时：改为单轮整数圈

2026-10-09实际采用单轮3圈，FR隔离复测采用1圈；5圈并非必须值。每次先把唯一轮毂/轮壳标记对齐车架固定参考，再读取新基线；一圈一停，最后一圈对齐后不补转，现场回报真实圈数，再读取结束端点。不要选择小滚子等周期性重复特征来替代唯一整轮标记。其它三轮在该区间不应被碰动。

当前结果见[四轮CPR手转复测汇总](FOUR_WHEEL_CPR_RESULTS_2026-10-09.md)，仅初测，不是可直接启用PI的最终标定参数。
