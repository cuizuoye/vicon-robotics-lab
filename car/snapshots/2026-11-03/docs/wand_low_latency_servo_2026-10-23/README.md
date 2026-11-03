# 低延迟连续伺服：离线完成，未切换实车

后续真实Sign驱动仿真的结论见[本轮实时验收记录](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/real_sign_acceptance.md)：
停车、遮挡停车和禁止自动恢复已验证；同步体感未通过，最后的手动重新准备尚未操作，不能视为整体验收通过。

2026-10-23。只使用合成输入和虚拟时间，没有接入Vicon、创建真实ROS节点、打开小车socket/串口、
刷写或启动实车。原矩形程序、STM32/ESP32、原操作台及网络均未修改。
本轮没有改变已运行程序，也没有将离线候选自动设为默认版本。

## 实现

- 保持Sign水平位移1:1；Z和杆姿态不转换为车辆平移/偏航，车头仍保持启动朝向。
- 保持固定原起点/原朝向的前3米、左3米名义目标区；每次准备不挪围栏。
- 车体190×210mm；车体外各0.50m物理缓冲和原0.20m额外余量保留。
  目标限幅仍为3×3米；缓冲不是额外目标区。没有证明50cm就是实际停车距离。
- 不提高能力上限：合速度0.20m/s、合加速度0.08m/s²、偏航0.10rad/s、120秒PC会话。
  没有修改4W20固件，35% PWM、600ms底层断令及150秒底层上限未改。
- 去掉80ms位置低通及参考层0.04m/s²重复加速；参考仍限速并保留20cm领先限制，
  最终指令只保留一次正常运动加速度限制。末级门仍独立校验限速/加速度，不是另加一个慢滤波器。
- 速度前馈用最近40ms源时间戳的因果回归，不用回调到达间隔估计速度；
  不外推或伪造位姿，60ms没有新样本便撤掉前馈，200ms定位超时停车仍保留。
- 运动时位置增益0.20/s，停稳后恢复1.4/s，80ms运动滞回避免过零切换。
  参考触及20cm领先限制时恢复正常位置纠偏，防止大目标在限制边缘被长期拖慢。
- 独立FinalGate代码不变；保持松键/租约、遮挡、反馈、来源身份/签名、重放、围栏、
  抬车/倾斜、时钟、无进展、时限和故障停车；恢复数据不会自动继续运动。

## 离线效果（同一车辆近似模型，同一限速/加速度）

模型为0.12秒一阶响应、前/侧增益0.98/0.88、偏航增益0.95，**不是实车辨识结果**。
对比双方均经过完整监督器和同一独立末级门，使用同一时刻的杆目标与车位姿，未平移曲线对齐。

| 合成轨迹 | 旧版位置误差RMS | 新版位置误差RMS |
| --- | ---: | ---: |
| 0.1m/s前移、停住、返回 | 7.04cm | 4.32cm |
| 连续圆周，半径0.10m、角频率0.6rad/s | 5.77cm | 1.46cm |
| 0.06m/s反复往返、突变换向 | 6.56cm | 5.47cm |

模型车速达到0.01m/s的起步时间从0.36～0.38秒降到0.24秒；这是指定轨迹下的模型阈值，
**不是相机曝光到轮子动作的实测延迟，也不是保证任意手势同步**。
急停/急反向输入超出0.08m/s²时仍有明显误差，不能凭离线结果宣称已经复制任意人手动作。
大目标和越界输入测试均限幅且最终收敛至15mm容差内；0.2mm合成定位抖动未驱动车辆。

## 验证与工件

最终运行包：`verified_runtime/install`；实现源：`verified_runtime/src`。
`runtime/`、`comparison_initial/`、`comparison/`是开发中间记录，不是最终验收结果。
最终结果由下面离线检查脚本生成的`check_*/tests.xml`和`check_*/comparison/comparison.json`记录。

本轮最终记录：

- [71项检查](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/check_qxttqY/tests.xml)
- [完整A/B数值与模型边界](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/check_qxttqY/comparison/comparison.json)
- [同一时刻的轨迹/误差曲线](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/check_qxttqY/comparison/comparison.png)

![离线对比，非实车测量](/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/check_qxttqY/comparison/comparison.png)

71项检查覆盖源时间戳/重放/异常值、估计器复位、1:1与四边限幅、旋转完整车体、固定围栏、
实际ROS节点构造（IO替身，无DDS）、27类故障注入、不自动恢复、真实传输入口拒绝、签名校验、
生成可复现性、六类A/B轨迹及三组未辨识动力学扰动。两个独立ROS包构建成功。
旧3×3版全部输入文件及既有矩形/固件保护清单重新核对哈希，未变化。

最终独立包名`mecanum_wand_servo_control`、`mecanum_wand_servo_hardware`。
域112、合成源域113；没有实际启动节点；hardware/emulator模式在任何socket构造前无条件拒绝。
只安装`wand_servo_offline`，不提供可运行实车桥或真实Sign操作台入口。

仅离线重跑（无实车、Vicon或ROS节点启动）：

```bash
bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/run_wand_servo_offline_checks.sh
```

每次生成全新`check_XXXXXX`目录，不覆盖旧结果。脚本只在自身环境加载ROS/私有包路径。
旧实车启动指令仍运行旧版；本候选没有自动迁移，也不应拿旧指令声称启用了新版。

下一阶段可先进行真实Sign驱动仿真和完整时间戳测量，仍保持实车断电；
真实端到端延迟、换向/停车余量、电流、温升及更高速度需要后续单独授权和测量。
