# ROS 2 麦克纳姆小车

## 基线

STM32F407VGT6、4个MG310霍尔电机、两块D157B/AT8236、60mm麦轮、ESP32-S3 N16R8。
X排列：FL `\`、FR `/`、RL `/`、RR `\`；轴距122mm、轮距170mm均为近似轮心距离。
CPR近似值为 `[3179/3,1049,3167/3,1061]`，方向符号 `[+,-,+,-]`。
四个PI反馈独立，复用的是初始增益，不是已完成带载整定。

数据流：Vicon `car01` → ROS 2控制/安全监督 → UDP46000/46001 → ESP32 UART921600 → STM32。
MCU独立指令租约150ms、ESP会话750ms，断流锁停，重连不能恢复旧运动权限。

原电脑STM32板内：26988字节的**无PWM只读版**，SHA256：
`12d4224f5337b42d3bdc3eef30f4142e957d65d900d61b64d8b8fb22033e9aa6`。
最后停发后的超时是预期状态，无UART故障、PI禁用、PWM定时器关闭、8路控制脚低。
这是迁移前实测状态，不代表新电脑或重新上电后已经验证。

## 运动候选，不自动部署

`mecanum_f407_four_link_motion` 为独立、默认构建排除的4WM1目标：
支持流式速度、同向更新、换向前全停及500ms实测静止。每次最多120秒，PWM≤10%，
轮缘≤50mm/s、车体各轴≤50mm/s、yaw≤0.2rad/s；换向/零速不延长总期限。
故障锁存和一次额度保留。未在真实硬件执行，不能直接开启无限矩形循环。

显式主机 `bounded_motion_mode` 配合 `src/mecanum_vicon_control/config/bounded_motion.yaml`：
新HELLO后至少600ms禁用零速和新鲜4WM1静止反馈，再请求使能；还需要独立的安全使能。
实际收发、使能流程已做过ROS发送器→真实C状态机的内存联调；不是上电实测。

## 离线命令

从仓库根目录运行 `bash scripts/test_offline.sh car`。
ARM工具链在PATH后，可选：`bash scripts/build_car_stm32.sh`。该脚本只构建，不烧录。
ESP-IDF v6.1环境已激活且设置私密 `MC_AP_PASSWORD` 后，可选：
`bash scripts/build_esp32_softap.sh`，只构建，不打开串口。

## 现有接线参考，不代表允许上电

- ESP GPIO17/TX→STM32 PD6/RX；GPIO18/RX←PD5/TX；公共GND，不并接电源输出。
- 第一板：FL Motor_A，PE5/PE6→AIN1/2，E1A/B→PB4/5；
  FR Motor_B，PE9/PE11→BIN1/2，E2A/B→PB6/7。
- 第二板：RL Motor_A，PE13/PE14→AIN1/2，E1A/B→PA15/PB3(SWO)；
  RR Motor_B，PB14/15→BIN1/2，E2A/B→PC6/7。
- 原异常左后六芯线停用；不能因换电脑省略现场接线/供电确认。

## 真实Vicon进度

实际话题 `/vicon/car01/car01`，世界坐标`map`，单位米。
手推421.5mm时yaw变化1.26°，位移与临时显示车头差4.62°；左转得到+91.08°。
遮挡后有效标志失效1.924s，期间无车体位姿，恢复后数据有效；全程没有控制发布者。
不同窗口订阅约154–330Hz，时间戳年龄不是曝光端到端延迟或绝对精度证明。
临时+135°只用于显示，车体中心和精确车头偏移仍待标定。

下一步：充电后现场验证车载逻辑供电，确认后备份/烧录候选、无动力检查、悬空及低速落地。
不重复已有轮向/CPR实验，除非硬件、线束或配置确实变化。
