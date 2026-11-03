# 高速软件准备结果（2026-10-23）

用户反馈低速全向基本正常；接受首档0.20m/s、后续逐级提高。独立4WGD候选上限为每轮400RPM请求目标，理想60mm轮缘直行速度约1.26m/s，不保证1.4kg负载速度。原4WGC镜像、电脑端及原矩形/ESP/Tenda不改；本轮没有实车操作、刷写或真实ROS执行。

主说明与安全边界：`stm32/mecanum_f407/candidates/high_speed_4wgd/README.md`（工作区根目录）。

交付：

- `verified/arm/mecanum_f407_high_speed_4wgd_OFFLINE.bin`：48476字节，SHA256 `d89f9e73669abb0578a3b69e6d49f8dc05daaa7fca42d0f55777a13caaf6cc44`，未刷写。
- `verified/private_runtime/install/`：独立控制/硬件软件包，域95、`/wand_highspeed_acceptance`，未启动。只支持审核后的首档0.20m/s，不能直接开放后续速度。
- `verified/private_runtime/src/mecanum_wand_highspeed_hardware/config/high_speed_020.REVIEW_REQUIRED.json`：现场确认false、滑行距离null，故意在Node/socket构造前阻断hardware模式；不是可直接采用的实机配置。
- `verified/summary.json`：37组C检查、25项高速参数/解析检查、11项独立运行时安全合同检查、ARM与两个ROS包构建通过，239个原文件前后哈希一致。
- `accepted_low_speed_regression.log`：原低速198项通过，6个原动态模型场景未运行。

早期`offline_01/`与`runtime_01/`是本轮保留的中间构建；以`verified/`为最终交付。新候选只是编译/静态与单元验证，不是高速动力、刹停或实车动作验收。驱动00为滑行，不把零输出当作已刹停；下一阶段需复核实际停车余量、供电/电流/温升，再单独授权备份刷写和初档测试。无需重做已采用的CPR、轮序或低速全向工作。
