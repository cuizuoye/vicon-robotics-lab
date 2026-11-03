# 小车源码快照 2026年11月3日

此目录是原机小车工程的脱敏增量归档，不覆盖仓库已有的10月28日可复现基线。
包含当前通用源码、历史候选开发配方、浏览器直接目标仿真、后台演示和静止数据源修复，
以及4WSC STOPFIX_V1固件源代码。它不是拷贝后即可刷写或运行的现场部署包。

## 代码索引

| 路径 | 用途 |
| --- | --- |
| `src/` | 通用ROS控制、矩形轨迹、原标定杆核心 |
| `protocol/` | MC-Link协议与消息约束 |
| `stm32/mecanum_f407/` | 基础STM32工程、HAL/CMSIS、早期独立固件层 |
| `esp32/mecanum_wifi_bridge/` | Wi-Fi UDP与UART通信桥 |
| `candidates/` | 独立候选生成器、仿真、诊断与故障注入测试源码 |
| `docs/wand_browser_direct_2026-10-30/prepared_v1/` | 直接1:1目标的浏览器软件车版本 |
| `docs/wand_stopfix_2026-11-02/prepared_v1/firmware/` | 4WSC STOPFIX_V1控制、协议、停车与原生测试 |
| `docs/wand_background_session_display_2026-11-02/prepared_v1/host/` | 后台会话、浏览器显示、停止确认版本 |
| `docs/wand_static_source_2026-11-02/prepared_v1/host/` | 最新静止数据源修复候选，尚未发布为实机入口 |

## 可复现的新增离线检查

在仓库根目录运行：

```bash
bash scripts/test_car_update_20261103.sh
```

该命令从本目录构建4WSC七组原生C测试（ASan/UBSan），再执行27项数据源单元测试。
不调用历史生成器，不创建ROS节点，不打开硬件或复用现场端口，不产生运动指令。
依赖Ubuntu24.04、GCC/CMake/Ninja、Python3与ROS2 Jazzy Python消息包、NumPy。

## 公开版本的边界

`sign_geometry.json`替换为合成T形测试数据，相关几何摘要随之更新；真实设备ID、MAC、
主目录均已脱敏。现场确认被撤销，不收录私有刷写收据、准入清单、会话密钥、原始日志、
Flash/NVS、厂商SDK二进制或构建产物。旧文档内的路径和命令仅是历史来源说明。

层叠生成器保留原逻辑和占位路径，用于代码审查，并非可移植的一键实机部署入口。
公开离线构建入口另外提供路径适配，不绕过任何原机运行准入。
新增C相对include只是公开副本的路径修正，不改变控制算法或安全阈值。
公开副本统一了换行和行尾空白；导出清单同时保留原文件与公开文件摘要。

完整性见[导出清单](../../../handoff/CAR_EXPORT_2026-11-03.json)和
[验证记录](../../../handoff/CAR_VALIDATION_2026-11-03.json)。
测试通过不转移物理运行许可；当前成果范围与待办见[状态交接](../../../handoff/CAR_STATE_2026-11-03.json)。
