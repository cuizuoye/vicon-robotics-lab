# 两台电脑接力与回退

## 新电脑

按根目录README克隆，切换到 `dev/laptop/<任务名>` 分支，再运行环境检查和离线测试。
参考宿主环境为Ubuntu24.04/Python3.12/ROS2 Jazzy；Python脚本不会代装ROS和系统编译器。
完整测试需要已有Jazzy、colcon、C/C++编译器、CMake及Python venv支持。
没有ROS时可先运行 `bash scripts/test_offline.sh drone` 做纯Python回归。

交接时先说明：

> 先阅读AGENTS.md、handoff/CURRENT_STATE.json、handoff/VALIDATION.json和对应项目README。
> 先跑离线检查，报告已验证和未验证的部分。只在当前开发分支修改。
> 不连接硬件、不发实机指令、不改网络、不自动烧录；需要这些操作时先停下来说明。

不要复制原电脑登录凭证。接力靠仓库中的状态和测试，不靠假定另一段对话可见。

完成一次小任务后，在该开发分支提交并推送：

```bash
python3 scripts/public_scan.py
git status --short
# 只添加本次审查过的文件，然后提交
git push -u origin HEAD
```

这不会自动修改main，也不会更新主电脑原工作区。没有配置自动合并、部署或刷机。

## 主电脑接回

主电脑原小车/无人机工作区保持不动。先在本仓库独立副本中fetch开发分支，
查看提交与差异，再新建review分支运行测试。通过后由人选择需要的提交，
逐项集成到主要项目；不把整个原目录替换为克隆目录。

## 新方向错误时

保留错误方向分支供分析，另建稳定基线目录，不会删除任何现有修改：

```bash
git fetch origin --tags
git worktree add --detach ../vicon-baseline baseline-2026-10-15
```

若错误已经合入共享分支，应审查后用 `git revert <提交>` 新增反向提交；
不默认强推或 `reset --hard`。回退代码不会恢复实机固件、NVS、PX4参数、
供电接线、marker位置或有效校准。公开仓库故意不含这些私密原始备份。
