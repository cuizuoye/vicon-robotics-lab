# 真机固件源码交接（不是刷机入口）

真机为自定义PX4 1.14.3、MicoAir H743-V2、board id1179。
`upstream.json`记录原厂源码提交、公开基底、修补SHA与已安装包SHA。

原厂源码克隆没有origin，原厂提交未假定可从官方远端直接取得。为避免只留下
无法重建的提交号，这里同时提供两份纯文本差异：

1. 官方PX4 `v1.14.3` → 原厂H743-V2板级源码：`vendor-board.patch`。
2. 原厂源码 → 当前已安装修复对应源码：`changes.patch`。

在**单独、干净的PX4源码克隆**中，先核对 `upstream.json` 的公开基底SHA，
再依次用 `git apply --check` 和 `git apply` 应用这两份patch。
主机迁移时已在隔离临时目录验证两份patch可应用，13个修复文件逐字节等于原源码。
这不等于在新电脑完成了全固件重建或复现了厂商整个镜像。

构建目标：`micoair_h743-v2_default`；原验证编译器：GNU Arm 10.2.1 20201103。
PX4子模块须按该版本自己的gitlink取得，不能套用SITL的依赖。
厂商bootloader二进制未公开分发；若构建流程需要它，应从合法原厂资料本地取得，
不可改board id、强刷bootloader或绕过兼容检查。本仓库不提供自动烧录步骤。

附带PX4 BSD-3-Clause `LICENSE`，源码中的原版权声明保留。
