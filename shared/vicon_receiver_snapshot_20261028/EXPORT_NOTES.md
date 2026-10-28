# Vicon接收器开发源码快照

保留GPL许可证。此目录是10月28日源码工作树快照，含尚未提交到上游的捕获队列与
FrameSample接口开发；不是原机已安装接收器的二进制等价证明，也不是新硬件验收。
旧共享接收器目录保留不变，无人机代码未覆盖。

专有DataStream SDK、Boost厂商二进制和原机ELF来源许可未上传。
`integration/`保留小车依赖的位姿桥接与来源核验源码；私有来源清单不存在时必须拒绝，
不得把本快照的构建结果自动替代旧安装或自动生成认可清单。

可以不接Vicon执行队列纯C++测试（源码不初始化ROS或SDK）：

```bash
mkdir -p .local/vicon-queue
g++ -std=c++17 -Wall -Wextra -Werror -pthread \
  -Ishared/vicon_receiver_snapshot_20261028/vicon_receiver/include \
  shared/vicon_receiver_snapshot_20261028/vicon_receiver/test/test_capture_queue.cpp \
  -o .local/vicon-queue/test_capture_queue
.local/vicon-queue/test_capture_queue
```

完整接收器构建仍需按许可证在本机补齐SDK及声明的依赖，本次未执行完整SDK链接。
