# Shared Vicon receiver

`vicon_receiver_ws/src/ros2-vicon-receiver` contains the actual modified source,
including the uncommitted eight-line rigid-body occlusion filter present on the
primary PC at the snapshot. Upstream GPL notices remain in place.

The proprietary Vicon DataStream SDK and vendored Boost binaries were deliberately
not copied into the public repository. Obtain the SDK under its applicable terms
and install the matching headers/libraries locally. The receiver's CMake currently
expects `vicon_receiver/third_party/<x86_64|aarch64>/{vicon_sdk,boost}`. These local
directories are Git-ignored; do not replace them with an assumed public license.

Offline source/coordinate/protocol tests do not need the Vicon SDK. Building and
running the actual receiver does. On a prepared machine, build in this workspace
with `colcon build --base-paths src --symlink-install`; do not restart the shared
lab receiver just to test a new checkout.

Use rigid-body PoseStamped/Odometry for control. Individual markers are only for
geometry, visibility and quality debugging. A static frame or an old displayed
RViz model does not prove a current pose is being received.
