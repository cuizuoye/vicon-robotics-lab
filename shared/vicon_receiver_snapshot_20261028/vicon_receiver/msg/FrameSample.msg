# Opt-in maintenance side channel. All SDK values come from ONE GetFrame.
# Neither ROS stamp nor monotonic timestamps claim to be exposure time.
uint64 sequence
uint64 sdk_frame
float64 sdk_hz
bool sdk_ok
bool occluded
bool pose_published
uint64 sdk_epoch
uint64 missing_frames
uint64 queue_dropped
uint64 sdk_begin_monotonic_ns
uint64 sdk_end_monotonic_ns
builtin_interfaces/Time ros_received_stamp
std_msgs/Header source_header
string subject
string segment
geometry_msgs/Pose sdk_pose
geometry_msgs/Pose world_pose
bool exposure_time_known
