#include "vicon_receiver/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

namespace
{

// ROS 2 话题名只允许字母、数字和下划线，而 Vicon 里的 marker 名字可能带空格、
// 冒号等字符（例如 "T2:1"）。这里做一次安全替换，非法字符统一换成下划线。
// ROS 2 topic names only allow letters, digits and '_', so sanitise the raw name.
std::string topic_fragment(const std::string & name)
{
    std::string safe;
    safe.reserve(name.size());
    for (char c : name)
    {
        const bool is_alnum =
            (c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9');
        safe.push_back(is_alnum || c == '_' ? c : '_');
    }
    return safe;
}

}  // namespace

// Constructor for the Communicator class
Communicator::Communicator() : Node("vicon_client")
{
    // Declare parameters for hostname, buffer size, and namespace
    this->declare_parameter<std::string>("hostname", "127.0.0.1");
    this->declare_parameter<int>("buffer_size", 200);
    this->declare_parameter<std::string>("namespace", "vicon");
    this->declare_parameter<std::string>("world_frame", "map");
    this->declare_parameter<std::string>("vicon_frame", "vicon");
    this->declare_parameter<std::vector<double>>("map_xyz",  {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("map_rpy",  {0.0, 0.0, 0.0});
    this->declare_parameter<bool>("map_rpy_in_degrees", false);
    // 以下为二次开发新增的参数，默认值即为推荐的“低延迟”配置
    this->declare_parameter<std::string>("stream_mode", "ServerPush");
    this->declare_parameter<bool>("enable_marker_ray_data", false);
    this->declare_parameter<bool>("enable_debug_data", false);
    this->declare_parameter<bool>("enable_device_data", false);
    this->declare_parameter<bool>("publish_latency", true);

    // Retrieve parameters values
    this->get_parameter("hostname", hostname);
    this->get_parameter("buffer_size", buffer_size);
    this->get_parameter("namespace", ns_name);

    this->get_parameter("world_frame", world_frame);
    this->get_parameter("vicon_frame", vicon_frame);
    this->get_parameter("map_xyz", map_xyz);
    this->get_parameter("map_rpy", map_rpy);
    this->get_parameter("map_rpy_in_degrees", map_rpy_in_degrees);
    this->get_parameter("stream_mode", stream_mode_name);
    this->get_parameter("enable_marker_ray_data", enable_marker_ray_data);
    this->get_parameter("enable_debug_data", enable_debug_data);
    this->get_parameter("enable_device_data", enable_device_data);
    this->get_parameter("publish_latency", publish_latency);

    last_frame_number_ = 0;
    have_last_frame_number_ = false;
    latency_unavailable_reported_ = false;

    // 帧号与延迟话题：帧号由 Vicon 服务器给出、单调递增；
    // 延迟为服务器上报的“曝光到客户端”总耗时，单位秒。
    frame_number_pub_ = this->rclcpp::Node::create_publisher<std_msgs::msg::Int64>(
        ns_name + "/frame_number", 10);
    latency_pub_ = this->rclcpp::Node::create_publisher<std_msgs::msg::Float64>(
        ns_name + "/latency", 10);

    // Publish static transform from map to vicon origin
    if (map_rpy_in_degrees) {
        for (unsigned int i=0; i<map_rpy.size(); i++) {
            map_rpy[i] = map_rpy[i] * M_PI / 180.0;
        }
    }
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_static_transform();

    // Initialize the tf2 broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

// Publish the static transform from map to vicon origin
void Communicator::publish_static_transform()
{
    static_tf.header.stamp = this->get_clock()->now();
    static_tf.header.frame_id = world_frame;
    static_tf.child_frame_id = vicon_frame;

    static_tf.transform.translation.x = map_xyz[0];
    static_tf.transform.translation.y = map_xyz[1];
    static_tf.transform.translation.z = map_xyz[2];
    tf2::Quaternion q;
    q.setRPY(
        map_rpy[0],
        map_rpy[1],
        map_rpy[2]
    );
    static_tf.transform.rotation.x = q.x();
    static_tf.transform.rotation.y = q.y();
    static_tf.transform.rotation.z = q.z();
    static_tf.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(static_tf);

    string msg = "Published static transform from " + world_frame + " to " + vicon_frame;
    cout << msg << endl;
}


// Connect to the Vicon server
bool Communicator::connect()
{
    // Log connection attempt
    string msg = "Connecting to " + hostname + " ...";
    cout << msg << endl;

    int counter = 0;
    // Retry connection until successful
    while (!vicon_client.IsConnected().Connected && rclcpp::ok())
    {
        bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            counter++;
            msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
            cout << msg << endl;
        }
    }
    if (!rclcpp::ok()) {
        std::cout << "Shutdown requested before connection established." << std::endl;
        return false;
    }

    // Log successful connection
    msg = "Connection successfully established with " + hostname;
    cout << msg << endl;

    // 只开启刚体与反光点必需的数据流。marker ray / debug 数据量很大，
    // 会拉长服务器每帧的准备时间、抬高端到端延迟，因此改为按参数开启。
    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    if (enable_marker_ray_data) {
        vicon_client.EnableMarkerRayData();
    }
    if (enable_device_data) {
        vicon_client.EnableDeviceData();
    }
    if (enable_debug_data) {
        vicon_client.EnableDebugData();
    }

    // ServerPush 是延迟最低的模式：服务器主动推帧，
    // 不需要像 ClientPull 那样每帧再发一次请求、等一次回包。
    if (stream_mode_name == "ClientPull") {
        vicon_client.SetStreamMode(StreamMode::ClientPull);
    } else if (stream_mode_name == "ClientPullPreFetch") {
        vicon_client.SetStreamMode(StreamMode::ClientPullPreFetch);
    } else {
        if (stream_mode_name != "ServerPush") {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown stream_mode '%s', falling back to ServerPush",
                stream_mode_name.c_str());
        }
        vicon_client.SetStreamMode(StreamMode::ServerPush);
    }
    vicon_client.SetBufferSize(buffer_size);

    msg = "Stream mode: " + stream_mode_name;
    cout << msg << endl;

    // Log initialization completion
    msg = "Initialization complete";
    cout << msg << endl;

    return true;
}

// Disconnect from the Vicon server
bool Communicator::disconnect()
{
    // If already disconnected, return true
    if (!vicon_client.IsConnected().Connected)
        return true;

    sleep(1); // Wait before disconnecting

    // Disable all data streams
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();
    vicon_client.DisableCentroidData();

    // Log disconnection attempt
    string msg = "Disconnecting from " + hostname + "...";
    cout << msg << endl;

    // Disconnect from the server
    vicon_client.Disconnect();

    // Log successful disconnection
    msg = "Successfully disconnected";
    cout << msg << endl;

    // Verify disconnection
    return !vicon_client.IsConnected().Connected;
}

// Retrieve and process a frame of data from the Vicon server
void Communicator::get_frame()
{
    // Request a new frame
    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    // 每次取帧都发布帧号与延迟，方便下游做时间对齐、量化“曝光→收到”的时延
    publish_frame_status(frame_number);

    if (frame_number.Result != Result::Success) {
        // No frame has been received yet.
        return;
    }

    // 跳过重复帧：某些流模式下 SDK 会重复返回上一帧，
    // 若原样重发会让话题频率虚高，并使下游求速度/加速度时出现假数据。
    if (have_last_frame_number_ && frame_number.FrameNumber == last_frame_number_) {
        return;
    }
    last_frame_number_ = frame_number.FrameNumber;
    have_last_frame_number_ = true;

    // Get the number of subjects in the frame
    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    map<string, Publisher>::iterator pub_it;

    // Iterate through each subject
    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        // Get the subject name
        string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        // Get the number of segments for the subject
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        // Iterate through each segment
        for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
        {
            // Get the segment name
            string segment_name = vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

            // Retrieve the segment's global position and rotation
            Output_GetSegmentGlobalTranslation trans =
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name);
            Output_GetSegmentGlobalRotationQuaternion quat =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

            // 遮挡/SDK失败不是有效静止位姿：本帧不发布，供下游新鲜度检查判失效。
            // 不能给旧值或默认零值换上新时间戳后继续用于闭环。
            if (trans.Result != Result::Success || quat.Result != Result::Success ||
                trans.Occluded || quat.Occluded)
            {
                continue;
            }

            // Build a TF message for this segment
            geometry_msgs::msg::TransformStamped tf_msg;

            // Use node clock to timestamp the transform
            tf_msg.header.stamp = this->get_clock()->now();

            // Parent and child frames: Vicon global origin -> subject_segment
            tf_msg.header.frame_id = vicon_frame;
            tf_msg.child_frame_id = subject_name + "_" + segment_name;

            // Vicon translations are in millimeters; convert to meters for ROS
            tf_msg.transform.translation.x = trans.Translation[0] / 1000.0;
            tf_msg.transform.translation.y = trans.Translation[1] / 1000.0;
            tf_msg.transform.translation.z = trans.Translation[2] / 1000.0;

            // Vicon quaternion order is [x, y, z, w]; copy directly
            tf_msg.transform.rotation.x = quat.Rotation[0];
            tf_msg.transform.rotation.y = quat.Rotation[1];
            tf_msg.transform.rotation.z = quat.Rotation[2];
            tf_msg.transform.rotation.w = quat.Rotation[3];

            // Publish the position data
            boost::mutex::scoped_try_lock lock(mutex);
            if (lock.owns_lock())
            {
                // Check if a publisher exists for the segment
                pub_it = pub_map.find(subject_name + "/" + segment_name);
                if (pub_it != pub_map.end())
                {
                    Publisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        // Build a PoseStamped in the Vicon frame from the computed TransformStamped.
                        geometry_msgs::msg::PoseStamped vicon_pose_msg;

                        // Header: copy timestamp and frame_id ("vicon") from the transform header.
                        vicon_pose_msg.header = tf_msg.header;

                        // Position: copy the already meter-converted translation components.
                        vicon_pose_msg.pose.position.x = tf_msg.transform.translation.x;
                        vicon_pose_msg.pose.position.y = tf_msg.transform.translation.y;
                        vicon_pose_msg.pose.position.z = tf_msg.transform.translation.z;

                        // Orientation: copy the quaternion (x, y, z, w) directly from the transform.
                        vicon_pose_msg.pose.orientation = tf_msg.transform.rotation;
                        
                        // Update timestamp of static transform
                        static_tf.header.stamp = tf_msg.header.stamp;

                        // Transform the pose to the global frame
                        geometry_msgs::msg::PoseStamped global_pose_msg;
                        tf2::doTransform(vicon_pose_msg, global_pose_msg, static_tf);

                        // Publish the transformed pose
                        pub.publish(global_pose_msg);
                    }
                }
                else
                {
                    // Create a publisher if it doesn't exist, de-duplicating concurrent attempts
                    std::string key = subject_name + "/" + segment_name;
                    if (pending_publishers.find(key) == pending_publishers.end())
                    {
                        pending_publishers.insert(key);
                        lock.unlock();
                        create_publisher(subject_name, segment_name);
                    }
                    else
                    {
                        // Another thread is already creating this publisher
                        lock.unlock();
                    }
                }
            }

            // Broadcast the transform
            tf_broadcaster_->sendTransform(tf_msg);

        }

        // 发布该刚体上所有反光点的原始坐标
        publish_marker_points(subject_name);
    }
}

// 发布当前帧的帧号与 Vicon 上报的延迟
void Communicator::publish_frame_status(const Output_GetFrameNumber & frame_number)
{
    if (frame_number.Result == Result::Success) {
        std_msgs::msg::Int64 frame_number_msg;
        frame_number_msg.data = static_cast<int64_t>(frame_number.FrameNumber);
        frame_number_pub_->publish(frame_number_msg);
    }

    if (!publish_latency) {
        return;
    }

    // 服务器上报的总延迟单位是秒，是采集链路各阶段耗时之和；
    // 若该服务器不导出延迟数据，则保持静默并只提示一次。
    Output_GetLatencyTotal latency = vicon_client.GetLatencyTotal();
    if (latency.Result == Result::Success) {
        std_msgs::msg::Float64 latency_msg;
        latency_msg.data = latency.Total;
        latency_pub_->publish(latency_msg);
    } else if (!latency_unavailable_reported_) {
        latency_unavailable_reported_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "Vicon server does not report latency data; %s/latency stays silent",
            ns_name.c_str());
    }
}

// 读取某个刚体下的全部反光点，逐个发布其全局坐标
void Communicator::publish_marker_points(const string subject_name)
{
    Output_GetMarkerCount marker_count = vicon_client.GetMarkerCount(subject_name);
    if (marker_count.Result != Result::Success || marker_count.MarkerCount == 0)
    {
        return;
    }

    for (unsigned int marker_index = 0; marker_index < marker_count.MarkerCount; ++marker_index)
    {
        // 取坐标前必须先拿到 marker 名称
        Output_GetMarkerName marker_name_out =
            vicon_client.GetMarkerName(subject_name, marker_index);
        if (marker_name_out.Result != Result::Success)
        {
            continue;
        }

        const string marker_name = marker_name_out.MarkerName;

        // 坐标是相对 Vicon 原点的全局坐标，单位为毫米
        Output_GetMarkerGlobalTranslation marker_translation =
            vicon_client.GetMarkerGlobalTranslation(subject_name, marker_name);
        if (marker_translation.Result != Result::Success || marker_translation.Occluded)
        {
            // 被遮挡时 SDK 会返回 [0,0,0]，那不是真实位置，
            // 所以这一帧对该点直接跳过：宁可曲线断点，也不发假数据。
            continue;
        }

        geometry_msgs::msg::PointStamped vicon_point;
        vicon_point.header.stamp = this->get_clock()->now();
        vicon_point.header.frame_id = vicon_frame;
        // Vicon 返回毫米，ROS 侧统一用米，这里做单位换算
        vicon_point.point.x = marker_translation.Translation[0] / 1000.0;
        vicon_point.point.y = marker_translation.Translation[1] / 1000.0;
        vicon_point.point.z = marker_translation.Translation[2] / 1000.0;

        const string key = subject_name + "/" + marker_name;

        boost::mutex::scoped_lock lock(mutex);
        map<string, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr>::iterator pub_it =
            marker_pub_map.find(key);

        if (pub_it != marker_pub_map.end())
        {
            // 变换到 world 系，与刚体位姿的发布保持一致
            geometry_msgs::msg::PointStamped world_point;
            tf2::doTransform(vicon_point, world_point, static_tf);
            pub_it->second->publish(world_point);
        }
        else
        {
            // Create the publisher once, de-duplicating concurrent attempts.
            if (pending_marker_publishers.find(key) == pending_marker_publishers.end())
            {
                pending_marker_publishers.insert(key);
                lock.unlock();
                create_marker_publisher(subject_name, marker_name);
            }
        }
    }
}

// Create a publisher for a specific marker of a subject.
void Communicator::create_marker_publisher(const string subject_name, const string marker_name)
{
    // Launch a thread to create the publisher
    boost::thread(&Communicator::create_marker_publisher_thread, this, subject_name, marker_name);
}

// Thread function to create a marker publisher.
void Communicator::create_marker_publisher_thread(const string subject_name, const string marker_name)
{
    // 构造话题名与内部索引键：<命名空间>/<刚体名>/markers/<marker名>
    std::string topic_name = ns_name + "/" + subject_name + "/markers/" + topic_fragment(marker_name);
    std::string key = subject_name + "/" + marker_name;

    // Log publisher creation
    string msg = "Creating publisher for marker " + marker_name + " from subject " + subject_name;
    cout << msg << endl;

    // Create and store the publisher; then clear the pending flag
    boost::mutex::scoped_lock lock(mutex);
    marker_pub_map.insert(std::make_pair(
        key,
        this->rclcpp::Node::create_publisher<geometry_msgs::msg::PointStamped>(topic_name, 10)));
    pending_marker_publishers.erase(key);
}

// Create a publisher for a specific subject and segment
void Communicator::create_publisher(const string subject_name, const string segment_name)
{
    // Launch a thread to create the publisher
    boost::thread(&Communicator::create_publisher_thread, this, subject_name, segment_name);
}

// Thread function to create a publisher
void Communicator::create_publisher_thread(const string subject_name, const string segment_name)
{
    // Construct the topic name and key
    std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
    std::string key = subject_name + "/" + segment_name;

    // Log publisher creation
    string msg = "Creating publisher for segment " + segment_name + " from subject " + subject_name;
    cout << msg << endl;

    // Create and store the publisher; then clear the pending flag
    boost::mutex::scoped_lock lock(mutex);
    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));
    pending_publishers.erase(key);
    lock.unlock();
}

// Main function
int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();

    // Connect to the Vicon server
    node->connect();

    // 主循环：持续取帧并发布。
    // 注意这里是直接忙轮询、没有走 executor->spin()，
    // 所以该节点的参数服务/话题自省等接口不会被响应，只对外发布话题与 TF。
    while (rclcpp::ok()){
        node->get_frame();
    }

    // Disconnect from the Vicon server and shut down ROS 2
    node->disconnect();
    rclcpp::shutdown();
    return 0;
}
