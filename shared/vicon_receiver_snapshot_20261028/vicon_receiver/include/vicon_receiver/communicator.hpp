#if !defined(COMMUNICATOR_HPP)
#define COMMUNICATOR_HPP

#include "vicon-datastream-sdk/DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include <iostream>
#include <map>
#include <chrono>
#include <string>
#include <unistd.h>
#include "boost/thread.hpp"
#include <set>
#include <thread>
#include "capture_queue.hpp"
#include "vicon_receiver/msg/frame_sample.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Transform.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std;

// Main Node class
class Communicator : public rclcpp::Node
{
private:
    ViconDataStreamSDK::CPP::Client vicon_client;

    string hostname;
    unsigned int buffer_size;
    string ns_name;
    map<string, Publisher> pub_map;
    map<string, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> marker_pub_map;
    boost::mutex mutex;
    std::set<std::string> pending_publishers;
    std::set<std::string> pending_marker_publishers;
    
    geometry_msgs::msg::TransformStamped static_tf;
    string world_frame;
    string vicon_frame;
    vector<double> map_xyz;
    vector<double> map_rpy;
    bool map_rpy_in_degrees;

    // 数据流配置（由同名 ROS 参数传入，见 README 的参数表）
    // stream_mode_name       : DataStream 工作模式，ServerPush 延迟最低
    // enable_*_data          : marker ray / debug / device 三类可选数据流，
    //                          默认关闭，因为它们会显著增加每帧数据量
    // publish_latency        : 是否发布 Vicon 服务器上报的总延迟
    string stream_mode_name;
    bool enable_marker_ray_data;
    bool enable_debug_data;
    bool enable_device_data;
    bool publish_latency;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 帧状态话题发布器
    // frame_number_pub_ -> <namespace>/frame_number，原始帧号，用于丢帧检测与时间对齐
    // latency_pub_      -> <namespace>/latency，服务器上报的总延迟，单位秒
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr frame_number_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

    // 去重用：上一帧的帧号，以及"是否已经收到过帧"的标记
    unsigned int last_frame_number_;
    bool have_last_frame_number_;

    // 服务器不支持延迟数据时只提示一次，避免刷屏
    bool latency_unavailable_reported_;

    // Default OFF; only this explicitly selected subject/segment is recorded.
    bool capture_enabled_{false};
    string capture_subject_, capture_segment_;
    vicon_capture::Queue<257> capture_queue_;
    rclcpp::Publisher<vicon_receiver::msg::FrameSample>::SharedPtr capture_pub_;
    std::thread capture_worker_;
    std::atomic<bool> capture_stop_{false};
    uint64_t capture_sequence_{0}, capture_epoch_{0}, capture_missing_{0};
    void publish_capture_loop();

    void publish_static_transform();

public:
    Communicator();
    ~Communicator();

    // Initialises the connection to the DataStream server
    bool connect();

    // Stops the current connection to a DataStream server (if any).
    bool disconnect();

    // Main loop that request frames from the currently connected DataStream server and send the 
    // received segment data to the Publisher class.
    void get_frame();

    // Publish the frame counter and the Vicon-reported capture-to-client latency.
    // 发布帧号与 Vicon 上报的延迟，供下游做时间对齐和时延量化
    void publish_frame_status(const ViconDataStreamSDK::CPP::Output_GetFrameNumber & frame_number);

    // functions to create a segment publisher in a new thread
    void create_publisher(const string subject_name, const string segment_name);
    void create_publisher_thread(const string subject_name, const string segment_name);

    // Read every marker attached to a subject and publish it as PointStamped.
    // 读取刚体上的每一个反光点，各自以 PointStamped 单独发布
    void publish_marker_points(const string subject_name);

    // functions to create a marker publisher in a new thread
    void create_marker_publisher(const string subject_name, const string marker_name);
    void create_marker_publisher_thread(const string subject_name, const string marker_name);
};

#endif // COMMUNICATOR_HPP
