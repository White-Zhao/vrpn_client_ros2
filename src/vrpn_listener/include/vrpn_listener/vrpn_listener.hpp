#ifndef VRPN_LISTENER_HPP
#define VRPN_LISTENER_HPP

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>
#include <unistd.h>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h> // 必须包含这个

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp> // <--- 必须在这里包含


class ConstantVelocityKF;
class OrientationESKF;
struct Synchronizer
{
    std::string sender_name;
    void *listener_ptr;
    std::shared_ptr<vrpn_Tracker_Remote> vrpn_tracker;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    // 注意：虽然我们精简了 cpp，但保留这些结构定义不会报错，或者你可以根据需要删除
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher; // 新增：用于发布欧拉角
    // 新增：用于存储每个刚体的 7 个滤波器 (3位置 + 4姿态)
    std::vector<std::shared_ptr<ConstantVelocityKF>> kfs; // 仅用于位置 (3路)
    std::shared_ptr<OrientationESKF> eskf_orient;         // 用于姿态 (ESKF)

};

class VRPNListener : public rclcpp::Node
{
private:
    // 1. Params
    int _port;
    std::string _server;
    std::string _frame_id;
    double _mainloop_frequency;
    double _refresh_trackers_frequency;
    double _tracker_mainloop_frequency;

    // 2. VRPN Entities
    std::shared_ptr<vrpn_Connection> _vrpn_connection;
    std::unordered_map<std::string, std::shared_ptr<Synchronizer>> _synchronizers;

    // 3. Timers
    rclcpp::TimerBase::SharedPtr _mainloop_timer;
    rclcpp::TimerBase::SharedPtr _trackers_refresh_timer;

    // 4. 新增：TF 广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    // 核心函数
    static void VRPN_CALLBACK handlePose(void *userData, const vrpn_TRACKERCB trackerData);
    // 如果你在 cpp 中删除了这两个函数的实现，可以把声明也删掉
    static void VRPN_CALLBACK handleTwist(void *userData, const vrpn_TRACKERVELCB trackerData);
    static void VRPN_CALLBACK handleAccel(void *userData, const vrpn_TRACKERACCCB trackerData);

    void vrpnConnectionMainloop();
    void refresh_trackers();
    void mainloop();
    void createSynchronizer(std::string sender_name);
    
    // 新增：DDS 初始化函数
    void setupWrenchSubscriber();

    template <class T>
    void loadParam(std::string param_name, T default_value, T &param);
    void loadParams();
    void replaceSpace(std::string &ori_str);

public:
    VRPNListener(std::string name);
};

#endif