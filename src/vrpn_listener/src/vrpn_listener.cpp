#include "vrpn_listener/vrpn_listener.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

// ==========================================
// 核心算法类：位置恒定速度卡尔曼滤波器 (CV-KF)
// ==========================================
class ConstantVelocityKF {
public:
    ConstantVelocityKF(double dt, double q_noise, double r_noise) {
        _dt = dt; 
        _R = r_noise;
        _x[0] = 0.0; _x[1] = 0.0; // [位置, 速度]
        _Q[0][0] = q_noise; _Q[0][1] = 0.0;
        _Q[1][0] = 0.0;     _Q[1][1] = q_noise;
        _P[0][0] = 1.0;     _P[0][1] = 0.0;
        _P[1][0] = 0.0;     _P[1][1] = 1.0;
    }

    void init(double val) { _x[0] = val; _x[1] = 0.0; }

    double update(double z) {
        // 预测步
        _x[0] += _x[1] * _dt; 
        _P[0][0] += _dt * (_P[1][0] + _P[0][1]) + _dt * _dt * _P[1][1] + _Q[0][0];
        _P[0][1] += _dt * _P[1][1];
        _P[1][0] += _dt * _P[1][1];
        _P[1][1] += _Q[1][1];
        // 更新步
        double S = _P[0][0] + _R;
        double K0 = _P[0][0] / S;
        double K1 = _P[1][0] / S;
        double res = z - _x[0];
        _x[0] += K0 * res;
        _x[1] += K1 * res;
        double p00_t = _P[0][0]; double p01_t = _P[0][1];
        _P[0][0] = (1.0 - K0) * p00_t;
        _P[0][1] = (1.0 - K0) * p01_t;
        _P[1][0] -= K1 * p00_t;
        _P[1][1] -= K1 * p01_t;
        return _x[0];
    }

    double getVel() const { return _x[1]; }

private:
    double _dt, _R, _x[2], _P[2][2], _Q[2][2];
};

// ==========================================
// 核心算法类：姿态误差状态卡尔曼滤波器 (ESKF)
// ==========================================
class OrientationESKF {
public:
    OrientationESKF(double q_noise, double r_noise, double dt) : _dt(dt) {
        _q_nominal = Eigen::Quaterniond::Identity();
        _angular_vel = Eigen::Vector3d::Zero();
        _P = Eigen::Matrix3d::Identity() * 0.1;
        _Q = Eigen::Matrix3d::Identity() * q_noise;
        _R = Eigen::Matrix3d::Identity() * r_noise;
    }

    void init(const tf2::Quaternion& q) {
        _q_nominal = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    tf2::Quaternion update(const tf2::Quaternion& z_tf) {
        Eigen::Quaterniond z(z_tf.w(), z_tf.x(), z_tf.y(), z_tf.z());
        if (_q_nominal.coeffs().dot(z.coeffs()) < 0.0) z.coeffs() *= -1.0;
        
        _P += _Q; // 预测
        Eigen::Quaterniond dq = _q_nominal.inverse() * z;
        dq.normalize();
        Eigen::AngleAxisd aa(dq);
        Eigen::Vector3d z_err = aa.angle() * aa.axis();
        
        Eigen::Matrix3d S = _P + _R;
        Eigen::Matrix3d K = _P * S.inverse();
        Eigen::Vector3d dx = K * z_err;
        _P = (Eigen::Matrix3d::Identity() - K) * _P;

        _angular_vel = dx / _dt; // 估计角速度

        if (dx.norm() > 1e-9) {
            Eigen::Quaterniond dq_corr(Eigen::AngleAxisd(dx.norm(), dx.normalized()));
            _q_nominal = (_q_nominal * dq_corr).normalized();
        }
        return tf2::Quaternion(_q_nominal.x(), _q_nominal.y(), _q_nominal.z(), _q_nominal.w());
    }

    Eigen::Vector3d getAngVel() const { return _angular_vel; }

private:
    Eigen::Quaterniond _q_nominal;
    Eigen::Vector3d _angular_vel;
    Eigen::Matrix3d _P, _Q, _R;
    double _dt;
};

// ==========================================
// VRPNListener 成员函数实现
// ==========================================

VRPNListener::VRPNListener(std::string name) : Node(name) {
    loadParams();
    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // 已移除宇树机器人的初始化和力矩传感器订阅

    std::string host = _server + ":" + std::to_string(_port);
    _vrpn_connection = std::shared_ptr<vrpn_Connection>(vrpn_get_connection_by_name(host.c_str()));

    _mainloop_timer = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(1e9 / _mainloop_frequency)), 
        std::bind(&VRPNListener::mainloop, this));
    
    _trackers_refresh_timer = this->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(1e9 / _refresh_trackers_frequency)), 
        std::bind(&VRPNListener::refresh_trackers, this));

    RCLCPP_INFO(this->get_logger(), "VRPN node initialized. Align: NewX=RawZ, NewY=RawX, NewZ=RawY");
}

void VRPN_CALLBACK VRPNListener::handlePose(void *userData, const vrpn_TRACKERCB poseData) {
    Synchronizer *sync = static_cast<Synchronizer *>(userData);
    VRPNListener *listener = static_cast<VRPNListener *>(sync->listener_ptr);
    double dt = 1.0 / listener->_mainloop_frequency;

    // 1. 坐标系对齐 (相似变换)
    tf2::Vector3 p_raw(poseData.pos[0], poseData.pos[1], poseData.pos[2]);
    tf2::Quaternion q_raw(poseData.quat[0], poseData.quat[1], poseData.quat[2], poseData.quat[3]);
    tf2::Matrix3x3 R_align(0, 0, 1,
                           1, 0, 0,
                           0, 1, 0);

    tf2::Vector3 p_final = R_align * p_raw;
    tf2::Matrix3x3 m_final = R_align * tf2::Matrix3x3(q_raw) * R_align.transpose();
    tf2::Quaternion q_final;
    m_final.getRotation(q_final); 
    q_final.normalize();

    // ---  混合卡尔曼滤波 (CV for Pos, ESKF for Ori) ---

    // 2. 初始化位置滤波器 (3路)
    if (sync->kfs.empty()) {
        for (int i = 0; i < 3; ++i) {
            auto kf = std::make_shared<ConstantVelocityKF>(dt, 0.001, 0.01);
            kf->init(i == 0 ? p_final.x() : (i == 1 ? p_final.y() : p_final.z()));
            sync->kfs.push_back(kf);
        }
    }
    // 初始化姿态 ESKF (1路)
    if (!sync->eskf_orient) {
        sync->eskf_orient = std::make_shared<OrientationESKF>(0.0001, 0.01, dt);
        sync->eskf_orient->init(q_final);
    }

    double f_pos[3], f_vel[3];
    f_pos[0] = sync->kfs[0]->update(p_final.x()); f_vel[0] = sync->kfs[0]->getVel();
    f_pos[1] = sync->kfs[1]->update(p_final.y()); f_vel[1] = sync->kfs[1]->getVel();
    f_pos[2] = sync->kfs[2]->update(p_final.z()); f_vel[2] = sync->kfs[2]->getVel();

    tf2::Quaternion q_filt = sync->eskf_orient->update(q_final);
    Eigen::Vector3d a_vel = sync->eskf_orient->getAngVel();

    // --- [核心计算：提取欧拉角] ---
    double f_roll, f_pitch, f_yaw;
    tf2::Matrix3x3(q_filt).getRPY(f_roll, f_pitch, f_yaw);

    // 3. 发布数据
    auto now = listener->get_clock()->now();
    
    // 发布 Pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now; pose_msg.header.frame_id = listener->_frame_id;
    pose_msg.pose.position.x = f_pos[0]; 
    pose_msg.pose.position.y = f_pos[1]; 
    pose_msg.pose.position.z = f_pos[2];
    pose_msg.pose.orientation = tf2::toMsg(q_filt);
    sync->pose_publisher->publish(pose_msg);

    // 发布 Twist
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now; 
    twist_msg.header.frame_id = listener->_frame_id;
    twist_msg.twist.linear.x = f_vel[0]; 
    twist_msg.twist.linear.y = f_vel[1];
    twist_msg.twist.linear.z = f_vel[2];
    twist_msg.twist.angular.x = a_vel.x(); 
    twist_msg.twist.angular.y = a_vel.y(); 
    twist_msg.twist.angular.z = a_vel.z();
    sync->twist_publisher->publish(twist_msg);

    // --- [发布欧拉角消息] ---
    geometry_msgs::msg::Vector3Stamped euler_msg;
    euler_msg.header.stamp = now;
    euler_msg.header.frame_id = listener->_frame_id;
    euler_msg.vector.x = f_roll;  
    euler_msg.vector.y = f_pitch; 
    euler_msg.vector.z = f_yaw;   
    sync->euler_publisher->publish(euler_msg);

    // TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = pose_msg.header; 
    tf_msg.child_frame_id = sync->sender_name + "/base_link";
    tf_msg.transform.translation.x = f_pos[0]; 
    tf_msg.transform.translation.y = f_pos[1]; 
    tf_msg.transform.translation.z = f_pos[2];
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    listener->_tf_broadcaster->sendTransform(tf_msg);

    // 4. 对比打印
    double r_r, r_p, r_y, f_r, f_p, f_y;
    tf2::Matrix3x3(q_final).getRPY(r_r, r_p, r_y);
    tf2::Matrix3x3(q_filt).getRPY(f_r, f_p, f_y);

    RCLCPP_INFO_THROTTLE(listener->get_logger(), *listener->get_clock(), 500,
        "\n--- [%s] Filtered ---\n"
        "Pos Raw: [%.3f, %.3f, %.3f] | Filt: [%.3f, %.3f, %.3f]\n"
        "RPY Raw: [%.1f, %.1f, %.1f] | Filt: [%.1f, %.1f, %.1f] (deg)\n"
        "Vel Lin: [%.2f, %.2f, %.2f] | Ang: [%.2f, %.2f, %.2f] (rad/s)",
        sync->sender_name.c_str(), p_final.x(), p_final.y(), p_final.z(), f_pos[0], f_pos[1], f_pos[2],
        r_r*57.3, r_p*57.3, r_y*57.3, f_r*57.3, f_p*57.3, f_y*57.3,
        f_vel[0], f_vel[1], f_vel[2], a_vel.x(), a_vel.y(), a_vel.z());
}

// 已移除 void VRPNListener::setupWrenchSubscriber() 函数体

void VRPNListener::createSynchronizer(std::string name) {
    std::string topic_name = name; replaceSpace(topic_name);
    auto sync = std::make_shared<Synchronizer>();
    sync->sender_name = name; sync->listener_ptr = this;
    sync->vrpn_tracker = std::make_shared<vrpn_Tracker_Remote>(name.c_str(), _vrpn_connection.get());
    sync->pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vrpn/" + topic_name + "/pose", 1);
    sync->twist_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/vrpn/" + topic_name + "/twist", 1);
    sync->euler_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/vrpn/" + topic_name + "/euler", 1);
    sync->vrpn_tracker->register_change_handler(sync.get(), &VRPNListener::handlePose);

    this->create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / _tracker_mainloop_frequency)),
        [sync]() { sync->vrpn_tracker->mainloop(); });
    _synchronizers[name] = sync;
}

void VRPNListener::refresh_trackers() {
    for (int i = 0; _vrpn_connection->sender_name(i) != NULL; i++) {
        std::string name = _vrpn_connection->sender_name(i);
        if (_synchronizers.count(name) == 0) createSynchronizer(name);
    }
}

void VRPNListener::mainloop() { _vrpn_connection->mainloop(); }

template <class T> void VRPNListener::loadParam(std::string n, T dv, T &p) {
    this->declare_parameter(n, dv); this->get_parameter(n, p);
}

void VRPNListener::loadParams() {
    loadParam(std::string("server"), std::string(""), _server);
    loadParam(std::string("port"), 3883, _port);
    loadParam(std::string("frame_id"), std::string("world"), _frame_id);
    loadParam(std::string("mainloop_frequency"), 100.0, _mainloop_frequency);
    loadParam(std::string("refresh_trackers_frequency"), 1.0, _refresh_trackers_frequency);
    loadParam(std::string("tracker_mainloop_frequency"), 100.0, _tracker_mainloop_frequency);
}

void VRPNListener::replaceSpace(std::string &s) { for (auto &c : s) if (c == ' ') c = '_'; }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VRPNListener>("vrpn_listener"));
    rclcpp::shutdown();
    return 0;
}