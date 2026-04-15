/************************************************************
 * rover_odom.cpp
 * Differential drive odometry from encoder ticks.
 * Publishes /odom and TF odom→base_link for SLAM.
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

constexpr double WHEEL_RADIUS  = 0.065;
constexpr double WHEEL_BASE    = 0.32;
constexpr double TICKS_PER_REV = 140.0;  // 70 physical ticks × 2 (half-quad encoding counts both edges)
constexpr double DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

class RoverOdom : public rclcpp::Node {
public:
    RoverOdom() : Node("rover_odom"),
        x_(0.0), y_(0.0), theta_(0.0),
        last_ticks_right_(0), last_ticks_left_(0),
        initialized_(false)
    {
        sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "encoder_raw", 10,
            std::bind(&RoverOdom::encoderCallback, this, std::placeholders::_1)
        );

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        debug_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("odom_debug", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // odom→base_link TF now published by EKF node
        // Keep timer running but only publish laser TF
        tf_timer_ = this->create_wall_timer(
            50ms, std::bind(&RoverOdom::publishLaserTF, this)
        );

        // base_link→laser refreshed by tf_timer above

        RCLCPP_INFO(this->get_logger(), "rover_odom ready");
    }

private:
    void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        int32_t ticks_right = -msg->data[0];  // M1 rear right — negated (inverted mounting)
        int32_t ticks_left  =  msg->data[1];  // M4 rear left

        if (!initialized_) {
            last_ticks_right_ = ticks_right;
            last_ticks_left_  = ticks_left;
            last_time_        = this->now();
            initialized_      = true;
            return;
        }

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = now;

        int32_t d_right = ticks_right - last_ticks_right_;
        int32_t d_left  = ticks_left  - last_ticks_left_;
        last_ticks_right_ = ticks_right;
        last_ticks_left_  = ticks_left;

        // Debug topic
        std_msgs::msg::Int32MultiArray dbg;
        dbg.data = {d_right, d_left};
        debug_pub_->publish(dbg);

        double dist_right = d_right * DIST_PER_TICK;
        double dist_left  = d_left  * DIST_PER_TICK;

        double dist    = (dist_right + dist_left) / 2.0;
        double d_theta = (dist_right - dist_left) / WHEEL_BASE;

        theta_ += d_theta;
        x_ += dist * std::cos(theta_);
        y_ += dist * std::sin(theta_);

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        odom.pose.pose.position.x    = x_;
        odom.pose.pose.position.y    = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x    = dist    / dt;
        odom.twist.twist.angular.z   = d_theta / dt;
        odom.pose.covariance[0]  = 0.01;   // x
        odom.pose.covariance[7]  = 0.01;   // y
        odom.pose.covariance[35] = 0.5;    // yaw — high uncertainty, let gyro override
        odom.twist.covariance[0]  = 0.01;
        odom.twist.covariance[35] = 0.5;   // yaw rate — high uncertainty

        odom_pub_->publish(odom);
    }

    void publishTF() {
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = this->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf);
    }

    void publishLaserTF() {
        geometry_msgs::msg::TransformStamped laser_tf;
        laser_tf.header.stamp    = this->now();
        laser_tf.header.frame_id = "base_link";
        laser_tf.child_frame_id  = "laser";
        laser_tf.transform.translation.x = 0.32;
        laser_tf.transform.translation.y = 0.0;
        laser_tf.transform.translation.z = 0.16;
        laser_tf.transform.rotation.w    = 1.0;

        tf_broadcaster_->sendTransform(laser_tf);
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr laser_tf_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
    int32_t last_ticks_right_ = 0, last_ticks_left_ = 0;
    rclcpp::Time last_time_;
    bool initialized_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverOdom>());
    rclcpp::shutdown();
    return 0;
}
