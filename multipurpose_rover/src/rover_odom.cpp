/************************************************************
 * rover_odom.cpp
 *
 * Subscribes to /encoder_raw (CAN 0x201 data from Pi CAN reader)
 * Computes differential drive odometry
 * Publishes:
 *   /odom  → nav_msgs/msg/Odometry   (for SLAM)
 *   /tf    → odom → base_link transform (for SLAM)
 *
 * Rover physical params — update these to match your rover:
 *   WHEEL_RADIUS    : metres
 *   WHEEL_BASE      : metres (distance between left and right wheel)
 *   TICKS_PER_REV   : encoder ticks per full wheel revolution
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// ---- Rover physical parameters ---- //
// Wheel diameter = 8cm → radius = 0.04m
// Circumference = 2π × 0.04 = 0.2513m
// Ticks per rev = 70
// Dist per tick = 0.2513 / 70 = 0.003590m
// Ticks for 1m  = 1 / 0.003590 ≈ 279 ticks
constexpr double WHEEL_RADIUS  = 0.04;   // 8cm diameter → 4cm radius
constexpr double WHEEL_BASE    = 0.30;   // metres — measure left to right wheel centre
constexpr double TICKS_PER_REV = 70.0;  // ticks per full wheel revolution

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
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // Publish static TF: base_link → laser
        // Adjust x/y/z offset to match where the RPLIDAR is mounted on your rover
        publishLaserTF();

        // Publish odom→base_link TF at 20Hz regardless of encoder data
        // This ensures slam_toolbox always has a valid TF even when rover is stationary
        tf_timer_ = this->create_wall_timer(
            50ms, std::bind(&RoverOdom::publishTF, this)
        );

        RCLCPP_INFO(this->get_logger(), "rover_odom ready");
    }

private:
    void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        int32_t ticks_right = -msg->data[0];  // M1 rear right — negated (encoder mounted inverted)
        int32_t ticks_left  =  msg->data[1];  // M4 rear left — raw

        if (!initialized_) {
            last_ticks_right_ = ticks_right;
            last_ticks_left_  = ticks_left;
            last_time_ = this->now();
            initialized_ = true;
            return;
        }

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0) return;
        last_time_ = now;

        // Delta ticks since last callback
        int32_t d_right = ticks_right - last_ticks_right_;
        int32_t d_left  = ticks_left  - last_ticks_left_;
        last_ticks_right_ = ticks_right;
        last_ticks_left_  = ticks_left;

        // Publish d_right and d_left for debugging
        std_msgs::msg::Int32MultiArray dbg;
        dbg.data = {d_right, d_left};
        debug_pub_->publish(dbg);

        // Distance travelled by each wheel
        double dist_right = d_right * DIST_PER_TICK;
        double dist_left  = d_left  * DIST_PER_TICK;

        // Differential drive odometry
        double dist   = (dist_right + dist_left) / 2.0;
        double d_theta = (dist_right - dist_left) / WHEEL_BASE;

        // Update pose
        theta_ += d_theta;
        x_ += dist * std::cos(theta_);
        y_ += dist * std::sin(theta_);

        // ---- Publish /odom ----
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // Velocity using actual dt
        odom.twist.twist.linear.x  = dist    / dt;
        odom.twist.twist.angular.z = d_theta / dt;

        // Covariance — diagonal, tuned conservatively
        odom.pose.covariance[0]  = 0.01;   // x
        odom.pose.covariance[7]  = 0.01;   // y
        odom.pose.covariance[35] = 0.05;   // yaw
        odom.twist.covariance[0]  = 0.01;
        odom.twist.covariance[35] = 0.05;

        odom_pub_->publish(odom);
    }

    void publishTF() {
        auto now = this->now();
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id  = "base_link";

        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
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

        // Adjust these offsets to match your RPLIDAR mounting position
        laser_tf.transform.translation.x = 0.0;   // metres forward from base_link
        laser_tf.transform.translation.y = 0.0;   // metres left from base_link
        laser_tf.transform.translation.z = 0.1;   // metres up from base_link

        // No rotation — lidar faces forward same as robot
        laser_tf.transform.rotation.x = 0.0;
        laser_tf.transform.rotation.y = 0.0;
        laser_tf.transform.rotation.z = 0.0;
        laser_tf.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(laser_tf);
    }

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    double x_, y_, theta_;
    int32_t last_ticks_right_, last_ticks_left_;
    rclcpp::Time last_time_;
    bool initialized_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverOdom>());
    rclcpp::shutdown();
    return 0;
}
