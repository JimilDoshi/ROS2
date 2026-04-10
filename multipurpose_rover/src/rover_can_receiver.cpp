/************************************************************
 * rover_can_receiver.cpp
 * Reads CAN frames from can0 and publishes:
 *   CAN 0x201 → /encoder_raw  (Int32MultiArray)
 *   CAN 0x200 → /imu/data_raw (sensor_msgs/Imu) for EKF
 *
 * Angular velocity is derived from lateral acceleration:
 *   omega = ay / vx  (centripetal: ay = omega * vx)
 * This gives EKF a yaw rate signal during turns.
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <cmath>

#define CAN_ID_ENCODER  0x201
#define CAN_ID_FEEDBACK 0x200
#define CAN_INTERFACE   "can0"

// Additional low-pass filter on Pi side for extra noise rejection
#define LPF_ALPHA 0.3f

class RoverCANReceiver : public rclcpp::Node {
public:
    RoverCANReceiver() : Node("rover_can_receiver"), can_fd_(-1), running_(true),
        lpf_ax_(0.0f), lpf_ay_(0.0f), lpf_az_(0.0f), vx_(0.0f)
    {
        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "encoder_raw", 10
        );
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_raw", 10
        );

        // Subscribe to odom to get current linear velocity for omega estimation
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                vx_ = msg->twist.twist.linear.x;
            }
        );

        if (!initCAN()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", CAN_INTERFACE);
            return;
        }

        can_thread_ = std::thread(&RoverCANReceiver::canReadLoop, this);
        RCLCPP_INFO(this->get_logger(), "rover_can_receiver ready on %s", CAN_INTERFACE);
    }

    ~RoverCANReceiver() {
        running_ = false;
        if (can_fd_ >= 0) { shutdown(can_fd_, SHUT_RDWR); close(can_fd_); }
        if (can_thread_.joinable()) can_thread_.join();
    }

private:
    bool initCAN() {
        can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd_ < 0) return false;

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ);
        if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
            close(can_fd_); can_fd_ = -1; return false;
        }

        struct sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(can_fd_); can_fd_ = -1; return false;
        }
        return true;
    }

    void canReadLoop() {
        struct can_frame frame;
        while (running_) {
            ssize_t n = read(can_fd_, &frame, sizeof(frame));
            if (n < (ssize_t)sizeof(frame)) continue;

            if (frame.can_id == CAN_ID_ENCODER && frame.can_dlc == 8) {
                int32_t m1 = (int32_t)(int16_t)((frame.data[0] << 8) | frame.data[1]);
                int32_t m4 = (int32_t)(int16_t)((frame.data[2] << 8) | frame.data[3]);
                std_msgs::msg::Int32MultiArray msg;
                msg.data = {m1, m4};
                encoder_pub_->publish(msg);
            }

            if (frame.can_id == CAN_ID_FEEDBACK && frame.can_dlc == 8) {
                float raw_ax = (int16_t)((frame.data[0] << 8) | frame.data[1]) / 100.0f;
                float raw_ay = (int16_t)((frame.data[2] << 8) | frame.data[3]) / 100.0f;
                float raw_az = (int16_t)((frame.data[4] << 8) | frame.data[5]) / 100.0f;

                // Second-stage low-pass filter on Pi
                lpf_ax_ = LPF_ALPHA * raw_ax + (1.0f - LPF_ALPHA) * lpf_ax_;
                lpf_ay_ = LPF_ALPHA * raw_ay + (1.0f - LPF_ALPHA) * lpf_ay_;
                lpf_az_ = LPF_ALPHA * raw_az + (1.0f - LPF_ALPHA) * lpf_az_;

                // Derive angular velocity from lateral acceleration
                // Centripetal: ay = omega * vx  →  omega = ay / vx
                // Only valid when moving (vx > threshold to avoid division noise)
                double omega_z = 0.0;
                double vx = vx_;
                if (std::abs(vx) > 0.05) {
                    omega_z = -lpf_ay_ / vx;  // negative: left turn = positive omega in ROS
                    // Clamp to reasonable range (±3 rad/s)
                    omega_z = std::max(-3.0, std::min(3.0, omega_z));
                }

                sensor_msgs::msg::Imu imu;
                imu.header.stamp    = this->now();
                imu.header.frame_id = "base_link";

                // Angular velocity derived from lateral accel
                imu.angular_velocity.x = 0.0;
                imu.angular_velocity.y = 0.0;
                imu.angular_velocity.z = omega_z;

                // Covariance — higher when stationary (omega derived from noisy accel)
                double omega_cov = (std::abs(vx) > 0.05) ? 0.1 : 9999.0;
                imu.angular_velocity_covariance[0] = 9999.0;
                imu.angular_velocity_covariance[4] = 9999.0;
                imu.angular_velocity_covariance[8] = omega_cov;

                // Linear acceleration
                imu.linear_acceleration.x = lpf_ax_;
                imu.linear_acceleration.y = lpf_ay_;
                imu.linear_acceleration.z = lpf_az_;
                imu.linear_acceleration_covariance[0] = 0.04;
                imu.linear_acceleration_covariance[4] = 0.04;
                imu.linear_acceleration_covariance[8] = 0.04;

                // No orientation
                imu.orientation_covariance[0] = -1.0;

                imu_pub_->publish(imu);
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::thread can_thread_;
    int can_fd_;
    std::atomic<bool> running_;

    float lpf_ax_, lpf_ay_, lpf_az_;
    std::atomic<double> vx_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCANReceiver>());
    rclcpp::shutdown();
    return 0;
}
