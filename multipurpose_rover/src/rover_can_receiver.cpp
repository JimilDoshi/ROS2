/************************************************************
 * rover_can_receiver.cpp
 * CAN 0x201 → /encoder_raw  (Int32MultiArray)
 * CAN 0x200 → accel buffer
 * CAN 0x202 → /imu/data_raw (sensor_msgs/Imu) — real gyro from MPU6050
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>

#define CAN_ID_ACCEL    0x200
#define CAN_ID_ENCODER  0x201
#define CAN_ID_GYRO     0x202
#define CAN_INTERFACE   "can0"

class RoverCANReceiver : public rclcpp::Node {
public:
    RoverCANReceiver() : Node("rover_can_receiver"), can_fd_(-1), running_(true) {

        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("encoder_raw", 10);
        imu_pub_     = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

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
        if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) { close(can_fd_); can_fd_=-1; return false; }
        struct sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(can_fd_); can_fd_=-1; return false; }
        return true;
    }

    void canReadLoop() {
        struct can_frame frame;
        while (running_) {
            ssize_t n = read(can_fd_, &frame, sizeof(frame));
            if (n < (ssize_t)sizeof(frame)) continue;

            if (frame.can_id == CAN_ID_ENCODER && frame.can_dlc == 8) {
                int32_t m1 = (int32_t)(int16_t)((frame.data[0]<<8)|frame.data[1]);
                int32_t m4 = (int32_t)(int16_t)((frame.data[2]<<8)|frame.data[3]);

                // Slip detection: if accel shows no forward motion but encoders are
                // counting, wheels are likely in the air or slipping — freeze odom
                // ax threshold 0.3 m/s² — below this = no real movement
                bool real_motion = (std::abs(ax_) > 0.3f) || (std::abs(gz_) > 0.05f);

                if (real_motion) {
                    std_msgs::msg::Int32MultiArray msg;
                    msg.data = {m1, m4};
                    encoder_pub_->publish(msg);
                } else {
                    // Publish same ticks as last time to freeze odom position
                    std_msgs::msg::Int32MultiArray msg;
                    msg.data = {last_m1_, last_m4_};
                    encoder_pub_->publish(msg);
                }
                last_m1_ = m1;
                last_m4_ = m4;
            }

            if (frame.can_id == CAN_ID_ACCEL && frame.can_dlc == 8) {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                ax_ = (int16_t)((frame.data[0]<<8)|frame.data[1]) / 100.0f;
                ay_ = (int16_t)((frame.data[2]<<8)|frame.data[3]) / 100.0f;
                az_ = (int16_t)((frame.data[4]<<8)|frame.data[5]) / 100.0f;
                accel_ready_ = true;
                publishIfReady();
            }

            if (frame.can_id == CAN_ID_GYRO && frame.can_dlc == 8) {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                gx_ = (int16_t)((frame.data[0]<<8)|frame.data[1]) / 100.0f;
                gy_ = (int16_t)((frame.data[2]<<8)|frame.data[3]) / 100.0f;
                gz_ = (int16_t)((frame.data[4]<<8)|frame.data[5]) / 100.0f;
                gyro_ready_ = true;
                publishIfReady();
            }
        }
    }

    // Publish only when both accel and gyro have been received
    void publishIfReady() {
        if (!accel_ready_ || !gyro_ready_) return;
        accel_ready_ = gyro_ready_ = false;

        sensor_msgs::msg::Imu imu;
        imu.header.stamp    = this->now();
        imu.header.frame_id = "base_link";

        // Real gyro from MPU6050 — direct yaw rate, no derivation needed
        // Dead zone — suppress gyro noise below 0.05 rad/s (stationary drift)
        constexpr float GYRO_DZ = 0.05f;
        imu.angular_velocity.x = (std::abs(gx_) > GYRO_DZ) ? gx_ : 0.0f;
        imu.angular_velocity.y = (std::abs(gy_) > GYRO_DZ) ? gy_ : 0.0f;
        imu.angular_velocity.z = (std::abs(gz_) > GYRO_DZ) ? gz_ : 0.0f;
        // MPU6050 gyro — tight covariance = EKF trusts this strongly over encoder yaw
        imu.angular_velocity_covariance[0] = 0.01;
        imu.angular_velocity_covariance[4] = 0.01;
        imu.angular_velocity_covariance[8] = 0.01;

        imu.linear_acceleration.x = ax_;
        imu.linear_acceleration.y = ay_;
        imu.linear_acceleration.z = az_;
        // MPU6050 accel noise ~0.01 m/s² RMS
        imu.linear_acceleration_covariance[0] = 0.01;
        imu.linear_acceleration_covariance[4] = 0.01;
        imu.linear_acceleration_covariance[8] = 0.01;

        // No orientation — EKF will integrate gyro for yaw
        imu.orientation_covariance[0] = -1.0;

        imu_pub_->publish(imu);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::thread can_thread_;
    std::mutex imu_mutex_;
    int can_fd_;
    std::atomic<bool> running_;

    float ax_=0, ay_=0, az_=0;
    float gx_=0, gy_=0, gz_=0;
    bool accel_ready_=false, gyro_ready_=false;
    int32_t last_m1_=0, last_m4_=0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCANReceiver>());
    rclcpp::shutdown();
    return 0;
}
