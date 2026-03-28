/************************************************************
 * cmd_vel_to_ecu.cpp
 * Subscribes to /cmd_vel, converts to motor ECU CAN frame,
 * and sends over SocketCAN (e.g. can0 on Raspberry Pi).
 *
 * CAN ID : 0x101  (CAN_ID_RECEIVER_DATA in Motor_Control_Kiro.ino)
 * Frame  : 8 bytes, 500 kbps
 *   byte[0..1] = y      (steering,  int16_t big-endian) ← angular.z scaled
 *   byte[2..3] = x      (throttle,  int16_t big-endian) ← linear.x scaled
 *   byte[4]    = speed  (uint8_t, constant = 50)
 *   byte[5]    = enable (uint8_t, 1=run / 0=stop)
 *   byte[6]    = mode   (uint8_t, 0=eco 1=normal 2=sport)
 *   byte[7]    = 0x00   (reserved)
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Linux SocketCAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

#define CAN_ID_RECEIVER_DATA 0x101
#define CAN_INTERFACE        "can0"

// ECU expects linear.x  in [-1.0, 1.0] m/s  → [-100, 100]
// ECU expects angular.z in [-1.0, 1.0] rad/s → [-100, 100]
#define MAX_LINEAR  1.0
#define MAX_ANGULAR 1.0

class CmdVelToECU : public rclcpp::Node {
public:
    CmdVelToECU() : Node("cmd_vel_to_ecu"), can_fd_(-1) {

        // Constants matching ECU expectations
        speed_  = 50;
        enable_ = 1;
        mode_   = 1;   // 0=eco, 1=normal, 2=sport

        if (!initCAN()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open SocketCAN on %s. Check interface is up.", CAN_INTERFACE);
        }

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToECU::cmdVelCallback, this, std::placeholders::_1)
        );

        // Watchdog: checks every 100ms if last msg > 0.5s ago
        watchdog_timer_ = this->create_wall_timer(
            100ms, std::bind(&CmdVelToECU::watchdogCallback, this)
        );

        last_msg_time_ = this->now();
        RCLCPP_INFO(this->get_logger(),
            "cmd_vel_to_ecu ready — sending CAN frames to %s @ 0x%X",
            CAN_INTERFACE, CAN_ID_RECEIVER_DATA);
    }

    ~CmdVelToECU() {
        if (can_fd_ >= 0) close(can_fd_);
    }

private:

    bool initCAN() {
        can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd_ < 0) return false;

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ);
        if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
            close(can_fd_);
            can_fd_ = -1;
            return false;
        }

        struct sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(can_fd_);
            can_fd_ = -1;
            return false;
        }
        return true;
    }

    int16_t scaleAndClamp(double value, double max_input) {
        if (max_input == 0.0) return 0;
        double scaled = (value / max_input) * 100.0;
        scaled = std::max(-100.0, std::min(100.0, scaled));
        return static_cast<int16_t>(scaled);
    }

    void sendCANFrame() {
        if (can_fd_ < 0) return;

        struct can_frame frame{};
        frame.can_id  = CAN_ID_RECEIVER_DATA;
        frame.can_dlc = 8;

        // y (steering) → bytes [0..1] big-endian
        frame.data[0] = (y_ >> 8) & 0xFF;
        frame.data[1] =  y_       & 0xFF;

        // x (throttle) → bytes [2..3] big-endian
        frame.data[2] = (x_ >> 8) & 0xFF;
        frame.data[3] =  x_       & 0xFF;

        frame.data[4] = static_cast<uint8_t>(speed_);
        frame.data[5] = static_cast<uint8_t>(enable_);
        frame.data[6] = static_cast<uint8_t>(mode_);
        frame.data[7] = 0x00;

        write(can_fd_, &frame, sizeof(frame));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_msg_time_ = this->now();
        enable_ = 1;

        x_ = scaleAndClamp(msg->linear.x,  MAX_LINEAR);   // throttle
        y_ = scaleAndClamp(msg->angular.z, MAX_ANGULAR);  // steering

        sendCANFrame();

        RCLCPP_INFO(this->get_logger(),
            "x(throttle)=%d  y(steering)=%d  speed=%d  enable=%d  mode=%d",
            x_, y_, speed_, enable_, mode_);
    }

    void watchdogCallback() {
        double elapsed = (this->now() - last_msg_time_).seconds();
        if (elapsed > 0.5 && enable_ == 1) {
            enable_ = 0;
            x_ = 0;
            y_ = 0;
            sendCANFrame();  // send stop frame to ECU
            RCLCPP_WARN(this->get_logger(),
                "STOP — no /cmd_vel for %.2fs  enable=0 sent to ECU", elapsed);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_msg_time_;

    int can_fd_;

    int16_t x_      = 0;
    int16_t y_      = 0;
    int16_t speed_  = 50;
    int16_t enable_ = 1;
    int16_t mode_   = 1;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToECU>());
    rclcpp::shutdown();
    return 0;
}
