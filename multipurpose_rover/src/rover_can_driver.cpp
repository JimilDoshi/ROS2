/************************************************************
 * rover_can_driver.cpp
 * Subscribes to /rover_cmd (direction) and /rover_enable_state
 * (enable toggle), combines both and sends CAN frames to
 * Motor ECU at 50Hz.
 *
 * CAN ID : 0x101
 *   byte[0..1] = y      (steering,  int16_t big-endian)
 *   byte[2..3] = x      (throttle,  int16_t big-endian)
 *   byte[4]    = speed
 *   byte[5]    = enable
 *   byte[6]    = mode
 *   byte[7]    = 0x00
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "multipurpose_rover/msg/rover_cmd.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;
using RoverCmd = multipurpose_rover::msg::RoverCmd;

#define CAN_ID_RECEIVER_DATA 0x101
#define CAN_INTERFACE        "can0"

class RoverCANDriver : public rclcpp::Node {
public:
    RoverCANDriver() : Node("rover_can_driver"), can_fd_(-1) {

        if (!initCAN()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open SocketCAN on %s", CAN_INTERFACE);
        }

        // QoS: best effort, depth 1 — always use latest, never queue stale msgs
        auto qos = rclcpp::QoS(1).best_effort();

        cmd_sub_ = this->create_subscription<RoverCmd>(
            "rover_cmd", qos,
            std::bind(&RoverCANDriver::cmdCallback, this, std::placeholders::_1)
        );

        enable_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "rover_enable_state", 10,
            std::bind(&RoverCANDriver::enableCallback, this, std::placeholders::_1)
        );

        // Send CAN frames at 50Hz
        timer_ = this->create_wall_timer(
            20ms, std::bind(&RoverCANDriver::timerCallback, this)
        );

        last_cmd_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "rover_can_driver ready on %s", CAN_INTERFACE);
    }

    ~RoverCANDriver() {
        if (can_fd_ >= 0) close(can_fd_);
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

    void sendCANFrame() {
        if (can_fd_ < 0) return;

        struct can_frame frame{};
        frame.can_id  = CAN_ID_RECEIVER_DATA;
        frame.can_dlc = 8;

        frame.data[0] = (y_ >> 8) & 0xFF;
        frame.data[1] =  y_       & 0xFF;
        frame.data[2] = (x_ >> 8) & 0xFF;
        frame.data[3] =  x_       & 0xFF;
        frame.data[4] = speed_;
        frame.data[5] = enable_;
        frame.data[6] = mode_;
        frame.data[7] = 0x00;

        write(can_fd_, &frame, sizeof(frame));
    }

    void cmdCallback(const RoverCmd::SharedPtr msg) {
        last_cmd_time_ = this->now();
        x_     = msg->x;
        y_     = msg->y;
        speed_ = msg->speed;
        mode_  = msg->mode;
    }

    void enableCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
        enable_ = msg->data;
    }

    void timerCallback() {
        // Safety watchdog: no /rover_cmd for 500ms → force stop
        double elapsed = (this->now() - last_cmd_time_).seconds();
        if (elapsed > 0.5 && enable_ == 1) {
            enable_ = 0;
            x_ = 0; y_ = 0;
            RCLCPP_WARN(this->get_logger(),
                "STOP — no /rover_cmd for %.2fs", elapsed);
        }

        sendCANFrame();

        RCLCPP_DEBUG(this->get_logger(),
            "x=%d  y=%d  speed=%d  enable=%d  mode=%d",
            x_, y_, speed_, enable_, mode_);
    }

    rclcpp::Subscription<RoverCmd>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr enable_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_cmd_time_;
    int can_fd_;

    int16_t x_      = 0;
    int16_t y_      = 0;
    uint8_t speed_  = 50;
    uint8_t enable_ = 0;
    uint8_t mode_   = 1;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCANDriver>());
    rclcpp::shutdown();
    return 0;
}
