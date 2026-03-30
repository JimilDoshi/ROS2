/************************************************************
 * rover_control_server.cpp
 *
 * Topic   /rover_cmd    → receives x, y, speed, mode at 50Hz
 * Service /rover_enable → receives enable toggle (0 or 1)
 *
 * Combines both and sends CAN frame to Motor ECU at 50Hz.
 * Safety: if no topic msg for 500ms → force enable=0
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "multipurpose_rover/srv/rover_control.hpp"
#include "multipurpose_rover/msg/rover_cmd.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;
using RoverControl = multipurpose_rover::srv::RoverControl;
using RoverCmd     = multipurpose_rover::msg::RoverCmd;

#define CAN_ID_RECEIVER_DATA 0x101
#define CAN_INTERFACE        "can0"

class RoverControlServer : public rclcpp::Node {
public:
    RoverControlServer() : Node("rover_control_server"), can_fd_(-1) {

        if (!initCAN()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open SocketCAN on %s", CAN_INTERFACE);
        }

        // Subscribe to direction topic
        subscription_ = this->create_subscription<RoverCmd>(
            "rover_cmd", 10,
            std::bind(&RoverControlServer::cmdCallback, this, std::placeholders::_1)
        );

        // Service for enable toggle only
        service_ = this->create_service<RoverControl>(
            "rover_enable",
            std::bind(&RoverControlServer::enableCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // 50Hz CAN publish timer
        publish_timer_ = this->create_wall_timer(
            20ms, std::bind(&RoverControlServer::publishTimerCallback, this)
        );

        last_cmd_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "rover_control_server ready.");
    }

    ~RoverControlServer() {
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

    void enableCallback(
        const RoverControl::Request::SharedPtr req,
        const RoverControl::Response::SharedPtr res)
    {
        enable_ = req->enable;
        RCLCPP_INFO(this->get_logger(), "[ENABLE] %s", enable_ ? "ARMED" : "DISARMED");
        res->success = true;
    }

    void publishTimerCallback() {
        // Safety watchdog: no topic msg for 500ms → force stop
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

    rclcpp::Subscription<RoverCmd>::SharedPtr subscription_;
    rclcpp::Service<RoverControl>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
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
    rclcpp::spin(std::make_shared<RoverControlServer>());
    rclcpp::shutdown();
    return 0;
}
