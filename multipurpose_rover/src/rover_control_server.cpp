/************************************************************
 * rover_control_server.cpp
 * Receives RoverControl service calls from the teleop client,
 * builds a CAN frame and sends it to the Motor ECU.
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
#include "multipurpose_rover/srv/rover_control.hpp"

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

#define CAN_ID_RECEIVER_DATA 0x101
#define CAN_INTERFACE        "can0"

class RoverControlServer : public rclcpp::Node {
public:
    RoverControlServer() : Node("rover_control_server"), can_fd_(-1) {

        if (!initCAN()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to open SocketCAN on %s", CAN_INTERFACE);
        }

        service_ = this->create_service<RoverControl>(
            "rover_control",
            std::bind(&RoverControlServer::handleRequest, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Safety watchdog: if no service call for 200ms → send stop frame
        watchdog_timer_ = this->create_wall_timer(
            50ms, std::bind(&RoverControlServer::watchdogCallback, this)
        );

        last_call_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "rover_control_server ready on CAN %s", CAN_INTERFACE);
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

    void sendCANFrame(int16_t x, int16_t y, uint8_t speed, uint8_t enable, uint8_t mode) {
        if (can_fd_ < 0) return;

        struct can_frame frame{};
        frame.can_id  = CAN_ID_RECEIVER_DATA;
        frame.can_dlc = 8;

        frame.data[0] = (y >> 8) & 0xFF;   // steering high
        frame.data[1] =  y       & 0xFF;   // steering low
        frame.data[2] = (x >> 8) & 0xFF;   // throttle high
        frame.data[3] =  x       & 0xFF;   // throttle low
        frame.data[4] = speed;
        frame.data[5] = enable;
        frame.data[6] = mode;
        frame.data[7] = 0x00;

        write(can_fd_, &frame, sizeof(frame));
    }

    void handleRequest(
        const RoverControl::Request::SharedPtr req,
        const RoverControl::Response::SharedPtr res)
    {
        last_call_time_ = this->now();
        last_x_      = req->x;
        last_y_      = req->y;
        last_speed_  = req->speed;
        last_enable_ = req->enable;
        last_mode_   = req->mode;

        sendCANFrame(req->x, req->y, req->speed, req->enable, req->mode);

        RCLCPP_INFO(this->get_logger(),
            "x=%d  y=%d  speed=%d  enable=%d  mode=%d",
            req->x, req->y, req->speed, req->enable, req->mode);

        res->success = true;
    }

    void watchdogCallback() {
        double elapsed = (this->now() - last_call_time_).seconds();
        // If no call received for 200ms, send stop immediately
        if (elapsed > 0.2 && last_enable_ != 0) {
            last_enable_ = 0;
            last_x_ = 0;
            last_y_ = 0;
            sendCANFrame(0, 0, last_speed_, 0, last_mode_);
            RCLCPP_WARN(this->get_logger(),
                "STOP — no teleop for %.2fs", elapsed);
        }
    }

    rclcpp::Service<RoverControl>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_call_time_;
    int can_fd_;

    int16_t last_x_      = 0;
    int16_t last_y_      = 0;
    uint8_t last_speed_  = 50;
    uint8_t last_enable_ = 0;
    uint8_t last_mode_   = 1;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverControlServer>());
    rclcpp::shutdown();
    return 0;
}
