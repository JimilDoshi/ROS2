/************************************************************
 * rover_can_receiver.cpp
 * Reads CAN frames from can0 and publishes encoder data.
 *
 * CAN ID 0x201 → /encoder_raw (Int32MultiArray)
 *   data[0] = M1 ticks (rear right, int32)
 *   data[1] = M4 ticks (rear left,  int32)
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>

using namespace std::chrono_literals;

#define CAN_ID_ENCODER  0x201
#define CAN_INTERFACE   "can0"

class RoverCANReceiver : public rclcpp::Node {
public:
    RoverCANReceiver() : Node("rover_can_receiver"), can_fd_(-1) {

        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "encoder_raw", 10
        );

        if (!initCAN()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", CAN_INTERFACE);
            return;
        }

        // Poll CAN socket at 50Hz
        timer_ = this->create_wall_timer(
            20ms, std::bind(&RoverCANReceiver::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "rover_can_receiver ready on %s", CAN_INTERFACE);
    }

    ~RoverCANReceiver() {
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

        // Non-blocking reads
        fcntl(can_fd_, F_SETFL, O_NONBLOCK);
        return true;
    }

    void timerCallback() {
        struct can_frame frame;
        ssize_t n;

        // Drain all available frames
        while ((n = read(can_fd_, &frame, sizeof(frame))) > 0) {
            if (frame.can_id == CAN_ID_ENCODER && frame.can_dlc == 8) {
                int32_t m1 = ((int32_t)frame.data[0] << 24) |
                             ((int32_t)frame.data[1] << 16) |
                             ((int32_t)frame.data[2] <<  8) |
                              (int32_t)frame.data[3];

                int32_t m4 = ((int32_t)frame.data[4] << 24) |
                             ((int32_t)frame.data[5] << 16) |
                             ((int32_t)frame.data[6] <<  8) |
                              (int32_t)frame.data[7];

                std_msgs::msg::Int32MultiArray msg;
                msg.data = {m1, m4};
                encoder_pub_->publish(msg);
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int can_fd_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCANReceiver>());
    rclcpp::shutdown();
    return 0;
}
