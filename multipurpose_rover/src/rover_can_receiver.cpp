/************************************************************
 * rover_can_receiver.cpp
 * Reads CAN frames from can0 in a dedicated thread (blocking)
 * and publishes encoder data to /encoder_raw at consistent rate.
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
#include <cstring>
#include <thread>
#include <atomic>

#define CAN_ID_ENCODER  0x201
#define CAN_INTERFACE   "can0"

class RoverCANReceiver : public rclcpp::Node {
public:
    RoverCANReceiver() : Node("rover_can_receiver"), can_fd_(-1), running_(true) {

        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "encoder_raw", 10
        );

        if (!initCAN()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", CAN_INTERFACE);
            return;
        }

        // Dedicated blocking read thread — never misses a frame
        can_thread_ = std::thread(&RoverCANReceiver::canReadLoop, this);

        RCLCPP_INFO(this->get_logger(), "rover_can_receiver ready on %s", CAN_INTERFACE);
    }

    ~RoverCANReceiver() {
        running_ = false;
        if (can_fd_ >= 0) {
            shutdown(can_fd_, SHUT_RDWR);
            close(can_fd_);
        }
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
            // Blocking read — wakes immediately when a frame arrives
            ssize_t n = read(can_fd_, &frame, sizeof(frame));
            if (n < (ssize_t)sizeof(frame)) continue;

            if (frame.can_id == CAN_ID_ENCODER && frame.can_dlc == 8) {
                // ECU sends int16 per encoder (2 bytes each)
                int32_t m1 = (int32_t)(int16_t)((frame.data[0] << 8) | frame.data[1]);
                int32_t m4 = (int32_t)(int16_t)((frame.data[2] << 8) | frame.data[3]);

                std_msgs::msg::Int32MultiArray msg;
                msg.data = {m1, m4};
                encoder_pub_->publish(msg);
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    std::thread can_thread_;
    int can_fd_;
    std::atomic<bool> running_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverCANReceiver>());
    rclcpp::shutdown();
    return 0;
}
