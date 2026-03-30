/************************************************************
 * rover_enable_server.cpp
 * Handles the rover_enable service (toggle arm/disarm).
 * Publishes enable state to /rover_enable_state topic
 * so the CAN node can read it.
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "multipurpose_rover/srv/rover_control.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;
using RoverControl = multipurpose_rover::srv::RoverControl;

class RoverEnableServer : public rclcpp::Node {
public:
    RoverEnableServer() : Node("rover_enable_server") {

        enable_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
            "rover_enable_state", 10
        );

        service_ = this->create_service<RoverControl>(
            "rover_enable",
            std::bind(&RoverEnableServer::enableCallback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "rover_enable_server ready.");
    }

private:
    void enableCallback(
        const RoverControl::Request::SharedPtr req,
        const RoverControl::Response::SharedPtr res)
    {
        auto msg  = std_msgs::msg::UInt8();
        msg.data  = req->enable;
        enable_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "[ENABLE] %s", req->enable ? "ARMED" : "DISARMED");
        res->success = true;
    }

    rclcpp::Service<RoverControl>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr enable_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverEnableServer>());
    rclcpp::shutdown();
    return 0;
}
