/************************************************************
 * rover_enable_client.cpp
 * Sends enable toggle to rover_enable service.
 * Press E to arm, press E again to disarm. Q to quit.
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "multipurpose_rover/srv/rover_control.hpp"

#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <chrono>

using namespace std::chrono_literals;
using RoverControl = multipurpose_rover::srv::RoverControl;

struct TermRAII {
    termios old_tio;
    TermRAII() {
        tcgetattr(STDIN_FILENO, &old_tio);
        termios raw = old_tio;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    ~TermRAII() { tcsetattr(STDIN_FILENO, TCSANOW, &old_tio); }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node   = rclcpp::Node::make_shared("rover_enable_client");
    auto client = node->create_client<RoverControl>("rover_enable");

    printf("\n=== Rover Enable Client ===\n");
    printf("  E : toggle enable (arm/disarm)\n");
    printf("  Q : quit\n\n");

    while (!client->wait_for_service(1s)) {
        printf("Waiting for rover_enable service...\n");
        if (!rclcpp::ok()) return 0;
    }

    TermRAII term;

    bool enabled = false;
    bool running = true;

    rclcpp::Rate rate(50);

    while (rclcpp::ok() && running) {
        char key = 0;
        read(STDIN_FILENO, &key, 1);

        if (key == 'e' || key == 'E') {
            enabled = !enabled;
            auto req   = std::make_shared<RoverControl::Request>();
            req->enable = enabled ? 1 : 0;
            client->async_send_request(req);
            printf("\n[ENABLE] %s\n", enabled ? "ARMED" : "DISARMED");
            fflush(stdout);
        } else if (key == 'q' || key == 'Q') {
            running = false;
        }

        rate.sleep();
    }

    // Disarm on exit
    auto req   = std::make_shared<RoverControl::Request>();
    req->enable = 0;
    client->async_send_request(req);
    printf("\nExiting.\n");

    rclcpp::shutdown();
    return 0;
}
