/************************************************************
 * rover_teleop_client.cpp
 *
 * Topic  /rover_cmd  → x, y, speed, mode  (published at 50Hz)
 * Service rover_enable → enable toggle     (called on E keypress)
 *
 * Controls:
 *   W/S   → forward/backward
 *   A/D   → steer left/right
 *   E     → toggle enable (press once to arm, again to disarm)
 *   1/2/3 → eco/normal/sport mode
 *   +/-   → speed up/down
 *   Q     → quit
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "multipurpose_rover/srv/rover_control.hpp"
#include "multipurpose_rover/msg/rover_cmd.hpp"

#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <chrono>

using namespace std::chrono_literals;
using RoverControl = multipurpose_rover::srv::RoverControl;
using RoverCmd     = multipurpose_rover::msg::RoverCmd;

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
    auto node      = rclcpp::Node::make_shared("rover_teleop_client");
    auto publisher = node->create_publisher<RoverCmd>("rover_cmd", 10);
    auto client    = node->create_client<RoverControl>("rover_enable");

    printf("\n=== Rover Teleop ===\n");
    printf("  W/S   : forward/backward\n");
    printf("  A/D   : steer left/right\n");
    printf("  E     : toggle enable (arm/disarm)\n");
    printf("  1/2/3 : eco/normal/sport mode\n");
    printf("  +/-   : speed up/down\n");
    printf("  Q     : quit\n\n");

    while (!client->wait_for_service(1s)) {
        printf("Waiting for rover_enable service...\n");
        if (!rclcpp::ok()) return 0;
    }

    TermRAII term;

    int16_t x = 0, y = 0;
    uint8_t speed   = 50;
    uint8_t mode    = 1;
    bool    enabled = false;   // toggle state
    bool    running = true;

    rclcpp::Rate rate(50);

    while (rclcpp::ok() && running) {
        char key = 0;
        read(STDIN_FILENO, &key, 1);

        // Reset direction each tick — hold key to keep moving
        x = 0; y = 0;

        switch (key) {
            case 'w': case 'W': x =  100; break;
            case 's': case 'S': x = -100; break;
            case 'a': case 'A': y =  100; break;
            case 'd': case 'D': y = -100; break;

            case 'e': case 'E': {
                // Toggle enable and send service call once
                enabled = !enabled;
                auto req = std::make_shared<RoverControl::Request>();
                req->enable = enabled ? 1 : 0;
                client->async_send_request(req);
                printf("\n[ENABLE] %s\n", enabled ? "ARMED" : "DISARMED");
                break;
            }

            case '1': mode = 0; printf("\nMode: ECO\n");    break;
            case '2': mode = 1; printf("\nMode: NORMAL\n"); break;
            case '3': mode = 2; printf("\nMode: SPORT\n");  break;
            case '+': speed = (speed >= 100) ? 100 : speed + 10;
                      printf("\nSpeed: %d\n", speed); break;
            case '-': speed = (speed <= 10)  ? 10  : speed - 10;
                      printf("\nSpeed: %d\n", speed); break;
            case 'q': case 'Q': running = false; break;
            default: break;
        }

        // Always publish direction topic at 50Hz
        auto msg   = RoverCmd();
        msg.x      = x;
        msg.y      = y;
        msg.speed  = speed;
        msg.mode   = mode;
        publisher->publish(msg);

        printf("\rx=%4d  y=%4d  speed=%3d  mode=%d  enabled=%s    ",
               x, y, speed, mode, enabled ? "YES" : "NO ");
        fflush(stdout);

        rate.sleep();
    }

    // Disarm on exit
    auto req = std::make_shared<RoverControl::Request>();
    req->enable = 0;
    client->async_send_request(req);
    printf("\nExiting teleop.\n");

    rclcpp::shutdown();
    return 0;
}
