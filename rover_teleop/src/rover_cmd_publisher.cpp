/************************************************************
 * rover_cmd_publisher.cpp
 * Uses /dev/input/eventX for true key press/release events.
 * No OS key repeat delay — instant response on hold/release.
 ************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "multipurpose_rover/msg/rover_cmd.hpp"

#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <cstring>
#include <cstdio>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;
using RoverCmd = multipurpose_rover::msg::RoverCmd;

struct KeyState {
    std::atomic<bool> forward{false};
    std::atomic<bool> backward{false};
    std::atomic<bool> left{false};
    std::atomic<bool> right{false};
    std::atomic<bool> quit{false};
    std::atomic<int>  mode{1};
    std::atomic<int>  speed{80};
};

// Find the keyboard event device automatically
std::string findKeyboard() {
    DIR *dir = opendir("/dev/input");
    if (!dir) return "";
    struct dirent *entry;
    std::string best = "";
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "event", 5) != 0) continue;
        std::string path = "/dev/input/" + std::string(entry->d_name);
        int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        char name[256] = {};
        ioctl(fd, EVIOCGNAME(sizeof(name)), name);

        // Check if device has key capabilities (EV_KEY bit set)
        uint8_t evbits[EV_MAX/8 + 1] = {};
        ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits);
        bool has_keys = evbits[EV_KEY / 8] & (1 << (EV_KEY % 8));

        // Check if it has letter keys (KEY_W = 17)
        uint8_t keybits[KEY_MAX/8 + 1] = {};
        ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits);
        bool has_w = keybits[KEY_W / 8] & (1 << (KEY_W % 8));

        close(fd);

        if (has_keys && has_w) {
            best = path;
            break;  // first device with letter keys is the main keyboard
        }
    }
    closedir(dir);
    return best;
}

void keyboardThread(KeyState &keys, const std::string &dev) {
    int fd = open(dev.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        printf("ERROR: Cannot open %s — try: sudo chmod a+r %s\n",
               dev.c_str(), dev.c_str());
        keys.quit = true;
        return;
    }

    struct input_event ev;
    while (!keys.quit) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n < (ssize_t)sizeof(ev)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        if (ev.type != EV_KEY) continue;
        if (ev.value == 2) continue;  // ignore repeat

        bool pressed = (ev.value == 1);

        switch (ev.code) {
            case KEY_W: keys.forward  = pressed; break;
            case KEY_S: keys.backward = pressed; break;
            case KEY_A: keys.left     = pressed; break;
            case KEY_D: keys.right    = pressed; break;
            case KEY_Q: if (pressed) keys.quit = true; break;
            case KEY_1: if (pressed) { keys.mode = 0; printf("\nMode: ECO\n");    fflush(stdout); } break;
            case KEY_2: if (pressed) { keys.mode = 1; printf("\nMode: NORMAL\n"); fflush(stdout); } break;
            case KEY_3: if (pressed) { keys.mode = 2; printf("\nMode: SPORT\n");  fflush(stdout); } break;
            case KEY_EQUAL:
                if (pressed) { int s = keys.speed + 10; keys.speed = s > 100 ? 100 : s;
                               printf("\nSpeed: %d\n", (int)keys.speed); fflush(stdout); } break;
            case KEY_MINUS:
                if (pressed) { int s = keys.speed - 10; keys.speed = s < 10 ? 10 : s;
                               printf("\nSpeed: %d\n", (int)keys.speed); fflush(stdout); } break;
            default: break;
        }
    }
    close(fd);
}

class RoverCmdPublisher : public rclcpp::Node {
public:
    RoverCmdPublisher(KeyState &keys) : Node("rover_cmd_publisher"), keys_(keys) {
        auto qos   = rclcpp::QoS(1).best_effort();
        publisher_ = this->create_publisher<RoverCmd>("rover_cmd", qos);
        timer_     = this->create_wall_timer(
            10ms, std::bind(&RoverCmdPublisher::timerCallback, this));
    }

    bool running() { return !keys_.quit; }

private:
    void timerCallback() {
        int16_t x = 0, y = 0;
        if (keys_.forward)  x =  100;
        if (keys_.backward) x = -100;
        if (keys_.left)     y =  100;
        if (keys_.right)    y = -100;

        RoverCmd msg;
        msg.x     = x;
        msg.y     = y;
        msg.speed = static_cast<uint8_t>(keys_.speed.load());
        msg.mode  = static_cast<uint8_t>(keys_.mode.load());
        publisher_->publish(msg);

        printf("\rx=%4d  y=%4d  speed=%3d  mode=%d    ",
               x, y, (int)keys_.speed, (int)keys_.mode);
        fflush(stdout);
    }

    rclcpp::Publisher<RoverCmd>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    KeyState &keys_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::string dev = findKeyboard();
    if (dev.empty()) {
        // fallback: let user specify
        printf("Keyboard not found automatically.\n");
        printf("Run: ls /dev/input/by-id/ to find yours, then:\n");
        printf("  ros2 run rover_teleop rover_cmd_publisher /dev/input/eventX\n");
        if (argc > 1) dev = argv[1];
        else { rclcpp::shutdown(); return 1; }
    }
    printf("Using keyboard device: %s\n", dev.c_str());

    printf("\n=== Rover Direction Publisher ===\n");
    printf("  W/S   : forward/backward\n");
    printf("  A/D   : steer left/right\n");
    printf("  1/2/3 : eco/normal/sport mode\n");
    printf("  +/-   : speed up/down\n");
    printf("  Q     : quit\n\n");

    KeyState keys;
    std::thread kb(keyboardThread, std::ref(keys), dev);

    auto node = std::make_shared<RoverCmdPublisher>(keys);
    while (rclcpp::ok() && node->running()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    keys.quit = true;
    kb.join();

    printf("\nExiting.\n");
    rclcpp::shutdown();
    return 0;
}
