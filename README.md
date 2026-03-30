# Multipurpose Rover — ROS2 Teleoperation System

A complete low-latency teleoperation stack for a 4-wheel rover, built on **ROS2 Jazzy**, **SocketCAN**, and an **ESP32 FreeRTOS ECU**. Keyboard input on a laptop drives four DC motors in under 50 ms, end-to-end.

> **Stack at a glance**
> Laptop (C++ evdev teleop) → Wi-Fi LAN (ROS2 DDS/CycloneDDS) → Raspberry Pi (SocketCAN bridge) → MCP2515 → ESP32 TWAI → 4× IBT-2 motor drivers

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Repository Structure](#repository-structure)
3. [ROS2 Node Graph](#ros2-node-graph)
4. [CAN Frame Format](#can-frame-format)
5. [ESP32 FreeRTOS Tasks](#esp32-freertos-tasks)
6. [Quick Start](#quick-start)
7. [Configuration](#configuration)
8. [Deployment (systemd)](#deployment-systemd)
9. [Testing & Validation](#testing--validation)
10. [Debugging Guide](#debugging-guide)
11. [Key Engineering Decisions](#key-engineering-decisions)
12. [Future Work](#future-work)

---

## System Architecture

```
┌─────────────────────────────────────┐
│         LAPTOP  (Teleop Client)     │
│  rover_cmd_publisher                │  ← reads /dev/input/eventX (evdev)
│  rover_enable_client                │  ← E to arm/disarm
└────────────────┬────────────────────┘
                 │  Wi-Fi LAN
                 │  /rover_cmd topic @ 50 Hz  (RoverCmd msg)
                 │  rover_enable service      (RoverControl srv)
                 ▼
┌─────────────────────────────────────┐
│     RASPBERRY PI  (ROS2 Gateway)    │
│  rover_enable_server                │  ← owns arm/disarm state
│  rover_can_driver                   │  ← sends CAN frames @ 100 Hz
└────────────────┬────────────────────┘
                 │  SPI → MCP2515 → SocketCAN (can0) @ 500 kbps
                 │  CAN ID 0x101, 8-byte frame
                 ▼
┌─────────────────────────────────────┐
│        ESP32  (Motor ECU)           │
│  TWAI (CAN) → FreeRTOS tasks        │
│  4× IBT-2 motor drivers + ADXL345  │
└────────────────┬────────────────────┘
                 │  LEDC PWM
          [Motor 1–4]
```

### Four-Layer Pipeline

| Layer | Hardware | Role |
|-------|----------|------|
| **Teleop** | Laptop | Reads keyboard via evdev; publishes `RoverCmd` at 50 Hz |
| **Gateway** | Raspberry Pi | Translates ROS2 messages → SocketCAN frames |
| **ECU** | ESP32 | Decodes CAN; applies differential drive mixing; drives PWM |
| **Actuators** | 4× DC motors via IBT-2 | Physical motion |

---

## Repository Structure

```
.
├── multipurpose_rover/          # Pi-side ROS2 package
│   ├── msg/
│   │   └── RoverCmd.msg         # Custom message: x, y, speed, mode
│   ├── srv/
│   │   └── RoverControl.srv     # Custom service: enable / success
│   ├── src/
│   │   ├── rover_enable_server.cpp   ← handles arm/disarm service
│   │   ├── rover_can_driver.cpp      ← core: ROS2 → SocketCAN @ 100 Hz
│   │   ├── rover_control_server.cpp  ← legacy combined node (reference)
│   │   └── cmd_vel_to_ecu.cpp        ← Nav2-compatible /cmd_vel bridge
│   ├── systemd/
│   │   ├── rover_enable_server.service
│   │   └── rover_can_driver.service
│   ├── CMakeLists.txt
│   └── package.xml
│
└── rover_teleop/                # Laptop-side ROS2 package
    ├── src/
    │   ├── rover_cmd_publisher.cpp   ← evdev keyboard → RoverCmd @ 50 Hz
    │   ├── rover_enable_client.cpp   ← E to arm/disarm
    │   └── rover_teleop_client.cpp   ← legacy monolithic node (reference)
    ├── CMakeLists.txt
    └── package.xml
```

### Active vs Legacy Nodes

| Node | Status | Description |
|------|--------|-------------|
| `rover_cmd_publisher` | ✅ Active | evdev keyboard, publishes direction @ 50 Hz |
| `rover_enable_client` | ✅ Active | One-shot arm/disarm via service call |
| `rover_enable_server` | ✅ Active | Owns enable state; publishes to topic |
| `rover_can_driver` | ✅ Active | Subscribes to cmd + enable; sends CAN @ 100 Hz |
| `rover_control_server` | 🗃 Reference | Legacy combined node; not deployed |
| `rover_teleop_client` | 🗃 Reference | Legacy monolithic teleop; not deployed |
| `cmd_vel_to_ecu` | 🔌 Optional | Nav2 bridge via `/cmd_vel` |

---

## ROS2 Node Graph

```
[Laptop]                                    [Raspberry Pi]

rover_cmd_publisher ──/rover_cmd──────────► rover_can_driver ──► SocketCAN (can0)
                                                    ▲
rover_enable_client ──rover_enable (srv)──► rover_enable_server
                                                    │
                                            /rover_enable_state
```

### Topics & Services

| Name | Type | Direction | Rate |
|------|------|-----------|------|
| `/rover_cmd` | `RoverCmd` | Laptop → Pi | 50 Hz |
| `/rover_enable_state` | `std_msgs/UInt8` | enable_server → can_driver | On change |
| `rover_enable` | `RoverControl` (service) | Laptop → Pi | On key-E |

### Custom Message: `RoverCmd.msg`

```
int16  x      # throttle  -100 to +100  (negated on Pi for polarity)
int16  y      # steering  -100 to +100
uint8  speed  # 0 to 100  (applied as a multiplier on Pi side)
uint8  mode   # 0=eco  1=normal  2=sport
```

### Custom Service: `RoverControl.srv`

```
uint8  enable   # 1 = run, 0 = stop
---
bool   success
```

---

## CAN Frame Format

### Command Frame — ID `0x101` (Pi → ESP32)

| Bytes | Field | Type | Notes |
|-------|-------|------|-------|
| 0–1 | `y` (steering) | `int16_t` big-endian | −100 to +100 |
| 2–3 | `x` (throttle) | `int16_t` big-endian | −100 to +100; pre-scaled on Pi |
| 4 | `speed` | `uint8_t` | Always **100** in final design (speed baked into x/y) |
| 5 | `enable` | `uint8_t` | 1 = run, 0 = stop |
| 6 | `mode` | `uint8_t` | 0 = Eco (0.3×), 1 = Normal (0.6×), 2 = Sport (1.0×) |
| 7 | reserved | `0x00` | — |

**Why speed is always 100:** The Pi pre-scales `x` and `y` before transmission (`x_scaled = x * speed / 100`). This eliminates a race-condition where an old `speed` byte arrives with a new direction byte, causing a momentary burst.

### Feedback Frame — ID `0x200` (ESP32 → Pi)

| Bytes | Field | Notes |
|-------|-------|-------|
| 0–1 | `ax × 100` | Accelerometer X (m/s²) |
| 2–3 | `ay × 100` | Accelerometer Y |
| 4–5 | `az × 100` | Accelerometer Z |
| 6 | `speed` | Current commanded speed |
| 7 | `faults` | Fault status byte (see below) |

### Fault Frame — ID `0x300`

| Bit | Flag | Condition |
|-----|------|-----------|
| 0 | `FAULT_CAN_RX_TIMEOUT` | No valid CAN frame for 500 ms |
| 1 | `FAULT_ADXL_ERROR` | ADXL345 read failure |
| 2 | `FAULT_MOTOR_FAULT` | Reserved for H-bridge fault detection |

---

## ESP32 FreeRTOS Tasks

```
Core 0                          Core 1
──────────────────────────      ──────────────────────────
can_rx_task   (Priority 3)      control_task  (Priority 3)
accel_task    (Priority 2)
can_tx_task   (Priority 2)
fault_tx_task (Priority 1)
```

| Task | Core | Priority | Stack | Function |
|------|------|----------|-------|----------|
| `can_rx_task` | 0 | 3 | 2048 B | Receives CAN frames; updates control struct; resets watchdog |
| `control_task` | 1 | 3 | 2048 B | Differential drive mixing; drives LEDC PWM at 100 Hz |
| `accel_task` | 0 | 2 | 2048 B | Reads ADXL345 at 10 Hz; mutex-protected |
| `can_tx_task` | 0 | 2 | 2048 B | Sends feedback frame at 5 Hz |
| `fault_tx_task` | 0 | 1 | 1024 B | Broadcasts fault byte at 2 Hz |

### Differential Drive Mixing

```
throttle = (x / 100.0) × speed × mode_scale
turn     = -(y / 100.0) × speed × mode_scale

left  = constrain(throttle + turn, -100, 100)
right = constrain(throttle - turn, -100, 100)
```

Motors 2 and 3 (right-side group) have inverted wiring and are compensated in firmware:

| Motor | Position | Polarity |
|-------|----------|----------|
| 1 | Front-left | Normal |
| 2 | Front-right | Inverted in firmware |
| 3 | Rear-left | Inverted in firmware |
| 4 | Rear-right | Normal |

### Motor Pin Mapping

| Motor | RPWM pin | LPWM pin |
|-------|----------|----------|
| 1 (FL) | GPIO 26 | GPIO 27 |
| 2 (FR) | GPIO 32 | GPIO 33 |
| 3 (RL) | GPIO 25 | GPIO 14 |
| 4 (RR) | GPIO 12 | GPIO 13 |

---

## Quick Start

### Prerequisites

- ROS2 Jazzy installed on both Raspberry Pi and laptop
- CycloneDDS: `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`
- SocketCAN tools: `sudo apt install can-utils`
- ESP32 Arduino IDE with Adafruit ADXL345 library

### 1. Clone and build

```bash
# On both machines
cd ~/ros2_ws/src
git clone <this-repo>
cd ..
colcon build
source install/setup.bash
```

### 2. Configure environment (both machines)

Add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=40
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Both machines **must** be on the same subnet (e.g. `192.168.1.x/24`).

### 3. Flash ESP32

Open `multipurpose_rover/Motor_Control_Kiro.ino` in Arduino IDE, select your ESP32 board, and flash.

### 4. Configure MCP2515

In `/boot/firmware/config.txt` on the Raspberry Pi:

```
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
dtoverlay=spi-bcm2835
```

> ⚠️ **Critical:** Match `oscillator=` to the crystal on your MCP2515 module (check the marking — common values are 8 MHz and 16 MHz). A mismatch causes silent TX failures.

### 5. Bring up CAN interface

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 6. Run

**On the Raspberry Pi:**
```bash
ros2 run multipurpose_rover rover_enable_server &
ros2 run multipurpose_rover rover_can_driver
```

**On the laptop (two terminals):**
```bash
# Terminal 1 — continuous direction commands
ros2 run rover_teleop rover_cmd_publisher

# Terminal 2 — arm/disarm
ros2 run rover_teleop rover_enable_client
```

### Controls

| Key | Action |
|-----|--------|
| `W` / `S` | Forward / Backward |
| `A` / `D` | Steer left / right |
| `E` | Toggle arm / disarm |
| `1` / `2` / `3` | Eco / Normal / Sport mode |
| `+` / `-` | Speed up / down (10% steps, range 10–100) |
| `Q` | Quit |

---

## Configuration

All tunable constants are compile-time for now (parameter server is a planned improvement):

| File | Constant | Default | Description |
|------|----------|---------|-------------|
| `rover_can_driver.cpp` | watchdog timeout | 200 ms | Force-stop if no `/rover_cmd` |
| `Motor_Control_Kiro.ino` | watchdog timeout | 500 ms | ECU halts if no CAN frame |
| `Motor_Control_Kiro.ino` | `MIN_PWM` | 30 | Minimum PWM to overcome stall |
| `Motor_Control_Kiro.ino` | `PWM_FREQ` | 5000 Hz | LEDC frequency |
| `cmd_vel_to_ecu.cpp` | `MAX_LINEAR` | 1.0 m/s | For Nav2 bridge scaling |

---

## Deployment (systemd)

Auto-start the rover nodes on boot without any SSH login:

```bash
sudo cp multipurpose_rover/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rover_enable_server rover_can_driver
sudo systemctl start rover_enable_server rover_can_driver
```

### Boot Sequence

```
t=0s    Power on
t=~15s  network.target reached
t=20s   rover_enable_server starts (15s + 5s ExecStartPre sleep)
t=21s   rover_can_driver starts (15s + 6s sleep; After= ensures enable_server is up)
t=21s   can0 configured: ip link set can0 type can bitrate 500000 && up
t=22s   rover_can_driver spinning, sending CAN frames
```

### Sudoers for CAN interface

The `ip link` commands require root. Add a narrow `NOPASSWD` rule (do not run the whole service as root):

```
# /etc/sudoers.d/rover-can
camero ALL=(ALL) NOPASSWD: /sbin/ip link set can0 type can bitrate 500000
camero ALL=(ALL) NOPASSWD: /sbin/ip link set can0 up
```

### Useful service commands

```bash
systemctl status rover_can_driver
journalctl -u rover_can_driver -f          # live log
sudo systemctl restart rover_can_driver
```

---

## Testing & Validation

### Unit tests

```bash
# Publish a manual command
ros2 topic pub /rover_cmd multipurpose_rover/msg/RoverCmd \
  '{x: 50, y: 0, speed: 60, mode: 1}' --rate 10

# Arm the rover
ros2 service call /rover_enable multipurpose_rover/srv/RoverControl '{enable: 1}'

# Monitor enable state
ros2 topic echo /rover_enable_state

# Verify publish rate
ros2 topic hz /rover_cmd   # expect ~50 Hz
```

### Network validation

```bash
# Confirm cross-machine node discovery
ros2 node list
# Expected: /rover_cmd_publisher, /rover_enable_client (laptop)
#           /rover_enable_server, /rover_can_driver   (Pi)

ros2 topic info /rover_cmd
# Expected: Publisher count: 1 / Subscription count: 1
```

### CAN bus validation

```bash
# Monitor all frames in real time
candump can0
# Expected when driving forward: can0 101 [8] 00 00 00 32 64 01 01 00

# Inject a manual stop frame
cansend can0 101#0000000000000100

# Check for TX/RX error counters
ip -details link show can0
```

### End-to-end procedure

1. Start Pi nodes (or verify systemd started them); open `candump can0` in a separate terminal.
2. Start `rover_cmd_publisher` on the laptop.
3. Arm via `rover_enable_client` (press `E`).
4. Hold `W` → verify non-zero `x` bytes in candump, motors spin forward.
5. Hold `S` → verify `x` bytes reverse sign.
6. Hold `A` / `D` → verify `y` bytes go positive / negative.
7. Release all keys → within 200 ms the Pi watchdog zeros out the frame; motors stop.
8. Kill `rover_cmd_publisher` → motors stop within 200 ms.

---

## Debugging Guide

### Nodes not discovering each other across machines

Most cross-machine ROS2 failures are network issues, not ROS2 bugs. Check in this order:

1. **Subnet** — both machines must be on the same `/24` subnet. Multicast does not traverse routers.
   ```bash
   ping <other-machine-ip>
   ```
2. **Domain ID** — must match on all machines.
   ```bash
   echo $ROS_DOMAIN_ID   # must be 40 (or whatever you chose) on both
   ```
3. **DDS implementation** — FastDDS and CycloneDDS are not wire-compatible.
   ```bash
   echo $RMW_IMPLEMENTATION   # must be rmw_cyclonedds_cpp on both
   ```
4. **systemd services** don't source `.bashrc`. Make sure `RMW_IMPLEMENTATION` is set in the service `Environment=` or `ExecStart=` line, not just in your shell.

### CAN frames sent but never received by ESP32

- **Oscillator mismatch** is the most common culprit. Verify the crystal frequency printed on the MCP2515 module and ensure `oscillator=<value>` in `/boot/firmware/config.txt` matches exactly.
- Check `ip -details link show can0` for TX error counters — rising errors confirm a baud rate mismatch.
- Use `candump can0` to confirm frames appear at the Linux level before suspecting ESP32.

### Node exits immediately after starting

You forgot `rclcpp::spin()` in `main()`. Any node with subscriptions, services, or timers must call `rclcpp::spin(node)` — it blocks and processes callbacks. Nodes that only publish once do not need it.

### Key-repeat delay in teleop

The legacy `rover_teleop_client.cpp` uses `termios` raw mode, which still applies the OS keyboard repeat rate (~250 ms initial delay). `rover_cmd_publisher.cpp` uses the Linux `evdev` API (`/dev/input/eventX`) which delivers true key-press/release events with zero repeat lag. Always use `rover_cmd_publisher`.

### Motor burst when changing speed

This was a bug in the legacy `rover_control_server.cpp` where `speed` was sent as a separate CAN byte. Between an old `speed` byte and updated direction bytes the ECU would momentarily apply full thrust. Fixed in `rover_can_driver.cpp` by pre-multiplying `x` and `y` by `speed` on the Pi, then always sending `speed=100`.

### CAN interface not found by ROS2 node

The `can0` interface must be up before `initCAN()` runs. The systemd service handles this by running `ip link set can0 up` in `ExecStart` before `ros2 run`. If running manually, bring up the interface first.

### CMake undefined symbol errors (rosidl type-support)

ROS2 Jazzy requires explicit type-support linkage after `rosidl_generate_interfaces`. Add:

```cmake
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(your_executable ${cpp_typesupport_target})
```

Simply calling `ament_target_dependencies` is not sufficient.

---

## Key Engineering Decisions

### evdev over termios for keyboard input

`termios` raw mode still applies the OS keyboard repeat rate, causing a characteristic "pause then burst" feel when holding a key. The `evdev` API delivers raw press/release events synchronously, giving true zero-lag key-hold behaviour essential for real-time control.

### Speed pre-scaling on Pi (atomicity)

When multiple CAN frame bytes are semantically coupled (direction × speed), they must be combined into a single atomic value before transmission. Sending them separately creates a window where a torn update causes a momentary burst. The Pi pre-computes `x_scaled = x * speed / 100` and always sends `speed=100`.

### Decoupled enable/disable

The arm/disarm state is a service call, not a topic. This prevents enable state from being corrupted by topic network jitter and allows the CAN driver to be restarted independently without losing the armed state.

### Two independent watchdogs

- **Pi watchdog (200 ms):** `rover_can_driver` zeroes the command if no `/rover_cmd` arrives.
- **ECU watchdog (500 ms):** ESP32 halts motors if no CAN frame arrives.

If the Pi ROS2 node crashes, the ESP32 halts within 500 ms regardless.

### QoS: best-effort, depth 1

For a 100 Hz control loop, a 200 ms old command is worse than no command. `best_effort` + `depth=1` ensures the CAN driver always processes the latest command and never dequeues a burst of stale ones after a brief network hiccup.

---

## Future Work

- **Hardware e-stop:** GPIO pin wired to CAN transceiver disable — motor halt independent of software state.
- **Tilt protection:** Use the ADXL345 data already telemetered in the feedback frame to auto-disarm above a tilt threshold.
- **ROS2 Nav2 integration:** `cmd_vel_to_ecu.cpp` already bridges `/cmd_vel` to CAN; connecting Nav2 requires no firmware changes.
- **LIDAR + SLAM:** Add RPLidar A1 and `slam_toolbox` for map building.
- **Launch file:** Replace multi-terminal startup with a single `ros2 launch` file.
- **Parameter server:** Move watchdog timeouts, CAN interface name, and speed defaults into ROS2 parameters to avoid recompilation.
- **CAN feedback loop:** Subscribe to feedback frame `0x200` on the Pi and republish as `diagnostic_msgs` for monitoring.
- **Multi-rover fleet:** ROS2 namespaces (`/rover1/`, `/rover2/`) allow multiple rovers on the same DDS domain without topic collision.
