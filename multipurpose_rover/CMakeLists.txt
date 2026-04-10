cmake_minimum_required(VERSION 3.8)
project(multipurpose_rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom message and service
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverCmd.msg"
  "srv/RoverControl.srv"
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Enable service server
add_executable(rover_enable_server src/rover_enable_server.cpp)
ament_target_dependencies(rover_enable_server rclcpp std_msgs)
target_link_libraries(rover_enable_server ${cpp_typesupport_target})

# CAN driver — subscribes to topic and drives ECU
add_executable(rover_can_driver src/rover_can_driver.cpp)
ament_target_dependencies(rover_can_driver rclcpp std_msgs)
target_link_libraries(rover_can_driver ${cpp_typesupport_target})

# CAN receiver — reads raw CAN frames and publishes encoder + IMU data
add_executable(rover_can_receiver src/rover_can_receiver.cpp)
ament_target_dependencies(rover_can_receiver rclcpp std_msgs sensor_msgs nav_msgs)

# Odometry — computes and publishes /odom and /tf from encoder data
add_executable(rover_odom src/rover_odom.cpp)
ament_target_dependencies(rover_odom rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros)

# Old combined server (kept for reference)
add_executable(rover_control_server src/rover_control_server.cpp)
ament_target_dependencies(rover_control_server rclcpp)
target_link_libraries(rover_control_server ${cpp_typesupport_target})

install(TARGETS
  rover_enable_server
  rover_can_driver
  rover_can_receiver
  rover_odom
  rover_control_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
