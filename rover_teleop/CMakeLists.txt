cmake_minimum_required(VERSION 3.8)
project(rover_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(multipurpose_rover REQUIRED)

add_executable(rover_teleop_client src/rover_teleop_client.cpp)
ament_target_dependencies(rover_teleop_client rclcpp multipurpose_rover)

add_executable(rover_cmd_publisher src/rover_cmd_publisher.cpp)
ament_target_dependencies(rover_cmd_publisher rclcpp multipurpose_rover)

add_executable(rover_enable_client src/rover_enable_client.cpp)
ament_target_dependencies(rover_enable_client rclcpp multipurpose_rover)

install(TARGETS
  rover_teleop_client
  rover_cmd_publisher
  rover_enable_client
  DESTINATION lib/${PROJECT_NAME}
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
