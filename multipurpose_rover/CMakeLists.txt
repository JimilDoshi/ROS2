cmake_minimum_required(VERSION 3.8)
project(multipurpose_rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RoverControl.srv"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(cmd_vel_to_ecu src/cmd_vel_to_ecu.cpp)
ament_target_dependencies(cmd_vel_to_ecu rclcpp geometry_msgs)

# rover_control_server
add_executable(rover_control_server src/rover_control_server.cpp)
ament_target_dependencies(rover_control_server rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rover_control_server ${cpp_typesupport_target})

install(TARGETS
  cmd_vel_to_ecu
  rover_control_server
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
