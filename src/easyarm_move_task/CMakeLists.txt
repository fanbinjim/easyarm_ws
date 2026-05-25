cmake_minimum_required(VERSION 3.8)
project(easyarm_move_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(move_to_ready src/move_to_ready.cpp)
target_include_directories(move_to_ready PRIVATE include/${PROJECT_NAME})
ament_target_dependencies(
  move_to_ready
  moveit_ros_planning_interface
  rclcpp)

add_executable(switch_controller_mode src/switch_controller_mode.cpp)
target_include_directories(switch_controller_mode PRIVATE include/${PROJECT_NAME})
ament_target_dependencies(
  switch_controller_mode
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs)

install(
  PROGRAMS scripts/safe_shutdown_demo.sh
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS move_to_ready switch_controller_mode
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
