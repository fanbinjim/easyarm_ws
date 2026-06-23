cmake_minimum_required(VERSION 3.8)
project(easyarm_motion_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(easyarm_interfaces REQUIRED)

add_executable(easyarm_motion_server
  src/main.cpp
  src/motion_server_node.cpp
  src/joint_state_cache.cpp
  src/hardware_mode_client.cpp
  src/hold_trajectory_sender.cpp
  src/moveit_servo_runtime.cpp
  src/position_servo_executor.cpp
  src/moveit_motion_executor.cpp)
target_include_directories(
  easyarm_motion_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  easyarm_motion_server
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  control_msgs
  trajectory_msgs
  controller_manager_msgs
  rcl_interfaces
  tf2_ros
  std_srvs
  std_msgs
  easyarm_interfaces)

install(
  TARGETS easyarm_motion_server
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
