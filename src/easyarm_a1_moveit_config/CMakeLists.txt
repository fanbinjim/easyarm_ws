cmake_minimum_required(VERSION 3.22)
project(easyarm_a1_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(move_to_ready src/move_to_ready.cpp)
ament_target_dependencies(
  move_to_ready
  moveit_ros_planning_interface
  rclcpp)

add_executable(switch_controller_mode src/switch_controller_mode.cpp)
ament_target_dependencies(
  switch_controller_mode
  rclcpp
  rclcpp_action
  sensor_msgs
  control_msgs)

install(
  TARGETS move_to_ready switch_controller_mode
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS scripts/safe_shutdown_demo.sh
  DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
