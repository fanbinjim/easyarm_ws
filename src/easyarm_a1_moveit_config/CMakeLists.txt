cmake_minimum_required(VERSION 3.22)
project(easyarm_a1_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(move_to_ready src/move_to_ready.cpp)
ament_target_dependencies(
  move_to_ready
  moveit_ros_planning_interface
  rclcpp)

install(
  TARGETS move_to_ready
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
