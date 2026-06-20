cmake_minimum_required(VERSION 3.8)
project(easyarm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(easyarm_dynamics REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(easyarm_controller SHARED
  src/dynamics_provider.cpp
  src/easyarm_servo_controller.cpp
)
target_include_directories(easyarm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(easyarm_controller
  controller_interface
  easyarm_dynamics
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  trajectory_msgs
)
target_link_libraries(easyarm_controller
  Eigen3::Eigen
)
pluginlib_export_plugin_description_file(
  controller_interface
  easyarm_controller_plugins.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS easyarm_controller
  EXPORT export_easyarm_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(include)
ament_export_targets(export_easyarm_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  easyarm_dynamics
  Eigen3
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  trajectory_msgs
)

ament_package()
