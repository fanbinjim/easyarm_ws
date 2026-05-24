cmake_minimum_required(VERSION 3.8)
project(robstride_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robstride_can_driver.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Threads::Threads
)

add_executable(single_motor_demo
  src/single_motor_demo.cpp
)

target_link_libraries(single_motor_demo
  PRIVATE
    ${PROJECT_NAME}
)

add_executable(discover_motors
  src/discover_motors.cpp
)

target_link_libraries(discover_motors
  PRIVATE
    ${PROJECT_NAME}
)

add_executable(set_zero_pos
  src/set_zero_pos.cpp
)

target_link_libraries(set_zero_pos
  PRIVATE
    ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    single_motor_demo
    discover_motors
    set_zero_pos
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Threads)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
