cmake_minimum_required(VERSION 3.8)
project(wheel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../incl.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(utils REQUIRED)

add_executable(wheel_controller src/wheel_controller.cpp)
ament_target_dependencies(wheel_controller rclcpp control_msgs sensor_msgs utils)

install(TARGETS
  wheel_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
