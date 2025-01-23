cmake_minimum_required(VERSION 3.8)
project(pose_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../incl.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(utils REQUIRED)

add_executable(pose_controller src/pose_controller.cpp)

ament_target_dependencies(pose_controller rclcpp std_msgs geometry_msgs control_msgs tf2 tf2_ros utils)

install(TARGETS
  pose_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
