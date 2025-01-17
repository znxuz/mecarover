cmake_minimum_required(VERSION 3.8)
project(odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../incl.cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(utils REQUIRED)

add_executable(odom src/odom.cpp)

# target_link_libraries(odom utils)
ament_target_dependencies(odom rclcpp control_msgs geometry_msgs tf2 tf2_ros utils)

install(TARGETS
  odom
  DESTINATION lib/${PROJECT_NAME})

ament_package()
