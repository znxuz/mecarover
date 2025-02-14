cmake_minimum_required(VERSION 3.8)
project(vel2d_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(vel2d_bridge src/vel2d_bridge.cpp src/vel2d_frame.cpp)
ament_target_dependencies(vel2d_bridge rclcpp geometry_msgs)

install(TARGETS
  vel2d_bridge
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(vel2d_bridge PRIVATE include/${PROJECT_NAME})

# add crc lib
include(FetchContent)
FetchContent_Declare(
  crcpp
  GIT_REPOSITORY https://github.com/d-bahr/CRCpp
)
FetchContent_MakeAvailable(crcpp)
target_include_directories(vel2d_bridge PRIVATE ${crcpp_SOURCE_DIR}/inc)

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
