# Prerequisites

- `docker`

# ROS2 package dependencies

- working `ros2` and `micro-ros-agent` in `jazzy`
- `control_msgs`

## `robot_description` package dependencies

- `rviz2`
- `xacro`
- `joint_state_publisher`
- `robot_state_publisher`
- `slam_toolbox`

Either install the dependencies locally via the system's package manager, or build from source

# Pull/Update submodules

`git submodule update --init --recursive`

# Build `micro_ros_stm32cubemx_utils` static library

```sh
docker pull microros/micro_ros_static_library_builder:jazzy
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:jazzy
```

Or just the use the convenient script `micro_ros_static_lib_gen.sh`

# `stm32cubemx` configuration

- assign a local IP address and a gateway address in `stm32cubemx`, which should
  be from the same subnet as micro-ROS agent, and regenerate the configuration
  files and compile
