# Prerequisites

- `docker`
- `tmux`

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

# source `control_msgs` before for `ros2`

- either install locally or build & source from the submodule
