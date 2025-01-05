# Prerequisites

- `docker`
- `ros2` in jazzy

## ROS2 package dependencies

- `micro-ros-agent` in `jazzy`
- `control_msgs`

Either install the dependencies locally via the system's package manager, or
build from source

# Pull/Update submodules

`git submodule update --init --recursive`

# Build `micro_ros_stm32cubemx_utils` static library

```sh
docker pull microros/micro_ros_static_library_builder:jazzy
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:jazzy
```

Or just the use the convenient script `./scripts/micro_ros_static_lib_gen.sh`

# `stm32cubemx` configuration

- assign a local IP address and a gateway address in `stm32cubemx`, which
  should be from the same subnet as micro-ROS agent, and regenerate the
  configuration files and recompile

# host ethernet interface configuration

Configure the host Ethernet interface with the IP address specified in the
`Makefile` for the micro-ROS agent, or use the convenient script
`./scripts/configure_ethernet_if.sh` for temporarily adding the IP for the
ethernet interface

```sh
sudo ip a add 192.168.188.200/24 dev <eth interface>
sudo ip link set <eth interface> up
```

This IP address will be the micro-ROS agent IP address, which the client on the
MCU uses for establishing the connection with the agent
