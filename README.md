# Git submodules

`git submodule update --init --recursive`

# On host

## Prerequisites

- `ros2` in jazzy

## Build `vel2d_bridge`

```sh
# source default ros2 jazzy underlay

cd host/vel2d_bridge
colcon build --symlink-install
```

## install `teleop_twist_keyboard` and run its binary

Either via the system's package manager or build from source.

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

Velocity values published from `teleop_twist_keyboard` will be parsed as 2D
velocity data frames with embedded CRC, which will then be sent over to the MCU
through a USB-CDC port defaulted to `/dev/ttyACM0`

# Firmware

# License

This project uses the CRC++ library (by Daniel Bahr), which is licensed under
the included LICENSE file.
