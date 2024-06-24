# Prerequisites

- `docker`

# Pull/Update submodules

# Build `micro_ros_stm32cubemx_utils` static library

```sh
docker pull microros/micro_ros_static_library_builder:jazzy
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:jazzy
```
