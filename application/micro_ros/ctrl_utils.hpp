#pragma once

#include <application/mrcpptypes.hpp>
#include <application/robot_params.hpp>

// avoid the runtime overhead for scalar multiplication by multiplying each elem
// separately, because constexpr initialization doesn't seem to support scalar
// multiplication

inline constexpr robot_params::Robot2WheelMatrix bt_mtx{
    {1.0 / robot_params::WHEEL_RADIUS, 1.0 / robot_params::WHEEL_RADIUS,
     robot_params::L_W_HALF / robot_params::WHEEL_RADIUS,
     1.0 / robot_params::WHEEL_RADIUS},
    {1.0 / robot_params::WHEEL_RADIUS, -1.0 / robot_params::WHEEL_RADIUS,
     -robot_params::L_W_HALF / robot_params::WHEEL_RADIUS,
     1.0 / robot_params::WHEEL_RADIUS},
    {1.0 / robot_params::WHEEL_RADIUS, 1.0 / robot_params::WHEEL_RADIUS,
     -robot_params::L_W_HALF / robot_params::WHEEL_RADIUS,
     -1.0 / robot_params::WHEEL_RADIUS},
    {1.0 / robot_params::WHEEL_RADIUS, -1.0 / robot_params::WHEEL_RADIUS,
     robot_params::L_W_HALF / robot_params::WHEEL_RADIUS,
     -1.0 / robot_params::WHEEL_RADIUS}};

inline constexpr robot_params::Wheel2RobotMatrix ft_mtx{
    {1.0 * robot_params::WHEEL_RADIUS / 4, 1.0 * robot_params::WHEEL_RADIUS / 4,
     1.0 * robot_params::WHEEL_RADIUS / 4,
     1.0 * robot_params::WHEEL_RADIUS / 4},
    {1.0 * robot_params::WHEEL_RADIUS / 4,
     -1.0 * robot_params::WHEEL_RADIUS / 4,
     1.0 * robot_params::WHEEL_RADIUS / 4,
     -1.0 * robot_params::WHEEL_RADIUS / 4},
    {1.0 / robot_params::L_W_HALF * robot_params::WHEEL_RADIUS / 4,
     -1.0 / robot_params::L_W_HALF* robot_params::WHEEL_RADIUS / 4,
     -1.0 / robot_params::L_W_HALF* robot_params::WHEEL_RADIUS / 4,
     1.0 / robot_params::L_W_HALF* robot_params::WHEEL_RADIUS / 4},
    {4.0 / robot_params::WHEEL_RADIUS * robot_params::WHEEL_RADIUS / 4,
     4.0 / robot_params::WHEEL_RADIUS* robot_params::WHEEL_RADIUS / 4,
     -4.0 / robot_params::WHEEL_RADIUS* robot_params::WHEEL_RADIUS / 4,
     -4.0 / robot_params::WHEEL_RADIUS* robot_params::WHEEL_RADIUS / 4}};

inline robot_params::VelRF vWheel2vRF(const robot_params::VelWheel& u) {
  return ft_mtx * u;
}

inline robot_params::VelWheel vRF2vWheel(const robot_params::VelRF& v) {
  return bt_mtx * v;
}
