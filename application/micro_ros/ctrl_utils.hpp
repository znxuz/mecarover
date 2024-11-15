#pragma once

#include <application/mrcpptypes.hpp>
#include <application/robot_params.hpp>

inline constexpr robot_params::Robot2WheelMatrix bt_mtx{
    {1.0, 1.0, robot_params::L_W_HALF, 1.0},
    {1.0, -1.0, -robot_params::L_W_HALF, 1.0},
    {1.0, 1.0, -robot_params::L_W_HALF, -1.0},
    {1.0, -1.0, robot_params::L_W_HALF, -1.0}};

inline constexpr robot_params::Wheel2RobotMatrix ft_mtx{
    {1.0, 1.0, 1.0, 1.0},
    {1.0, -1.0, 1.0, -1.0},
    {1.0 / robot_params::L_W_HALF, -1.0 / robot_params::L_W_HALF,
     -1.0 / robot_params::L_W_HALF, 1.0 / robot_params::L_W_HALF},
    {4.0 / robot_params::WHEEL_RADIUS, 4.0 / robot_params::WHEEL_RADIUS,
     -4.0 / robot_params::WHEEL_RADIUS, -4.0 / robot_params::WHEEL_RADIUS}};

inline robot_params::VelRF vWheel2vRF(const robot_params::VelWheel& u) {
  return ft_mtx * robot_params::WHEEL_RADIUS / 4 * u;
}

inline robot_params::VelWheel vRF2vWheel(const robot_params::VelRF& v) {
  return bt_mtx / robot_params::WHEEL_RADIUS * v;
}
