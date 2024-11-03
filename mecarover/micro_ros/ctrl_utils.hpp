#pragma once

#include <mecarover/mrcpptypes.hpp>
#include <mecarover/robot_params.hpp>

inline constexpr Robot2WheelMatrix bt_mtx{
    {1.0, 1.0, robot_params.l_w_half, 1.0},
    {1.0, -1.0, -robot_params.l_w_half, 1.0},
    {1.0, 1.0, -robot_params.l_w_half, -1.0},
    {1.0, -1.0, robot_params.l_w_half, -1.0}};

inline constexpr Wheel2RobotMatrix ft_mtx{
    {1.0, 1.0, 1.0, 1.0},
    {1.0, -1.0, 1.0, -1.0},
    {1.0 / robot_params.l_w_half, -1.0 / robot_params.l_w_half,
     -1.0 / robot_params.l_w_half, 1.0 / robot_params.l_w_half},
    {4.0 / robot_params.wheel_radius, 4.0 / robot_params.wheel_radius,
     -4.0 / robot_params.wheel_radius, -4.0 / robot_params.wheel_radius}};

inline VelRF vWheel2vRF(const VelWheel& u) { return ft_mtx * u; }

inline VelWheel vRF2vWheel(const VelRF& v) { return bt_mtx * v; }
