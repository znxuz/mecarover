#pragma once

#include <mecarover/mrtypes.h>
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

inline imsl::vPose<real_t> velocity_smoothen(const imsl::vPose<real_t>& vel_sp,
                                             const imsl::vPose<real_t>& vel_old) {
    real_t diff_max = MAX_VELOCITY * 0.1;

    auto v_diff = vel_sp - vel_old;
    v_diff.vx = std::clamp(v_diff.vx, -diff_max, diff_max);
    v_diff.vy = std::clamp(v_diff.vy, -diff_max, diff_max);
    diff_max /= robot_params.l_w_half;
    v_diff.omega = std::clamp(v_diff.omega, -diff_max, diff_max);

    return vel_old + v_diff;
}
