#pragma once

#include <stdint.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <numbers>

namespace robot_params {
using std::numbers::pi;

inline constexpr double WHEEL_CTRL_PERIOD_S = 0.02;
inline constexpr double POSE_CTRL_PERIOD_S = 0.03;
inline constexpr double LIDAR_PERIOD_S = 0.2;
inline constexpr uint16_t S_TO_MS = 1000;

inline constexpr uint8_t DOF = 4;
inline constexpr uint8_t N_WHEEL = 4;

inline constexpr double GEAR_RATIO = 64.0;  // 1 wheel rot = 64 motor rot
inline constexpr double INCREMENTS = 48.0;  // 1 motor rot = 48 encoder incs
inline constexpr double INC2RAD = 2.0 * pi / (INCREMENTS * GEAR_RATIO);
inline constexpr double LENGTH = 325;
inline constexpr double WIDTH = 300;
inline constexpr double L_W_HALF = (LENGTH + WIDTH) / 2;
inline constexpr double WHEEL_RADIUS = 50;
/*
 * RPM given from the manufacturer: 120
 * experiment:
 *  encoder increment after 1 full rotation: GEAR_RATIO * INCREMENTS = 3072
 *  encoder increment delta on max PWM in 1 second: ~5375
 *  max rotation per minute = 5375 / 3072 * 60 ~= 105
 */
inline constexpr double WHEEL_MAX_RPM = 105;

inline constexpr double MAX_VELOCITY_WHEEL_ANGULAR =
    WHEEL_MAX_RPM / 60 * (2 * pi);
inline constexpr double MAX_VELOCITY_WHEEL_LINEAR =
    MAX_VELOCITY_WHEEL_ANGULAR * WHEEL_RADIUS;

using VelWheel = Eigen::Matrix<double, N_WHEEL, 1>;
using VelRF = Eigen::Matrix<double, DOF, 1>;
}  // namespace robot_params
