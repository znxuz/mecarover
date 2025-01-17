#pragma once

#include <stdint.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <chrono>
#include <numbers>

#include "real_t.hpp"

namespace robot_params {
using namespace std::chrono;
using namespace Eigen;

using std::numbers::pi;

inline constexpr milliseconds WHEEL_CTRL_PERIOD_MS = 20ms;
inline constexpr milliseconds POSE_CTRL_PERIOD_MS = 33ms;
inline constexpr milliseconds LIDAR_PERIOD_MS = 200ms;

inline constexpr uint8_t DOF = 4;
inline constexpr uint8_t N_WHEEL = 4;

inline constexpr real_t GEAR_RATIO = 64.0; // 1 wheel rot = 64 motor rot
inline constexpr real_t INCREMENTS = 48.0; // 1 motor rot = 48 encoder incs
inline constexpr real_t INC2RAD = 2.0 * pi / (INCREMENTS * GEAR_RATIO);
inline constexpr real_t LENGTH = 325;
inline constexpr real_t WIDTH = 300;
inline constexpr real_t L_W_HALF = (LENGTH + WIDTH) / 2;
inline constexpr real_t WHEEL_RADIUS = 50;
/*
 * RPM given from the manufacturer: 120
 * experiment:
 *  encoder increment after 1 full rotation: GEAR_RATIO * INCREMENTS = 3072
 *  encoder increment delta on max PWM in 1 second: ~5375
 *  max rotation per minute = 5375 / 3072 * 60 ~= 105
 */
inline constexpr real_t WHEEL_MAX_RPM = 105;

inline constexpr real_t MAX_VELOCITY_WHEEL_ANGULAR =
    WHEEL_MAX_RPM / 60 * (2 * pi);
inline constexpr real_t MAX_VELOCITY_WHEEL_LINEAR =
    MAX_VELOCITY_WHEEL_ANGULAR * WHEEL_RADIUS;

using VelWheel = Matrix<real_t, N_WHEEL, 1>;
using VelRF = Matrix<real_t, DOF, 1>;
using HomogenousTransformMatrix = Matrix<real_t, N_WHEEL, DOF>;
} // namespace robot_params
