#pragma once

#include <stdint.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include "real_t.h"

namespace robot_params {
inline constexpr uint8_t DOF = 4;
inline constexpr uint8_t N_WHEEL = 4;

inline constexpr real_t INCREMENTS = 48.0;
inline constexpr real_t GEAR_RATIO = 64.0;
inline constexpr real_t INC2RAD = 2.0 * M_PI / (INCREMENTS * GEAR_RATIO);
inline constexpr real_t LENGTH = 325;
inline constexpr real_t WIDTH = 300;
inline constexpr real_t L_W_HALF = (LENGTH + WIDTH) / 2;
inline constexpr real_t WHEEL_RADIUS = 50;
inline constexpr real_t WHEEL_MAX_RPM = 120; // TODO: tune this value

inline constexpr real_t MAX_VELOCITY_WHEEL_ANGULAR =
    WHEEL_MAX_RPM / 60 * (2 * M_PI);
inline constexpr real_t MAX_VELOCITY_WHEEL_LINEAR =
    WHEEL_MAX_RPM / 60 * 2 * M_PI * WHEEL_RADIUS;
inline constexpr real_t MAX_LINEAR_DEVIATION = 300;
inline constexpr real_t MAX_ANGULAR_DEVIATION = M_PI;

inline constexpr real_t UROS_FREQ_MOD_WHEEL_CTRL_SEC = 0.05;
inline constexpr real_t UROS_FREQ_MOD_INTERPOLATION_SEC = 0.10;
inline constexpr real_t UROS_FREQ_MOD_LIDAR_SEC = 2;
inline constexpr uint16_t S_TO_MS = 1000;

using VelWheel = Eigen::Matrix<real_t, N_WHEEL, 1>;
using VelRF = Eigen::Matrix<real_t, DOF, 1>;
using Robot2WheelMatrix = Eigen::Matrix<real_t, N_WHEEL, DOF>;
using Wheel2RobotMatrix = Eigen::Matrix<real_t, DOF, N_WHEEL>;
}  // namespace robot_params
