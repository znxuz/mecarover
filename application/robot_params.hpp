#pragma once

#include <stdint.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include "real_t.h"

namespace robot_params {
inline constexpr real_t UROS_FREQ_MOD_WHEEL_CTRL_SEC = 0.05;
inline constexpr real_t UROS_FREQ_MOD_INTERPOLATION_SEC = 0.10;
inline constexpr real_t UROS_FREQ_MOD_LIDAR_SEC = 0.5;
inline constexpr uint16_t S_TO_MS = 1000;

inline constexpr uint8_t DOF = 4;
inline constexpr uint8_t N_WHEEL = 4;

inline constexpr real_t GEAR_RATIO = 64.0; // wheel rotation : motor rotation
inline constexpr real_t INCREMENTS = 48.0; // motor rotation : encoder increment
inline constexpr real_t INC2RAD = 2.0 * M_PI / (INCREMENTS * GEAR_RATIO);
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
    WHEEL_MAX_RPM / 60 * (2 * M_PI);
inline constexpr real_t MAX_VELOCITY_WHEEL_LINEAR =
    MAX_VELOCITY_WHEEL_ANGULAR * WHEEL_RADIUS;
inline constexpr real_t MAX_POSE_DEVIATION_LINEAR = 300;
inline constexpr real_t MAX_POSE_DEVIATION_ANGULAR = M_PI;

using VelWheel = Eigen::Matrix<real_t, N_WHEEL, 1>;
using VelRF = Eigen::Matrix<real_t, DOF, 1>;
}  // namespace robot_params
