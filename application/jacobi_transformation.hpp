#pragma once

#include <application/pose_types.hpp>
#include <application/robot_params.hpp>

using BackwardKinematic =
    Eigen::Matrix<real_t, robot_params::N_WHEEL, robot_params::DOF>;
using ForwardKinematic =
    Eigen::Matrix<real_t, robot_params::DOF, robot_params::N_WHEEL>;

// avoid the runtime overhead for scalar multiplication by multiplying each
// elem separately, because I haven't found a way to *constexpr intialize* the
// matrix with scalar multiplication.
// Just multiplying the matrix with the scalar using `*` would result the
// multiplication being calculated every time in run time

inline constexpr BackwardKinematic JACOBI_MTX{
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

inline constexpr ForwardKinematic INV_JACOBI_MTX{
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
// or {1, 1, -1, -1} for the last row

inline robot_params::VelRF forward_transform(const robot_params::VelWheel& u) {
  return INV_JACOBI_MTX * u;
}

inline robot_params::VelWheel backward_transform(const robot_params::VelRF& v) {
  return JACOBI_MTX * v;
}
