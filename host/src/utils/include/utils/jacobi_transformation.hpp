#pragma once

#include "robot_params.hpp"

using BackwardKinematic =
    Eigen::Matrix<real_t, robot_params::N_WHEEL, robot_params::DOF>;
using ForwardKinematic =
    Eigen::Matrix<real_t, robot_params::DOF, robot_params::N_WHEEL>;

inline const auto JACOBI_MTX = []() {
  return BackwardKinematic{{1.0, 1.0, robot_params::L_W_HALF, 1.0},
                           {1.0, -1.0, -robot_params::L_W_HALF, 1.0},
                           {1.0, 1.0, -robot_params::L_W_HALF, -1.0},
                           {1.0, -1.0, robot_params::L_W_HALF, -1.0}} /
         robot_params::WHEEL_RADIUS;
}();

inline const auto INV_JACOBI_MTX = []() {
  return ForwardKinematic{
             {1.0, 1.0, 1.0, 1.0},
             {1.0, -1.0, 1.0, -1.0},
             {1.0 / robot_params::L_W_HALF, -1.0 / robot_params::L_W_HALF,
              -1.0 / robot_params::L_W_HALF, 1.0 / robot_params::L_W_HALF},
             {4.0 / robot_params::WHEEL_RADIUS,
              4.0 / robot_params::WHEEL_RADIUS,
              -4.0 / robot_params::WHEEL_RADIUS,
              -4.0 / robot_params::WHEEL_RADIUS}} *
         robot_params::WHEEL_RADIUS / 4;
}();

inline robot_params::VelRF forward_transform(const robot_params::VelWheel &u) {
  return INV_JACOBI_MTX * u;
}

inline robot_params::VelWheel backward_transform(const robot_params::VelRF &v) {
  return JACOBI_MTX * v;
}
