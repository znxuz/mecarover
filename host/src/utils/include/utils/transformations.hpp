#pragma once

#include "robot_params.hpp"

inline const auto JACOBI_MTX = []() {
  using namespace robot_params;
  return HomogenousTransformMatrix{{1.0, 1.0, L_W_HALF, 1.0},
                                   {1.0, -1.0, -L_W_HALF, 1.0},
                                   {1.0, 1.0, -L_W_HALF, -1.0},
                                   {1.0, -1.0, L_W_HALF, -1.0}} /
         WHEEL_RADIUS;
}();

inline const auto INV_JACOBI_MTX = []() {
  using namespace robot_params;
  return HomogenousTransformMatrix{
             {1.0, 1.0, 1.0, 1.0},
             {1.0, -1.0, 1.0, -1.0},
             {1.0 / L_W_HALF, -1.0 / L_W_HALF, -1.0 / L_W_HALF, 1.0 / L_W_HALF},
             {4.0 / WHEEL_RADIUS, 4.0 / WHEEL_RADIUS, -4.0 / WHEEL_RADIUS,
              -4.0 / WHEEL_RADIUS}} *
         WHEEL_RADIUS / 4;
}();

inline robot_params::VelRF forward_transform(const robot_params::VelWheel &u) {
  return INV_JACOBI_MTX * u;
}

inline robot_params::VelWheel backward_transform(const robot_params::VelRF &v) {
  return JACOBI_MTX * v;
}

template <typename T>
concept FltType = std::is_same_v<T, float> || std::is_same_v<T, double>;

template <typename FltType>
inline robot_params::VelRF rotate_to_wframe(const robot_params::VelRF &dpose,
                                            FltType th) {
  return robot_params::HomogenousTransformMatrix{
             {std::cos(th), -std::sin(th), 0.0, 0.0},
             {std::sin(th), std::cos(th), 0.0, 0.0},
             {0.0, 0.0, 1.0, 0.0},
             {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}

template <typename FltType>
inline robot_params::VelRF rotate_to_rframe(const robot_params::VelRF &dpose,
                                            FltType th) {
  return robot_params::HomogenousTransformMatrix{
             {std::cos(th), std::sin(th), 0.0, 0.0},
             {-std::sin(th), std::cos(th), 0.0, 0.0},
             {0.0, 0.0, 1.0, 0.0},
             {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}
