#pragma once

#include "robot_params.hpp"

/*
 * eigen pitfalls: https://eigen.tuxfamily.org/dox/TopicPitfalls.html
 */

namespace transform {

template <typename T>
concept FltType = std::is_same_v<T, float> || std::is_same_v<T, double>;

using namespace robot_params;

inline const auto JACOBI_MTX = []() -> HomogenousTransformMatrix {
  using namespace robot_params;
  return HomogenousTransformMatrix{{1.0, 1.0, L_W_HALF, 1.0},
                                   {1.0, -1.0, -L_W_HALF, 1.0},
                                   {1.0, 1.0, -L_W_HALF, -1.0},
                                   {1.0, -1.0, L_W_HALF, -1.0}} /
         WHEEL_RADIUS;
}();

inline const auto INV_JACOBI_MTX = []() -> HomogenousTransformMatrix {
  using namespace robot_params;
  return HomogenousTransformMatrix{
             {1.0, 1.0, 1.0, 1.0},
             {1.0, -1.0, 1.0, -1.0},
             {1.0 / L_W_HALF, -1.0 / L_W_HALF, -1.0 / L_W_HALF, 1.0 / L_W_HALF},
             {4.0 / WHEEL_RADIUS, 4.0 / WHEEL_RADIUS, -4.0 / WHEEL_RADIUS,
              -4.0 / WHEEL_RADIUS}} *
         WHEEL_RADIUS / 4;
}();

inline VelRF forward_transform(const VelWheel &u) { return INV_JACOBI_MTX * u; }

inline VelWheel backward_transform(const VelRF &v) { return JACOBI_MTX * v; }

template <typename FltType>
inline VelRF rotate_to_wframe(const VelRF &dpose, FltType th) {
  return HomogenousTransformMatrix{{std::cos(th), -std::sin(th), 0.0, 0.0},
                                   {std::sin(th), std::cos(th), 0.0, 0.0},
                                   {0.0, 0.0, 1.0, 0.0},
                                   {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}

template <typename FltType>
inline VelRF rotate_to_rframe(const VelRF &dpose, FltType th) {
  return HomogenousTransformMatrix{{std::cos(th), std::sin(th), 0.0, 0.0},
                                   {-std::sin(th), std::cos(th), 0.0, 0.0},
                                   {0.0, 0.0, 1.0, 0.0},
                                   {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}

}; // namespace transform
