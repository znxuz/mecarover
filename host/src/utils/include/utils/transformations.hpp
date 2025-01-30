#pragma once

#include "robot_params.hpp"

/*
 * eigen pitfalls: https://eigen.tuxfamily.org/dox/TopicPitfalls.html
 */

namespace transform {

using namespace robot_params;

inline const auto JACOBI_MTX = []() -> TransformationMatrix {
  return TransformationMatrix{{1.0, 1.0, L_W_HALF, 1.0},
                              {1.0, -1.0, -L_W_HALF, 1.0},
                              {1.0, 1.0, -L_W_HALF, -1.0},
                              {1.0, -1.0, L_W_HALF, -1.0}} /
         WHEEL_RADIUS;
}();

inline const auto INV_JACOBI_MTX = []() -> TransformationMatrix {
  return TransformationMatrix{
             {1.0, 1.0, 1.0, 1.0},
             {1.0, -1.0, 1.0, -1.0},
             {1.0 / L_W_HALF, -1.0 / L_W_HALF, -1.0 / L_W_HALF, 1.0 / L_W_HALF},
             {4.0 / WHEEL_RADIUS, 4.0 / WHEEL_RADIUS, -4.0 / WHEEL_RADIUS,
              -4.0 / WHEEL_RADIUS}} *
         WHEEL_RADIUS / 4;
}();

inline VelRF forward_transform(const VelWheel &u) { return INV_JACOBI_MTX * u; }

inline VelWheel backward_transform(const VelRF &v) { return JACOBI_MTX * v; }

inline VelRF rotate_to_wframe(const VelRF &dpose, double th) {
  return TransformationMatrix{{std::cos(th), -std::sin(th), 0.0, 0.0},
                              {std::sin(th), std::cos(th), 0.0, 0.0},
                              {0.0, 0.0, 1.0, 0.0},
                              {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}

inline VelRF rotate_to_rframe(const VelRF &dpose, double th) {
  return TransformationMatrix{{std::cos(th), std::sin(th), 0.0, 0.0},
                              {-std::sin(th), std::cos(th), 0.0, 0.0},
                              {0.0, 0.0, 1.0, 0.0},
                              {0.0, 0.0, 0.0, 1.0}} *
         dpose;
}

};  // namespace transform
