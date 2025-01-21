#pragma once

#include <numbers>

namespace utils {

using std::numbers::pi;

inline double normalize_theta(double theta) {
  while (theta > pi) theta -= (2 * pi);
  while (theta < -pi) theta += (2 * pi);
  return theta;
}

}
