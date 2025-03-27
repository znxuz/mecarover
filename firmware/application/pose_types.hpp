#pragma once

#include <cmath>
#include <numbers>

namespace imsl {

using std::numbers::pi;

/* Heading of a mobile robot in the range of -pi ... +pi */
class Heading {
 private:
  double theta{};

  static constexpr double maxPI(double theta) {
    while (theta > pi) theta -= 2 * pi;
    while (theta < -pi) theta += 2 * pi;
    return theta;
  }

 public:
  constexpr Heading() = default;

  constexpr Heading(double theta) { this->theta = maxPI(theta); }

  constexpr Heading& operator=(const double val) {
    this->theta = maxPI(val);
    return *this;
  }

  constexpr Heading& operator+=(const double& rhs) {
    theta = maxPI(this->theta + rhs);
    return *this;
  }

  constexpr Heading& operator-=(const double divisor) {
    this->theta = maxPI(this->theta - divisor);
    return *this;
  }

  constexpr Heading operator+(double rhs) const { return this->theta + rhs; }

  constexpr Heading operator-(double rhs) const { return this->theta - rhs; }

  constexpr Heading operator*(double factor) const {
    return this->theta * factor;
  }

  constexpr operator double() const { return this->theta; }
};

class Pose;

class vPose {
 public:
  double vx{};
  double vy{};
  double omega{};

  constexpr vPose() = default;

  constexpr vPose(double vx, double vy, double omega)
      : vx{vx}, vy{vy}, omega{omega} {}

  explicit vPose(const Pose& pose);

  constexpr bool operator==(const vPose& rhs) const {
    return this->vx == rhs.vx && this->vy == rhs.vy && this->omega == rhs.omega;
  }

  constexpr vPose& operator+=(const vPose& rhs) {
    this->vx += rhs.vx;
    this->vy += rhs.vy;
    this->omega += rhs.omega;
    return *this;
  }

  friend constexpr vPose operator*(const vPose& lhs, const double factor) {
    return {lhs.vx * factor, lhs.vy * factor, lhs.omega * factor};
  }

  friend constexpr vPose operator+(const vPose& lhs, const vPose& rhs) {
    return {lhs.vx + rhs.vx, lhs.vy + rhs.vy, lhs.omega + rhs.omega};
  }

  friend constexpr vPose operator-(const vPose& lhs, const vPose& rhs) {
    return {lhs.vx - rhs.vx, lhs.vy - rhs.vy, lhs.omega - rhs.omega};
  }

  /* transformation of velocities from robot frame into world frame  */
  friend constexpr vPose vRF2vWF(vPose vRF, double theta) {
    return {vRF.vx * cos(theta) - vRF.vy * sin(theta),
            vRF.vx * sin(theta) + vRF.vy * cos(theta), vRF.omega};
  }

  /* transformation of velocities from world frame into robot frame  */
  friend constexpr vPose vWF2vRF(vPose vWF, double theta) {
    return {vWF.vx * cos(theta) + vWF.vy * sin(theta),
            -vWF.vx * sin(theta) + vWF.vy * cos(theta), vWF.omega};
  }
};

class Pose {
 public:
  double x{};
  double y{};
  Heading theta{};

  constexpr Pose() = default;

  constexpr Pose(double x, double y, Heading theta)
      : x{x}, y{y}, theta{theta} {}

  explicit Pose(const vPose& v_pose)
      : x{v_pose.vx}, y{v_pose.vy}, theta{v_pose.omega} {}

  constexpr bool operator==(const Pose& rhs) const {
    return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta;
  }

  constexpr Pose& operator+=(const Pose& rhs) {
    this->x += rhs.x;
    this->y += rhs.y;
    this->theta += rhs.theta;
    return *this;
  }

  friend constexpr Pose operator+(const Pose& lhs, const Pose& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.theta + rhs.theta};
  }

  friend constexpr Pose operator-(const Pose& lhs, const Pose& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.theta - rhs.theta};
  }

  friend constexpr Pose operator*(const Pose& lhs, const double factor) {
    return {lhs.x * factor, lhs.y * factor, lhs.theta * factor};
  }

  friend constexpr Pose operator/(const Pose& lhs, const double divisor) {
    if (divisor == 0.0)
      for (;;);
    return {lhs.x / divisor, lhs.y / divisor, lhs.theta / divisor};
  }

  /* transformation of small movements from robot frame into world frame  */
  friend constexpr Pose pRF2pWF(const Pose& dpose, double th) {
    return {dpose.x * cos(th) - dpose.y * sin(th),
            dpose.x * sin(th) + dpose.y * cos(th), dpose.theta};
  }

  /* transformation of small movements from world frame into robot frame  */
  friend constexpr Pose pWF2pRF(const Pose& dpose, double th) {
    return {dpose.x * cos(th) + dpose.y * sin(th),
            -dpose.x * sin(th) + dpose.y * cos(th), dpose.theta};
  }
};

inline vPose::vPose(const Pose& pose)
    : vx{pose.x}, vy{pose.y}, omega{pose.theta} {}

}  // namespace imsl
