#pragma once

#include <application/real_t.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <numbers>

using namespace Eigen;

namespace imsl {

using std::numbers::pi;

template <typename T>
concept FltType = std::is_same_v<T, float> || std::is_same_v<T, double>;

/* Heading of a mobile robot in the range of -pi ... +pi */
template <typename FltType>
class Heading {
 private:
  FltType theta{};

  static constexpr FltType maxPI(FltType theta) {
    while (theta > static_cast<FltType>(pi))
      theta -= static_cast<FltType>(2 * pi);
    while (theta < static_cast<FltType>(-pi))
      theta += static_cast<FltType>(2 * pi);
    return theta;
  }

 public:
  Heading() = default;

  Heading(FltType theta) { this->theta = maxPI(theta); }

  Heading& operator=(const FltType val) {
    this->theta = maxPI(val);
    return *this;
  }

  Heading& operator+=(const FltType& rhs) {
    theta = maxPI(this->theta + rhs);
    return *this;
  }

  Heading& operator-=(const FltType divisor) {
    this->theta = maxPI(this->theta - divisor);
    return *this;
  }

  Heading operator+(FltType rhs) const { return this->theta + rhs; }

  Heading operator-(FltType rhs) const { return this->theta - rhs; }

  Heading operator*(FltType factor) const { return this->theta * factor; }

  operator FltType() const { return this->theta; }
};

template <typename FltType>
class Pose;

template <typename FltType>
class vPose {
 public:
  FltType vx{};
  FltType vy{};
  FltType omega{};

  vPose() = default;

  vPose(FltType vx, FltType vy, FltType omega) : vx{vx}, vy{vy}, omega{omega} {}

  explicit vPose(const Pose<FltType>& pose)
      : vx{pose.x}, vy{pose.y}, omega{pose.theta} {}

  bool operator==(const vPose<FltType>& rhs) const {
    return this->vx == rhs.vx && this->vy == rhs.vy && this->omega == rhs.omega;
  }

  vPose& operator+=(const vPose<FltType>& rhs) {
    this->vx += rhs.vx;
    this->vy += rhs.vy;
    this->omega += rhs.omega;
    return *this;
  }

  friend constexpr vPose<FltType> operator*(const vPose<FltType>& lhs,
                                            const FltType factor) {
    return {lhs.vx * factor, lhs.vy * factor, lhs.omega * factor};
  }

  friend constexpr vPose<FltType> operator+(const vPose<FltType>& lhs,
                                            const vPose<FltType>& rhs) {
    return {lhs.vx + rhs.vx, lhs.vy + rhs.vy, lhs.omega + rhs.omega};
  }

  friend constexpr vPose<FltType> operator-(const vPose<FltType>& lhs,
                                            const vPose<FltType>& rhs) {
    return {lhs.vx - rhs.vx, lhs.vy - rhs.vy, lhs.omega - rhs.omega};
  }

  friend constexpr vPose<FltType> rotate_z(const vPose<FltType>& v,
                                           FltType th) {
    return {v.vx * cos(th) - v.vy * sin(th), v.vx * sin(th) + v.vy * cos(th),
            v.omega};
  }

  /* transformation of velocities from robot frame into world frame  */
  friend constexpr vPose<FltType> vRF2vWF(const vPose<FltType>& vRF,
                                          FltType th) {
    return rotate_z(vRF, th);
  }

  /* transformation of velocities from world frame into robot frame  */
  friend constexpr vPose<FltType> vWF2vRF(const vPose<FltType>& vWF,
                                          FltType th) {
    return rotate_z(vWF, -th);
  }
};

template <typename FltType>
class Pose {
 public:
  FltType x{};
  FltType y{};
  Heading<FltType> theta{};

  Pose() = default;

  Pose(FltType x, FltType y, Heading<FltType> theta)
      : x{x}, y{y}, theta{theta} {}

  explicit Pose(const vPose<real_t>& v_pose)
      : x{v_pose.vx}, y{v_pose.vy}, theta{v_pose.omega} {}

  bool operator==(const Pose<FltType>& rhs) const {
    return this->x == rhs.x && this->y == rhs.y && this->theta == rhs.theta;
  }

  Pose& operator+=(const Pose<FltType>& rhs) {
    this->x += rhs.x;
    this->y += rhs.y;
    this->theta += rhs.theta;
    return *this;
  }

  friend constexpr Pose<FltType> operator+(const Pose<FltType>& lhs,
                                           const Pose<FltType>& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.theta + rhs.theta};
  }

  friend constexpr Pose<FltType> operator-(const Pose<FltType>& lhs,
                                           const Pose<FltType>& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.theta - rhs.theta};
  }

  friend constexpr Pose<FltType> operator*(const Pose<FltType>& lhs,
                                           const FltType factor) {
    return {lhs.x * factor, lhs.y * factor, lhs.theta * factor};
  }

  friend constexpr Pose<FltType> operator/(const Pose<FltType>& lhs,
                                           const FltType divisor) {
    if (divisor == 0.0) {
      std::perror("division by zero!");
      return {};
    }
    return {lhs.x / divisor, lhs.y / divisor, lhs.theta / divisor};
  }

  friend constexpr Pose<FltType> rotate_z(const Pose<FltType>& p, FltType th) {
    return {p.x * cos(th) - p.y * sin(th), p.x * sin(th) + p.y * cos(th),
            p.theta};
  }

  /* transformation of small movements from robot frame into world frame  */
  friend constexpr Pose<FltType> pRF2pWF(const Pose<FltType>& dpose,
                                         FltType th) {
    return rotate_z(dpose, th);
  }

  /* transformation of small movements from world frame into robot frame  */
  friend constexpr Pose<FltType> pWF2pRF(const Pose<FltType>& dpose,
                                         FltType th) {
    return rotate_z(dpose, -th);
  }
};

}  // namespace imsl
