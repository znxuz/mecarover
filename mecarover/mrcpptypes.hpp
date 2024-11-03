#pragma once

#include <mecarover/mrtypes.h>

#include <cmath>
#include <cstdio>
#include <limits>

namespace imsl {

constexpr static inline real_t inf = std::numeric_limits<real_t>::infinity();
constexpr static inline real_t eps = std::numeric_limits<real_t>::epsilon();

template <typename T>
concept FltType = std::is_same_v<T, float> || std::is_same_v<T, double>;

/* Heading of a mobile robot in the range of -pi ... +pi */
template <typename FltType>
class Heading {
 private:
  FltType theta{};

  static constexpr FltType maxPI(FltType theta) {
    while (theta > static_cast<FltType>(M_PI))
      theta -= static_cast<FltType>(2 * M_PI);
    while (theta < static_cast<FltType>(-M_PI))
      theta += static_cast<FltType>(2 * M_PI);
    return theta;
  }

 public:
  Heading() = default;

  Heading(FltType theta) { this->theta = maxPI(theta); }

  Heading& operator=(const FltType& th) {
    this->theta = maxPI(th);
    return *this;
  }

  Heading operator+(FltType rhs) const {
    Heading t = maxPI(this->theta + rhs);
    return t;
  }

  Heading& operator+=(FltType rhs) {
    theta = maxPI(this->theta + rhs);
    return *this;
  }

  Heading operator-(FltType rhs) const {
    Heading t = maxPI(this->theta - rhs);
    return t;
  }

  Heading& operator-=(FltType divisor) {
    this->theta = maxPI(this->theta - divisor);
    return *this;
  }

  Heading& operator*(FltType factor) {
    this->theta = maxPI(this->theta * factor);
    return *this;
  }

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

  friend vPose<FltType> operator*(const vPose<FltType>& lhs,
                                  const FltType factor) {
    return vPose<FltType>{lhs.vx * factor, lhs.vy * factor, lhs.omega * factor};
  }

  friend vPose<FltType> operator+(const vPose<FltType>& lhs,
                                  const vPose<FltType>& rhs) {
    return vPose<FltType>{lhs.vx + rhs.vx, lhs.vy + rhs.vy,
                          lhs.omega + rhs.omega};
  }

  friend vPose<FltType> operator-(const vPose<FltType>& lhs,
                                  const vPose<FltType>& rhs) {
    return vPose<FltType>{lhs.vx - rhs.vx, lhs.vy - rhs.vy,
                          lhs.omega - rhs.omega};
  }

  /* transformation of velocities from robot frame into world frame  */
  friend vPose<FltType> vRF2vWF(vPose<FltType> vRF, FltType theta) {
    return vPose<FltType>{vRF.vx * cos(theta) - vRF.vy * sin(theta),
                          vRF.vx * sin(theta) + vRF.vy * cos(theta), vRF.omega};
  }

  /* transformation of velocities from world frame into robot frame  */
  friend vPose<FltType> vWF2vRF(vPose<FltType> vWF, FltType theta) {
    return vPose<FltType>{vWF.vx * cos(theta) + vWF.vy * sin(theta),
                          -vWF.vx * sin(theta) + vWF.vy * cos(theta),
                          vWF.omega};
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

  friend Pose<FltType> operator+(const Pose<FltType>& lhs,
                                 const Pose<FltType>& rhs) {
    return Pose<FltType>{lhs.x + rhs.x, lhs.y + rhs.y, lhs.theta + rhs.theta};
  }

  friend Pose<FltType> operator-(const Pose<FltType>& lhs,
                                 const Pose<FltType>& rhs) {
    return Pose<FltType>{lhs.x - rhs.x, lhs.y - rhs.y, lhs.theta - rhs.theta};
  }

  friend Pose<FltType> operator/(const Pose<FltType>& p1,
                                 const FltType divisor) {
    if (divisor == 0.0) {
      std::perror("division by zero!");
      return {};
    }
    return Pose<FltType>{p1.x / divisor, p1.y / divisor, p1.theta / divisor};
  }

  /* transformation of small movements from robot frame into world frame  */
  friend Pose<FltType> pRF2pWF(const Pose<FltType>& pose, FltType th) {
    return Pose<FltType>{pose.x * cos(th) - pose.y * sin(th),
                         pose.x * sin(th) + pose.y * cos(th), pose.theta};
  }

  /* transformation of small movements from world frame into robot frame  */
  friend Pose<FltType> pWF2pRF(const Pose<FltType>& d_pose, FltType th) {
    return Pose<FltType>{d_pose.x * cos(th) + d_pose.y * sin(th),
                         -d_pose.x * sin(th) + d_pose.y * cos(th),
                         d_pose.theta};
  }
};

}  // namespace imsl
