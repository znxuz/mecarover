#pragma once

#include <Eigen/Dense>
#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <numbers>

namespace utils {

using Eigen::Vector4;
using geometry_msgs::msg::Pose2D;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using std::is_same_v;
using std::numbers::pi;

using DriveState = control_msgs::msg::MecanumDriveControllerState;

template <typename T>
concept MsgType = std::is_same_v<T, Twist> || std::is_same_v<T, Pose2D> ||
                  std::is_same_v<T, DriveState>;

template <MsgType T>
inline Vector4<double> msg2vec(const T& msg) {
  if constexpr (std::is_same_v<T, Twist>)
    return Vector4<double>{msg.linear.x, msg.linear.y, msg.angular.z, 0};
  if constexpr (std::is_same_v<T, Pose2D>)
    return Vector4<double>{msg.x, msg.y, msg.theta, 0};
  if constexpr (std::is_same_v<T, DriveState>)
    return Vector4<double>{
        msg.front_right_wheel_velocity, msg.front_left_wheel_velocity,
        msg.back_left_wheel_velocity, msg.back_right_wheel_velocity};
}

template <MsgType T>
inline T vec2msg(const Vector4<double>& v) {
  if constexpr (std::is_same_v<T, Twist>)
    return Twist{}
        .set__linear(Vector3{}.set__x(v(0)).set__y(v(1)))
        .set__angular(Vector3{}.set__z(v(2)));
  if constexpr (std::is_same_v<T, Pose2D>)
    return Pose2D{}.set__x(v(0)).set__y(v(1)).set__theta(v(2));
  if constexpr (std::is_same_v<T, DriveState>)
    return DriveState{}
        .set__front_right_wheel_velocity(v(0))
        .set__front_left_wheel_velocity(v(1))
        .set__back_left_wheel_velocity(v(2))
        .set__back_right_wheel_velocity(v(3));
}

inline double normalize_theta(double theta) {
  while (theta > pi) theta -= (2 * pi);
  while (theta < -pi) theta += (2 * pi);
  return theta;
}

}  // namespace utils
