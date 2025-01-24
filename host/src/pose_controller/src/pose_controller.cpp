#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.hpp>
#include <utils/robot_params.hpp>
#include <utils/transformations.hpp>
#include <utils/utils.hpp>

using namespace rclcpp;
using namespace std::chrono;
using namespace robot_params;
using namespace tf2_ros;
using namespace transform;
using namespace utils;

using geometry_msgs::msg::Pose2D;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Float64;

using WheelState = control_msgs::msg::MecanumDriveControllerState;

class PoseController : public rclcpp::Node {
 public:
  PoseController() : rclcpp::Node{"pose_controller"} {
    cmd_vel_subscription_ = this->create_subscription<Twist>(
        "/cmd_vel", 10, [this](Twist::UniquePtr msg) {
          vel_rf_target_ = msg2vec<Twist>(*msg);
          vel_rf_target_(0) *= 1000;
          vel_rf_target_(1) *= 1000;
        });
    odom_subscription_ = this->create_subscription<Pose2D>(
        "/odom", 10, [this](Pose2D::UniquePtr msg) { odom_ = msg2vec(*msg); });
    epsilon_subscription_ = this->create_subscription<Float64>(
        "/epsilon", 10,
        [this](Float64::UniquePtr msg) { epsilon_ = msg->data; });

    robot_vel_publisher_ = this->create_publisher<Twist>("/robot_vel", 10);

    timer_ =
        this->create_wall_timer(timer_period_, [this]() { pose_control(); });
  }

 private:
  const milliseconds timer_period_ = POSE_CTRL_PERIOD_MS;
  const double dt_ = duration<double>(timer_period_).count();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<Pose2D>::SharedPtr odom_subscription_;
  rclcpp::Subscription<Float64>::SharedPtr epsilon_subscription_;
  rclcpp::Publisher<Twist>::SharedPtr robot_vel_publisher_;

  VelRF vel_rf_prev_{};
  VelRF vel_rf_target_{};
  PoseWithEpsilon odom_{};
  PoseWithEpsilon pose_sp_{};
  double epsilon_{};
  const double K_e = -0.2;

  void pose_control() {
    auto vel_rf_cur_ = velocity_smoothen();
    vel_rf_prev_ = vel_rf_cur_;

    pose_sp_ += rotate_to_wframe(vel_rf_cur_, pose_sp_(2)) * dt_;

    Vector4<double> delta_vel_wf = pid_control() / dt_;
    Vector4<double> vel_rf_corrected =
        vel_rf_cur_ + rotate_to_rframe(delta_vel_wf, odom_(2));
    vel_rf_corrected(3) = epsilon_ * K_e;

    robot_vel_publisher_->publish(vec2msg<Twist>(vel_rf_corrected));
  }

  Vector4<double> pid_control() const {
    // TODO
    return {};
  }

  Vector4<double> velocity_smoothen() const {
    // TODO tune this
    constexpr auto DELTA_MAX_LINEAR = MAX_VELOCITY_WHEEL_LINEAR * 0.03;
    constexpr auto DELTA_MAX_ANGULAR = DELTA_MAX_LINEAR / L_W_HALF;

    Vector4<double> delta_vel = vel_rf_target_ - vel_rf_prev_;

    using std::clamp;
    for (int i = 0; i < 3; ++i) {
      auto delta_max = (i < 2) ? DELTA_MAX_LINEAR : DELTA_MAX_ANGULAR;
      delta_vel(i) = clamp(delta_vel(i), -delta_max, delta_max);
    }

    return vel_rf_prev_ + delta_vel;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseController>());
  rclcpp::shutdown();
  return 0;
}
