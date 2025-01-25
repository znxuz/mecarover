#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <utils/robot_params.hpp>
#include <utils/transformations.hpp>
#include <utils/utils.hpp>

using namespace utils;
using namespace std::chrono;
using namespace rclcpp;
using namespace geometry_msgs::msg;
using namespace transform;
using namespace robot_params;

using std_msgs::msg::Float64;

class Odom : public Node {
 public:
  Odom() : Node{"odom"} {
    epsilon_publisher_ = this->create_publisher<Float64>("/epsilon", 10);
    odom_publisher_ = this->create_publisher<Pose2D>("/odom", 10);

    delta_phi_subscription_ = this->create_subscription<WheelState>(
        "/delta_phi", rclcpp::SensorDataQoS(),
        [this](WheelState::UniquePtr msg) { this->odometry(*msg); });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(timer_period_, [this]() {
      publish_odom();
      epsilon_publisher_->publish(Float64{}.set__data(epsilon_));
    });
  }

 private:
  Subscription<WheelState>::SharedPtr delta_phi_subscription_;
  Publisher<Float64>::SharedPtr epsilon_publisher_;
  Publisher<Pose2D>::SharedPtr odom_publisher_;
  // TODO try make stack allocated
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  TimerBase::SharedPtr timer_;

  Vector4<double> odom_{};
  double epsilon_{};
  milliseconds timer_period_ = POSE_CTRL_PERIOD_MS;
  double dt_ = std::chrono::duration<double>(timer_period_).count();

  /*
   * odom: encoder delta gets fed directly into the inverted jacobian
   * matrix without dividing the dt for the reason being:
   * enc delta in rad / dt = vel -> forward_transform(vel) = vel_rf * dt =
   * dpose
   * => dt gets canceled out on both sides
   */
  void odometry(const WheelState& msg) {
    auto pose_delta = forward_transform(utils::msg2vec(msg));
    pose_delta(2) = normalize_theta(pose_delta(2));
    epsilon_ += pose_delta(3);

    odom_ += rotate_to_wframe(pose_delta, odom_(2) + pose_delta(2) / 2);
  }

  void publish_odom() const {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = odom_(0);
    t.transform.translation.y = odom_(1);
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_(2));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    odom_publisher_->publish(vec2msg<Pose2D>(odom_));
    tf_broadcaster_->sendTransform(t);
  }
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(std::make_shared<Odom>());
  shutdown();
}
