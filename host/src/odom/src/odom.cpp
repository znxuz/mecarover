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

using namespace std::chrono;
using namespace rclcpp;
using namespace geometry_msgs::msg;
using namespace transform;
using namespace robot_params;

using drive_state = control_msgs::msg::MecanumDriveControllerState;

class Odom : public Node {
 public:
  Odom() : Node{"odom"} {
    epsilon_publisher_ =
        this->create_publisher<std_msgs::msg::Float64>("/epsilon", 10);
    odom_publisher_ = this->create_publisher<Pose2D>("/odom", 10);

    delta_phi_subscription_ = this->create_subscription<drive_state>(
        "/delta_phi", rclcpp::SensorDataQoS(),
        [this](drive_state::UniquePtr msg) { this->odometry(std::move(msg)); });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(timer_period_, [this]() {
      this->publish_odom();
      this->publish_epsilon();
    });
  }

 private:
  Subscription<drive_state>::SharedPtr delta_phi_subscription_;
  Publisher<std_msgs::msg::Float64>::SharedPtr epsilon_publisher_;
  Publisher<Pose2D>::SharedPtr odom_publisher_;
  // TODO try make stack allocated
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  TimerBase::SharedPtr timer_;

  Pose2D odom{};
  double epsilon{};
  milliseconds timer_period_ = POSE_CTRL_PERIOD_MS;
  double dt_ = std::chrono::duration<double>(timer_period_).count();

  /*
   * odom: encoder delta gets fed directly into the inverted jacobian
   * matrix without dividing the dt for the reason being:
   * enc delta in rad / dt = vel -> forward_transform(vel) = vel_rf * dt =
   * dpose
   * => dt gets canceled out on both sides
   */
  void odometry(drive_state::UniquePtr msg) {
    auto mtx_to_pose2d = [](const VelWheel& dpose_mtx) -> Pose2D {
      Pose2D dpose{};
      dpose.set__x(dpose_mtx(0)).set__y(dpose_mtx(1)).set__theta(dpose_mtx(2));

      return dpose;
    };

    auto dpose_rf_mtx = forward_transform(
        {msg->front_right_wheel_velocity, msg->front_left_wheel_velocity,
         msg->back_left_wheel_velocity, msg->back_right_wheel_velocity});
    dpose_rf_mtx(2) = utils::normalize_theta(dpose_rf_mtx(2));
    this->epsilon = dpose_rf_mtx(3);

    // TODO
    // odom += mtx_to_pose2d(
    //     rotate_to_wframe(dpose_rf_mtx, odom.theta + dpose_rf_mtx(2) / 2));
  }

  void publish_odom() const {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = odom.x;
    t.transform.translation.y = odom.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    odom_publisher_->publish(odom);
    tf_broadcaster_->sendTransform(t);
  }

  void publish_epsilon() const {
    auto epsilon_msg = std_msgs::msg::Float64{};
    epsilon_msg.data = this->epsilon;
    epsilon_publisher_->publish(std::move(epsilon_msg));
  }
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(std::make_shared<Odom>());
  shutdown();
}
