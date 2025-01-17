#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <utils/robot_params.hpp>
#include <utils/transformations.hpp>

using namespace std::chrono_literals;
using namespace rclcpp;
using namespace geometry_msgs::msg;

using drive_state = control_msgs::msg::MecanumDriveControllerState;
using robot_params::VelRF;
using robot_params::VelWheel;

using std::numbers::pi;

static constexpr double max_pi(double theta) {
  while (theta > pi) theta -= 2 * pi;
  while (theta < -pi) theta += 2 * pi;
  return theta;
}

class Odom : public Node {
 public:
  Odom() : Node{"odom"} {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    encoder_data_subscription_ = this->create_subscription<drive_state>(
        "/mecarover/encoder_data", rclcpp::SensorDataQoS(),
        [this](drive_state::UniquePtr msg) { this->odometry(std::move(msg)); });

    timer_ = this->create_wall_timer(timer_period_, [this]() {
      RCLCPP_INFO(this->get_logger(), "gear ratio: %f",
                  robot_params::GEAR_RATIO);
    });
  }

 private:
  Subscription<drive_state>::SharedPtr encoder_data_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  TimerBase::SharedPtr timer_;
  Pose odom{};
  std::chrono::milliseconds timer_period_ = robot_params::POSE_CTRL_PERIOD_MS;
  double dt_ = std::chrono::duration<double>(timer_period_).count();

  void odometry(drive_state::UniquePtr msg) {
    /*
     * odometry: encoder delta gets fed directly into the inverted jacobian
     * matrix without dividing the dt for the reason being:
     * enc delta in rad / dt = vel -> forward_transform(vel) = vel_rf * dt =
     * dpose
     * => dt gets canceled out on both sides
     */
    auto dpose_mtx = forward_transform(VelWheel{
        msg->front_right_wheel_velocity, msg->front_left_wheel_velocity,
        msg->back_left_wheel_velocity, msg->back_right_wheel_velocity});

    tf2::Quaternion q{};
    q.setRPY(0, 0, dpose_mtx(2));

    geometry_msgs::msg::Pose dpose;
    dpose.position.x = dpose_mtx(0);
    dpose.position.y = dpose_mtx(1);
    dpose.orientation.x = q.x();
    dpose.orientation.y = q.y();
    dpose.orientation.z = q.z();
    dpose.orientation.w = q.w();

    // TODO dpose_mtx(3) for coupling error
  }
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(std::make_shared<Odom>());
  shutdown();
}
