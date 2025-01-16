#include <chrono>
#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <utils/jacobi_transformation.hpp>
#include <utils/robot_params.hpp>

using namespace std::chrono_literals;
using namespace rclcpp;
using namespace geometry_msgs::msg;

using drive_state = control_msgs::msg::MecanumDriveControllerState;
using robot_params::VelRF;
using robot_params::VelWheel;

class Odom : public Node {
 public:
  Odom() : Node("odom") {
    encoder_data_subscription_ = this->create_subscription<drive_state>(
        "encoder-data", rclcpp::SensorDataQoS(),
        [this](drive_state::UniquePtr msg) { this->odometry(std::move(msg)); });

    timer_ = this->create_wall_timer(timer_period_, [this]() {
      RCLCPP_INFO(this->get_logger(), "gear ratio: %f",
                  robot_params::GEAR_RATIO);
    });
  }

 private:
  Subscription<drive_state>::SharedPtr encoder_data_subscription_;
  TimerBase::SharedPtr timer_;
  std::chrono::milliseconds timer_period_ = robot_params::POSE_CTRL_PERIOD_MS;
  double dt_ = std::chrono::duration<double>(timer_period_).count();
  Pose2D pose_wf{};

  void odometry(drive_state::UniquePtr msg) const {
    /*
     * odometry: encoder delta gets fed directly into the inverted jacobian
     * matrix without dividing the dt for the reason being:
     * enc delta in rad / dt = vel -> forward_transform(vel) = vel_rf * dt =
     * dpose
     * => dt gets canceled out on both sides
     */
    auto dpose_rf_mtx = forward_transform(VelWheel{
        msg->front_right_wheel_velocity, msg->front_left_wheel_velocity,
        msg->back_left_wheel_velocity, msg->back_right_wheel_velocity});

    Pose2D dpose_rf{};
    dpose_rf.set__x(dpose_rf_mtx(0));
    dpose_rf.set__y(dpose_rf_mtx(1));
    dpose_rf.set__theta(dpose_rf_mtx(2));

    // TODO
  }
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(std::make_shared<Odom>());
  shutdown();
}
