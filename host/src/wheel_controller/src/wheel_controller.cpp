#include <rclcpp/rclcpp.hpp>

#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <sensor_msgs/msg/joint_state.h>
#include <utils/robot_params.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

using drive_state = control_msgs::msg::MecanumDriveControllerState;

class WheelController : public Node {
public:
  WheelController() : Node("wheel_controller") {
    encoder_data_subscription_ = this->create_subscription<drive_state>(
        "/mecarover/encoder_data", rclcpp::SensorDataQoS(),
        [this](drive_state::UniquePtr msg) {
          get_wheel_vel_rad_actual(std::move(msg));
        });

    wheel_vel_subscription_ = this->create_subscription<drive_state>(
        "/mecarover/wheel_vel", rclcpp::SensorDataQoS(),
        [this](drive_state::UniquePtr msg) { wheel_vel_rad_sp_ = *msg.get(); });

    timer_ =
        this->create_wall_timer(timer_period_, [this]() { this->wheel_pid(); });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<drive_state>::SharedPtr encoder_data_subscription_;
  rclcpp::Subscription<drive_state>::SharedPtr wheel_vel_subscription_;
  drive_state wheel_vel_rad_actual_;
  drive_state wheel_vel_rad_sp_;
  std::chrono::milliseconds timer_period_ = robot_params::WHEEL_CTRL_PERIOD_MS;
  double dt_ = std::chrono::duration<double>(timer_period_).count();

  void wheel_pid() const {
    // TODO
  }

  void get_wheel_vel_rad_actual(drive_state::UniquePtr msg) {
    auto vel = *msg.get();
    vel.set__front_right_wheel_velocity(msg->front_right_wheel_velocity / dt_);
    vel.set__front_left_wheel_velocity(msg->front_left_wheel_velocity / dt_);
    vel.set__back_left_wheel_velocity(msg->back_left_wheel_velocity / dt_);
    vel.set__back_right_wheel_velocity(msg->back_right_wheel_velocity / dt_);

    wheel_vel_rad_actual_ = std::move(vel);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}
