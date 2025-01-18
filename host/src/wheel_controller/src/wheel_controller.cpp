#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utils/robot_params.hpp>

using namespace rclcpp;
using namespace std::chrono;
using namespace robot_params;

using drive_state = control_msgs::msg::MecanumDriveControllerState;

class WheelController : public Node {
 public:
  WheelController() : Node("wheel_controller") {
    output_wheel_vel_publisher_ =
        this->create_publisher<drive_state>("topic", 10);

    delta_phi_subscription_ = this->create_subscription<drive_state>(
        "/delta_phi", rclcpp::SensorDataQoS(),
        [this](drive_state::UniquePtr msg) {
          wheel_vel_rad_actual_ = msg_to_mtx(*msg) / dt_;
        });

    wheel_vel_subscription_ = this->create_subscription<drive_state>(
        "/wheel_vel", 10, [this](drive_state::UniquePtr msg) {
          wheel_vel_rad_sp_ = msg_to_mtx(*msg);
        });

    timer_ =
        this->create_wall_timer(timer_period_, [this]() { wheel_control(); });
  }

 private:
  const milliseconds timer_period_ = WHEEL_CTRL_PERIOD_MS;
  const double dt_ = duration<double>(timer_period_).count();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<drive_state>::SharedPtr delta_phi_subscription_;
  rclcpp::Subscription<drive_state>::SharedPtr wheel_vel_subscription_;
  rclcpp::Publisher<drive_state>::SharedPtr output_wheel_vel_publisher_;
  VelWheel wheel_vel_rad_actual_;
  VelWheel wheel_vel_rad_sp_;

  VelWheel msg_to_mtx(const drive_state &msg) const {
    return {msg.front_right_wheel_velocity, msg.front_left_wheel_velocity,
            msg.back_left_wheel_velocity, msg.back_right_wheel_velocity};
  }

  VelWheel pid_control() const {
    // TODO tune with param server
    static constexpr double K_P = 0, K_I = 0, K_D = 0, INTEGRAL_MAX = 10;
    static auto integral = VelWheel{}, prev_err = VelWheel{};

    const auto err = wheel_vel_rad_sp_ - wheel_vel_rad_actual_;

    integral += err * dt_;
    integral = integral.unaryExpr([&](double val) {
      return std::clamp(val, INTEGRAL_MAX, -INTEGRAL_MAX);
    });
    if (std::any_of(std::begin(integral), std::end(integral),
                    [](double val) { return val >= 0.8 * INTEGRAL_MAX; })) {
      RCLCPP_WARN(this->get_logger(),
                  "[wheel_ctrl]: PID integral: [%0.2f, %.02f, %.02f, %.02f]",
                  integral(0), integral(1), integral(2), integral(3));
    }

    const auto derivative =
        (err - std::exchange(prev_err, err)) / dt_;  // unused

    return wheel_vel_rad_sp_ + K_P * err + K_I * integral + K_D * derivative;
  }

  void wheel_control() const {
    auto output_vel = pid_control();

    drive_state msg{};
    output_wheel_vel_publisher_->publish(
        std::move(msg.set__front_right_wheel_velocity(output_vel(0))
                      .set__front_left_wheel_velocity(output_vel(1))
                      .set__back_left_wheel_velocity(output_vel(2))
                      .set__back_right_wheel_velocity(output_vel(3))));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}
