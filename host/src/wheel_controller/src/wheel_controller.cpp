#include <control_msgs/msg/mecanum_drive_controller_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utils/robot_params.hpp>
#include <utils/transformations.hpp>
#include <utils/utils.hpp>

using namespace utils;
using namespace rclcpp;
using namespace std::chrono;
using namespace robot_params;
using namespace transform;

using geometry_msgs::msg::Twist;

using DriveState = control_msgs::msg::MecanumDriveControllerState;

class WheelController : public Node {
 public:
  WheelController() : Node("wheel_controller") {
    output_wheel_vel_publisher_ =
        this->create_publisher<DriveState>("wheel_vel", 10);

    delta_phi_subscription_ = this->create_subscription<DriveState>(
        "/delta_phi", rclcpp::SensorDataQoS(),
        [this](DriveState::UniquePtr msg) {
          wheel_vel_rad_actual_ = msg2vec(*msg) / dt_;
        });

    robot_vel_subscription_ = this->create_subscription<Twist>(
        "/robot_vel", 10, [this](Twist::UniquePtr msg) {
          wheel_vel_rad_sp_ = backward_transform(msg2vec<Twist>(*msg));
        });

    timer_ =
        this->create_wall_timer(timer_period_, [this]() { wheel_control(); });
  }

 private:
  const milliseconds timer_period_ = WHEEL_CTRL_PERIOD_MS;
  const double dt_ = duration<double>(timer_period_).count();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Twist>::SharedPtr robot_vel_subscription_;
  rclcpp::Subscription<DriveState>::SharedPtr delta_phi_subscription_;
  rclcpp::Publisher<DriveState>::SharedPtr output_wheel_vel_publisher_;
  VelWheel wheel_vel_rad_actual_;
  VelWheel wheel_vel_rad_sp_;

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

    output_wheel_vel_publisher_->publish(vec2msg<DriveState>(output_vel));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelController>());
  rclcpp::shutdown();
  return 0;
}
