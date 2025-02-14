#include <cstdio>
#include <cstring>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string_view>
#include <vel2d_frame.hpp>

#include "serial_port.hpp"

using geometry_msgs::msg::Twist;

constexpr std::string_view DEFAULT_PORT = "/dev/ttyACM0";

class Vel2dBridge : public rclcpp::Node {
 public:
  Vel2dBridge() : Node{"vel2d_bridge"} {
    twist_sub_ = create_subscription<Twist>(
        "cmd_vel", 10, [this](Twist::UniquePtr twist) {
          auto frame =
              Vel2dFrame{{twist->linear.x, twist->linear.y, twist->angular.z}};

          uart.send(frame.data());
          RCLCPP_INFO(this->get_logger(), "sending [%f, %f, %f], crc: %u",
                      frame.vel.x, frame.vel.y, frame.vel.z, frame.crc);
        });
  }

 private:
  rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
  SerialPort<VEL2D_FRAME_LEN> uart = SerialPort<VEL2D_FRAME_LEN>(DEFAULT_PORT);
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vel2dBridge>());
  rclcpp::shutdown();
  return 0;
}
