#pragma once

#include <control_msgs/msg/mecanum_drive_controller_state.h>
#include <rcl/time.h>
#include <rmw_microros/time_sync.h>

#include <application/robot_params.hpp>
#include <cmath>
#include <cstring>

#include "wheel_state_type.hpp"

using WheelState = control_msgs__msg__MecanumDriveControllerState;

template <WheelStateType E>
struct WheelStateWrapper {
  WheelStateWrapper() {
    this->state.header.frame_id.data =
        const_cast<char*>(to_string(E).value().data());
    this->state.header.frame_id.size = to_string(E).value().size();
    auto nano_sec = rmw_uros_epoch_nanos();
    this->state.header.stamp.nanosec = nano_sec;
    this->state.header.stamp.sec = RCL_NS_TO_S(nano_sec);
  }

  WheelStateWrapper(const std::array<real_t, robot_params::N_WHEEL>& data)
      : WheelStateWrapper() {
    this->state.front_right_wheel_velocity = data[0];
    this->state.front_left_wheel_velocity = data[1];
    this->state.back_left_wheel_velocity = data[2];
    this->state.back_right_wheel_velocity = data[3];
  }

  WheelStateWrapper(const robot_params::VelRF& data) : WheelStateWrapper() {
    this->state.front_right_wheel_velocity = data(0);
    this->state.front_left_wheel_velocity = data(1);
    this->state.back_left_wheel_velocity = data(2);
    this->state.back_right_wheel_velocity = data(3);
  }

  static const rosidl_message_type_support_t* get_msg_type_support() {
    return ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg,
                                       MecanumDriveControllerState);
  }

  WheelState state{};
};
