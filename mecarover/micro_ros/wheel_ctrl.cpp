#include "wheel_ctrl.hpp"

#include <mecarover/mrlogger/mrlogger.h>
#include <mecarover/mrtypes.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>

#include <algorithm>
#include <mecarover/hal/stm_hal.hpp>

#include "WheelDataWrapper.hpp"
#include "rcl_ret_check.hpp"

static constexpr uint8_t N_EXEC_HANDLES = 2;

static std::array<real_t, N_WHEEL> vel_to_duty_cycle(
    const std::array<real_t, N_WHEEL>& wheel_vel) {
  auto ret = std::array<real_t, N_WHEEL>{};

  std::transform(begin(wheel_vel), end(wheel_vel), begin(ret), [](real_t elem) {
    return std::clamp(elem / MAX_VELOCITY, -1.0, 1.0);
  });
  return ret;
}

extern "C" {
static auto wheel_ctrl_exe = rclc_executor_get_zero_initialized_executor();

static rcl_subscription_t sub_encoder_data;
static auto enc_msg_buf =
    WheelDataWrapper<real_t, WheelDataType::ENC_DELTA_RAD>{};

static rcl_subscription_t sub_wheel_vel;
static auto wheel_vel_buf = WheelDataWrapper<real_t, WheelDataType::VEL_SP>{};

/* void arg points to the static msg object as given in the init function */
static void wheel_ctrl_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const MsgType<real_t>*>(arg);
  const auto* label = extract_label(*msg);

  switch (parse_wheel_data_type(label).value()) {
    case WheelDataType::ENC_DELTA_RAD:
      log_message(log_info, "%s: %.2f, %.2f, %.2f, %.2f",
                  "d_enc: ", enc_msg_buf[0], enc_msg_buf[1], enc_msg_buf[2],
                  enc_msg_buf[3]);
      break;
    case WheelDataType::VEL_SP:
      log_message(log_info, "%s: %.2f, %.2f, %.2f, %.2f",
                  "vel sp: ", wheel_vel_buf[0], wheel_vel_buf[1],
                  wheel_vel_buf[2], wheel_vel_buf[3]);
      break;
  }
}

rclc_executor_t* wheel_ctrl_init(rcl_node_t* node, rclc_support_t* support,
                                 const rcl_allocator_t* allocator) {
  rclc_executor_init(&wheel_ctrl_exe, &support->context, N_EXEC_HANDLES,
                     allocator);

  rclc_subscription_init_default(
      &sub_encoder_data, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "encoder_data");
  rcl_ret_check(rclc_executor_add_subscription(
      &wheel_ctrl_exe, &sub_encoder_data, &enc_msg_buf.msg, &wheel_ctrl_cb,
      ON_NEW_DATA));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_wheel_vel, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "wheel_vel"));
  rcl_ret_check(rclc_executor_add_subscription(&wheel_ctrl_exe, &sub_wheel_vel,
                                               &wheel_vel_buf.msg,
                                               &wheel_ctrl_cb, ON_NEW_DATA));

  return &wheel_ctrl_exe;
}
}
