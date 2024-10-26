#include "wheel_ctrl.hpp"

#include <mecarover/mrlogger/mrlogger.h>
#include <mecarover/mrtypes.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>

#include <algorithm>
#include <mecarover/hal/stm_hal.hpp>

#include "WheelDataWrapper.hpp"
#include "mecarover/robot_params.hpp"
#include "rcl_ret_check.hpp"
#include "rclc/executor_handle.h"

static constexpr uint8_t N_EXEC_HANDLES = 2;

static std::array<real_t, N_WHEEL> vel_to_duty_cycle(
    const std::array<real_t, N_WHEEL>& wheel_vel) {
  static constexpr uint8_t PERCENT = 100;
  auto ret = std::array<real_t, N_WHEEL>{};

  std::transform(begin(wheel_vel), end(wheel_vel), begin(ret), [](real_t elem) {
    return std::clamp(elem / MAX_VELOCITY * PERCENT, -100.0, 100.0);
  });
  return ret;
}

static std::array<real_t, N_WHEEL> vel_to_duty_cycle(const VelWheel& vel) {
  static constexpr uint8_t PERCENT = 100;
  auto ret = std::array<real_t, N_WHEEL>{};

  std::transform(vel.data(), vel.data() + N_WHEEL, begin(ret), [](real_t elem) {
    return std::clamp(elem / MAX_VELOCITY * PERCENT, -100.0, 100.0);
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

static std::array<real_t, N_WHEEL> wheel_vel_cur{};

static void enc2vel_cb(const void* arg) {
  if (!arg) return;
  log_message(log_debug, "%s: %.2f, %.2f, %.2f, %.2f",
              "[wheel ctrl - enc_data_cb]: d_enc in wheel_ctrl module: ",
              enc_msg_buf[0], enc_msg_buf[1], enc_msg_buf[2], enc_msg_buf[3]);

  // TODO: add iterators to the wrapper struct
  // FIXME: the calculated velocity is always lower than the setpoint, see log
  std::transform(enc_msg_buf.msg.data.data, enc_msg_buf.msg.data.data + N_WHEEL,
                 begin(wheel_vel_cur),
                 [dt = UROS_FREQ_SEC,
                  r = robot_params.wheel_radius](real_t encoder_delta_rad) {
                   // return encoder_delta_rad / dt;
                   return encoder_delta_rad / dt * r;
                 });
  log_message(log_debug,
              "[wheel ctrl - enc_data_cb]: vel_wheel_cur from enc: [%.2f, "
              "%.2f, %.2f, %.2f]",
              wheel_vel_cur[0], wheel_vel_cur[1], wheel_vel_cur[2],
              wheel_vel_cur[3]);
}

static void wheel_ctrl_cb(const void* arg) {
  VelWheel vel_cur = VelWheel(wheel_vel_cur.data());
  VelWheel vel_sp = VelWheel(wheel_vel_buf.msg.data.data);

  // TODO: wheel PID ctrl

  log_message(
      log_info, "%s: %.2f, %.2f, %.2f, %.2f",
      "[wheel ctrl - wheel_ctrl_cb]: vel_wheel_sp from interpolation cb: ",
      vel_sp(0), vel_sp(1), vel_sp(2), vel_sp(3));

  auto vel_corrected = vel_sp + (vel_sp - vel_cur) * 0.01;

  log_message(log_info, "%s: %.2f, %.2f, %.2f, %.2f",
              "[wheel ctrl - wheel_ctrl_cb]: vel corrected: ", vel_corrected(0),
              vel_corrected(1), vel_corrected(2), vel_corrected(3));

  auto vel_pwm = vel_to_duty_cycle(vel_corrected);
  hal_wheel_vel_set_pwm(vel_pwm);
}

rclc_executor_t* wheel_ctrl_init(rcl_node_t* node, rclc_support_t* support,
                                 const rcl_allocator_t* allocator) {
  rcl_ret_check(rclc_executor_init(&wheel_ctrl_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_encoder_data, node,
      WheelDataWrapper<real_t,
                       WheelDataType::ENC_DELTA_RAD>::get_msg_type_support(),
      "encoder_data"));
  rcl_ret_check(
      rclc_executor_add_subscription(&wheel_ctrl_exe, &sub_encoder_data,
                                     &enc_msg_buf.msg, &enc2vel_cb, ALWAYS));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_wheel_vel, node,
      WheelDataWrapper<real_t, WheelDataType::VEL_SP>::get_msg_type_support(),
      "wheel_vel"));
  rcl_ret_check(rclc_executor_add_subscription(&wheel_ctrl_exe, &sub_wheel_vel,
                                               &wheel_vel_buf.msg,
                                               &wheel_ctrl_cb, ON_NEW_DATA));

  rcl_ret_check(rclc_executor_set_trigger(&wheel_ctrl_exe,
                                          &rclc_executor_trigger_any, NULL));

  return &wheel_ctrl_exe;
}
}
