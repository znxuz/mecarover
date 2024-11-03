#include "wheel_ctrl.hpp"

#include <rclc/executor.h>
#include <rclc/subscription.h>

#include <algorithm>
#include <mecarover/hal/stm_hal.hpp>
#include <utility>

#include "WheelDataWrapper.hpp"
#include "mecarover/robot_params.hpp"
#include "rcl_ret_check.hpp"
#include "rclc/executor_handle.h"

static constexpr uint8_t N_EXEC_HANDLES = 2;

static std::array<real_t, N_WHEEL> vel_to_duty_cycle(const VelWheel& vel) {
  static constexpr double PERCENT = 100.0;
  auto ret = std::array<real_t, N_WHEEL>{};

  std::transform(vel.data(), vel.data() + N_WHEEL, begin(ret), [](real_t elem) {
    return std::clamp(elem / MAX_VELOCITY_MM_S * PERCENT, -PERCENT, PERCENT);
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

static VelWheel wheel_vel_cur{};
static VelWheel wheel_vel_sp{};

static void enc2vel_cb(const void* arg) {
  const auto* enc_msg = reinterpret_cast<const MsgType<real_t>*>(arg);

  std::transform(enc_msg->data.data, enc_msg->data.data + N_WHEEL,
                 std::begin(wheel_vel_cur),
                 [dt = UROS_FREQ_MOD_ENC_SEC, r = robot_params.wheel_radius](
                     real_t enc_delta) { return enc_delta * r / dt; });
  // ULOG_DEBUG("[wheel - enc_cb]: wheel vel from enc: [%.2f, %.2f, %.2f, %.2f]",
  //            wheel_vel_cur[0], wheel_vel_cur[1], wheel_vel_cur[2],
  //            wheel_vel_cur[3]);
}

static VelWheel vel_pid() {
  static constexpr real_t k_p = 0.6, k_i = 0.015, k_d = 0, i_gain_max = 100,
                          dt = UROS_FREQ_MOD_ENC_SEC;
  static auto i_gain = VelWheel{}, prev_err = VelWheel{};

  const auto err = wheel_vel_sp - wheel_vel_cur;

  const auto p_gain = k_p * err;

  i_gain += k_i * err * dt;
  std::for_each(std::begin(i_gain), std::end(i_gain), [](real_t& val) {
    val = std::clamp(val, i_gain_max, -i_gain_max);
  });
  if (std::any_of(std::begin(i_gain), std::end(i_gain),
                  [](real_t val) { return val >= 0.8 * i_gain_max; })) {
    ULOG_DEBUG("[wheel]: pid I gain: [%0.2f, %.02f, %.02f, %.02f]", i_gain(0),
               i_gain(1), i_gain(2), i_gain(3));
  }
  // TODO: wtf why does it jump abruptly when not moving??? bug from micro ros msg transport?

  const auto d_gain = k_d * (err - std::exchange(prev_err, err)) / dt; // unused

  return wheel_vel_sp + p_gain + i_gain + d_gain;
}

static void wheel_ctrl_cb(const void* arg) {
  if (arg) {
    const auto* msg = reinterpret_cast<const MsgType<real_t>*>(arg);
    std::copy(msg->data.data, msg->data.data + N_WHEEL,
              std::begin(wheel_vel_sp));
  }

  // ULOG_DEBUG("%s: %.2f, %.2f, %.2f, %.2f",
  //            "[wheel - wheel_ctrl_cb]: wheel_vel_sp from intrpl",
  //            wheel_vel_sp(0), wheel_vel_sp(1), wheel_vel_sp(2),
  //            wheel_vel_sp(3));

  auto vel_corrected = vel_pid();
  // ULOG_DEBUG("%s: %.2f, %.2f, %.2f, %.2f",
  //            "[wheel - wheel_ctrl_cb]: vel corrected", vel_corrected(0),
  //            vel_corrected(1), vel_corrected(2), vel_corrected(3));

  hal_wheel_vel_set_pwm(vel_to_duty_cycle(vel_corrected));
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
  rcl_ret_check(rclc_executor_add_subscription(
      &wheel_ctrl_exe, &sub_encoder_data, &enc_msg_buf.msg, &enc2vel_cb,
      ON_NEW_DATA));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_wheel_vel, node,
      WheelDataWrapper<real_t, WheelDataType::VEL_SP>::get_msg_type_support(),
      "wheel_vel"));
  /*
   * ALWAYS: let wheel ctrl callback execute at the same frequency as the
   * encoder data publisher, and also execute when a new wheel vel is available
   */
  rcl_ret_check(rclc_executor_add_subscription(&wheel_ctrl_exe, &sub_wheel_vel,
                                               &wheel_vel_buf.msg,
                                               &wheel_ctrl_cb, ALWAYS));

  rcl_ret_check(rclc_executor_set_trigger(&wheel_ctrl_exe,
                                          &rclc_executor_trigger_any, NULL));

  return &wheel_ctrl_exe;
}
}
