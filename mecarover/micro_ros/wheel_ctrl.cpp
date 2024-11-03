#include "wheel_ctrl.hpp"

#include <rcl/time.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>

#include <algorithm>
#include <mecarover/hal/stm_hal.hpp>
#include <utility>

#include "WheelDataWrapper.hpp"
#include "mecarover/mrtypes.h"
#include "mecarover/robot_params.hpp"
#include "rcl_ret_check.hpp"

static constexpr uint8_t N_EXEC_HANDLES = 2;

static std::array<real_t, N_WHEEL> vel_to_duty_cycle(const VelWheel& vel) {
  static constexpr real_t PERCENT = 100.0;
  auto ret = std::array<real_t, N_WHEEL>{};

  std::transform(vel.data(), vel.data() + vel.size(), begin(ret),
                 [](real_t elem) {
                   return std::clamp(elem / MAX_VELOCITY_MM_S * PERCENT,
                                     -PERCENT, PERCENT);
                 });
  return ret;
}

extern "C" {
static auto wheel_ctrl_exe = rclc_executor_get_zero_initialized_executor();

static auto wheel_ctrl_timer = rcl_get_zero_initialized_timer();

static rcl_publisher_t pub_encoder_data;

static rcl_subscription_t sub_wheel_vel;
static auto msg_wheel_vel_sp =
    WheelDataWrapper<real_t, WheelDataType::VEL_SP>{};

static VelWheel wheel_vel_cur{};
static VelWheel wheel_vel_sp{};

static void wheel_vel_sp_cb(const void* arg) {
  const auto* msg = reinterpret_cast<const MsgType<real_t>*>(arg);
  std::copy(msg->data.data, msg->data.data + N_WHEEL, std::begin(wheel_vel_sp));
}

static VelWheel vel_pid(const real_t dt) {
  static constexpr real_t k_p = 0.40, k_i = 0.015, k_d = 0, i_gain_max = 100;
  static auto integral = VelWheel{}, prev_err = VelWheel{};

  const auto err = wheel_vel_sp - wheel_vel_cur;

  const auto proportional = k_p * err;

  integral += k_i * err * dt;
  integral = integral.unaryExpr(
      [&](real_t val) { return std::clamp(val, i_gain_max, -i_gain_max); });
  if (std::any_of(std::begin(integral), std::end(integral),
                  [](real_t val) { return val >= 0.8 * i_gain_max; })) {
    ULOG_DEBUG("[wheel_ctrl]: pid I gain: [%0.2f, %.02f, %.02f, %.02f]",
               integral(0), integral(1), integral(2), integral(3));
  }

  const auto derivative =
      k_d * (err - std::exchange(prev_err, err)) / dt;  // unused

  return wheel_vel_sp + proportional + integral + derivative;
}

static void wheel_ctrl_cb(rcl_timer_t* timer, int64_t last_call_time) {
  auto enc_delta_rad = hal_encoder_delta_rad();
  const auto dt = RCL_NS_TO_S(static_cast<real_t>(last_call_time));

  WheelDataWrapper<real_t, WheelDataType::ENC_DELTA_RAD> enc_data{
      enc_delta_rad};
  rcl_ret_softcheck(rcl_publish(&pub_encoder_data, &enc_data.msg, NULL));

  std::transform(begin(enc_delta_rad), end(enc_delta_rad),
                 std::begin(wheel_vel_cur),
                 [dt, r = robot_params.wheel_radius](real_t enc_delta) {
                   return enc_delta * r / dt;
                 });

  const auto vel_corrected = vel_pid(dt);
  hal_wheel_vel_set_pwm(vel_to_duty_cycle(vel_corrected));
}

rclc_executor_t* wheel_ctrl_init(rcl_node_t* node, rclc_support_t* support,
                                 const rcl_allocator_t* allocator) {
  rcl_ret_check(rclc_executor_init(&wheel_ctrl_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rcl_ret_check(rclc_subscription_init_default(
      &sub_wheel_vel, node,
      WheelDataWrapper<real_t, WheelDataType::VEL_SP>::get_msg_type_support(),
      "wheel_vel"));
  rcl_ret_check(rclc_executor_add_subscription(&wheel_ctrl_exe, &sub_wheel_vel,
                                               &msg_wheel_vel_sp.msg,
                                               &wheel_vel_sp_cb, ON_NEW_DATA));

  constexpr uint16_t TIMER_TIMEOUT_MS = UROS_FREQ_MOD_WHEEL_CTRL_SEC * S_TO_MS;
  rcl_ret_check(rclc_timer_init_default2(&wheel_ctrl_timer, support,
                                         RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
                                         &wheel_ctrl_cb, true));
  rcl_ret_check(rclc_executor_add_timer(&wheel_ctrl_exe, &wheel_ctrl_timer));

  rcl_ret_check(rclc_publisher_init_default(
      &pub_encoder_data, node,
      WheelDataWrapper<real_t,
                       WheelDataType::ENC_DELTA_RAD>::get_msg_type_support(),
      "encoder_data"));

  return &wheel_ctrl_exe;
}
}
