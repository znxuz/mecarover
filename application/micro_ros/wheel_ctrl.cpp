#include "wheel_ctrl.hpp"

#include <rcl/time.h>
#include <rclc/executor.h>
#include <rclc/executor_handle.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <ulog.h>

#include <algorithm>
#include <application/hal/hal.hpp>
#include <application/robot_params.hpp>
#include <utility>

#include "drive_state_wrapper.hpp"
#include "rcl_guard.hpp"

using namespace robot_params;

static constexpr uint8_t N_EXEC_HANDLES = 2;
static constexpr uint16_t TIMER_TIMEOUT_MS = WHEEL_CTRL_PERIOD_S * S_TO_MS;

static std::array<real_t, N_WHEEL> vel_to_duty_cycle(const VelWheel& vel) {
  static constexpr real_t PERCENT = 100.0;
  auto ret = std::array<real_t, N_WHEEL>{};

  std::transform(vel.data(), vel.data() + vel.size(), begin(ret),
                 [](real_t val) {
                   return std::clamp(val / MAX_VELOCITY_WHEEL_ANGULAR * PERCENT,
                                     -PERCENT, PERCENT);
                 });
  return ret;
}

extern "C" {
static auto exe = rclc_executor_get_zero_initialized_executor();

static auto timer = rcl_get_zero_initialized_timer();

static rcl_publisher_t pub_encoder_data;

static rcl_subscription_t sub_wheel_vel;
static auto msg_wheel_vel_sp =
    DriveStateWrapper<DriveStateType::VEL_SP_ANGULAR>{};

static VelWheel wheel_vel_actual{};
static VelWheel wheel_vel_sp{};

static void vel_sp_cb(const void* arg) {
  if (!arg) [[likely]]
    return;

  const auto* msg = reinterpret_cast<const DriveState*>(arg);
  wheel_vel_sp(0) = msg->front_right_wheel_velocity;
  wheel_vel_sp(1) = msg->front_left_wheel_velocity;
  wheel_vel_sp(2) = msg->back_left_wheel_velocity;
  wheel_vel_sp(3) = msg->back_right_wheel_velocity;
}

static VelWheel pid_ctrl(const real_t dt) {
  static constexpr real_t K_P = 0.025, K_I = 0.015, K_D = 0, MAX_INTEGRAL = 10;
  static auto integral = VelWheel{}, prev_err = VelWheel{};

  const auto err = wheel_vel_sp - wheel_vel_actual;
  // ULOG_WARNING("[wheel_ctrl]: wheel vel err: [%0.2f, %.02f, %.02f, %.02f]",
  //              err(0), err(1), err(2), err(3));

  integral += err * dt;
  integral = integral.unaryExpr(
      [&](real_t val) { return std::clamp(val, MAX_INTEGRAL, -MAX_INTEGRAL); });
  if (std::any_of(std::begin(integral), std::end(integral),
                  [](real_t val) { return val >= 0.8 * MAX_INTEGRAL; }))
    ULOG_WARNING("[wheel_ctrl]: PID integral: [%0.2f, %.02f, %.02f, %.02f]",
                 integral(0), integral(1), integral(2), integral(3));

  const auto derivative = (err - std::exchange(prev_err, err)) / dt;  // unused

  return wheel_vel_sp + K_P * err + K_I * integral + K_D * derivative;
}

static void wheel_ctrl_cb(rcl_timer_t* timer, int64_t last_call_time) {
  const auto dt = RCL_NS_TO_S(static_cast<real_t>(last_call_time));
  const auto enc_delta_rad = hal_encoder_delta_rad();

  DriveStateWrapper<DriveStateType::ENC_DELTA_RAD> enc_data{enc_delta_rad};
  rcl_softguard(rcl_publish(&pub_encoder_data, &enc_data.state, NULL));

  std::transform(begin(enc_delta_rad), end(enc_delta_rad),
                 std::begin(wheel_vel_actual),
                 [dt](real_t enc_delta) { return enc_delta / dt; });

  hal_wheel_vel_set_pwm(vel_to_duty_cycle(pid_ctrl(dt)));
}

rclc_executor_t* wheel_ctrl_init(rcl_node_t* node, rclc_support_t* support,
                                 const rcl_allocator_t* allocator) {
  hal_init();

  rcl_guard(
      rclc_executor_init(&exe, &support->context, N_EXEC_HANDLES, allocator));

  rcl_guard(rclc_subscription_init_best_effort(
      &sub_wheel_vel, node,
      DriveStateWrapper<DriveStateType::VEL_SP_ANGULAR>::get_msg_type_support(),
      "wheel_vel"));
  rcl_guard(rclc_executor_add_subscription(
      &exe, &sub_wheel_vel, &msg_wheel_vel_sp.state, &vel_sp_cb, ALWAYS));

  rcl_guard(rclc_timer_init_default2(
      &timer, support, RCL_MS_TO_NS(TIMER_TIMEOUT_MS), &wheel_ctrl_cb, true));
  rcl_guard(rclc_executor_add_timer(&exe, &timer));

  rcl_guard(rclc_publisher_init_best_effort(
      &pub_encoder_data, node,
      DriveStateWrapper<DriveStateType::ENC_DELTA_RAD>::get_msg_type_support(),
      "encoder_data"));

  rclc_executor_set_trigger(&exe, rclc_executor_trigger_one, &timer);
  rclc_executor_set_semantics(&exe, RCLC_SEMANTICS_LOGICAL_EXECUTION_TIME);

  return &exe;
}
}
