#include "main_exec.hpp"

#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>

#include <application/hal/hal.hpp>
#include <application/robot_params.hpp>

#include "rcl_guard.hpp"
#include "wheel_state_type.hpp"
#include "wheel_state_wrapper.hpp"

static constexpr uint8_t N_EXEC_HANDLES = 2;
static constexpr real_t DELTA_PHI_PERIOD_S = 0.01;

using namespace robot_params;

extern "C" {
static rclc_executor_t exe;

static auto delta_phi_timer = rcl_get_zero_initialized_timer();
static rcl_publisher_t delta_phi_pub;

static rcl_subscription_t wheel_vel_sub;

static WheelStateWrapper<WheelStateType::VEL_RAD> wheel_vel_msg;

static std::array<real_t, N_WHEEL> vel_to_duty_cycle(const WheelState* msg) {
  static constexpr real_t PERCENT = 100.0;
  auto ret = std::array<real_t, N_WHEEL>{
      msg->front_right_wheel_velocity, msg->front_left_wheel_velocity,
      msg->back_left_wheel_velocity, msg->back_right_wheel_velocity};

  std::for_each(begin(ret), end(ret), [](real_t& elem) {
    elem = std::clamp(elem / MAX_VELOCITY_WHEEL_ANGULAR * PERCENT, -PERCENT,
                      PERCENT);
  });

  return ret;
}

static void set_wheel_vel_cb(const void* msg) {
  hal_wheel_set_pwm(
      vel_to_duty_cycle(reinterpret_cast<const WheelState*>(msg)));
}

static void pub_delta_phi_cb(rcl_timer_t*, int64_t) {
  auto delta_phi = hal_wheel_delta_phi();
  auto msg = WheelStateWrapper<WheelStateType::DELTA_PHI>{delta_phi};

  rcl_guard(rcl_publish(&delta_phi_pub, &msg.state, NULL));
}

rclc_executor_t* main_exec_init(rcl_node_t* node, rclc_support_t* support,
                                const rcl_allocator_t* allocator) {
  hal_init();

  rcl_softguard(
      rclc_executor_init(&exe, &support->context, N_EXEC_HANDLES, allocator));

  rcl_softguard(rclc_subscription_init_default(
      &wheel_vel_sub, node,
      WheelStateWrapper<WheelStateType::VEL_RAD>::get_msg_type_support(),
      "wheel_vel"));
  rcl_softguard(rclc_executor_add_subscription(&exe, &wheel_vel_sub,
                                               &wheel_vel_msg.state,
                                               &set_wheel_vel_cb, ON_NEW_DATA));

  rcl_softguard(rclc_publisher_init_default(
      &delta_phi_pub, node,
      WheelStateWrapper<WheelStateType::DELTA_PHI>::get_msg_type_support(),
      "delta_phi"));
  rcl_softguard(rclc_timer_init_default2(&delta_phi_timer, support,
                                         RCL_S_TO_NS(DELTA_PHI_PERIOD_S),
                                         &pub_delta_phi_cb, true));
  rcl_softguard(rclc_executor_add_timer(&exe, &delta_phi_timer));

  return &exe;
}
}
