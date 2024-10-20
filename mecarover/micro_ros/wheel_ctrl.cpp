#include "wheel_ctrl.hpp"

#include <mecarover/mrlogger/mrlogger.h>
#include <mecarover/mrtypes.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>

#include <mecarover/hal/stm_hal.hpp>

#include "WheelDataWrapper.hpp"

static constexpr uint8_t N_EXEC_HANDLES = 1;
// static constexpr uint16_t TIMER_TIMEOUT_MS = 15;

extern "C" {
static auto wheel_ctrl_exe = rclc_executor_get_zero_initialized_executor();
static auto encoder_delta_timer = rcl_get_zero_initialized_timer();
static auto wheel_enc_deltas = WheelDataWrapper<real_t>{};
static auto msg = WheelDataWrapper<real_t>{};
static rcl_subscription_t sub_wheel_vel;

// static void publish_encoder_delta(rcl_timer_t* timer, int64_t) {
//   auto d_enc = hal_encoder_delta_rad();
// }

static void wheel_vel_callback(const void* arg) {
  /* arg points to the static msg object as given in the init function */
  log_message(log_debug,
              "wheel_vel_cb wheel velocities: [%.2f], [%.2f], [%.2f], [%.2f]",
              msg[0], msg[1], msg[2], msg[3]);
}

rclc_executor_t* wheel_ctrl_init(rcl_node_t* node, rclc_support_t* support,
                                 const rcl_allocator_t* allocator) {
  rclc_executor_init(&wheel_ctrl_exe, &support->context, N_EXEC_HANDLES,
                     allocator);

  // rclc_timer_init_default2(&encoder_delta_timer, support,
  //                          RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
  //                          publish_encoder_delta, true);
  // rclc_executor_add_timer(&wheel_ctrl_exe, &encoder_delta_timer);

  rclc_subscription_init_default(&sub_wheel_vel, node,
                                 msg.get_msg_type_support(), "wheel_vel");

  rclc_executor_add_subscription(&wheel_ctrl_exe, &sub_wheel_vel, &msg.msg,
                                 &wheel_vel_callback, ON_NEW_DATA);

  return &wheel_ctrl_exe;
}
}
