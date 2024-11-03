#include "encoder_data.hpp"

#include <rclc/executor.h>
#include <rclc/publisher.h>
#include <rclc/timer.h>
#include <rmw_microros/time_sync.h>
#include <ulog.h>

#include <mecarover/hal/stm_hal.hpp>

#include "WheelDataWrapper.hpp"
#include "mecarover/micro_ros/WheelDataType.hpp"
#include "mecarover/micro_ros/rcl_ret_check.hpp"
#include "mecarover/robot_params.hpp"

static constexpr uint8_t N_EXEC_HANDLES = 1;
static constexpr uint16_t TIMER_TIMEOUT_MS =
    UROS_FREQ_MOD_WHEEL_CTRL_SEC * S_TO_MS;

extern "C" {
static auto encoder_pub_exe = rclc_executor_get_zero_initialized_executor();
static auto enc_data_timer = rcl_get_zero_initialized_timer();
static rcl_publisher_t pub_encoder;

static void encoder_data_cb(rcl_timer_t* timer, int64_t dt) {
  WheelDataWrapper<real_t, WheelDataType::ENC_DELTA_RAD> enc_data{
      hal_encoder_delta_rad()};

  rcl_ret_softcheck(rcl_publish(&pub_encoder, &enc_data.msg, NULL));
}

rclc_executor_t* encoder_data_exe_init(const rcl_node_t* node,
                                       rclc_support_t* support,
                                       const rcl_allocator_t* allocator) {
  rcl_ret_check(rclc_executor_init(&encoder_pub_exe, &support->context,
                                   N_EXEC_HANDLES, allocator));

  rcl_ret_check(rclc_timer_init_default2(&enc_data_timer, support,
                                         RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
                                         &encoder_data_cb, true));
  rcl_ret_check(rclc_executor_add_timer(&encoder_pub_exe, &enc_data_timer));

  rcl_ret_check(rclc_publisher_init_default(
      &pub_encoder, node,
      WheelDataWrapper<real_t,
                       WheelDataType::ENC_DELTA_RAD>::get_msg_type_support(),
      "encoder_data"));

  return &encoder_pub_exe;
}
}
