#include "micro_ros.hpp"

#include <lwip.h>

// micro-ros headers
#include <mecarover/lidar/lidar.h>
#include <rcl/allocator.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include "eth_transport.h"
#include "interpolation.hpp"
#include "odometry.hpp"
#include "rcl_ret_check.hpp"
#include "wheel_PID.hpp"

extern LaserScanner laser_scanner;

extern "C" {
/* cannot be const, because the transport function requires non const */
static eth_transport_params_t TRANSPORT_PARAMS = {
    {0, 0, 0}, {MICRO_ROS_AGENT_IP}, {MICRO_ROS_AGENT_PORT}};

static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;

void init() {
  MX_LWIP_Init();
  vTaskDelay(pdMS_TO_TICKS(200));

  rcl_ret_check(rmw_uros_set_custom_transport(
      false, (void*)&TRANSPORT_PARAMS, eth_transport_open, eth_transport_close,
      eth_transport_write, eth_transport_read));

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_check(rcl_init_options_init(&init_options, allocator));
  rcl_ret_check(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  vTaskDelay(pdMS_TO_TICKS(5000)); // delay for the agent to be available

  /* probing the agent until success or timeout */
  for (uint8_t cnt = 0, max_cnt = 10; cnt < max_cnt; ++cnt) {
    if (rmw_uros_ping_agent(50, 2) == RCL_RET_OK) break;
    rcl_ret_check(cnt + 1 == max_cnt);
    log_message(log_warning, "agent not responding, retrying...");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  rcl_ret_check(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                               &allocator));
  log_message(log_debug, "free heap: %d, free stack: %lu",
              xPortGetMinimumEverFreeHeapSize(),
              uxTaskGetStackHighWaterMark(NULL));

  rcl_ret_check(rclc_node_init_default(&node, "micro_ros_node", "", &support));
}

void micro_ros(void* arg) {
  init();

  auto* odometry_exe = odometry_init(&node, &support, &allocator);
  auto* interpolation_exe = interpolation_init(&node, &support, &allocator);
  auto* wheel_ctrl_exe = wheel_ctrl_init(&node, &support, &allocator);

  log_message(log_info, "micro_ros starting the loop");
  for (;;) {
    rclc_executor_spin_some(odometry_exe, RCL_MS_TO_NS(1));
    rclc_executor_spin_some(interpolation_exe, RCL_MS_TO_NS(1));
    rclc_executor_spin_some(wheel_ctrl_exe, RCL_MS_TO_NS(1));
  }
}
}
