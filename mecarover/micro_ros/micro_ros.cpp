#include "micro_ros.hpp"

#include <lwip.h>
#include <mecarover/lidar/lidar.h>
#include <mecarover/mrlogger/mrlogger.h>
#include <rcl/allocator.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include "encoder_data.hpp"
#include "eth_transport.h"
#include "interpolation.hpp"
#include "odometry.hpp"
#include "rcl_ret_check.hpp"
#include "wheel_ctrl.hpp"

extern LaserScanner laser_scanner;

extern "C" {
void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void* state);

/* cannot be const, because the transport function requires non const */
static eth_transport_params_t TRANSPORT_PARAMS = {
    {0, 0, 0}, {MICRO_ROS_AGENT_IP}, {MICRO_ROS_AGENT_PORT}};

static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;

static void init() {
  MX_LWIP_Init();
  vTaskDelay(pdMS_TO_TICKS(200));

  rcl_ret_check(rmw_uros_set_custom_transport(
      false,  // framing disable here. udp should use packet-oriented mode
      (void*)&TRANSPORT_PARAMS, eth_transport_open, eth_transport_close,
      eth_transport_write, eth_transport_read));

  allocator = rcutils_get_zero_initialized_allocator();
  allocator.allocate = microros_allocate;
  allocator.deallocate = microros_deallocate;
  allocator.reallocate = microros_reallocate;
  allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }

  init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_check(rcl_init_options_init(&init_options, allocator));
  rcl_ret_check(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  // vTaskDelay(pdMS_TO_TICKS(4000));  // delay for the agent to be available

  /* probing the agent until success */
  while (rmw_uros_ping_agent(50, 2) != RCL_RET_OK) {
    log_message(log_warning, "agent not responding yet, retrying...");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  log_message(log_debug, "agent responded. continuing initialization...");
  rcl_ret_check(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                               &allocator));
  rcl_ret_check(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  log_message(log_debug, "init finished; free heap: %d, free stack: %lu",
              xPortGetMinimumEverFreeHeapSize(),
              uxTaskGetStackHighWaterMark(NULL));
}

void micro_ros(void* arg) {
  init();

  auto* encoder_pub_exe = encoder_data_exe_init(&node, &support, &allocator);
  // auto* odometry_exe = odometry_init(&node, &support, &allocator);
  // auto* interpolation_exe = interpolation_init(&node, &support, &allocator);
  // auto* wheel_ctrl_exe = wheel_ctrl_init(&node, &support, &allocator);

  log_message(log_info, "micro_ros starting the loop");
  for (;;) {
    rclc_executor_spin_some(encoder_pub_exe, RCL_MS_TO_NS(10));
    // rclc_executor_spin_some(odometry_exe, RCL_MS_TO_NS(10));
    // rclc_executor_spin_some(interpolation_exe, RCL_MS_TO_NS(10));
    // rclc_executor_spin_some(wheel_ctrl_exe, RCL_MS_TO_NS(10));
  }
}
}
