#include "micro_ros.hpp"

#include <lwip.h>
#include <rcl/allocator.h>
#include <rcl/time.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <ulog.h>

#include "interpolation.hpp"
#include "lidar.hpp"
#include "odometry.hpp"
#include "rcl_ret_check.hpp"
#include "udp_transport.h"
#include "wheel_ctrl.hpp"

extern "C" {
void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void* state);

/* cannot be const, because the transport function requires non const */
static udp_transport_params_t TRANSPORT_PARAMS = {
    {0, 0, 0}, {MICRO_ROS_AGENT_IP}, {MICRO_ROS_AGENT_PORT}};

static auto allocator = rcutils_get_zero_initialized_allocator();
static rcl_node_t node;
static rclc_support_t support;
static rcl_init_options_t init_options;

static void init() {
  ULOG_INFO("initializing micro-ROS module");
  MX_LWIP_Init();
  vTaskDelay(pdMS_TO_TICKS(200));

  rcl_ret_check(rmw_uros_set_custom_transport(
      false,  // framing disable here. udp should use packet-oriented mode
      (void*)&TRANSPORT_PARAMS, udp_transport_open, udp_transport_close,
      udp_transport_write, udp_transport_read));

  allocator.allocate = microros_allocate;
  allocator.deallocate = microros_deallocate;
  allocator.reallocate = microros_reallocate;
  allocator.zero_allocate = microros_zero_allocate;
  init_options = rcl_get_zero_initialized_init_options();
  rcl_ret_check(rcl_init_options_init(&init_options, allocator));
  rcl_ret_check(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  /* probing the agent until success */
  while (rmw_uros_ping_agent(50, 1) != RCL_RET_OK) {
    ULOG_INFO("agent not responding yet, retrying...");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  ULOG_INFO("agent responded. continuing initialization...");
  rcl_ret_check(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                               &allocator));
  rcl_ret_check(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  ULOG_INFO("init finished; free heap: %d, free stack: %lu",
            xPortGetMinimumEverFreeHeapSize(),
            uxTaskGetStackHighWaterMark(NULL));

  /* synchronize the ROS time for getting epoch time from rmw */
  rcl_ret_softcheck(rmw_uros_sync_session(1000));
}

void micro_ros(void* arg) {
  init();

  auto* wheel_ctrl_exe = wheel_ctrl_init(&node, &support, &allocator);
  auto* odometry_exe = odometry_init(&node, &support, &allocator);
  auto* interpolation_exe = interpolation_init(&node, &support, &allocator);
  auto* lidar_exe = lidar_init(&node, &support, &allocator);

  ULOG_INFO("micro-ROS: starting executors");
  for (;;) {
    rclc_executor_spin_some(wheel_ctrl_exe, 1);
    rclc_executor_spin_some(odometry_exe, 1);
    rclc_executor_spin_some(interpolation_exe, 1);
    rclc_executor_spin_some(lidar_exe, 1);
  }
  // TODO: clean up functions from the executor modules, or even better: RAII
  // that shit
}
}
