#include "micro_ros.hpp"

#include <usart.h>
#include <lwip.h>
#include <rcl/allocator.h>
#include <rcl/time.h>
#include <rcl/types.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <ulog.h>

#include "pose_ctrl.hpp"
#include "lidar.hpp"
#include "odometry.hpp"
#include "rcl_guard.hpp"
#include "udp_transport.h"
#include "wheel_ctrl.hpp"

extern "C" {
bool cubemx_transport_open(struct uxrCustomTransport* transport);
bool cubemx_transport_close(struct uxrCustomTransport* transport);
// TODO: pull request to change buf to `const uint8_t*`
size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              const uint8_t* buf, size_t len, uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf,
                             size_t len, int timeout, uint8_t* err);

void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void* state);

/* cannot be const, because the transport function requires non const */
static eth_transport_params_t TRANSPORT_PARAMS = {
    {0, 0, 0}, {MICRO_ROS_AGENT_IP}, {MICRO_ROS_AGENT_PORT}};

static auto allocator = rcutils_get_zero_initialized_allocator();
static rcl_node_t node;
static rclc_support_t support;
static rcl_init_options_t init_options;

static void init() {
  ULOG_INFO("initializing micro-ROS module");

#ifdef USE_UDP_TRANSPORT
  ULOG_INFO("using UDP transport");
  MX_LWIP_Init();
  rcl_guard(rmw_uros_set_custom_transport(
      false,  // framing disable here. udp should use packet-oriented mode
      (void*)&TRANSPORT_PARAMS, eth_transport_open, eth_transport_close,
      eth_transport_write, eth_transport_read));
#else
  rcl_guard(rmw_uros_set_custom_transport(
      true, (void*)&huart3, cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read));
#endif

  allocator.allocate = microros_allocate;
  allocator.deallocate = microros_deallocate;
  allocator.reallocate = microros_reallocate;
  allocator.zero_allocate = microros_zero_allocate;
  init_options = rcl_get_zero_initialized_init_options();
  rcl_guard(rcl_init_options_init(&init_options, allocator));
  rcl_guard(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  /* probing the agent until success */
  while (rmw_uros_ping_agent(50, 1) != RCL_RET_OK) {
    ULOG_INFO("agent not responding yet, retrying...");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  ULOG_INFO("agent responded. continuing initialization...");
  rcl_guard(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                               &allocator));
  rcl_guard(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  ULOG_INFO("init finished; free heap: %d, free stack: %lu",
            xPortGetMinimumEverFreeHeapSize(),
            uxTaskGetStackHighWaterMark(NULL));

  /* synchronize the ROS time for getting epoch time from rmw */
  rcl_softguard(rmw_uros_sync_session(1000));
}

void micro_ros(void* arg) {
  init();

  auto* odometry_exe = odometry_init(&node, &support, &allocator);
  auto* wheel_ctrl_exe = wheel_ctrl_init(&node, &support, &allocator);
  auto* pose_ctrl_exe = pose_ctrl_init(&node, &support, &allocator);
  auto* lidar_exe = lidar_init(&node, &support, &allocator);

  ULOG_INFO("micro-ROS: starting executors");
  for (;;) {
    rclc_executor_spin_some(wheel_ctrl_exe, RCL_MS_TO_NS(1));
    rclc_executor_spin_some(odometry_exe, RCL_MS_TO_NS(1));
    rclc_executor_spin_some(pose_ctrl_exe, RCL_MS_TO_NS(1));
    rclc_executor_spin_some(lidar_exe, RCL_MS_TO_NS(1));
  }
}
}
