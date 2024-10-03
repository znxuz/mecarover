#include <mecarover/controls/MecanumControllerTask.h> // FIXME: has to be before micro_ros.hpp

#include "micro_ros.hpp"

#include <lwip.h>
#include <stm32f7xx_hal.h>

// micro-ros headers
#include <rcl/allocator.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc/timer.h>
#include <rmw_microros/rmw_microros.h>

#include <experimental/source_location>

#include <mecarover/controls/ControllerTask.h>
#include <mecarover/lidar/lidar.h>
#include <mecarover/mrlogger/mrlogger.h>

#include "eth_transport.h"
#include "interpolation.hpp"
#include "odometry.hpp"
#include "rcl_ret_check.hpp"
#include "wheel_PID.hpp"

extern LaserScanner ls;

extern "C"
{
// transport params cannot be const, because the middleware will change it
static eth_transport_params_t TRANSPORT_PARAMS
	= {{0, 0, 0}, {MICRO_ROS_AGENT_IP}, {MICRO_ROS_AGENT_PORT}};

static rcl_node_t node;
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_init_options_t init_options;

void init()
{
	MX_LWIP_Init();
	HAL_Delay(1000);

	rmw_uros_set_custom_transport(false, (void*)&TRANSPORT_PARAMS,
								  eth_transport_open, eth_transport_close,
								  eth_transport_write, eth_transport_read);

	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	rcl_ret_check(rcl_init_options_init(&init_options, allocator));
	rcl_ret_check(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
	rcl_ret_check(rclc_support_init_with_options(&support, 0, NULL,
												 &init_options, &allocator));

	rclc_node_init_default(&node, "micro_ros_node", "", &support);
}

void micro_ros(void* arg)
{
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
