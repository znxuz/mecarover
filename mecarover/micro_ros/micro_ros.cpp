#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/detail/laser_scan__struct.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <tf2_msgs/msg/tf_message.h>

// micro-ros headers
#include <rcl/allocator.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc/timer.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

// nav2
#include <nav_msgs/msg/odometry.h>

#include <lwip.h>
#include <mecarover/controls/ControllerTask.h>
#include <mecarover/lidar/lidar.h>

#include <experimental/source_location>

#include "eth_transport.h"
#include "mecarover/mrlogger/mrlogger.h"
#include "rcl/subscription.h"
#include "rclc/types.h"
#include "micro_ros.h"

using namespace imsl;
using namespace imsl::vehiclecontrol;

static inline void
rcl_ret_check(rcl_ret_t ret_code,
			  const std::experimental::source_location location
			  = std::experimental::source_location::current())
{
	if (ret_code) {
		printf("Failed status on line %d: %d in %s. Aborting.\n",
			   static_cast<int>(location.line()), static_cast<int>(ret_code),
			   location.file_name());
		vTaskDelete(NULL);
	}
}

static inline void
rcl_ret_softcheck(rcl_ret_t ret_code,
				  const std::experimental::source_location location
				  = std::experimental::source_location::current())
{
	if (ret_code)
		printf("Failed status on line %d: %d in %s. Continuing.\n",
			   static_cast<int>(location.line()), static_cast<int>(ret_code),
			   location.file_name());
}

extern LaserScanner ls;

extern "C"
{
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
};

namespace imsl
{

void init()
{
	// ethernet communication, change to the local lan ip address
	rmw_uros_set_custom_transport(false, (void*)&TRANSPORT_PARAMS,
								  eth_transport_open, eth_transport_close,
								  eth_transport_write, eth_transport_read);

	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	rcl_ret_check(rcl_init_options_init(&init_options, allocator));
	rcl_ret_check(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
	rcl_ret_check(rclc_support_init_with_options(&support, 0, NULL,
												 &init_options, &allocator));

	executor = rclc_executor_get_zero_initialized_executor();
	rcl_ret_check(
		rclc_executor_init(&executor, &support.context, 3, &allocator));
}

void callback(const void* cmd_msg)
{
	log_message(log_debug, "vel received\n");
	const auto* msg
		= reinterpret_cast<const geometry_msgs__msg__Twist*>(cmd_msg);
	log_message(log_debug, "cmd_vel x: %f, y: %f, theta: %f", msg->linear.x,
				msg->linear.y, msg->angular.z);
}

void init_sub_cmd_vel(rcl_subscription_t* sub_cmd_vel, rcl_node_t* node, rclc_executor_t* executor)
{
	rcl_ret_check(rclc_subscription_init_default(
		sub_cmd_vel, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

	geometry_msgs__msg__Twist* cmd_vel_msg = new geometry_msgs__msg__Twist();
	rcl_ret_check(rclc_executor_add_subscription(
		executor, sub_cmd_vel, cmd_vel_msg, &callback, ON_NEW_DATA));
}

void micro_ros(void* arg)
{
	MX_LWIP_Init();
	init();

	rcl_node_t node;
	rcl_ret_check(rclc_node_init_default(&node, "test_node", "", &support));

	rcl_subscription_t sub_cmd_vel;
	init_sub_cmd_vel(&sub_cmd_vel, &node, &executor);

	while (true) {
		rcl_ret_check(rclc_executor_spin_some(
			&executor,
			RCL_MS_TO_NS(1))); // before 1000 now 1, because microRos Task
							   // waited too long for new Task to work
	}

	rcl_ret_check(rcl_subscription_fini(&sub_cmd_vel, &node));
	rcl_ret_check(rcl_node_fini(&node));
	vTaskDelete(NULL);
}
}
