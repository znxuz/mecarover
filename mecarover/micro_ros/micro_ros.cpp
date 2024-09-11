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
#include <rcl/subscription.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc/timer.h>
#include <rclc/types.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

// nav2
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <lwip.h>
#include <mecarover/controls/ControllerTask.h>
#include <mecarover/lidar/lidar.h>
#include <mecarover/mrlogger/mrlogger.h>

#include <experimental/source_location>

#include "eth_transport.hpp"
#include "micro_ros.hpp"
#include "wheel_vel_msg.h"

using namespace imsl;
using namespace imsl::vehiclecontrol;

static inline void
rcl_ret_check(rcl_ret_t ret_code,
			  const std::experimental::source_location location
			  = std::experimental::source_location::current())
{
	if (ret_code) {
		log_message(log_info, "Failed status on line %d: %d in %s. Aborting.",
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
		log_message(log_info, "Failed status on line %d: %d in %s. Continuing.",
			   static_cast<int>(location.line()), static_cast<int>(ret_code),
			   location.file_name());
}

extern LaserScanner ls;

extern "C"
{
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;

void init()
{
	MX_LWIP_Init();
	rmw_uros_set_custom_transport(false, (void*)&TRANSPORT_PARAMS,
								  eth_transport_open, eth_transport_close,
								  eth_transport_write, eth_transport_read);

	allocator = rcl_get_default_allocator();
	init_options = rcl_get_zero_initialized_init_options();
	rcl_ret_check(rcl_init_options_init(&init_options, allocator));
	rcl_ret_check(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
	rcl_ret_check(rclc_support_init_with_options(&support, 0, NULL,
												 &init_options, &allocator));
}

rcl_publisher_t pub_wheel_vel;
void pose_callback(const void* arg)
{
	const auto* pose_msg = reinterpret_cast<const geometry_msgs__msg__Twist*>(arg);
	// log_message(log_info, "pose callback pose: [x: %.2f, y: %.2f, theta: %.2f]",
			   // pose_msg->linear.x, pose_msg->linear.y, pose_msg->angular.z);

	real_t encoder_delta[4];
	hal_encoder_read(encoder_delta);
	// log_message(log_info, "encoder values: [%.2f, %.2f, %.2f, %.2f]",
				// encoder_delta[0], encoder_delta[1], encoder_delta[2],
				// encoder_delta[3]);

	wheel_vel_msg wheel_msg;
	wheel_msg[0] = 1.3;
	wheel_msg[1] = 2.5;
	wheel_msg[2] = 3.7;
	wheel_msg[3] = 7.3;
	rcl_ret_check(rcl_publish(&pub_wheel_vel, &wheel_msg.msg, NULL));
}

rcl_publisher_t pub_odometry;
void odometry_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	geometry_msgs__msg__Twist pose_msg{};
	pose_msg.linear.x = 1;
	pose_msg.linear.y = 2;
	pose_msg.angular.z = 0.5;
	rcl_ret_check(rcl_publish(&pub_odometry, &pose_msg, NULL));
}

void wheel_vel_callback(const void* arg)
{
	const auto* wheel_vel_msg
		= reinterpret_cast<const struct wheel_vel_msg*>(arg);
	log_message(log_info,
		"wheel_vel_callback wheel velocities: [%.2f], [%.2f], [%.2f], [%.2f]",
		wheel_vel_msg->operator[](0), wheel_vel_msg->operator[](1),
		wheel_vel_msg->operator[](2), wheel_vel_msg->operator[](3));
}

void micro_ros(void* arg)
{
	init();

	rcl_node_t node;
	rclc_node_init_default(&node, "micro_ros_node", "", &support);

	auto odometry_exe = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&odometry_exe, &support.context, 1, &allocator);

	rclc_publisher_init_default(
		&pub_odometry, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "odometry");
	auto timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout),
							 odometry_callback, true);
	rclc_executor_add_timer(&odometry_exe, &timer);

	auto interpolate_exe = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(
		&interpolate_exe, &support.context, 2,
		&allocator); // TODO: number of handles is given here, extract as config

	rcl_subscription_t sub_odometry;
	rclc_subscription_init_default(
		&sub_odometry, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "odometry");
	auto odometry_msg = geometry_msgs__msg__Twist();
	rclc_executor_add_subscription(&interpolate_exe, &sub_odometry,
								   &odometry_msg, &pose_callback, ON_NEW_DATA);

	rcl_subscription_t sub_cmd_vel;
	rclc_subscription_init_default(
		&sub_cmd_vel, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
	auto cmd_vel_msg = geometry_msgs__msg__Twist();
	rclc_executor_add_subscription(&interpolate_exe, &sub_cmd_vel, &cmd_vel_msg,
								   &pose_callback, ON_NEW_DATA);

	rclc_publisher_init_default(
		&pub_wheel_vel, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
		"wheel_vel");

	rcl_subscription_t sub_wheel_vel;
	rclc_subscription_init_default(
		&sub_wheel_vel, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
		"wheel_vel");
	auto action_exe = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&action_exe, &support.context, 1, &allocator);

	wheel_vel_msg msg;
	rclc_executor_add_subscription(&action_exe, &sub_wheel_vel, &msg.msg,
								   &wheel_vel_callback, ON_NEW_DATA);

	log_message(log_info, "debug: starting the loop");
	for (;;) {
		rclc_executor_spin_some(&odometry_exe, RCL_MS_TO_NS(1));
		rclc_executor_spin_some(&interpolate_exe, RCL_MS_TO_NS(1));
		rclc_executor_spin_some(&action_exe, RCL_MS_TO_NS(1));
	}
}

}
