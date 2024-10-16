#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/detail/laser_scan__struct.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <tf2_msgs/msg/tf_message.h>

#include <rcl/allocator.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc/timer.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

#include <lwip.h>

#include <mecarover/controls/ControllerTask.h>
#include <mecarover/hal/stm_hal.hpp>
#include <mecarover/lidar/lidar.h>
#include <mecarover/micro_ros/rcl_ret_check.hpp>
#include <mecarover/mrlogger/mrlogger.h>

#include "eth_transport.h"

using namespace imsl;
using namespace imsl::vehiclecontrol;

extern "C"
{
void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
							 void* state);
}

extern LaserScanner laser_scanner;

vehiclecontrol::ControllerTask<real_t>* controller = nullptr;

rcl_publisher_t tf_pub;
rcl_publisher_t ctrl_status_pub;
rcl_publisher_t lidar_pub;

void rplidar_scan(rcl_timer_t* timer, int64_t last_call_time)
{
	// more info at `ros2 interface show sensor_msgs/msg/LaserScan`

	// Synchronize the ROS time
	rcl_ret_softcheck(rmw_uros_sync_session(1000));
	int64_t time_ms = rmw_uros_epoch_millis();
	// int64_t time_ns = rmw_uros_epoch_nanos();

	sensor_msgs__msg__LaserScan distance[2];
	distance[0].header.stamp.sec = time_ms / 1000;
	distance[0].header.stamp.nanosec = 0; // time_ns;
	distance[0].header.frame_id.data = (char*)"scan";

	// set the angle range (start angle and end angle)
	float startAngle = 0;
	int requestedNumberOfPoints = 360;
	float endAngle = ((2 * M_PI) / requestedNumberOfPoints)
		* (requestedNumberOfPoints - 1);
	distance[0].angle_min = startAngle;
	distance[0].angle_max = endAngle;
	distance[0].angle_increment = (2 * M_PI) / requestedNumberOfPoints;
	distance[0].time_increment = 0;
	distance[0].scan_time = 0;
	distance[0].range_min = 0.015;
	distance[0].range_max = 12;

	float er[360];
	laser_scanner.invert(laser_scanner.adjustedScan, er, 360);

	distance[0].ranges.size = 360;
	distance[0].ranges.data = er;

	// output the quality
	distance[0].intensities.size = 360;
	distance[0].intensities.data = laser_scanner.quality;

	(void)last_call_time;
	if (timer != NULL) {
		rcl_ret_softcheck(rcl_publish(&lidar_pub, &distance, NULL));
	}
}

void start_Scan_cb(const void* start_Scan)
{
	const std_msgs__msg__Bool* startScan
		= (const std_msgs__msg__Bool*)start_Scan;

	if (startScan->data) {
		laser_scanner.Start();
	} else {
		laser_scanner.stop();
		log_message(log_info, "LaserScanner Stop");
	}
}

void timer_cb(rcl_timer_t* timer, int64_t last_call_time)
{
	// tf odom <-> base_link
	auto p = controller->pose_current.get();

	rcl_ret_softcheck(rmw_uros_sync_session(1000));
	int64_t time_ms = rmw_uros_epoch_millis();

	geometry_msgs__msg__TransformStamped t[2];
	t[0].header.stamp.sec = time_ms / 1000;
	t[0].header.stamp.nanosec = 0; // time_ns;
	t[0].header.frame_id.data = (char*)"/odom";
	t[0].child_frame_id.data = (char*)"/base_link";

	t[0].transform.translation.x = (double)p.x / 1000.0; // 1000.0; // mm -> m
	t[0].transform.translation.y = (double)p.y / 1000.0; // 1000.0; // mm -> m

	t[0].transform.rotation.x = (double)0;
	t[0].transform.rotation.y = (double)0;
	t[0].transform.rotation.z = (double)sin(p.theta * 0.5);
	t[0].transform.rotation.w = (double)cos(p.theta * 0.5);

	tf2_msgs__msg__TFMessage tf;
	tf.transforms.size = 1;
	tf.transforms.data = t;

	std_msgs__msg__Byte ctrl_status_msg;
	ctrl_status_msg.data = static_cast<int>(controller->ctrl_mode.get());
	if (timer != NULL) {
		rcl_ret_softcheck(
			rcl_publish(&ctrl_status_pub, &ctrl_status_msg, NULL));
		rcl_ret_softcheck(rcl_publish(&tf_pub, &tf, NULL));
	}
}

void cmd_cb(const void* cmd_msg)
{
	const auto* msg
		= reinterpret_cast<const geometry_msgs__msg__Twist*>(cmd_msg);
	log_message(log_info, "v_x: %f, v_y: %f, omega: %f", msg->linear.x,
				msg->linear.y, msg->angular.z);

	imsl::vPose<real_t> v_ref;
	v_ref.vx = msg->linear.x * 1000.0; // m/s -> mm/s
	v_ref.vy = msg->linear.y * 1000.0; // m/s -> mm/s
	v_ref.omega = msg->angular.z; // rad/s

	if (controller) [[likely]]
		controller->vel_rframe_sp.set(v_ref);
}

void enable_topic_cb(const void* enable_topic)
{
	const auto* enable
		= reinterpret_cast<const std_msgs__msg__Bool*>(enable_topic);
	if (enable->data) {
		controller->ctrl_mode.set(imsl::vehiclecontrol::CtrlMode::TWIST);
		log_message(log_info, "amplifiers enabled, set mode TWIST");
	} else {
		controller->ctrl_mode.set(vehiclecontrol::CtrlMode::OFF);
		log_message(log_info, "amplifiers disabled, set mode OFF");
	}
}

// micro-ROS configuration
void micro_ros_legacy(void* arg)
{
	MX_LWIP_Init();
	vTaskDelay(pdMS_TO_TICKS(200));

	controller = reinterpret_cast<vehiclecontrol::ControllerTask<real_t>*>(arg);

	// ethernet communication, change to the local lan ip address
	eth_transport_params_t default_params
		= {{0, 0, 0}, {"192.168.199.157"}, {"8888"}};
	rmw_uros_set_custom_transport(false, (void*)&default_params,
								  eth_transport_open, eth_transport_close,
								  eth_transport_write, eth_transport_read);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	rcl_ret_check(rcl_init_options_init(&init_options, allocator));
	size_t domain_id = 42;
	rcl_ret_check(rcl_init_options_set_domain_id(&init_options, domain_id));
	rclc_support_t support;
	rcl_ret_check(rclc_support_init_with_options(&support, 0, NULL,
												 &init_options, &allocator));

	rcl_node_t node;
	rcl_ret_check(rclc_node_init_default(&node, "test_node", "", &support));

	rcl_subscription_t sub_startScan;
	rcl_subscription_t sub_cmd_vel;
	rcl_subscription_t sub_enable;

	// inittialize publisher for LaserScanner, default (qos) Settings
	rcl_ret_check(rclc_publisher_init_default(
		&lidar_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
	rcl_ret_check(rclc_subscription_init_default(
		&sub_startScan, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"start_scan"));
	// with deafault (qos) settings
	rcl_ret_check(rclc_publisher_init_default(
		&ctrl_status_pub, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Byte), "ctrl_status"));
	rcl_ret_check(rclc_publisher_init_default(
		&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
		"tf"));
	rcl_ret_check(rclc_subscription_init_default(
		&sub_cmd_vel, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
	rcl_ret_check(rclc_subscription_init_default(
		&sub_enable, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
		"enable"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 500;
	rcl_ret_check(rclc_timer_init_default2(
		&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_cb, true));

	// Create LaserScanner Timer
	rcl_timer_t lidar_timer = rcl_get_zero_initialized_timer();
	const unsigned int lidar_timer_timeout = 500;
	rcl_ret_check(rclc_timer_init_default2(&lidar_timer, &support,
										   RCL_MS_TO_NS(lidar_timer_timeout),
										   rplidar_scan, true));

	// Create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	rcl_ret_check(
		rclc_executor_init(&executor, &support.context, 3, &allocator));

	// Create laser scanner executor
	rclc_executor_t lidar_executor
		= rclc_executor_get_zero_initialized_executor();
	rcl_ret_check(rclc_executor_init(&lidar_executor, &support.context, 2,
									 &allocator)); // 1->2

	// add participants to executor
	geometry_msgs__msg__Twist cmd_vel;
	std_msgs__msg__Bool enable_msg;
	rcl_ret_check(rclc_executor_add_timer(&executor, &timer));
	rcl_ret_check(rclc_executor_add_subscription(
		&executor, &sub_enable, &enable_msg, &enable_topic_cb, ON_NEW_DATA));
	rcl_ret_check(rclc_executor_add_subscription(
		&executor, &sub_cmd_vel, &cmd_vel, &cmd_cb, ON_NEW_DATA));

	// add participants to LaserScanner executor
	std_msgs__msg__Bool start_Scan;
	rcl_ret_check(rclc_executor_add_timer(&lidar_executor, &lidar_timer));
	rcl_ret_check(rclc_executor_add_subscription(&lidar_executor,
												 &sub_startScan, &start_Scan,
												 &start_Scan_cb, ON_NEW_DATA));

	log_message(log_info, "micro_ros agent intialized - start spinning");
	while (true) {
		rcl_ret_check(rclc_executor_spin_some(
			&executor,
			RCL_MS_TO_NS(1))); // before 1000 now 1, because microRos Task
							   // waited too long for new Task to work
		rcl_ret_check(
			rclc_executor_spin_some(&lidar_executor, RCL_MS_TO_NS(1)));
	}

	rcl_ret_check(rcl_publisher_fini(&ctrl_status_pub, &node));
	rcl_ret_check(rcl_publisher_fini(&tf_pub, &node));
	rcl_ret_check(rcl_subscription_fini(&sub_cmd_vel, &node));
	rcl_ret_check(rcl_subscription_fini(&sub_enable, &node));
	rcl_ret_check(rcl_subscription_fini(&sub_startScan, &node));
	rcl_ret_check(rcl_publisher_fini(&lidar_pub, &node));
	rcl_ret_check(rcl_node_fini(&node));
	vTaskDelete(NULL);
}
