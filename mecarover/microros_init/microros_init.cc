#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/detail/laser_scan__struct.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/byte.h>
#include <tf2_msgs/msg/tf_message.h>

// ros2 headers
#include <lwip.h>
#include <rcl/allocator.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc/timer.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

// nav2
#include <nav_msgs/msg/odometry.h>

#include <mecarover/controls/ControllerTask.h>
#include <mecarover/lidar/lidar.h>

#include "eth_transport.h"
#include "microros_init.h"

using namespace imsl;
using namespace imsl::vehiclecontrol;

#define RCCHECK(ret_err)                                              \
	{                                                                 \
		if (ret_err) {                                                \
			printf("Failed status on line %d: %d in %s. Aborting.\n", \
				__LINE__, static_cast<int>(ret_err), __FILE__);       \
			vTaskDelete(NULL);                                        \
		}                                                             \
	}
#define RCSOFTCHECK(ret_err)                                            \
	{                                                                   \
		if (ret_err)                                                    \
			printf("Failed status on line %d: %d in %s. Continuing.\n", \
				__LINE__, static_cast<int>(ret_err), __FILE__);         \
	}
#define DEG2RAD(x) ((x) * M_PI / 180.)

extern LaserScanner ls;

extern "C"
{
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
	void *state);
}

namespace imsl
{
vehiclecontrol::ControllerTask<real_t> *ct = nullptr;

rcl_publisher_t tf_pub;
rcl_publisher_t ctrl_status_pub;
rcl_publisher_t lidar_pub;

void rplidar_scan(rcl_timer_t *timer, int64_t last_call_time)
{
	/* Package the data into the correct format
	   Here is what is in the LaserScan message:
	   Header header
	   timestamp in the header is the acquisition time of the first ray in the scan.
	   in frame frame_id, angles are measured around
	   the positive Z axis (counterclockwise, if Z is up)
	   with zero angle being forward along the x axis

	   float32 angle_min        # start angle of the scan [rad]
	   float32 angle_max        # end angle of the scan [rad]
	   float32 angle_increment  # angular distance between measurements [rad]
	   float32 time_increment   # time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
	   float32 scan_time        # time between scans [seconds]
	   float32 range_min        # minimum range value [m]
	   float32 range_max        # maximum range value [m]
	   float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
	   float32[] intensities    # intensity data [device-specific units].  If your device does not provide intensities, please leave the array empty.
	   */

	// Synchronize the ROS time
	RCSOFTCHECK(rmw_uros_sync_session(1000));
	int64_t time_ms = rmw_uros_epoch_millis();
	// int64_t time_ns = rmw_uros_epoch_nanos();

	sensor_msgs__msg__LaserScan distance[2];
	distance[0].header.stamp.sec = time_ms / 1000;
	distance[0].header.stamp.nanosec = 0; // time_ns;
	distance[0].header.frame_id.data = (char *)"scan";

	// set the angle range (start angle and end angle)
	float startAngle = 0;
	int requestedNumberOfPoints = 360;
	float endAngle = ((2 * 3.14159) / requestedNumberOfPoints)
		* (requestedNumberOfPoints - 1);
	distance[0].angle_min = startAngle;
	distance[0].angle_max = endAngle;
	distance[0].angle_increment = (2 * 3.14159) / requestedNumberOfPoints;
	distance[0].time_increment = 0;
	distance[0].scan_time = 0;
	distance[0].range_min = 0.015;
	distance[0].range_max = 12;

	float er[360];
	ls.invert(ls.adjustedScan, er, 360);

	distance[0].ranges.size = 360;
	distance[0].ranges.data = er;

	// output the quality
	distance[0].intensities.size = 360;
	distance[0].intensities.data = ls.quality;

	(void)last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&lidar_pub, &distance, NULL));
	}
}

void start_Scan_cb(const void *start_Scan)
{
	const std_msgs__msg__Bool *startScan = (const std_msgs__msg__Bool *)start_Scan;

	if (startScan->data) {
		ls.Start();
	} else {
		ls.Stop();
		log_message(log_info, "LaserScanner Stop");
	}
}

void timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
	// tf odom <-> base_link
	auto p = ct->getPose();

	RCSOFTCHECK(rmw_uros_sync_session(1000));
	int64_t time_ms = rmw_uros_epoch_millis();
	// int64_t time_ns = rmw_uros_epoch_nanos();

	geometry_msgs__msg__TransformStamped t[2];
	t[0].header.stamp.sec = time_ms / 1000;
	t[0].header.stamp.nanosec = 0; // time_ns;
	t[0].header.frame_id.data = (char *)"/odom";
	t[0].child_frame_id.data = (char *)"/base_link";

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
	ctrl_status_msg.data = static_cast<int>(ct->GetControllerMode());
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&ctrl_status_pub, &ctrl_status_msg, NULL));
		RCSOFTCHECK(rcl_publish(&tf_pub, &tf, NULL));
	}
}

void cmd_cb(const void *cmd_msg)
{
	const auto *msg = reinterpret_cast<const geometry_msgs__msg__Twist *>(cmd_msg);
	log_message(log_debug,"cmd_vel x: %f, y: %f, theta: %f",
			msg->linear.x, msg->linear.y, msg->angular.z);

	imsl::vPose<real_t> v_ref;
	v_ref.vx = msg->linear.x * 1000.0; // m/s -> mm/s
	v_ref.vy = msg->linear.y * 1000.0; // m/s -> mm/s
	v_ref.omega = msg->angular.z; // rad/s

	if (ct) [[unlikely]]
		ct->SetManuRef(v_ref);
}

void enable_topic_cb(const void *enable_topic)
{
	const auto *enable = reinterpret_cast<const std_msgs__msg__Bool *>(enable_topic);
	if (enable->data) {
		ct->SetControllerMode(vehiclecontrol::CtrlMode::TWIST);
		log_message(log_info, "amplifiers enabled, set mode TWIST");
	} else {
		ct->SetControllerMode(vehiclecontrol::CtrlMode::OFF);
		log_message(log_info, "amplifiers disabled, set mode OFF");
	}
}

// micro-ROS configuration
void uros_init(void *controller)
{
	MX_LWIP_Init();

	// ethernet communication, change to the local lan ip address
	static eth_transport_params_t default_params = {
		{ 0, 0, 0 }, { "192.168.1.228" }, { "8888" }
	};
	rmw_uros_set_custom_transport(
		false,
		(void *)&default_params,
		eth_transport_open,
		eth_transport_close,
		eth_transport_write,
		eth_transport_read);

	ct = reinterpret_cast<vehiclecontrol::ControllerTask<real_t> *>(controller);

	// Create init_options
	rcl_allocator_t allocater = rcl_get_default_allocator();
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocater));

	// Setup support structure.
	rclc_support_t support;
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocater));

	// Create node for Odometry and Status
	rcl_node_t node = rcl_get_zero_initialized_node();
	rcl_node_options_t node_ops = rcl_node_get_default_options();
	node_ops.domain_id = 56;
	RCCHECK(rclc_node_init_with_options(&node, "RobotNode", "", &support, &node_ops));

	rcl_subscription_t sub_startScan;
	rcl_subscription_t sub_cmd_vel;
	rcl_subscription_t sub_enable;

	// initialize publisher and subscriber
	// with best effort variant
	// RCCHECK(rclc_publisher_init_best_effort(&ctrl_status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Byte), "ctrl_status"));
	// RCCHECK(rclc_publisher_init_best_effort(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf"));
	// RCCHECK(rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
	// RCCHECK(rclc_subscription_init_best_effort(&sub_enable, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "enable"));

	// inittialize publisher for LaserScanner, default (qos) Settings
	RCCHECK(rclc_publisher_init_default(&lidar_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
	RCCHECK(rclc_subscription_init_default(&sub_startScan, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "start_scan"));
	// with deafault (qos) settings
	RCCHECK(rclc_publisher_init_default(&ctrl_status_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Byte), "ctrl_status"));
	RCCHECK(rclc_publisher_init_default(&tf_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf"));
	RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
	RCCHECK(rclc_subscription_init_default(&sub_enable, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "enable"));

	// with qos settings
	//	rmw_qos_profile_t qos = {RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE, false};
	//	RCCHECK(rclc_publisher_init(&ctrl_status, &node, rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Byte(), "ctrl_status", &qos));
	//	RCCHECK(rclc_publisher_init(&tf_publisher, &node, rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TransformStamped(), "tf", &qos));
	//	RCCHECK(rclc_subscription_init(&sub_cmd_vel, &node, rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Twist(), "cmd_vel", &qos));
	//	RCCHECK(rclc_subscription_init(&sub_enable, &node, rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Bool(), "enable", &qos));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_cb));

	// Create LaserScanner Timer
	rcl_timer_t sens_timer = rcl_get_zero_initialized_timer();
	const unsigned int sens_timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(&sens_timer, &support, RCL_MS_TO_NS(sens_timer_timeout), rplidar_scan));

	// Create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocater));

	// Create LaserScanner executor
	rclc_executor_t sens_executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&sens_executor, &support.context, 2, &allocater)); // 1->2

	// add participants to executor
	geometry_msgs__msg__Twist cmd_vel;
	std_msgs__msg__Bool enable_msg;
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &sub_enable, &enable_msg, &enable_topic_cb, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel, &cmd_cb, ON_NEW_DATA));

	// add participants to LaserScanner executor
	std_msgs__msg__Bool start_Scan;
	RCCHECK(rclc_executor_add_timer(&sens_executor, &sens_timer));
	RCCHECK(rclc_executor_add_subscription(&sens_executor, &sub_startScan, &start_Scan, &start_Scan_cb, ON_NEW_DATA));

	while (true) {
		RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); // before 1000 now 1, because microRos Task waited too long for new Task to work
		RCCHECK(rclc_executor_spin_some(&sens_executor, RCL_MS_TO_NS(1)));
	}

	RCCHECK(rcl_publisher_fini(&ctrl_status_pub, &node));
	RCCHECK(rcl_publisher_fini(&tf_pub, &node));
	RCCHECK(rcl_subscription_fini(&sub_cmd_vel, &node));
	RCCHECK(rcl_subscription_fini(&sub_enable, &node));
	RCCHECK(rcl_subscription_fini(&sub_startScan, &node));
	RCCHECK(rcl_publisher_fini(&lidar_pub, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}
}
