#include "ros_interface.h" // declarations (API) of the component
//#include <../../build/config/sdkconfig.h>
#include "ros_debug.h" //ros Los message
#include "hal.h"

// ros2 messages
#include <std_msgs/msg/bool.h>          // for enable topic
#include <std_msgs/msg/byte.h>          // for ctrl_status
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>

#include <tf2_msgs/msg/tf_message.h>

// ros2 headers
#include <rcl/rcl.h>
#include <rcl/allocator.h>
#include <rclc/rclc.h>
#include <rclc/timer.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

#include "usart.h"
#include <main.h> //gnetif
#include "retarget.h"

#include "eth_transport.h"
//#include "connection.h"
#include <lwip.h>
#include "api.h"

#include "sensor_msgs/msg/detail/laser_scan__struct.h"
#include <sensor_msgs/msg/laser_scan.h>
#include "LaserScanner.h"
#include <math.h>

//nav2
#include <nav_msgs/msg/odometry.h>

using namespace imsl;
using namespace imsl::vehiclecontrol;

#define RCCHECK(fn)                                                                \
  {                                                                                \
    rcl_ret_t temp_rc = fn;                                                        \
    if ((temp_rc != RCL_RET_OK))                                                   \
    {                                                                              \
      printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      vTaskDelete(NULL);                                                           \
    }                                                                              \
  }
#define RCSOFTCHECK(fn)                                                              \
  {                                                                                  \
    rcl_ret_t temp_rc = fn;                                                          \
    if ((temp_rc != RCL_RET_OK)){                                                    \
      printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
    }                                                                                \
  }

#define DEG2RAD(x) ((x)*M_PI/180.)

#ifdef __cplusplus
extern "C" {
#endif

namespace imsl {

//Ethernet Communication
static eth_transport_params_t default_params = { { 0, 0, 0 }, { "192.168.1.228" }, { "8888" } }; //{"192.168.20.123"}, {"8888"}};


//Usart Communication
//bool cubemx_transport_open(struct uxrCustomTransport * transport);
//bool cubemx_transport_close(struct uxrCustomTransport * transport);
//size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
//size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
		void *state);

// local objects
// Odometry, Status
vehiclecontrol::ControllerTasksInterfaces<real_t> *ct = nullptr;
char buf[100];

tf2_msgs__msg__TFMessage tf;
geometry_msgs__msg__TransformStamped t[2];
geometry_msgs__msg__Twist cmd_vel;
std_msgs__msg__Byte ctrl_status_msg;
std_msgs__msg__Bool enable_msg;

rcl_node_t node;
rcl_publisher_t tf_publisher;
rcl_publisher_t ctrl_status;
rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_enable;
rclc_executor_t executor;

//Laser Scanner
extern LaserScanner ls;
rcl_publisher_t laserscanner;
rclc_executor_t sens_executor;
rcl_subscription_t sub_startScan;
std_msgs__msg__Bool start_Scan;
sensor_msgs__msg__LaserScan distance[2];

int requestedNumberOfPoints = 360;
float er[360];


void rplidar_scan(rcl_timer_t *timer, int64_t last_call_time) {

        /* Package the data into the correct format
            Here is what is in the LaserScan message:
            Header header            # timestamp in the header is the acquisition time of
                                     # the first ray in the scan.
                                     #
                                     # in frame frame_id, angles are measured around
                                     # the positive Z axis (counterclockwise, if Z is up)
                                     # with zero angle being forward along the x axis

            float32 angle_min        # start angle of the scan [rad]
            float32 angle_max        # end angle of the scan [rad]
            float32 angle_increment  # angular distance between measurements [rad]
            float32 time_increment   # time between measurements [seconds] - if your scanner
                                     # is moving, this will be used in interpolating position
                                     # of 3d points
            float32 scan_time        # time between scans [seconds]
            float32 range_min        # minimum range value [m]
            float32 range_max        # maximum range value [m]
            float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
            float32[] intensities    # intensity data [device-specific units].  If your
                                     # device does not provide intensities, please leave
                                     # the array empty.
        */

	//Synchronize the ROS time
	RCSOFTCHECK(rmw_uros_sync_session(1000));
	int64_t time_ms = rmw_uros_epoch_millis();
	int64_t time_ns = rmw_uros_epoch_nanos();

	distance[0].header.stamp.sec = time_ms / 1000;
	distance[0].header.stamp.nanosec = 0; //time_ns;
	distance[0].header.frame_id.data = (char*) "scan";

	//set the angle range (start angle and end angle)
	float startAngle = 0;
	float endAngle = ((2 * 3.14159) / requestedNumberOfPoints)
			* (requestedNumberOfPoints - 1);
	distance[0].angle_min = startAngle;
	distance[0].angle_max = endAngle;
	distance[0].angle_increment = (2 * 3.14159) / requestedNumberOfPoints;
	distance[0].time_increment = 0;
	distance[0].scan_time = 0;
	distance[0].range_min = 0.015;
	distance[0].range_max = 12;

	ls.invert(ls.adjustedScan, er, 360);

	distance[0].ranges.size = 360;
	distance[0].ranges.data = er;

	//output the quality
	distance[0].intensities.size = 360;
	distance[0].intensities.data = ls.quality;


	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&laserscanner, &distance, NULL));
	}

}

void start_Scan_cb(const void *start_Scan){
	const std_msgs__msg__Bool *startScan =	(const std_msgs__msg__Bool*) start_Scan;

	if (startScan->data) {
		ls.Start();
	} else {
		ls.Stop();
		log_message(log_info, "LaserScanner Stop");
	}
}

void timer_cb(rcl_timer_t *timer, int64_t last_call_time) {
	vehiclecontrol::CtrlMode controllerMode = ct->GetControllerMode();

	ctrl_status_msg.data = int8_t(controllerMode);

	// tf odom <-> base_link
	PoseV_t p;
	ct->GetPose(&p);

	RCSOFTCHECK(rmw_uros_sync_session(1000));
	int64_t time_ms = rmw_uros_epoch_millis();
	int64_t time_ns = rmw_uros_epoch_nanos();

	t[0].header.stamp.sec = time_ms / 1000;
	t[0].header.stamp.nanosec = 0; //time_ns;
	t[0].header.frame_id.data = (char*) "/odom";
	t[0].child_frame_id.data = (char*) "/base_link";

	t[0].transform.translation.x = (double) p.x / 1000.0; // 1000.0; // mm -> m
	t[0].transform.translation.y = (double) p.y / 1000.0; // 1000.0; // mm -> m

	t[0].transform.rotation.x = (double) 0;
	t[0].transform.rotation.y = (double) 0;
	t[0].transform.rotation.z = (double) sin(p.theta * 0.5);
	t[0].transform.rotation.w = (double) cos(p.theta * 0.5);

	tf.transforms.size = 1;
	tf.transforms.data = t;


	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&ctrl_status, &ctrl_status_msg, NULL));
		RCSOFTCHECK(rcl_publish(&tf_publisher, &tf, NULL));
	}
}

void cmd_cb(const void *cmd_msg) {
	//  log_message(log_debug,"cmd_vel x: %f, y: %f, theta: %f", cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z);
	geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist*) cmd_msg;

	imsl::vPose<real_t> v_ref;
	v_ref.vx = (msg->linear.x * 1000.0); // m/s -> mm/s
	v_ref.vy = (msg->linear.y * 1000.0); // m/s -> mm/s
	v_ref.omega = (msg->angular.z);      // in radians / s

	if (ct) {
		ct->SetManuRef(v_ref);
	}
}

void enable_topic_cb(const void *enable_topic) {
	const std_msgs__msg__Bool *enable =
			(const std_msgs__msg__Bool*) enable_topic;
	if (enable->data) {
//      hal_amplifiers_enable();
		ct->SetControllerMode(vehiclecontrol::CtrlMode::TWIST);

		log_message(log_info, "amplifiers enable, going into twist mode");
	} else {
//      hal_amplifiers_disable();
		ct->SetControllerMode(vehiclecontrol::CtrlMode::OFF);

		log_message(log_info, "amplifiers disable, mode off");
	}
}

void rosInit(void *controller) {

//		 micro-ROS configuration

	//init lwip for Ethernet Communication
	MX_LWIP_Init();

	rmw_uros_set_custom_transport(
			false,
			(void*)	&default_params,
			eth_transport_open,
			eth_transport_close,
			eth_transport_write,
			eth_transport_read);

	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}

	ct = (vehiclecontrol::ControllerTasksInterfaces<real_t>*) controller;
	rcl_allocator_t allocater = rcl_get_default_allocator();
	rclc_support_t support;

	//Create init_options
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&init_options, allocater));


	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocater));

	//Create node for Odometry and Status
	node = rcl_get_zero_initialized_node();
	rcl_node_options_t node_ops = rcl_node_get_default_options();
	node_ops.domain_id = 56;
	RCCHECK(rclc_node_init_with_options(&node, "RobotNode", "", &support, &node_ops));

//	RCCHECK(rclc_node_init_default(&node, "RobotNode", "", &support));

	//initialize publisher and subscriber
	//with best effort variant
//	RCCHECK(rclc_publisher_init_best_effort(&ctrl_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Byte), "ctrl_status"));
//	RCCHECK(rclc_publisher_init_best_effort(&tf_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf"));
//	RCCHECK(rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
//	RCCHECK(rclc_subscription_init_best_effort(&sub_enable, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "enable"));

	//inittialize publisher for LaserScanner, default (qos) Settings
	RCCHECK(rclc_publisher_init_default(&laserscanner, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
	RCCHECK(rclc_subscription_init_default(&sub_startScan, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "start_scan"));

	//with deafault (qos) settings
	RCCHECK(rclc_publisher_init_default(&ctrl_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Byte), "ctrl_status"));
	RCCHECK(rclc_publisher_init_default(&tf_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf"));
	RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
	RCCHECK(rclc_subscription_init_default(&sub_enable, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "enable"));

	//with qos settings
//	rmw_qos_profile_t qos = {RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE, false};
//	RCCHECK(rclc_publisher_init(&ctrl_status, &node, rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Byte(), "ctrl_status", &qos));
//	RCCHECK(rclc_publisher_init(&tf_publisher, &node, rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TransformStamped(), "tf", &qos));
//	RCCHECK(rclc_subscription_init(&sub_cmd_vel, &node, rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Twist(), "cmd_vel", &qos));
//	RCCHECK(rclc_subscription_init(&sub_enable, &node, rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Bool(), "enable", &qos));

	//Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_cb));

	//Create LaserScanner Timer
	rcl_timer_t sens_timer = rcl_get_zero_initialized_timer();
	const unsigned int sens_timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(&sens_timer, &support, RCL_MS_TO_NS(sens_timer_timeout), rplidar_scan));

	//Create executor
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocater));

	//Create LaserScanner executor
	sens_executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&sens_executor, &support.context, 2, &allocater)); //1->2


	//add participants to executor
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &sub_enable, &enable_msg, &enable_topic_cb, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_vel, &cmd_cb, ON_NEW_DATA));

	//add participants to LaserScanner executor
	RCCHECK(rclc_executor_add_timer(&sens_executor, &sens_timer));
	RCCHECK(rclc_executor_add_subscription(&sens_executor, &sub_startScan, &start_Scan, &start_Scan_cb, ON_NEW_DATA));


	//start spinning
	while (true) {
		RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1))); //before 1000 now 1, because microRos Task waited too long for new Task to work
		RCCHECK(rclc_executor_spin_some(&sens_executor, RCL_MS_TO_NS(1)));

	}
	RCCHECK(rcl_publisher_fini(&ctrl_status, &node));
	RCCHECK(rcl_publisher_fini(&tf_publisher, &node));
	RCCHECK(rcl_subscription_fini(&sub_cmd_vel, &node));
	RCCHECK(rcl_subscription_fini(&sub_enable, &node));
	RCCHECK(rcl_subscription_fini(&sub_startScan, &node));
	RCCHECK(rcl_publisher_fini(&laserscanner, &node));
	RCCHECK(rcl_node_fini(&node));


	vTaskDelete(NULL);
}

bool rosConnected() {
	return rmw_uros_ping_agent(100, 3) == RCL_RET_OK ? true : false;
}

} // namespace imsl

#ifdef __cplusplus
}
#endif

void ros_log_message(mr_logprio_t prio, const char *msg) {
	return; // do nothing, at the moment rosserial_python crashes on ros log messages
}

