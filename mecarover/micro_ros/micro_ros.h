/**
 * Copyright (c) 2019, FH Dortmund,
 * Intelligent Mobile Systems Lab
 * All rights reserved.
 * @author Christof Roehrig

 * @brief API for the ros interface component

 needs rosserial on the server side, configuration: ros_config.h

 @see http://wiki.ros.org/rosserial_python
 run rosserial_python on ROS:
 roscore&
 rosrun rosserial_python serial_node.py tcp

 @see http://wiki.ros.org/rosserial_server
 the C++ rosserial_server is much more stable but didn't support ros services:
 "At this time, rosserial_server is experimental. It is missing key features
 of the rosserial_python-provided node, including parameters, logging, and services.
 If you require these features, please stick to the standard Python server for now."

 roslaunch rosserial_server socket.launch

 * ROS topics:
 /cmd_vel type Twist, velocity command
 /tf type tf, odometry transform odom base_link

 ROS service:
 /enable boolean-service to enable amplifiers

commands:
twist command: rostopic pub -r 1 /cmd_vel geometry_msgs/Twist  "{linear:  {x: $1, y: $2, z: 0.0}, angular: {x: 0.0,y: 0.0,z: $3}}"

enable/disable: rosservice call /enable 1

odometry: rostopic echo tf

*/
/////////////////////////////////////////////////////////////
/*
 *ROS2
 start ros agent: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

 * ROS topics:
 /cmd_vel type Twist,
 /ctrl_status type Byte,
 /tf type TransformStamped,
 /enable type Bool

 * commands:
 enable command: ros2 topic pub --once /enable std_msgs/msg/Bool "data: 1"
 twist command: ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
 or use teleop_twist_keyboard to send twist messages with keyboar: ros2 run teleop_twist_keyboard teleop_twist_keyboard

odometry: ros2 topic echo /tf
*/

#pragma once

#include <cstddef>

#include <mecarover/micro_ros/eth_transport.h>

static inline constexpr size_t ROS_DOMAIN_ID = 42;

extern "C"
{
static eth_transport_params_t TRANSPORT_PARAMS
	= {{0, 0, 0}, {"192.168.1.228"}, {"8888"}};
};

namespace imsl {
	void micro_ros(void *ct);
}
