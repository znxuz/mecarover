#!/usr/bin/env bash

export ROS_DOMAIN_ID=42

set_motor()
{
	ros2 topic list | grep -qo '/enable' || return 1
	ros2 topic pub --once /enable std_msgs/msg/Bool "data: $1"
}

set_scan()
{
	ros2 topic list | grep -qo '/start_scan' || return 1
	ros2 topic pub --once /start_scan std_msgs/msg/Bool "data: $1"
}

drive_x()
{
	vel="${1:-0.5}"
	ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
		"{linear: {x:  $vel, y: 0.0, z: 0.0}, angular: { x: 0.0, y: 0.0, z: 0.0}}"
}

drive_y()
{
	vel="${1:-0.5}"
	ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
			"{linear: {x:  0.0, y: $vel, z: 0.0}, angular: { x: 0.0, y: 0.0, z: 0.0}}"
}

turn()
{
	vel="${1:-1}"
	ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
			"{linear: {x:  0.0, y: 0.0, z: 0.0}, angular: { x: 0.0, y: 0.0, z: $vel}}"
}

clear_drive()
{
	ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
			"{linear: {x:  0.0, y: 0.0, z: 0.0}, angular: { x: 0.0, y: 0.0, z: 0.0}}"
	sleep 2
}

start_keyboard_control()
{
	echo "starting the keyboard control..." && /opt/ros/jazzy-base/bin/teleop_twist_keyboard
}

main()
{
	[[ -z "$ROS_DISTRO" ]] && echo "ros not sourced" >&2 && exit 1

	set_motor 1
	set_scan 1

	if [[ "$1" = "test" ]]; then
		drive_x 0.5 && sleep 3
		clear_drive
		drive_x -0.5 && sleep 3
		clear_drive
		turn 1 && sleep 3
		turn -1 && sleep 3
		clear_drive
	else
		start_keyboard_control
	fi

	set_motor 0
	set_scan 0
}

main "$@"
