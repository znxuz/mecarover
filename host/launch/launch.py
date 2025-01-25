#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom',
            executable='odom',
            name='odom'
        ),
        Node(
            package='pose_controller',
            executable='pose_controller',
            name='pose_controller'
        ),
        Node(
            package='wheel_controller',
            executable='wheel_controller',
            name='wheel_controller'
        )
    ])
