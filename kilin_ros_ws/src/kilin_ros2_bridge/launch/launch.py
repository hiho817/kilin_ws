#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_bridge',
            executable='ros2_bridge',
            name='ros2_bridge',
            output='screen'
        )
    ])
