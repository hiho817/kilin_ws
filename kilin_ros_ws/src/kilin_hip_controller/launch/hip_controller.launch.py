#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('kilin_hip_controller')
    resources_dir = os.path.join(pkg_dir, 'resources')

    # Declare launch arguments
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for the web server'
    )

    # Create the hip controller node
    hip_controller_node = Node(
        package='kilin_hip_controller',
        executable='kilin_hip_controller',
        name='kilin_hip_controller',
        parameters=[
            {
                'web_port': LaunchConfiguration('web_port'),
                # 'resources_dir': resources_dir,
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        web_port_arg,
        hip_controller_node,
    ])
