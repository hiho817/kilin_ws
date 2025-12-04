from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # User can pass csv_name:=xxxx.csv
    csv_name_arg = DeclareLaunchArgument(
        'csv_name',
        default_value='gait.csv',
        description='CSV filename located in the csv/ folder inside kilin_csv_control'
    )

    csv_name = LaunchConfiguration('csv_name')

    pkg_dir = get_package_share_directory('kilin_csv_control')

    # CORRECT way: use PathJoinSubstitution
    csv_full_path = PathJoinSubstitution([
        pkg_dir,
        'csv',
        csv_name
    ])

    return LaunchDescription([
        csv_name_arg,
        Node(
            package='kilin_csv_control',
            executable='kilin_csv_control',
            name='kilin_csv_control',
            output='screen',
            parameters=[
                {"csv_path": csv_full_path},
                {"rate_hz": 200.0}
            ]
        )
    ])
