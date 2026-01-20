from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),

        # Launch the IMU logger node
        Node(
            package='kilin_imu',
            executable='MIP_IMU_LOGGER',
            name='imu_logger',
            output='screen',
            arguments=[
                LaunchConfiguration('port'),
                LaunchConfiguration('baud')
            ]
        )
    ])
