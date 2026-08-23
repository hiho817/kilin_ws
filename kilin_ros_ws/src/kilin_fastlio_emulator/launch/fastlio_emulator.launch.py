from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(
        package="kilin_fastlio_emulator",
        executable="fastlio_odometry_emulator",
        name="kilin_fastlio_emulator",
        output="screen",
        parameters=[{
            "ground_truth_topic": "/kilin/isaac/ground_truth/odometry",
            "output_topic": "/kilin/fastlio/odometry",
            "map_frame": "map_lio",
            "base_frame": "base_link",
            "position_noise_std_m": 0.0,
            "yaw_noise_std_rad": 0.0,
        }],
    )])
