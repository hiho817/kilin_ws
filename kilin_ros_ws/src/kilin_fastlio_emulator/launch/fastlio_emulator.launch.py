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
            "output_topic": "/Odometry",
            "map_frame": "camera_init",
            "base_frame": "body",
            "position_noise_std_m": 0.0,
            "yaw_noise_std_rad": 0.0,
        }],
    )])
