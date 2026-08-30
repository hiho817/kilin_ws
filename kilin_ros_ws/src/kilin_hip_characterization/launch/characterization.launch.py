from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("armed", default_value="false"),
        DeclareLaunchArgument("command_topic", default_value="/kilin/hip_characterization/command_preview"),
        DeclareLaunchArgument("run_dir", default_value=""),
        DeclareLaunchArgument(
            "profile",
            default_value=PathJoinSubstitution([
                FindPackageShare("kilin_hip_characterization"),
                "config", "initial_screening.yaml",
            ]),
        ),
        Node(
            package="kilin_hip_characterization",
            executable="campaign_runner",
            name="kilin_hip_characterization",
            output="screen",
            parameters=[LaunchConfiguration("profile"), {
                "armed": LaunchConfiguration("armed"),
                "command_topic": LaunchConfiguration("command_topic"),
                "run_dir": LaunchConfiguration("run_dir"),
            }],
        ),
    ])
