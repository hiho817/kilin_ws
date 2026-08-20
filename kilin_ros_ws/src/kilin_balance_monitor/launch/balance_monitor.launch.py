"""Launch the Kilin shadow-mode balance monitor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("kilin_balance_monitor"))
        / "config"
        / "balance_monitor.yaml"
    )
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to the balance monitor parameter file.",
    )
    node = Node(
        package="kilin_balance_monitor",
        executable="kilin_balance_monitor",
        name="kilin_balance_monitor",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription([config_arg, node])
