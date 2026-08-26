"""Offline FAST-LIO2 replay: consumes bag topics and starts no Livox driver."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _static_transforms(config_path: Path):
    with config_path.open(encoding="utf-8") as handle:
        transforms = yaml.safe_load(handle)["static_transforms"]
    return [Node(
        package="tf2_ros", executable="static_transform_publisher", name=item["name"],
        arguments=["--x", str(item["translation_m"][0]), "--y", str(item["translation_m"][1]), "--z", str(item["translation_m"][2]),
                   "--roll", str(item["rpy_rad"][0]), "--pitch", str(item["rpy_rad"][1]), "--yaw", str(item["rpy_rad"][2]),
                   "--frame-id", item["parent_frame"], "--child-frame-id", item["child_frame"]],
    ) for item in transforms]


def generate_launch_description():
    share = get_package_share_directory("kilin_fastlio_bringup")
    config = LaunchConfiguration("fastlio_config")
    actions = [
        DeclareLaunchArgument("fastlio_config", default_value=PathJoinSubstitution([share, "config", "fastlio_mid360s_terrain_dense.yaml"])),
        Node(package="fast_lio", executable="fastlio_mapping", name="fastlio_mapping", output="screen", parameters=[config, {"use_sim_time": True}]),
        Node(package="kilin_fastlio_bringup", executable="hip_center_odometry_adapter.py", name="kilin_fastlio_hip_center_odometry", output="screen", parameters=[{
            "use_sim_time": True, "input_topic": "/Odometry", "output_topic": "/kilin/fastlio/odometry",
            "expected_source_parent_frame": "camera_init", "expected_source_child_frame": "body",
            "target_parent_frame": "map", "target_child_frame": "hip_axis_center",
        }]),
    ]
    return LaunchDescription(actions + _static_transforms(Path(share) / "config" / "robot_frames.yaml"))
