"""Offline, unarmed full-stack terrain-planner shadow launch.

The accompanying rosbag player supplies only recorded LiDAR, IMU, and motor
feedback.  This launch never starts a Livox driver and the controller is both
unarmed and remapped to an isolated command topic.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    fastlio_share = get_package_share_directory("kilin_fastlio_bringup")
    controller_config_dir = [FindPackageShare("kilin_known_terrain_controller"), "config"]
    return LaunchDescription([
        DeclareLaunchArgument(
            "fastlio_config",
            default_value=PathJoinSubstitution([fastlio_share, "config", "fastlio_mid360s_terrain_balanced.yaml"]),
        ),
        DeclareLaunchArgument("terrain_profile", default_value="terrain_150mm_20deg_single.yaml"),
        DeclareLaunchArgument("speed_m_s", default_value="0.10"),
        DeclareLaunchArgument("planner_debug", default_value="true"),
        DeclareLaunchArgument("terrain_resolution_m", default_value="0.10"),
        Node(
            package="fast_lio", executable="fastlio_mapping", name="fastlio_mapping",
            output="screen", parameters=[LaunchConfiguration("fastlio_config"), {"use_sim_time": True}],
        ),
        Node(
            package="kilin_fastlio_bringup", executable="hip_center_odometry_adapter.py",
            name="kilin_fastlio_hip_center_odometry", output="screen", parameters=[{
                "use_sim_time": True, "input_topic": "/Odometry", "output_topic": "/kilin/fastlio/odometry",
                "expected_source_parent_frame": "camera_init", "expected_source_child_frame": "body",
                "target_parent_frame": "map", "target_child_frame": "hip_axis_center",
            }],
        ),
        Node(
            package="kilin_local_terrain_mapping", executable="local_terrain_window",
            name="kilin_local_terrain_mapping", output="screen", parameters=[{
                "use_sim_time": True, "pointcloud_topic": "/cloud_registered",
                "resolution_m": LaunchConfiguration("terrain_resolution_m"),
            }],
        ),
        Node(
            package="kilin_known_terrain_controller", executable="known_terrain_controller",
            name="kilin_terrain_shadow_controller", output="screen", parameters=[
                PathJoinSubstitution([*controller_config_dir, "one_sided_ramp.yaml"]),
                PathJoinSubstitution([*controller_config_dir, LaunchConfiguration("terrain_profile")]),
                {
                    "use_sim_time": True,
                    "armed": False,
                    "mode": "known_ramp",
                    "command_topic": "/kilin/terrain_shadow/motor_command",
                    "feedback_source": "motor_state", "motor_state_topic": "/motor/state",
                    "use_odometry": True, "odometry_topic": "/kilin/fastlio/odometry",
                    "odometry_required_frame": "map", "odometry_relative_origin": True,
                    "use_terrain_window": True,
                    "use_speed_command": False, "speed_m_s": LaunchConfiguration("speed_m_s"),
                    "run_duration_s": 30.0, "hard_motion_limit_s": 35.0,
                    "debug_publish_enabled": LaunchConfiguration("planner_debug"),
                },
            ],
        ),
    ] + _static_transforms(Path(fastlio_share) / "config" / "robot_frames.yaml"))
