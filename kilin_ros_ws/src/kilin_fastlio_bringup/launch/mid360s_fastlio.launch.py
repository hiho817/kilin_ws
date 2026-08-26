"""Launch a MID360s driver and unmodified FAST-LIO2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml


def _static_transform_nodes(config_path: Path):
    """Create static TF publishers from the local, versioned frame config."""
    with config_path.open(encoding="utf-8") as config_file:
        transforms = yaml.safe_load(config_file)["static_transforms"]

    nodes = []
    for transform in transforms:
        x, y, z = transform["translation_m"]
        roll, pitch, yaw = transform["rpy_rad"]
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=transform["name"],
            arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                "--frame-id", transform["parent_frame"],
                "--child-frame-id", transform["child_frame"],
            ],
        ))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("kilin_fastlio_bringup")
    driver_config = LaunchConfiguration("driver_config")
    fastlio_config = LaunchConfiguration("fastlio_config")
    static_tf_nodes = _static_transform_nodes(Path(share) / "config" / "robot_frames.yaml")

    actions = [
        DeclareLaunchArgument(
            "driver_config",
            default_value=PathJoinSubstitution([share, "config", "MID360s_config.json"]),
            description="Livox MID360s network configuration JSON."),
        DeclareLaunchArgument(
            "fastlio_config",
            default_value=PathJoinSubstitution([share, "config", "fastlio_mid360s_windowed.yaml"]),
            description="FAST-LIO2 parameter YAML."),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": driver_config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[fastlio_config],
        ),
        Node(
            package="kilin_fastlio_bringup",
            executable="hip_center_odometry_adapter.py",
            name="kilin_fastlio_hip_center_odometry",
            output="screen",
            parameters=[{
                "input_topic": "/Odometry",
                "output_topic": "/kilin/fastlio/odometry",
                "expected_source_parent_frame": "camera_init",
                "expected_source_child_frame": "body",
                "target_parent_frame": "map",
                "target_child_frame": "hip_axis_center",
            }],
        ),
    ]
    return LaunchDescription(actions + static_tf_nodes)
