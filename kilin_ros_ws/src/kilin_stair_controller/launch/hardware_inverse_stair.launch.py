# Copyright 2026 Ian

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    csv_path = LaunchConfiguration("csv_path")
    controller_config = LaunchConfiguration("controller_config")
    estimator_config = LaunchConfiguration("estimator_config")
    ptp_config = LaunchConfiguration("ptp_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "csv_path",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "kilin_ws", "csv", "stairs_hardware.csv"]
                ),
            ),
            DeclareLaunchArgument(
                "controller_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kilin_stair_controller"),
                        "config",
                        "stair_controller_hardware_inverse.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "estimator_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kilin_com_estimator"),
                        "config",
                        "hardware_com_estimator.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "ptp_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("kinova_joint_ptp"), "config", "joint_ptp.yaml"]
                ),
            ),
            Node(
                package="kilin_com_estimator",
                executable="kilin_com_estimator",
                name="kilin_com_estimator",
                output="screen",
                parameters=[estimator_config],
            ),
            Node(
                package="kinova_joint_ptp",
                executable="kinova_joint_ptp_server",
                name="kinova_joint_ptp",
                output="screen",
                parameters=[ptp_config],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="kilin_stair_controller",
                        executable="kilin_stair_controller",
                        name="kilin_stair_controller",
                        output="screen",
                        parameters=[
                            controller_config,
                            {
                                "csv_path": csv_path,
                                "use_sim_time": False,
                                "arm_control_mode": "com_closed_loop",
                            },
                        ],
                    )
                ],
            ),
        ]
    )
