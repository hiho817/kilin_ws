# Copyright 2026 Ian

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    csv_name_arg = DeclareLaunchArgument(
        "csv_name",
        default_value="stairs.csv",
        description="CSV filename located in csv_dir",
    )
    csv_dir_arg = DeclareLaunchArgument(
        "csv_dir",
        default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "kilin_ws", "csv"]),
        description="Directory containing stair gait CSV files",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use Isaac Sim /clock when true; use system time when false",
    )
    config_name_arg = DeclareLaunchArgument(
        "config_name",
        default_value="stair_controller.yaml",
        description="Parameter YAML filename located in config_dir",
    )
    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value=PathJoinSubstitution(
            [EnvironmentVariable("HOME"), "kilin_ws", "config"]
        ),
        description="Directory containing stair controller parameter YAML files",
    )
    start_ptp_arg = DeclareLaunchArgument(
        "start_ptp",
        default_value="true",
        description="Start the kinova_joint_ptp action server",
    )
    arm_control_mode_arg = DeclareLaunchArgument(
        "arm_control_mode",
        default_value="fixed_phase",
        description="Arm strategy: fixed_phase or com_closed_loop",
    )
    ptp_config_arg = DeclareLaunchArgument(
        "ptp_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("kinova_joint_ptp"), "config", "joint_ptp.yaml"]
        ),
        description="Kinova JointPtp server parameter YAML",
    )
    ptp_warmup_sec_arg = DeclareLaunchArgument(
        "ptp_warmup_sec",
        default_value="2.0",
        description=(
            "Wall-clock delay before starting the stair controller when this launch "
            "also starts JointPtp; allows JointPtp to receive initial joint states"
        ),
    )
    csv_path = PathJoinSubstitution(
        [LaunchConfiguration("csv_dir"), LaunchConfiguration("csv_name")]
    )
    config_path = PathJoinSubstitution(
        [LaunchConfiguration("config_dir"), LaunchConfiguration("config_name")]
    )

    controller_parameters = [
        config_path,
        {
            "csv_path": csv_path,
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "arm_control_mode": LaunchConfiguration("arm_control_mode"),
        },
    ]

    def stair_controller_node(condition):
        return Node(
            package="kilin_stair_controller",
            executable="kilin_stair_controller",
            name="kilin_stair_controller",
            output="screen",
            parameters=controller_parameters,
            condition=condition,
        )

    return LaunchDescription(
        [
            csv_name_arg,
            csv_dir_arg,
            use_sim_time_arg,
            config_name_arg,
            config_dir_arg,
            start_ptp_arg,
            arm_control_mode_arg,
            ptp_config_arg,
            ptp_warmup_sec_arg,
            Node(
                package="kinova_joint_ptp",
                executable="kinova_joint_ptp_server",
                name="kinova_joint_ptp",
                output="screen",
                parameters=[LaunchConfiguration("ptp_config")],
                condition=IfCondition(LaunchConfiguration("start_ptp")),
            ),
            TimerAction(
                period=LaunchConfiguration("ptp_warmup_sec"),
                actions=[stair_controller_node(condition=None)],
                condition=IfCondition(LaunchConfiguration("start_ptp")),
            ),
            stair_controller_node(
                condition=UnlessCondition(LaunchConfiguration("start_ptp"))
            ),
        ]
    )
