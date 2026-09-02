"""Selectable real-Kilin stack from one trial-local manifest.

This launch is intentionally a convenience/orchestration entry point.  For an
armed experiment use ``run_real_terrain_trial`` instead: it additionally waits
for corrected odometry and retains each process's console in the trial folder.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from kilin_known_terrain_controller.trial_runner import REQUIRED_CONTROLLER_KEYS, load_trial_config


def _value(value):
    return "true" if value is True else "false" if value is False else str(value)


def _trial_actions(context):
    trial_dir = Path(LaunchConfiguration("trial_dir").perform(context)).resolve()
    allow_armed = LaunchConfiguration("allow_armed").perform(context).lower() == "true"
    config = load_trial_config(trial_dir)
    controller = config["controller"]
    if bool(controller["armed"]) and not allow_armed:
        raise RuntimeError(
            "trial_config.yaml requests controller.armed=true. Re-run with "
            "allow_armed:=true only after reviewing the trial-local profiles."
        )

    actions = []
    fastlio = config["fastlio"]
    if bool(fastlio.get("enabled", True)):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("kilin_fastlio_bringup"), "launch", "mid360s_fastlio.launch.py"]
                    )
                ),
                launch_arguments={
                    "fastlio_config": str(trial_dir / fastlio["config"]),
                }.items(),
            )
        )

    mapper = config["terrain_mapper"]
    if bool(mapper.get("enabled", False)):
        actions.append(
            TimerAction(
                period=float(mapper.get("startup_wait_s", 0.0)),
                actions=[
                    Node(
                        package="kilin_local_terrain_mapping",
                        executable="local_terrain_window",
                        name="local_terrain_window",
                        output="screen",
                        parameters=[str(trial_dir / mapper["profile"])],
                    )
                ],
            )
        )

    recording = config["recording"]
    if bool(recording.get("enabled", True)):
        output = trial_dir / str(recording.get("output_directory", "bag"))
        if output.exists():
            raise RuntimeError(
                f"Refusing to overwrite existing bag directory: {output}. "
                "Create a new trial directory or change recording.output_directory."
            )
        command = ["ros2", "bag", "record", "-o", str(output)]
        command.extend(["-a"] if recording["mode"] == "all" else recording["topics"])
        actions.append(TimerAction(
            period=float(recording.get("startup_wait_s", 0.0)),
            actions=[ExecuteProcess(cmd=command, output="screen")],
        ))

    launch_arguments = {
        key: _value(controller[key]) for key in REQUIRED_CONTROLLER_KEYS
    }
    launch_arguments.update(
        {
            "terrain_profile": str(trial_dir / controller["terrain_profile"]),
            "hip_pid_profile": str(trial_dir / controller["hip_pid_profile"]),
            "vicon_trigger_test": _value(controller.get("vicon_trigger_test", False)),
            "vicon_trigger_test_duration_s": _value(controller.get("vicon_trigger_test_duration_s", 3.0)),
        }
    )
    for config_key, launch_key in (
        ("angle_diff_compensation.gain", "angle_diff_compensation_gain"),
        (
            "angle_diff_compensation.maximum_abs_rad",
            "angle_diff_compensation_maximum_abs_rad",
        ),
    ):
        if config_key in controller:
            launch_arguments[launch_key] = _value(controller[config_key])
    actions.append(
        TimerAction(
            period=float(controller["startup_wait_s"]),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [FindPackageShare("kilin_known_terrain_controller"), "launch", "real_kilin_known_ramp.launch.py"]
                        )
                    ),
                    launch_arguments=launch_arguments.items(),
                )
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("trial_dir", description="Directory containing trial_config.yaml and its profiles."),
            DeclareLaunchArgument("allow_armed", default_value="false"),
            OpaqueFunction(function=_trial_actions),
        ]
    )
