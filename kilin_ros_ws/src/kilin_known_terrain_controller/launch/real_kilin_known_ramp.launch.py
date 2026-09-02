"""Known-ramp controller for real Kilin: no Isaac bridge or simulation clock."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = [FindPackageShare("kilin_known_terrain_controller"), "config"]
    base_config = PathJoinSubstitution([*config_dir, "one_sided_ramp.yaml"])
    def profile_path(argument_name):
        """Resolve a package-relative profile name or an absolute YAML path."""
        profile = LaunchConfiguration(argument_name)
        return IfElseSubstitution(
            PythonExpression(["'", profile, "'.startswith('/')"]),
            profile,
            PathJoinSubstitution([*config_dir, profile]),
        )

    terrain_config = profile_path("terrain_profile")
    hip_pid_config = profile_path("hip_pid_profile")
    return LaunchDescription(
        [
            DeclareLaunchArgument("armed", default_value="false"),
            DeclareLaunchArgument("mode", default_value="known_ramp"),
            DeclareLaunchArgument(
                "terrain_profile",
                default_value=PathJoinSubstitution(
                    [*config_dir, "terrain_150mm_20deg_single.yaml"]
                ),
                description="Package profile name by default, or an absolute analytical terrain YAML path.",
            ),
            DeclareLaunchArgument(
                "hip_pid_profile",
                # The base profile already carries the historical default
                # hip gains.  Per-trial PID overlays are supplied explicitly.
                default_value=PathJoinSubstitution([*config_dir, "one_sided_ramp.yaml"]),
                description=(
                    "Package profile name by default, or an absolute per-trial PID YAML path. "
                    "This overlay contains hip_kp, hip_ki, and hip_kd only."
                ),
            ),
            DeclareLaunchArgument("use_speed_command", default_value="false"),
            DeclareLaunchArgument("speed_m_s", default_value="0.05"),
            DeclareLaunchArgument("run_duration_s", default_value="30.0"),
            DeclareLaunchArgument("hard_motion_limit_s", default_value="35.0"),
            DeclareLaunchArgument("vicon_trigger", default_value="false"),
            DeclareLaunchArgument("vicon_trigger_test", default_value="false"),
            DeclareLaunchArgument("vicon_trigger_test_duration_s", default_value="3.0"),
            DeclareLaunchArgument("debug_publish", default_value="false"),
            DeclareLaunchArgument(
                "known_ramp_auto_initialize_stance",
                default_value="true",
                description=(
                    "Move motor-side hips to the nominal stance before planning. "
                    "Set false only after manually verifying the physical stance."
                ),
            ),
            DeclareLaunchArgument(
                "known_ramp_max_initial_hip_error_deg",
                default_value="5.0",
                description=(
                    "Maximum allowed motor-side difference from nominal when "
                    "automatic stance initialization is disabled."
                ),
            ),
            DeclareLaunchArgument(
                "angle_diff_compensation_gain",
                default_value="0.0",
                description="Experimental outward position_diff compensation gain; zero disables it.",
            ),
            DeclareLaunchArgument(
                "angle_diff_compensation_maximum_abs_rad",
                default_value="0.10",
                description="Hard per-hip bound for angle-difference compensation.",
            ),
            DeclareLaunchArgument(
                "use_terrain_window",
                default_value="false",
                description=(
                    "Use /kilin/terrain/local_window instead of the analytical profile. "
                    "Keep false for analytical-map or shadow-mapping experiments."
                ),
            ),
            DeclareLaunchArgument(
                "terrain_window_timeout_s",
                default_value="1.0",
                description="Maximum age of a live TerrainWindow before the controller safety hold.",
            ),
            DeclareLaunchArgument(
                "use_odometry",
                default_value="false",
                description=(
                    "Use corrected /kilin/fastlio/odometry for analytical-map progress. "
                    "The controller stops instead of falling back if it becomes stale."
                ),
            ),
            DeclareLaunchArgument(
                "odometry_relative_origin",
                default_value="true",
                description="Zero corrected odometry when ramp motion begins.",
            ),
            Node(
                package="kilin_known_terrain_controller",
                executable="known_terrain_controller",
                name="kilin_known_terrain_controller",
                output="screen",
                parameters=[
                    base_config,
                    terrain_config,
                    hip_pid_config,
                    {
                        "use_sim_time": False,
                        "armed": LaunchConfiguration("armed"),
                        "mode": LaunchConfiguration("mode"),
                        "command_topic": "/motor/command",
                        "feedback_source": "motor_state",
                        "motor_state_topic": "/motor/state",
                        "use_odometry": LaunchConfiguration("use_odometry"),
                        "odometry_topic": "/kilin/fastlio/odometry",
                        "odometry_required_frame": "map",
                        "odometry_relative_origin": LaunchConfiguration(
                            "odometry_relative_origin"
                        ),
                        "use_terrain_window": LaunchConfiguration("use_terrain_window"),
                        "terrain_window_timeout_s": LaunchConfiguration("terrain_window_timeout_s"),
                        "use_speed_command": LaunchConfiguration("use_speed_command"),
                        "speed_m_s": LaunchConfiguration("speed_m_s"),
                        "run_duration_s": LaunchConfiguration("run_duration_s"),
                        "hard_motion_limit_s": LaunchConfiguration("hard_motion_limit_s"),
                        "vicon_trigger.enabled": LaunchConfiguration("vicon_trigger"),
                        "vicon_trigger.test_mode": LaunchConfiguration("vicon_trigger_test"),
                        "vicon_trigger.test_duration_s": LaunchConfiguration(
                            "vicon_trigger_test_duration_s"
                        ),
                        "debug_publish_enabled": LaunchConfiguration("debug_publish"),
                        "known_ramp.auto_initialize_stance": LaunchConfiguration(
                            "known_ramp_auto_initialize_stance"
                        ),
                        "known_ramp.max_initial_hip_error_deg": LaunchConfiguration(
                            "known_ramp_max_initial_hip_error_deg"
                        ),
                        "angle_diff_compensation.gain": LaunchConfiguration(
                            "angle_diff_compensation_gain"
                        ),
                        "angle_diff_compensation.maximum_abs_rad": LaunchConfiguration(
                            "angle_diff_compensation_maximum_abs_rad"
                        ),
                    },
                ],
            ),
        ]
    )
