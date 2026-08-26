"""Known-ramp controller for real Kilin: no Isaac bridge or simulation clock."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = [FindPackageShare("kilin_known_terrain_controller"), "config"]
    base_config = PathJoinSubstitution([*config_dir, "one_sided_ramp.yaml"])
    terrain_config = PathJoinSubstitution([*config_dir, LaunchConfiguration("terrain_profile")])
    return LaunchDescription(
        [
            DeclareLaunchArgument("armed", default_value="false"),
            DeclareLaunchArgument("mode", default_value="known_ramp"),
            DeclareLaunchArgument("terrain_profile", default_value="terrain_150mm_20deg_single.yaml"),
            DeclareLaunchArgument("use_speed_command", default_value="false"),
            DeclareLaunchArgument("speed_m_s", default_value="0.05"),
            DeclareLaunchArgument("run_duration_s", default_value="30.0"),
            DeclareLaunchArgument("hard_motion_limit_s", default_value="35.0"),
            DeclareLaunchArgument("vicon_trigger", default_value="false"),
            DeclareLaunchArgument("vicon_trigger_test", default_value="false"),
            DeclareLaunchArgument("vicon_trigger_test_duration_s", default_value="3.0"),
            DeclareLaunchArgument("debug_publish", default_value="false"),
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
                        "use_terrain_window": False,
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
                    },
                ],
            ),
        ]
    )
