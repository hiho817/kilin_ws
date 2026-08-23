from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target = LaunchConfiguration("target")
    armed = LaunchConfiguration("armed")
    mode = LaunchConfiguration("mode")
    speed_m_s = LaunchConfiguration("speed_m_s")
    use_speed_command = LaunchConfiguration("use_speed_command")
    run_duration_s = LaunchConfiguration("run_duration_s")
    hard_motion_limit_s = LaunchConfiguration("hard_motion_limit_s")
    auto_initialize_stance = LaunchConfiguration("auto_initialize_stance")
    terrain_profile = LaunchConfiguration("terrain_profile")
    config = PathJoinSubstitution(
        [FindPackageShare("kilin_known_terrain_controller"), "config", "one_sided_ramp.yaml"]
    )
    terrain_config = PathJoinSubstitution(
        [FindPackageShare("kilin_known_terrain_controller"), "config", terrain_profile]
    )
    bridge = GroupAction(
        scoped=True,
        condition=IfCondition(PythonExpression(["'", target, "' == 'isaac'"])),
        actions=[
            SetRemap(src="/imu", dst="/kilin/isaac/imu"),
            # The Isaac graph publishes its articulation state below this
            # namespace.  The bridge translates it into canonical /motor/state.
            SetRemap(src="/kilin_joint_states", dst="/kilin/isaac/joint_states"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("isaac_bridge"), "launch", "launch.py"]
                    )
                ),
                launch_arguments={"start_cmd_converter": "false"}.items(),
            ),
        ],
    )
    isaac_controller = Node(
        package="kilin_known_terrain_controller",
        executable="known_terrain_controller",
        name="kilin_known_terrain_controller",
        output="screen",
        parameters=[
            config,
            terrain_config,
            {
                "armed": armed,
                "mode": mode,
                "speed_m_s": speed_m_s,
                "use_speed_command": use_speed_command,
                "run_duration_s": run_duration_s,
                "hard_motion_limit_s": hard_motion_limit_s,
                "known_ramp.auto_initialize_stance": auto_initialize_stance,
            },
        ],
        remappings=[("/clock", "/kilin/isaac/clock")],
        condition=IfCondition(PythonExpression(["'", target, "' == 'isaac'"])),
    )
    real_controller = Node(
        package="kilin_known_terrain_controller",
        executable="known_terrain_controller",
        name="kilin_known_terrain_controller",
        output="screen",
        parameters=[
            config,
            terrain_config,
            {
                "use_sim_time": False,
                "armed": armed,
                "mode": mode,
                "command_topic": "/motor/command",
                "feedback_source": "motor_state",
                "motor_state_topic": "/motor/state",
                "speed_m_s": speed_m_s,
                "use_speed_command": use_speed_command,
                "run_duration_s": run_duration_s,
                "hard_motion_limit_s": hard_motion_limit_s,
                "known_ramp.auto_initialize_stance": auto_initialize_stance,
            },
        ],
        condition=IfCondition(PythonExpression(["'", target, "' == 'real'"])),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target",
                default_value="real",
                description=(
                    "Execution target: real starts no Isaac component and publishes "
                    "to /motor/command; isaac starts isaac_bridge and uses Isaac time."
                ),
                choices=["real", "isaac"],
            ),
            DeclareLaunchArgument(
                "armed",
                default_value="false",
                description="Publish motor commands; false performs planning without actuation",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="hip_test",
                description="calibration, stance_initialization, planner_posture_test, or known_ramp",
            ),
            DeclareLaunchArgument(
                "speed_m_s",
                default_value="0.18",
                description="Fixed forward speed when use_speed_command is false",
            ),
            DeclareLaunchArgument(
                "use_speed_command",
                default_value="true",
                description="Use /kilin/control/target_speed_m_s; ignores run_duration_s",
            ),
            DeclareLaunchArgument(
                "run_duration_s",
                default_value="22.0",
                description="Fixed-speed run duration; used only when use_speed_command is false",
            ),
            DeclareLaunchArgument(
                "hard_motion_limit_s",
                default_value="22.0",
                description="Upper duration bound for fixed-speed runs",
            ),
            DeclareLaunchArgument(
                "auto_initialize_stance",
                default_value="true",
                description=(
                    "Move to the nominal [-45, -45, 45, 45] degree stance "
                    "before known_ramp; set false when already in that pose"
                ),
            ),
            DeclareLaunchArgument(
                "terrain_profile",
                default_value="terrain_80mm_two_ramps.yaml",
                description="Terrain profile YAML from this package's config directory",
            ),
            bridge,
            isaac_controller,
            real_controller,
        ]
    )
