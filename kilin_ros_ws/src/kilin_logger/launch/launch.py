from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================================
    # Launch Arguments
    # ============================================================
    csv_name = LaunchConfiguration("csv_name")
    log_dir = LaunchConfiguration("log_dir")
    motor_topic = LaunchConfiguration("motor_topic")
    power_topic = LaunchConfiguration("power_topic")
    flush_every_n = LaunchConfiguration("flush_every_n")
    qos_depth = LaunchConfiguration("qos_depth")

    return LaunchDescription([
        # ----------------------------
        # File naming / path
        # ----------------------------
        DeclareLaunchArgument(
            "csv_name",
            default_value="motor_power_state.csv",
            description="Output CSV filename (stored under <kilin_ws>/logs/YYYY-MM-DD/)."
        ),
        DeclareLaunchArgument(
            "log_dir",
            default_value="",
            description=(
                "Base log directory. If empty, node auto-resolves to <kilin_ws>/logs "
                "by searching ancestor folder named 'kilin_ros_ws' from __FILE__. "
                "You may also pass an absolute/relative path."
            )
        ),

        # ----------------------------
        # Topics
        # ----------------------------
        DeclareLaunchArgument(
            "motor_topic",
            default_value="/motor/state",
            description="Motor state topic."
        ),
        DeclareLaunchArgument(
            "power_topic",
            default_value="/power/state",
            description="Power state topic."
        ),

        # ----------------------------
        # Behavior
        # ----------------------------
        DeclareLaunchArgument(
            "flush_every_n",
            default_value="20",
            description="Flush CSV every N motor rows (0 = every row)."
        ),
        DeclareLaunchArgument(
            "qos_depth",
            default_value="50",
            description="QoS depth for subscriptions."
        ),

        # ============================================================
        # Node
        # ============================================================
        Node(
            package="kilin_logger",
            executable="kilin_logger",
            name="kilin_logger",
            output="screen",
            parameters=[{
                "csv_name": csv_name,
                "log_dir": log_dir,
                "motor_topic": motor_topic,
                "power_topic": power_topic,
                "flush_every_n": flush_every_n,
                "qos_depth": qos_depth,

                # fixed behavior
                "daily_folder": True,
                "add_suffix_if_exists": True,
            }],
        ),
    ])
