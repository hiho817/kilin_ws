from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Start the bridge immediately
    bridge_node = Node(
        package='kilin_ros2_bridge',
        executable='kilin_ros2_bridge',
        name='kilin_ros2_bridge',
        output='screen',
        emulate_tty=True,
    )

    # Start the panel after a short delay to give the bridge time to initialize
    panel_node = Node(
        package='kilin_panel',
        executable='kilin_panel',
        output='screen',
        emulate_tty=True,
    )

    delayed_panel = TimerAction(period=1.0, actions=[panel_node])

    return LaunchDescription([
        bridge_node,
        delayed_panel,
    ])
