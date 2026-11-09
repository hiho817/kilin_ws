from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vmax = ParameterValue(LaunchConfiguration('vmax'), value_type=float)
    wmax = ParameterValue(LaunchConfiguration('wmax'), value_type=float)
    keyboard = Node(
        package='kilin_joystick',
        executable='kilin_keyboard',
        name='kilin_keyboard',
        output='screen',
        parameters=[{
            'vmax': vmax,
            'wmax': wmax,
        }],
    )
    converter = Node(
        package='kilin_cmd_converter',
        executable='kilin_cmd_converter',
        name='kilin_cmd_converter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_cmd_converter')),
        parameters=[{'vmax': vmax, 'wmax': wmax, 'enable_rest_mask': False}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('vmax', default_value='0.3', description='Linear speed (m/s)'),
        DeclareLaunchArgument('wmax', default_value='1.0', description='Yaw speed (rad/s)'),
        DeclareLaunchArgument('start_cmd_converter', default_value='true',
                              description='Start kilin_cmd_converter alongside keyboard control'),
        RegisterEventHandler(OnProcessExit(
            target_action=keyboard,
            on_exit=[EmitEvent(event=Shutdown(reason='Keyboard control exited'))],
        )),
        keyboard,
        converter,
    ])
