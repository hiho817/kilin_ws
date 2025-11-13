from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # ------------------------------------
    # Shared control parameters
    # ------------------------------------
    control_params = {
        'vmax': 0.1,      # Max linear velocity [m/s]
        'wmax': 0.2,      # Max angular velocity [rad/s]
        'deadzone': 0.1   # Joystick deadzone ratio (0.0 ~ 1.0)
    }

    # ------------------------------------
    # Node definitions
    # ------------------------------------
    # Command Converter Node
    cmd_converter_node = Node(
        package='kilin_cmd_converter',
        executable='kilin_cmd_converter',
        name='kilin_cmd_converter',
        output='screen',
        parameters=[{
            'vmax': control_params['vmax'],
            'wmax': control_params['wmax']
        }]
    )

    # Joystick Node (translates /joy → /kilin/cmd_vel)
    joystick_node = Node(
        package='kilin_joystick',
        executable='kilin_joystick',
        name='kilin_joystick',
        output='screen',
        parameters=[control_params]
    )

    # ROS2 joy_node (reads hardware joystick → /joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # ------------------------------------
    # Startup sequence:
    # CmdConverter → Joystick → joy_node
    # ------------------------------------
    return LaunchDescription([
        # Start cmd_converter first
        cmd_converter_node,

        # When cmd_converter is running, start joystick
        RegisterEventHandler(
            OnProcessStart(
                target_action=cmd_converter_node,
                on_start=[joystick_node]
            )
        ),

        # When joystick is running, start joy_node
        RegisterEventHandler(
            OnProcessStart(
                target_action=joystick_node,
                on_start=[joy_node]
            )
        ),
    ])
