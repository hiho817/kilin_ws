from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    device = LaunchConfiguration('device').perform(context)
    vmax   = float(LaunchConfiguration('vmax').perform(context))

    # Axis mapping
    if device == "pc":
        omega_axes = 3
    else:
        omega_axes = 2

    joystick_node = Node(
        package='kilin_joystick',
        executable='kilin_joystick',
        name='kilin_joystick',
        output='screen',
        parameters=[{
            'omega_axes': omega_axes,
            'vmax': vmax,
            'wmax': 2.0,
            'deadzone': 0.1
        }]
    )

    converter_node = Node(
        package='kilin_cmd_converter',
        executable='kilin_cmd_converter',
        name='kilin_cmd_converter',
        output='screen',
        parameters=[{
            'vmax': vmax,
            'wmax': 2.0
        }]
    )

    joydrv_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return [converter_node, joystick_node, joydrv_node]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'device',
            default_value='orin',
            description='Select joystick mapping: pc or orin'
        ),

        DeclareLaunchArgument(
            'vmax',
            default_value='1.0',
            description='Max linear velocity'
        ),

        OpaqueFunction(function=launch_setup)
    ])
