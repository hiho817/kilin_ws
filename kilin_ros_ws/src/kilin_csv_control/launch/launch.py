from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

def generate_launch_description():

    # User can pass: csv_name:=xxxx.csv
    csv_name_arg = DeclareLaunchArgument(
        'csv_name',
        default_value='gait.csv',
        description='CSV filename located in ~/kilin_ws/csv/'
    )

    # Optional: allow user to override the folder too (nice to have)
    csv_dir_arg = DeclareLaunchArgument(
        'csv_dir',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'kilin_ws',
            'csv'
        ]),
        description='Directory that stores CSV files (default: ~/kilin_ws/csv)'
    )

    csv_name = LaunchConfiguration('csv_name')
    csv_dir  = LaunchConfiguration('csv_dir')

    csv_full_path = PathJoinSubstitution([
        csv_dir,
        csv_name
    ])

    return LaunchDescription([
        csv_name_arg,
        csv_dir_arg,
        Node(
            package='kilin_csv_control',
            executable='kilin_csv_control',
            name='kilin_csv_control',
            output='screen',
            parameters=[
                {"csv_path": csv_full_path},
                {"rate_hz": 200.0}
            ]
        )
    ])
