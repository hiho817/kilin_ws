#!/usr/bin/env python3
"""
Launch file to start both isaac_converter and kilin_cmd_converter nodes

Usage:
  Normal (no logging):
    ros2 launch isaac_bridge launch.py
  
  Enable logging with custom filename:
    ros2 launch isaac_bridge launch.py csv_name:=my_experiment.csv
    
  CSV will be saved to: kilin_ws/logs/YYYY-MM-DD/<csv_name>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with isaac_bridge and kilin_cmd_converter nodes"""
    
    # Declare launch arguments for CSV logging
    # csv_name: empty string = no logging, non-empty = enable logging with that filename
    csv_name_arg = DeclareLaunchArgument(
        'csv_name',
        default_value='',
        description='CSV filename (empty = no logging, set a name to enable logging)'
    )
    
    add_suffix_arg = DeclareLaunchArgument(
        'add_suffix_if_exists',
        default_value='true',
        description='Auto-increment filename if exists'
    )
    
    flush_every_n_arg = DeclareLaunchArgument(
        'flush_every_n',
        default_value='20',
        description='Flush CSV every N rows (0 = every row)'
    )

    # Isaac Bridge Node - 將 kilin motor commands 轉換為 Isaac Sim commands
    isaac_converter_node = Node(
        package='isaac_bridge',
        executable='isaac_converter',
        name='isaac_converter',
        output='screen',
        emulate_tty=True,
        parameters=[
            # enable_logging is true only when csv_name is not empty
            {'enable_logging': PythonExpression(["'", LaunchConfiguration('csv_name'), "' != ''"])},
            {'csv_name': LaunchConfiguration('csv_name')},
            {'daily_folder': True},           # 自動啟用日期文件夾
            {'add_suffix_if_exists': LaunchConfiguration('add_suffix_if_exists')},
            {'flush_every_n': LaunchConfiguration('flush_every_n')},
            {'log_dir': ''},                  # 自動使用 kilin_ws/logs
        ],
        remappings=[]
    )
    
    # Kilin Command Converter Node - 處理 kilin 的命令轉換
    kilin_cmd_converter_node = Node(
        package='kilin_cmd_converter',
        executable='kilin_cmd_converter',
        name='kilin_cmd_converter',
        output='screen',
        emulate_tty=True,
        parameters=[],
        remappings=[]
    )
    
    return LaunchDescription([
        # Launch arguments
        csv_name_arg,
        add_suffix_arg,
        flush_every_n_arg,
        # Nodes
        isaac_converter_node,
        kilin_cmd_converter_node,
    ])
