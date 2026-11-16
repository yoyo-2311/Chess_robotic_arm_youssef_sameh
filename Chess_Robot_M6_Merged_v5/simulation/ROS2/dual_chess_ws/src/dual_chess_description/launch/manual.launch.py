"""Separate joint sliders: no MoveIt or controller joint-state publishers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('dual_chess_description')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file',default_value=os.path.join(share,'config','chess_geometry.yaml')),
        DeclareLaunchArgument('rviz',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','display.launch.py')),
            launch_arguments={'gui':'true','rviz':LaunchConfiguration('rviz'),'geometry_file':LaunchConfiguration('geometry_file')}.items()),
    ])
