"""Standalone view of the new archive; fixed assembly, no joint publisher."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('chess_workcell_description'))
    return LaunchDescription([DeclareLaunchArgument('rviz',default_value='true'),
      Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(share/'urdf/workcell.urdf').read_text()}]),
      Node(package='rviz2',executable='rviz2',arguments=['-f','world'],condition=IfCondition(LaunchConfiguration('rviz')))])
