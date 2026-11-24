"""Reuse the description; the desktop owns visualization joint states (no JSP/GUI)."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=get_package_share_directory('dual_chess_description')
    xml=xacro.process_file(os.path.join(share,'urdf','dual_chess.urdf.xacro'),mappings={'geometry_file':LaunchConfiguration('geometry_file').perform(context)}).toxml()
    return [Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':xml}]),
            Node(package='rviz2',executable='rviz2',arguments=['-d',os.path.join(share,'rviz','dual_chess.rviz')],condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('geometry_file',default_value=os.path.join(get_package_share_directory('dual_chess_description'),'config','chess_geometry.yaml')),
                              DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
