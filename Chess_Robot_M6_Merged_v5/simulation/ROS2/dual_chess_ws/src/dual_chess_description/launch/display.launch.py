"""Manual description display; only GUI OR non-GUI joint_state_publisher runs."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('dual_chess_description')
    mappings = {key:LaunchConfiguration(key).perform(context) for key in ['geometry_file','robot1_enabled','robot2_enabled','use_meshes']}
    description = xacro.process_file(os.path.join(share,'urdf','dual_chess.urdf.xacro'),mappings=mappings).toxml()
    return [
        Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',parameters=[{'robot_description':description}]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher',executable='joint_state_publisher',condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2',executable='rviz2',arguments=['-d',os.path.join(share,'rviz','dual_chess.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    share = get_package_share_directory('dual_chess_description')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file',default_value=os.path.join(share,'config','chess_geometry.yaml')),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('robot1_enabled',default_value='true'),
        DeclareLaunchArgument('robot2_enabled',default_value='true'),
        DeclareLaunchArgument('use_meshes',default_value='true'),
        OpaqueFunction(function=setup),
    ])
