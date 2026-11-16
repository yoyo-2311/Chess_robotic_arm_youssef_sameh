"""Explicit all-mock integration: no Stockfish, physical camera, gripper or motors."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('dual_chess_moveit_config')
    geometry = os.path.join(get_package_share_directory('dual_chess_description'),'config','chess_geometry.yaml')
    fen = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'
    initial = ParameterValue(LaunchConfiguration('initial_fen'),value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('geometry_file',default_value=geometry),
        DeclareLaunchArgument('initial_fen',default_value=fen),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','demo.launch.py')),
                                 launch_arguments={'rviz':LaunchConfiguration('rviz'),'geometry_file':LaunchConfiguration('geometry_file'),'initial_fen':LaunchConfiguration('initial_fen'),'accept_move_plans':'true','publish_mock_readiness':'true'}.items()),
        Node(package='chess_brain_handover',executable='chess_brain_node',output='screen',parameters=[{'engine_mode':'mock','initial_fen':initial}]),
        Node(package='chess_motion_interface',executable='mock_vision',output='screen',parameters=[{'initial_fen':initial}]),
        Node(package='chess_motion_interface',executable='mock_gripper',output='screen'),
    ])
