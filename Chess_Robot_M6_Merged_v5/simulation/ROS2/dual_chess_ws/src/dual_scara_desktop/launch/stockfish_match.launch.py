"""Stockfish + existing motion, mock controllers, gripper and vision. Turn owner is the desktop."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc=get_package_share_directory('dual_chess_description')
    moveit=get_package_share_directory('dual_chess_moveit_config')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file',default_value=os.path.join(desc,'config','chess_geometry.yaml')),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('motion_start_delay',default_value='2.0'),
        DeclareLaunchArgument('stockfish_path',default_value='/usr/games/stockfish'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(moveit,'launch','demo.launch.py')),
            launch_arguments={'motion_start_delay':LaunchConfiguration('motion_start_delay'),'geometry_file':LaunchConfiguration('geometry_file'),'rviz':LaunchConfiguration('rviz'),
                              'initial_fen':'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1',
                              'accept_move_plans':'true','publish_mock_readiness':'true'}.items()),
        Node(package='chess_brain_handover',executable='chess_brain_node',output='screen',parameters=[
            {'engine_mode':'stockfish','engine_path':LaunchConfiguration('stockfish_path'),'white_preset':'medium','black_preset':'medium'}]),
        Node(package='chess_motion_interface',executable='mock_vision',output='screen'),
        Node(package='chess_motion_interface',executable='mock_gripper',output='screen'),
    ])
