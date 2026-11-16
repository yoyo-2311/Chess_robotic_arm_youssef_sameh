"""Start the robot executor. DEFAULT IS DRY-RUN: it only logs commands and never acknowledges plans.

  ros2 launch chess_robot_executor executor.launch.py
  ros2 launch chess_robot_executor executor.launch.py dry_run:=false sink:=cmd_pos     # real commands (needs the final arm model)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('chess_robot_executor')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'executor_params.yaml')),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('ack_in_dry_run', default_value='false'),
        DeclareLaunchArgument('sink', default_value='log'),
        DeclareLaunchArgument('speed_scale', default_value='-1.0'),
        DeclareLaunchArgument('publish_safety_ready', default_value='false',
                              description='true = this node publishes /chess/safety_ready (software state, not an interlock)'),
        Node(
            package='chess_robot_executor', executable='robot_executor', name='chess_robot_executor', output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                'ack_in_dry_run': ParameterValue(LaunchConfiguration('ack_in_dry_run'), value_type=bool),
                'sink': LaunchConfiguration('sink'),
                'speed_scale': ParameterValue(LaunchConfiguration('speed_scale'), value_type=float),
                'publish_safety_ready': ParameterValue(LaunchConfiguration('publish_safety_ready'), value_type=bool),
            }],
        ),
    ])
