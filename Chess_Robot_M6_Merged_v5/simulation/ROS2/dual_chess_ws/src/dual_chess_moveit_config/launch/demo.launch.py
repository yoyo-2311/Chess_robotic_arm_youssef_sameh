"""MoveIt with ros2_control mock hardware. Does not command physical motors or Gazebo."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('dual_chess_moveit_config')
    description = os.path.join(get_package_share_directory('dual_chess_description'), 'urdf', 'dual_chess.urdf.xacro')
    config = (MoveItConfigsBuilder('dual_chess', package_name='dual_chess_moveit_config')
              .robot_description(file_path=description, mappings={'hardware_mode': 'mock'})
              .robot_description_semantic(file_path='config/dual_chess.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
              .to_moveit_configs())
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[config.robot_description], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[os.path.join(share, 'config', 'ros2_controllers.yaml')],
             remappings=[('~/robot_description', '/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'robot1_controller', 'robot2_controller', '--controller-manager-timeout', '60'], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[config.to_dict()], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'config', 'moveit.rviz')],
             parameters=[config.to_dict()], condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
