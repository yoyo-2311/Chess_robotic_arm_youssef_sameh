"""MoveIt with ros2_control mock hardware. Does not command physical motors or Gazebo."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    share = get_package_share_directory('dual_chess_moveit_config')
    description = os.path.join(get_package_share_directory('dual_chess_description'), 'urdf', 'dual_chess.urdf.xacro')
    config = (MoveItConfigsBuilder('dual_chess', package_name='dual_chess_moveit_config')
              .robot_description(file_path=description, mappings={'hardware_mode': 'mock', 'include_board': 'false', 'geometry_file': LaunchConfiguration('geometry_file').perform(context)})
              .robot_description_semantic(file_path='config/dual_chess.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
              .to_moveit_configs())
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[config.robot_description], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[os.path.join(share, 'config', 'ros2_controllers.yaml')],
             remappings=[('~/robot_description', '/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'robot1_controller', 'robot2_controller', '--controller-manager-timeout', '60'], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[config.to_dict()], output='screen'),
        Node(package='chess_motion_interface', executable='motion_interface', output='screen', parameters=[{'geometry_file': LaunchConfiguration('geometry_file'), 'initial_fen': ParameterValue(LaunchConfiguration('initial_fen'), value_type=str), 'accept_move_plans': ParameterValue(LaunchConfiguration('accept_move_plans'), value_type=bool), 'publish_mock_readiness': ParameterValue(LaunchConfiguration('publish_mock_readiness'), value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'config', 'moveit.rviz')],
             parameters=[config.to_dict()], condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('dual_chess_description'), 'config', 'chess_geometry.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('geometry_file', default_value=default),
        DeclareLaunchArgument('initial_fen', default_value=''),
        DeclareLaunchArgument('accept_move_plans', default_value='false'),
        DeclareLaunchArgument('publish_mock_readiness', default_value='false'),
        OpaqueFunction(function=setup),
    ])
