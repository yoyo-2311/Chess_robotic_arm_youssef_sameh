"""Gazebo Harmonic simulation of the dual-arm chess robot (Week-1 foundation).

Starts: gz sim (chess_world.sdf), robot_state_publisher (sim time), ros_gz_sim `create`
(spawns a generated physics URDF as model "dual_chess"), ros_gz_bridge
(/clock, /joint_states, /robotN/joint1/cmd_pos (m),
 /robotN/joint2/cmd_pos and /robotN/joint3/cmd_pos (rad)).
RViz is NOT started here; use dual_chess_bringup simulation.launch.py for both.
"""
import os
import shlex
import xacro
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

WORLD_NAME = 'chess_world'   # must match worlds/chess_world.sdf and config/bridge.yaml
MODEL_NAME = 'dual_chess'    # must match config/bridge.yaml


def static_workcell_assets(robot_description,geometry,workcell_share):
    """Physics owns a separate static enclosure; RSP keeps the full TF model.

    SDFormat drops massless fixed intermediate URDF links and leaves a broken
    workcell_box_joint frame. A static SDF needs no fabricated mass/inertia.
    """
    raw=geometry.get('workcell',{})
    if not raw.get('enabled',False):return robot_description,None
    robot=ET.fromstring(robot_description)
    for element in list(robot):
        if element.tag in ['link','joint'] and element.get('name','').startswith('workcell_'):
            robot.remove(element)
    sdf=ET.Element('sdf',version='1.9');model=ET.SubElement(sdf,'model',name='chess_workcell')
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,[*raw['position'],*raw['rpy']]))
    link=ET.SubElement(model,'link',name='enclosure')
    # Gazebo's mesh loader does not decode percent-escaped file:// paths.
    uri=str((Path(workcell_share)/'meshes'/raw.get('mesh','controle_box.STL')).resolve())
    for kind in ['visual','collision']:
        part=ET.SubElement(link,kind,name='enclosure_'+kind)
        mesh=ET.SubElement(ET.SubElement(part,'geometry'),'mesh');ET.SubElement(mesh,'uri').text=uri
    return ET.tostring(robot,encoding='unicode'),ET.tostring(sdf,encoding='unicode')


def workcell_spawn_arguments(geometry,sdf):
    # ros_gz_sim create overrides SDF model poses with its default zero pose.
    # Pass all six components explicitly from the same canonical configuration.
    pose=[*geometry['workcell']['position'],*geometry['workcell']['rpy']]
    args=['-world',WORLD_NAME,'-name','chess_workcell','-string',sdf]
    for flag,value in zip(['-x','-y','-z','-R','-P','-Y'],pose):args.extend([flag,str(value)])
    return args


def world_for_workcell(world_xml,geometry,workcell_share):
    """Place the simulated floor at the enclosure's CAD lower extent.

    The old Z=0 floor cuts through the normalized enclosure. This does not
    change board height or arm mounts; physical floor installation is unknown.
    """
    if not geometry.get('workcell',{}).get('enabled',False):return world_xml,None
    import numpy as np
    from chess_workcell_description.assembly import Assembly,read_stl
    _,mesh,t=Assembly(workcell_share).components(geometry)[0]
    vertices=read_stl(mesh).reshape(-1,3)
    floor_z=float((t @ np.column_stack([vertices,np.ones(len(vertices))]).T)[2].min())
    root=ET.fromstring(world_xml);ground=root.find("world/model[@name='ground_plane']")
    if ground is None:raise ValueError('Configured world has no ground_plane')
    pose=ground.find('pose')
    if pose is None:pose=ET.SubElement(ground,'pose')
    pose.text=f'0 0 {floor_z} 0 0 0'
    return ET.tostring(root,encoding='unicode'),floor_z


def setup(context):
    desc_share = get_package_share_directory('dual_chess_description')
    gz_share = get_package_share_directory('dual_chess_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(gz_share, 'worlds', 'chess_world.sdf')
    bridge_cfg = os.path.join(gz_share, 'config', 'bridge.yaml')
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('dual_chess_description'), 'urdf', 'dual_chess.urdf.xacro'])


    # let Gazebo resolve package://dual_chess_description/meshes/robot_arm/*.STL
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.pathsep.join([os.path.dirname(desc_share),os.path.dirname(get_package_share_directory('chess_workcell_description'))]))


    mappings = {key:LaunchConfiguration(key).perform(context) for key in ['geometry_file','robot1_enabled','robot2_enabled','use_meshes']}
    mappings['sim_gazebo'] = 'true'
    robot_description = xacro.process_file(os.path.join(desc_share,'urdf','dual_chess.urdf.xacro'),mappings=mappings).toxml()

    with open(mappings['geometry_file']) as stream:geometry=yaml.safe_load(stream)
    physics_robot,static_workcell=static_workcell_assets(robot_description,geometry,get_package_share_directory('chess_workcell_description'))
    generated=tempfile.TemporaryDirectory(prefix='dual_chess_gazebo_')
    robot_file=Path(generated.name)/'robot.urdf';robot_file.write_text(physics_robot)
    if static_workcell:
        world_xml,floor_z=world_for_workcell(Path(world).read_text(),geometry,get_package_share_directory('chess_workcell_description'))
        world_file=Path(generated.name)/'world.sdf';world_file.write_text(world_xml);world=str(world_file)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-v 2 ', LaunchConfiguration('gz_args'), ' ', shlex.quote(world)],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    cleanup=RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda context:generated.cleanup())]))

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', WORLD_NAME, '-name', MODEL_NAME,
                       '-file', str(robot_file)],
        )],
    )

    environment_spawn=[]
    if static_workcell:
        environment_spawn=[TimerAction(period=LaunchConfiguration('spawn_delay'),actions=[Node(
            package='ros_gz_sim',executable='create',output='screen',
            arguments=workcell_spawn_arguments(geometry,static_workcell))])]

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg}],
    )

    return [resource_path, gz_sim, rsp, spawn, bridge, cleanup,*environment_spawn]


def generate_launch_description():
    desc_share = get_package_share_directory('dual_chess_description')
    declared = [
        DeclareLaunchArgument('geometry_file', default_value=os.path.join(desc_share,'config','chess_geometry.yaml')),
        DeclareLaunchArgument('gz_args', default_value='-r',
                              description="Extra gz sim flags, e.g. '-r -s' (headless server)"),
        DeclareLaunchArgument('robot1_enabled', default_value='true'),
        DeclareLaunchArgument('robot2_enabled', default_value='true'),
        DeclareLaunchArgument('use_meshes', default_value='false'),
        DeclareLaunchArgument('spawn_delay', default_value='3.0',
                              description='Seconds to wait for gz sim before spawning'),
    ]

    return LaunchDescription(declared + [OpaqueFunction(function=setup)])
