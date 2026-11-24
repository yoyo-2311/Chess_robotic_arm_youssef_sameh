"""The physics backend shares measured geometry without fake static inertias."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import xacro

ROOT=Path(__file__).parents[2]


def test_static_enclosure_has_one_physics_owner_and_full_rsp_tf_is_preserved():
 path=ROOT/'dual_chess_gazebo/launch/gazebo.launch.py'
 spec=importlib.util.spec_from_file_location('gazebo_workcell_launch',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 profile=ROOT/'chess_workcell_description/config/workcell_sim.yaml';geometry=yaml.safe_load(profile.read_text())
 original=xacro.process_file(str(ROOT/'dual_chess_description/urdf/dual_chess.urdf.xacro'),mappings={'geometry_file':str(profile)}).toxml()
 robot,environment=module.static_workcell_assets(original,geometry,ROOT/'chess_workcell_description')
 rsp=ET.fromstring(original);physics=ET.fromstring(robot);static=ET.fromstring(environment)
 assert rsp.find("link[@name='workcell_controle_box']") is not None
 assert not any(x.get('name','').startswith('workcell_') for x in physics)
 assert physics.find("link[@name='robot1_base_link']") is not None
 assert static.find('model/static').text=='true' and static.find('.//inertial') is None
 assert static.find('.//visual/geometry/mesh/uri').text==static.find('.//collision/geometry/mesh/uri').text
 assert Path(static.find('.//collision/geometry/mesh/uri').text).is_file()
 assert static.find('model/pose').text==' '.join(map(str,[*geometry['workcell']['position'],*geometry['workcell']['rpy']]))
 args=module.workcell_spawn_arguments(geometry,environment)
 for flag,value in zip(['-x','-y','-z','-R','-P','-Y'],[*geometry['workcell']['position'],*geometry['workcell']['rpy']]):assert float(args[args.index(flag)+1])==value
 for arm in ['robot1','robot2']:
  assert physics.find(f"joint[@name='{arm}_mount_joint']/origin").attrib==rsp.find(f"joint[@name='{arm}_mount_joint']/origin").attrib
 disabled=dict(geometry,workcell={'enabled':False})
 assert module.static_workcell_assets(original,disabled,ROOT/'chess_workcell_description')==(original,None)


def test_gazebo_floor_does_not_cut_through_the_normalized_platform():
 path=ROOT/'dual_chess_gazebo/launch/gazebo.launch.py'
 spec=importlib.util.spec_from_file_location('gazebo_workcell_world',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 profile=ROOT/'chess_workcell_description/config/workcell_sim.yaml';geometry=yaml.safe_load(profile.read_text())
 original=(ROOT/'dual_chess_gazebo/worlds/chess_world.sdf').read_text()
 converted,z=module.world_for_workcell(original,geometry,ROOT/'chess_workcell_description')
 assert -.232<z<-.231
 assert float(ET.fromstring(converted).find("world/model[@name='ground_plane']/pose").text.split()[2])==z
 assert module.world_for_workcell(original,{'workcell':{'enabled':False}},ROOT/'chess_workcell_description')==(original,None)


def test_vertical_simulation_tuning_respects_unchanged_effort_and_joint_limits():
 profile=ROOT/'chess_workcell_description/config/workcell_sim.yaml'
 xml=xacro.process_file(str(ROOT/'dual_chess_description/urdf/dual_chess.urdf.xacro'),mappings={'geometry_file':str(profile),'sim_gazebo':'true'}).toxml()
 root=ET.fromstring(xml)
 for arm in ['robot1','robot2']:
  joint=root.find(f"joint[@name='{arm}_joint1']");limit=joint.find('limit')
  assert (float(limit.get('lower')),float(limit.get('upper')),float(limit.get('effort')))==(0.,.1,100.)
  plugin=next(p for p in root.findall('gazebo/plugin') if p.find('joint_name') is not None and p.find('joint_name').text==arm+'_joint1')
  assert float(plugin.find('p_gain').text)==10000.
  assert float(plugin.find('cmd_max').text)<=float(limit.get('effort'))
  assert -float(plugin.find('cmd_min').text)<=float(limit.get('effort'))
