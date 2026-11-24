from pathlib import Path
import copy
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from chess_workcell_description.assembly import Assembly,read_stl,transform

ROOT=Path(__file__).parents[1]


def config():return yaml.safe_load((ROOT/'config/workcell_sim.yaml').read_text())


def test_original_meshes_preserved_and_static_ros2_model_has_valid_names():
 for name in ['chess_board.STL','controle_box.STL']:
  assert (ROOT/'meshes'/name).read_bytes()==(ROOT/'reference/original/meshes'/name).read_bytes()
 urdf=ET.parse(ROOT/'urdf/workcell.urdf').getroot()
 assert all(x.get('type')=='fixed' for x in urdf.findall('joint'))
 assert not urdf.findall('.//inertial')
 assert all(m.get('filename').startswith('package://chess_workcell_description/') for m in urdf.findall('.//mesh'))


def test_canonical_conversion_matches_original_fixed_joint_and_board_bounds():
 components=Assembly(ROOT).components(config());box,board=components
 original=ET.parse(ROOT/'reference/original/urdf/AssemBase.SLDASM.urdf').getroot()
 origin=original.find('joint/origin')
 xyz=np.fromstring(origin.get('xyz'),sep=' ');rpy=np.fromstring(origin.get('rpy'),sep=' ')
 assert np.allclose(box[2] @ transform(xyz,rpy),board[2],atol=1e-12)
 vertices=read_stl(board[1]).reshape(-1,3)
 world=(board[2] @ np.column_stack([vertices,np.ones(len(vertices))]).T).T[:,:3]
 assert np.allclose(world.min(0),[-.14,-.14,0],atol=1e-7)
 assert np.allclose(world.max(0),[.14,.14,.01],atol=1e-7)


def test_board_pose_update_never_moves_control_box():
 a=config();b=copy.deepcopy(a);b['board']['position']=[.02,.03,.004];b['board']['rpy']=[0,0,.2]
 first=Assembly(ROOT).components(a);second=Assembly(ROOT).components(b)
 assert np.allclose(first[0][2],second[0][2])
 assert not np.allclose(first[1][2],second[1][2])


def test_cad_never_silently_scales_to_another_board():
 data=config();data['board']['length']=.24
 with pytest.raises(ValueError,match='CAD board is'):Assembly(ROOT).components(data)


def test_invalid_stl_rejected(tmp_path):
 p=tmp_path/'bad.STL';p.write_bytes(b'not a mesh')
 with pytest.raises(ValueError):read_stl(p)

