"""Integration of unmodified robot foot geometry with measured CAD seats."""
from pathlib import Path
import numpy as np
import pytest
import yaml
from chess_workcell_description.assembly import Assembly,read_stl,transform
ROOT=Path(__file__).parents[2]/'chess_workcell_description'
def config():return yaml.safe_load((ROOT/'config/workcell_sim.yaml').read_text())


def test_measured_mount_centers_and_faces_match_unscaled_cad():
 from chess_workcell_description.mounting import measure_seats,measure_base,mount_for_seat
 box=Assembly(ROOT).components(config())[0]
 seats=measure_seats(ROOT/'meshes/controle_box.STL',box[2]);base=measure_base(ROOT.parent/'dual_chess_description/meshes/scara_v2/base_link.STL')
 assert [s['center'] for s in seats]==pytest.approx(np.array([[-.4,0,-.033],[.4,0,-.033]]),abs=1e-7)
 assert base['center_native']==pytest.approx([.0031,.43752130866,.0981],abs=1e-7)
 assert base['radius']==pytest.approx(.1397,abs=1e-7)
 for seat,arm in zip(seats,['robot1','robot2']):
  mount=config()['mounts'][arm];derived=mount_for_seat(seat,base,mount['rpy'][2])
  assert mount['position']==pytest.approx(derived['position'],abs=1e-9)
  world_base=transform(mount['position'],mount['rpy']) @ transform([0,0,0],[-np.pi/2,0,0])
  assert (world_base @ [*base['center_native'],1])[:3]==pytest.approx(seat['center'],abs=1e-9)
  assert world_base[:3,:3] @ base['normal_native']==pytest.approx([0,0,-1],abs=1e-9)
  assert seat['fit_max_error']<1e-7 and base['fit_max_error']<1e-7


def test_refined_pockets_remove_only_coarse_polygon_interference():
 from scipy.spatial import ConvexHull
 from chess_workcell_description.mounting import measure_seats
 raw=config();box=Assembly(ROOT).components(raw)[0]
 seats=measure_seats(ROOT/'meshes/controle_box.STL',box[2])
 original=read_stl(ROOT/'meshes/controle_box.STL');refined=read_stl(ROOT/'meshes/controle_box_refined.STL')
 assert refined.min((0,1))==pytest.approx(original.min((0,1)),abs=2e-7)
 assert refined.max((0,1))==pytest.approx(original.max((0,1)),abs=2e-7)
 base=read_stl(ROOT.parent/'dual_chess_description/meshes/scara_v2/base_link.STL').reshape(-1,3)
 base=base[abs(base[:,1]-base[:,1].max())<3e-7]
 for seat,arm in zip(seats,['robot1','robot2']):
  mount=raw['mounts'][arm];tb=transform(mount['position'],mount['rpy']) @ transform([0,0,0],[-np.pi/2,0,0])
  foot=(tb @ np.column_stack([base,np.ones(len(base))]).T).T[:,:2]
  clearances=[]
  for triangles in [original,refined]:
   v=triangles.reshape(-1,3);v=(box[2] @ np.column_stack([v,np.ones(len(v))]).T).T[:,:3]
   rim=v[(abs(v[:,2]-seat['center'][2])<3e-7)&(abs(np.linalg.norm(v[:,:2]-seat['center'][:2],axis=1)-.14)<3e-7),:2]
   hull=rim[ConvexHull(rim).vertices];following=np.roll(hull,-1,axis=0);edge=following-hull
   relative=foot[:,None,:]-hull
   signed=(edge[None,:,0]*relative[:,:,1]-edge[None,:,1]*relative[:,:,0])/np.linalg.norm(edge,axis=1)
   clearances.append(signed.min())
  assert clearances[0]<-.0009  # Reproduces the original ~1.05mm false overlap.
  assert clearances[1]>.00028  # Same 140mm circle; 139.7mm foot now fits.


def test_refined_visual_and_collision_mesh_agree():
 import xml.etree.ElementTree as ET
 import xacro
 path=ROOT.parent/'dual_chess_description/urdf/dual_chess.urdf.xacro'
 root=ET.fromstring(xacro.process_file(str(path),mappings={'geometry_file':str(ROOT/'config/workcell_sim.yaml')}).toxml())
 link=root.find("link[@name='workcell_controle_box']")
 assert ET.tostring(link.find('visual/geometry'))==ET.tostring(link.find('collision/geometry'))
 assert link.find('collision/geometry/mesh').get('filename').endswith('/controle_box_refined.STL')
