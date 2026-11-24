"""Check reported world contacts against the unchanged collision mesh surfaces."""
import argparse,hashlib,json,struct,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import xacro
from chess_motion_interface.model import Model


def triangles(path):
 data=path.read_bytes();n=struct.unpack_from('<I',data,80)[0]
 assert len(data)==84+50*n
 return np.frombuffer(data,offset=84,count=n,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]))['vertices'].astype(float)

def distance(point,t):
 a,b,c=t[:,0],t[:,1],t[:,2];ab=b-a;ac=c-a;ap=point-a
 d00=np.sum(ab*ab,1);d01=np.sum(ab*ac,1);d11=np.sum(ac*ac,1)
 d20=np.sum(ap*ab,1);d21=np.sum(ap*ac,1);den=d00*d11-d01*d01
 safe=np.where(abs(den)>1e-24,den,1.)
 v=(d11*d20-d01*d21)/safe;w=(d00*d21-d01*d20)/safe
 projection=a+v[:,None]*ab+w[:,None]*ac
 inside=(v>=0)&(w>=0)&(v+w<=1)&(abs(den)>1e-24)
 ds=np.where(inside,np.linalg.norm(point-projection,axis=1),np.inf)
 for first,last in [(a,b),(b,c),(c,a)]:
  edge=last-first;length=np.sum(edge*edge,1)
  u=np.clip(np.sum((point-first)*edge,1)/np.where(length>0,length,1),0,1)
  ds=np.minimum(ds,np.linalg.norm(point-(first+u[:,None]*edge),axis=1))
 return float(ds.min())

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',required=True);p.add_argument('--inspection',required=True);a=p.parse_args()
 ws=Path(a.workspace);desc=ws/'src/dual_chess_description';d=json.loads(Path(a.inspection).read_text())
 xml=ET.fromstring(xacro.process_file(str(desc/'urdf/dual_chess.urdf.xacro'),mappings={'include_board':'false'}).toxml());model=Model(xml)
 result={'mesh_audit':{},'contacts':[]}
 for link in ['link1','link3','link4']:
  mesh=desc/'meshes/scara_v2'/(link+'.STL');ref=desc/'reference/scara_v2/meshes'/(link+'.STL')
  t=triangles(mesh);element=xml.find("link[@name='robot1_"+link+"']")
  collision=element.find('collision');visual=element.find('visual')
  result['mesh_audit'][link]={'sha256':hashlib.sha256(mesh.read_bytes()).hexdigest(),'identical_to_archive_reference':mesh.read_bytes()==ref.read_bytes(),
   'bounds_m':[t.min((0,1)).tolist(),t.max((0,1)).tolist()],
   'same_visual_collision_origin':collision.find('origin').attrib==visual.find('origin').attrib,
   'same_visual_collision_geometry':ET.tostring(collision.find('geometry'))==ET.tostring(visual.find('geometry'))}
 for branch in d['branches']:
  sample=branch['first_invalid']
  if not sample:continue
  q=dict(d['initial_joints']);q.update({f'robot1_joint{i}':v for i,v in enumerate(sample['q'],1)})
  for contact in sample['contacts']:
   item={'branch':branch['index'],'tcp':sample['tcp'],'world_point':contact['point'],'depth_m':contact['depth'],'links':{}}
   for name in contact['pair']:
    if not name.startswith('robot1_'):continue
    local=(np.linalg.inv(model.matrix(name,q))@np.array([*contact['point'],1.]))[:3]
    mesh=desc/'meshes/scara_v2'/(name.removeprefix('robot1_')+'.STL')
    item['links'][name]={'point_in_collision_mesh_frame':local.tolist(),'nearest_triangle_distance_m':distance(local,triangles(mesh)),
                         'world_transform':model.matrix(name,q).tolist()}
   result['contacts'].append(item)
 output=Path(a.inspection).with_name('mesh_contact_audit.json');output.write_text(json.dumps(result,indent=2))
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
