"""Read-only MoveIt candidate sweep, except scoped selected-piece touch permissions."""
import argparse,copy,json,math,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy,xacro
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningSceneComponents,PlanningScene
from moveit_msgs.srv import GetPlanningScene,GetStateValidity
from chess_motion_interface.backend import Backend
from chess_motion_interface.geometry import Geometry
from chess_motion_interface.model import Model
from chess_motion_interface.scene import Scene


def main():
 p=argparse.ArgumentParser();p.add_argument('--geometry',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 rclpy.init();node=Node('pickup_scene_inspector');node.io_group=ReentrantCallbackGroup()
 g=Geometry.load(a.geometry);description=Path(get_package_share_directory('dual_chess_description'))
 xml=xacro.process_file(str(description/'urdf/dual_chess.urdf.xacro'),mappings={'geometry_file':a.geometry,'include_board':'false'}).toxml()
 model=Model(ET.fromstring(xml));b=Backend(node,model,g,lambda *args,**kwargs:None)
 executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node);worker=Thread(target=executor.spin,daemon=True);worker.start()
 client=node.create_client(GetPlanningScene,'/get_planning_scene',callback_group=node.io_group)
 def get_scene():return b.wait(client.call_async(GetPlanningScene.Request(components=PlanningSceneComponents(components=1023)))).scene
 deadline=time.monotonic()+10
 while b.latest is None and time.monotonic()<deadline:time.sleep(.05)
 assert client.wait_for_service(timeout_sec=10)
 before=get_scene();scene=Scene(b,g);
 for service in [scene.get_client,b.valid_client,b.cart_client,b.scene_client]:assert service.wait_for_service(timeout_sec=10)
 saved=copy.deepcopy(before.allowed_collision_matrix)
 report={'initial_joints':b.joints(),'geometry':g.raw,'branches':[],'meshes':{},'scene_objects':[]}
 for obj in before.world.collision_objects:
  if obj.id in ['chess_piece_g1','chess_piece_h2']:
   report['scene_objects'].append({'id':obj.id,'frame':obj.header.frame_id,'dimensions':[list(x.dimensions) for x in obj.primitives],
    'object_pose':([obj.pose.position.x,obj.pose.position.y,obj.pose.position.z,obj.pose.orientation.x,obj.pose.orientation.y,obj.pose.orientation.z,obj.pose.orientation.w] if hasattr(obj,'pose') else None),
    'pose':[[v.position.x,v.position.y,v.position.z,v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w] for v in obj.primitive_poses]})
 for name in ['robot1_link1','robot1_link3','robot1_link4']:
  link=ET.fromstring(xml).find("link[@name='"+name+"']")
  report['meshes'][name]={'collision':ET.tostring(link.find('collision'),encoding='unicode'),'visual':ET.tostring(link.find('visual'),encoding='unicode')}
 def validity(state):
  result=b.wait(b.valid_client.call_async(GetStateValidity.Request(robot_state=state)))
  return {'valid':result.valid,'contacts':[{'pair':[c.contact_body_1,c.contact_body_2],'frame':c.header.frame_id,'depth':c.depth,
     'point':[c.position.x,c.position.y,c.position.z],'normal':[c.normal.x,c.normal.y,c.normal.z]} for c in result.contacts]}
 def state_for(q):
  s=copy.deepcopy(b.state());values=dict(zip(s.joint_state.name,s.joint_state.position));values.update({f'robot1_joint{i}':v for i,v in enumerate(q,1)})
  s.joint_state.position=[float(values[n]) for n in s.joint_state.name];return s
 above=g.square('g1',g.travel(g.piece_height('N')));target=g.square('g1',g.grasp('N'))
 report.update(above=above.tolist(),target=target.tolist())
 try:
  scene.touch('chess_piece_g1','robot1',True)
  for index,solution in enumerate(model.solutions('robot1',above)):
   q=np.asarray(solution['q']);s=state_for(q);request=b._cartesian_request('robot1',target,s)
   path=b.wait(b.cart_client.call_async(request));item={'index':index,'above_q':q.tolist(),'above_validity':validity(s),
    'fraction':path.fraction,'code':path.error_code.val,'path':[],'sweep':[]}
   for point in path.solution.joint_trajectory.points:
    joints=dict(zip(path.solution.joint_trajectory.joint_names,point.positions));merged=b.joints();merged.update(joints)
    item['path'].append({'q':joints,'tcp':model.fk('robot1_tool0',merged).tolist()})
   current=model.fk('robot1_tool0',{f'robot1_joint{i}':v for i,v in enumerate(q,1)})
   shifted=q.copy();shifted[0]+=.001
   axis=(model.fk('robot1_tool0',{f'robot1_joint{i}':v for i,v in enumerate(shifted,1)})-current)/.001
   delta=float(np.dot(target-current,axis)/np.dot(axis,axis));steps=math.ceil(abs(delta)/.00025)
   for step in range(steps+1):
    sample=q.copy();sample[0]+=delta*step/steps;state=state_for(sample)
    joints=dict(zip(state.joint_state.name,state.joint_state.position))
    item['sweep'].append({'step':step,'q':sample.tolist(),'tcp':model.fk('robot1_tool0',joints).tolist(),**validity(state)})
   item['first_invalid']=next((v for v in item['sweep'] if not v['valid']),None)
   report['branches'].append(item)
 finally:
  b.apply(PlanningScene(is_diff=True,allowed_collision_matrix=saved))
  report['joint_state_after']=b.joints();Path(a.output).write_text(json.dumps(report,indent=2))
  executor.shutdown();node.destroy_node();rclpy.shutdown();worker.join(3)
if __name__=='__main__':main()
