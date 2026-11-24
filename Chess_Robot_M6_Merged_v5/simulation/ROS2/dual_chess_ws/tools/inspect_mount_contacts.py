#!/usr/bin/env python3
"""Read-only live collision contacts, joint feedback, scene and TF evidence."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from moveit_msgs.srv import GetPlanningScene,GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
from tf2_ros import Buffer,TransformListener


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);a=p.parse_args();rclpy.init();n=Node('mount_contact_audit')
 buffer=Buffer();listener=TransformListener(buffer,n)
 def call(client,request):
  assert client.wait_for_service(timeout_sec=15),client.srv_name
  f=client.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=15);assert f.done();return f.result()
 client=n.create_client(GetPlanningScene,'/get_planning_scene');valid=n.create_client(GetStateValidity,'/check_state_validity')
 scene=call(client,GetPlanningScene.Request(components=PlanningSceneComponents(components=1023))).scene
 state=scene.robot_state;result=call(valid,GetStateValidity.Request(robot_state=state))
 report={'valid':result.valid,'joints':dict(zip(state.joint_state.name,state.joint_state.position)),
  'contacts':[{'pair':[c.contact_body_1,c.contact_body_2],'depth':c.depth,'world_point':[c.position.x,c.position.y,c.position.z],
               'normal':[c.normal.x,c.normal.y,c.normal.z]} for c in result.contacts],
  'world_objects':[{'id':o.id,'frame':o.header.frame_id,'meshes':len(o.meshes),'primitives':len(o.primitives)} for o in scene.world.collision_objects], 'tf':{}}
 end=time.monotonic()+3
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 for arm in ['robot1','robot2']:
  for suffix in ['mount','base_link','tool0']:
   name=arm+'_'+suffix
   try:
    t=buffer.lookup_transform('world',name,rclpy.time.Time()).transform
    report['tf'][name]={'translation':[t.translation.x,t.translation.y,t.translation.z],'quaternion':[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]}
   except Exception as e:report['tf'][name]={'error':str(e)}
 a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
