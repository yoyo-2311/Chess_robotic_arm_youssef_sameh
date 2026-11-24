#!/usr/bin/env python3
"""Native RViz evidence, measured seats and provisional arm/yaw labels.

Uses the normal Qt manual backend (one joint publisher). RViz is a separately
owned companion. No camera/vision/game-state or collision permissions change.
"""
import argparse,ctypes,json,os,signal,subprocess,time
from pathlib import Path
import yaml


def main():
 p=argparse.ArgumentParser();p.add_argument('--domain',type=int,default=226);a=p.parse_args()
 ws=Path(__file__).resolve().parents[1];out=ws/'docs/mount_validation';out.mkdir(exist_ok=True)
 for lib in (ws/'.qt-deps').glob('os/usr/lib/*/libxcb-cursor.so.0'):ctypes.CDLL(str(lib),mode=ctypes.RTLD_GLOBAL)
 os.environ['QT_QPA_PLATFORM']='xcb'
 for key in ['QT_PLUGIN_PATH','QT_QPA_PLATFORM_PLUGIN_PATH']:os.environ.pop(key,None)
 from PySide6.QtWidgets import QApplication
 from PySide6.QtGui import QFont,QPainterPath
 from dual_scara_desktop.window import MainWindow
 from dual_scara_desktop.controller import DesktopController
 from dual_scara_desktop.modes import MODES
 import rclpy
 from visualization_msgs.msg import Marker,MarkerArray
 from geometry_msgs.msg import Point
 app=QApplication([]);profile=ws/'src/chess_workcell_description/config/workcell_sim.yaml'
 w=MainWindow(profile,a.domain);c=DesktopController(w,ws,a.domain);w.mode.setCurrentIndex(next(i for i,m in enumerate(MODES) if m.key=='manual'));w.rviz.setChecked(False);w.show()
 rclpy.init(domain_id=a.domain);node=rclpy.create_node('mount_alignment_labels');pub=node.create_publisher(MarkerArray,'/mount_alignment/markers',10)
 def pump():app.processEvents();rclpy.spin_once(node,timeout_sec=0);time.sleep(.03)
 def until(pred,timeout=40):
  end=time.monotonic()+timeout
  while not pred() and time.monotonic()<end:pump()
  assert pred(),w.status.text()
 raw=yaml.safe_load(profile.read_text());derivation=json.loads((out/'derivation.json').read_text());markers=MarkerArray()
 for i,(arm,seat) in enumerate(zip(['robot1','robot2'],derivation['seats'])):
  xyz=seat['center'];letter='AB'[i];yaw=raw['mounts'][arm]['rpy'][2]
  label=Marker();label.header.frame_id='world';label.ns='mount_labels';label.id=i;label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD;label.pose.orientation.w=1.
  label.pose.position.x=xyz[0];label.pose.position.y=.32;label.pose.position.z=1.05;label.scale.z=.028;label.color.r=label.color.g=label.color.b=label.color.a=1.
  label.type=Marker.LINE_LIST;label.pose.position.x=label.pose.position.y=label.pose.position.z=0.;label.scale.x=.001
  lines=[f'SEAT {letter} / {arm} PREVIEW',f'X={xyz[0]:+.3f} Y=0 Z=-0.033',f'YAW={yaw:.3f} UNCONFIRMED']
  for row,text in enumerate(lines):
   path=QPainterPath();path.addText(0,0,QFont('DejaVu Sans',100),text);bounds=path.boundingRect();factor=.025/bounds.height()
   for polygon in path.toSubpathPolygons():
    points=list(polygon)
    for one,two in zip(points[:-1],points[1:]):
     for point in [one,two]:label.points.append(Point(x=float(xyz[0]+(point.x()-bounds.center().x())*factor),y=float(.37-row*.04-(point.y()-bounds.center().y())*factor),z=1.05))
  markers.markers.append(label)
  center=Marker();center.header.frame_id='world';center.ns='centers';center.id=i;center.type=Marker.SPHERE;center.pose.orientation.w=1.
  center.pose.position.x=xyz[0];center.pose.position.y=xyz[1];center.pose.position.z=.015;center.scale.x=center.scale.y=center.scale.z=.015;center.color.a=1.;center.color.r=1.;center.color.g=.6;markers.markers.append(center)
  arrow=Marker();arrow.header.frame_id='world';arrow.ns='yaw';arrow.id=i;arrow.type=Marker.ARROW;arrow.pose.orientation.w=1.
  import math
  arrow.points=[Point(x=float(xyz[0]),y=float(xyz[1]),z=.025),Point(x=float(xyz[0]+.17*math.cos(yaw)),y=float(xyz[1]+.17*math.sin(yaw)),z=.025)]
  arrow.scale.x=.008;arrow.scale.y=.025;arrow.scale.z=.04;arrow.color.a=1.;arrow.color.r=1. if i==0 else .1;arrow.color.g=.3 if i==0 else 1.;arrow.color.b=.2;markers.markers.append(arrow)
  ring=Marker();ring.header.frame_id='world';ring.ns='seat_rings';ring.id=i;ring.type=Marker.LINE_STRIP;ring.pose.orientation.w=1.;ring.scale.x=.003;ring.color.a=1.;ring.color.r=arrow.color.r;ring.color.g=arrow.color.g;ring.color.b=arrow.color.b
  ring.points=[Point(x=float(xyz[0]+.14*math.cos(j*2*math.pi/128)),y=float(xyz[1]+.14*math.sin(j*2*math.pi/128)),z=.02) for j in range(129)];markers.markers.append(ring)
 def pump_labels():pub.publish(markers);pump()
 # Query X11 only for our separately launched RViz window and save that window.
 x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.restype=ctypes.c_void_p;x.XOpenDisplay.argtypes=[ctypes.c_char_p];display=x.XOpenDisplay(None);assert display
 x.XDefaultRootWindow.argtypes=[ctypes.c_void_p];x.XDefaultRootWindow.restype=ctypes.c_ulong
 x.XQueryTree.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),ctypes.POINTER(ctypes.c_uint)]
 x.XFetchName.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.POINTER(ctypes.c_char_p)];x.XFree.argtypes=[ctypes.c_void_p]
 def windows(root):
  parent=ctypes.c_ulong();r=ctypes.c_ulong();children=ctypes.POINTER(ctypes.c_ulong)();count=ctypes.c_uint()
  if not x.XQueryTree(display,root,ctypes.byref(r),ctypes.byref(parent),ctypes.byref(children),ctypes.byref(count)):return []
  ids=[children[i] for i in range(count.value)]
  if children:x.XFree(children)
  found=[]
  for wid in ids:
   name=ctypes.c_char_p()
   if x.XFetchName(display,wid,ctypes.byref(name)) and name.value:
    title=name.value.decode(errors='replace');x.XFree(name)
    if 'mount_alignment_' in title:found.append(wid)
   found+=windows(wid)
  return found
 rviz=None
 try:
  c.start_selected();until(lambda:c.ready)
  report={'profile':str(profile),'manual_ready':True,'joint_publishers':c.graph['joint_publishers'],'joints':c.data['joints'],'assignment':'A/B unconfirmed, preview only','screenshots':[]}
  for view in ['topdown','perspective']:
   config=yaml.safe_load((ws/'src/dual_chess_description/rviz/dual_chess.rviz').read_text());vm=config['Visualization Manager']
   vm['Displays']=[d for d in vm['Displays'] if d['Name'] not in ['TF','Grid','RobotModel (collision check)']]
   vm['Displays'].append({'Class':'rviz_default_plugins/MarkerArray','Enabled':True,'Name':'Measured seat centers and provisional yaw','Topic':{'Value':'/mount_alignment/markers','Depth':5,'Durability Policy':'Volatile','Reliability Policy':'Reliable','History Policy':'Keep Last'},'Value':True})
   if view=='topdown':vm['Views']['Current']={'Class':'rviz_default_plugins/TopDownOrtho','Name':'Measured mount centers','Scale':550.,'X':0.,'Y':0.,'Angle':0.,'Target Frame':'<Fixed Frame>'}
   config['Window Geometry']={'Height':900,'Width':1500}
   path=out/('mount_alignment_'+view+'.rviz');path.write_text(yaml.safe_dump(config,sort_keys=False))
   env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),ROS_LOG_DIR=str(ws/'.desktop-state/mount-rviz'))
   log=(out/('rviz_'+view+'.log')).open('w');rviz=subprocess.Popen(['ros2','run','rviz2','rviz2','-d',str(path)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   end=time.monotonic()+15
   while time.monotonic()<end:pump_labels()
   assert rviz.poll() is None,(out/('rviz_'+view+'.log')).read_text()
   ids=windows(x.XDefaultRootWindow(display));assert ids,'RViz X11 window missing'
   pix=app.primaryScreen().grabWindow(ids[0]);assert not pix.isNull();image=out/('rviz_'+view+'.png');assert pix.save(str(image));report['screenshots'].append(str(image))
   os.killpg(rviz.pid,signal.SIGINT)
   try:rviz.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(rviz.pid,signal.SIGTERM);rviz.wait(timeout=5)
   rviz=None;log.close()
  (out/'visual_evidence.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
 finally:
  if rviz and rviz.poll() is None:os.killpg(rviz.pid,signal.SIGTERM);rviz.wait(timeout=8)
  c.close();end=time.monotonic()+25
  while (c.supervisor.alive() or c.bridge.alive()) and time.monotonic()<end:pump()
  assert not c.supervisor.alive() and not c.bridge.alive();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
