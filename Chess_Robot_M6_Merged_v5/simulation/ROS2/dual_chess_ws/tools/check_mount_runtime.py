#!/usr/bin/env python3
"""Focused Qt/ROS comparison and Gazebo feedback checks for the seated layout."""
import argparse,json,time,uuid,subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt,QPoint,QTimer
from PySide6.QtTest import QTest
from dual_scara_desktop.controller import DesktopController
from dual_scara_desktop.window import MainWindow
from dual_scara_desktop.modes import MODES
from dual_scara_desktop.environment import local_environment
from chess_brain_handover.core import chess


def main():
 p=argparse.ArgumentParser();p.add_argument('--mode',choices=['exact','gazebo'],required=True);p.add_argument('--domain',type=int,default=228);a=p.parse_args()
 ws=Path(__file__).resolve().parents[1];out=ws/'docs/mount_validation';profile=ws/'src/chess_workcell_description/config/workcell_sim.yaml'
 app=QApplication([]);app.setQuitOnLastWindowClosed(False);w=MainWindow(profile,a.domain);c=DesktopController(w,ws,a.domain,headless_gazebo=True);w.rviz.setChecked(False);w.show()
 w.mode.setCurrentIndex(next(i for i,m in enumerate(MODES) if m.key==('mock' if a.mode=='exact' else 'gazebo')))
 if a.mode=='exact':w.scenario.setCurrentText('After c4 e5: g1f3')
 events=[];original=c.consume
 def consume(kind,data,stamp):
  if kind in ['plan','ack','event','motion']:events.append({'kind':kind,'data':data,'stamp':stamp})
  original(kind,data,stamp)
 c.consume=consume;pulses=[];timer=QTimer();timer.setInterval(20);timer.timeout.connect(lambda:pulses.append(time.monotonic()));timer.start()
 def pump():app.processEvents();time.sleep(.02)
 def until(pred,timeout=60):
  end=time.monotonic()+timeout
  while not pred() and time.monotonic()<end:pump()
  assert pred(),w.status.text()
 report={'mode':a.mode,'cases':[]}
 try:
  QTest.mouseClick(w.start,Qt.LeftButton);until(lambda:c.ready)
  report['launch_log']=c.log_path
  if a.mode=='exact':
   before=c.data['state']['fen'];assert chess.Board(before).piece_at(chess.H2).symbol()=='P'
   for transport in ['qt','ros']:
    previous=(c.data['state'].get('last_rejection') or {}).get('plan_id');start=len(events)
    if transport=='qt':
     for sq in ['g1','f3']:QTest.mouseClick(w.board,Qt.LeftButton,pos=QPoint(*map(round,w.board.square_centre(sq))));pump()
    else:
     c.policy.enabled=False;c.publish('/chess/command',{'action':'uci','uci':'g1f3','request_id':'mount-ros-'+uuid.uuid4().hex})
    until(lambda:bool(c.data['state'].get('last_rejection')) and c.data['state']['last_rejection']['plan_id']!=previous and c.ready)
    s=c.data['state'];assert s['fen']==before and s['committed_plies']==0 and not s['pending_plan_id']
    acks=[e['data'] for e in events[start:] if e['kind']=='ack'];assert len(acks)==1 and acks[0]['safe_rejection'] and not acks[0]['arm_activity'] and not acks[0]['gripper_activity']
    report['cases'].append({'transport':transport,'rejection':s['last_rejection'],'h2_present':True,'fen_unchanged':True,'no_activity':True})
  else:
   report['initial_joints']=dict(c.data['joints']);report['clock']=c.data['clock']
   for arm in ['robot1','robot2']:
    w.arm.setCurrentText(arm);pump()
    for joint,target,tolerance in [('joint3',.10,.02),('joint1',.01,.003)]:
     name=arm+'_'+joint;w.joint_controls[name][0].setValue(target);QTest.mouseClick(w.apply_joints,Qt.LeftButton)
     end=time.monotonic()+6
     while time.monotonic()<end:pump()
     observed=c.data['joints'][name]
     report['cases'].append({'arm':arm,'joint':joint,'requested':target,'observed':observed,'tracking_within_tolerance':abs(observed-target)<tolerance})
   pool=ThreadPoolExecutor(1)
   future=pool.submit(subprocess.run,['gz','service','-s','/world/chess_world/scene/info','--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','5000','--req',''],env=local_environment(a.domain),capture_output=True,text=True)
   until(future.done,15);result=future.result();pool.shutdown();report['scene_query_returncode']=result.returncode
   (out/'gazebo_scene.txt').write_text(result.stdout+result.stderr)
   report['enclosure_in_scene']='chess_workcell' in result.stdout
   from google.protobuf import text_format
   from gz.msgs10.scene_pb2 import Scene
   import yaml,numpy as np
   scene=Scene();text_format.Parse(result.stdout,scene);model=next(m for m in scene.model if m.name=='chess_workcell')
   raw=yaml.safe_load(profile.read_text());position=[model.pose.position.x,model.pose.position.y,model.pose.position.z]
   report['physics_enclosure_position']=position;assert np.allclose(position,raw['workcell']['position'],atol=1e-9)
   from scipy.spatial.transform import Rotation
   q=model.pose.orientation;actual_rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
   assert np.allclose(actual_rotation,Rotation.from_euler('xyz',raw['workcell']['rpy']).as_matrix(),atol=1e-9)
   robot=next(m for m in scene.model if m.name=='dual_chess');report['physics_mounts']={}
   for arm in ['robot1','robot2']:
    mount=next(link for link in robot.link if link.name==arm+'_mount');pos=[mount.pose.position.x,mount.pose.position.y,mount.pose.position.z]
    assert np.allclose(pos,raw['mounts'][arm]['position'],atol=1e-9);report['physics_mounts'][arm]=pos
   text=Path(c.log_path).read_text(errors='replace');report['mesh_load_errors']=[line for line in text.splitlines() if any(s in line for s in ['Failed to load mesh','Unable to find file','Could not resolve file','FrameAttachedToGraph'])]
   assert not report['mesh_load_errors'],report['mesh_load_errors']
  w.grab().save(str(out/('qt_'+a.mode+'_final.png')))
 finally:
  c.close();end=time.monotonic()+25
  while (c.supervisor.alive() or c.bridge.alive()) and time.monotonic()<end:pump()
  report['clean_shutdown']=not c.supervisor.alive() and not c.bridge.alive();assert report['clean_shutdown']
  report['max_qt_timer_gap']=max((b-a for a,b in zip(pulses,pulses[1:])),default=0);report['events']=events
  (out/('runtime_'+a.mode+'.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='events'},indent=2))


if __name__=='__main__':main()
