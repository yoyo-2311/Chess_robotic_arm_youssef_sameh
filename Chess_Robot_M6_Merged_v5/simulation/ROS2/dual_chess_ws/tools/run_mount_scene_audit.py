#!/usr/bin/env python3
"""Own and reap one ROS launch while collecting a read-only mount audit."""
import argparse,json,os,subprocess,time
from pathlib import Path
from dual_scara_desktop.supervisor import ProcessSupervisor


def main():
 p=argparse.ArgumentParser();p.add_argument('--domain',type=int,default=225);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
 ws=Path(__file__).resolve().parents[1];s=ProcessSupervisor(ws/'.desktop-state',a.domain,grace=3)
 try:
  s.start(['ros2','launch','dual_chess_moveit_config','demo.launch.py','geometry_file:='+str(ws/'src/chess_workcell_description/config/workcell_sim.yaml'),'rviz:=false','publish_mock_readiness:=true'],1)
  deadline=time.monotonic()+25
  while time.monotonic()<deadline:
   if any('"status":"idle"' in line or 'initialization_failed' in line for _,line in s.logs):break
   time.sleep(.1)
  env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain))
  run=subprocess.run(['/usr/bin/python3',str(ws/'tools/inspect_mount_contacts.py'),'--output',str(a.output)],env=env,capture_output=True,text=True,timeout=45)
  print(run.stdout);assert run.returncode==0,run.stderr
 finally:
  s.shutdown();end=time.monotonic()+20
  while s.alive() and time.monotonic()<end:time.sleep(.05)
  assert not s.alive(),'Owned launch did not stop'


if __name__=='__main__':main()
