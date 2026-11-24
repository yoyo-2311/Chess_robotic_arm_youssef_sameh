"""Opt-in: normal Qt modes with the correctly seated, mechanically constrained CAD.

Board execution is EXPECTED TO REJECT. Never calls a plan preview a successful
chess execution. Every requested case records its actual stage/result.
"""
import argparse,json,time
from pathlib import Path
from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
from PySide6.QtCore import Qt,QPoint,QTimer
from dual_scara_desktop.window import MainWindow
from dual_scara_desktop.controller import DesktopController
from dual_scara_desktop.modes import MODES
from chess_brain_handover.core import chess


def main():
 p=argparse.ArgumentParser();p.add_argument('--domain',type=int,default=227);a=p.parse_args()
 ws=Path(__file__).resolve().parents[3];out=ws/'docs/mount_validation';profile=ws/'src/chess_workcell_description/config/workcell_sim.yaml'
 app=QApplication([]);app.setQuitOnLastWindowClosed(False);w=MainWindow(profile,a.domain);c=DesktopController(w,ws,a.domain,headless_gazebo=True);w.rviz.setChecked(False);w.show()
 reports=[];events=[];pulses=[];original=c.consume
 def consume(kind,data,stamp):
  if kind in ['motion','ack','event','plan']:events.append({'kind':kind,'data':data,'time':stamp})
  original(kind,data,stamp)
 c.consume=consume;timer=QTimer();timer.setInterval(20);timer.timeout.connect(lambda:pulses.append(time.monotonic()));timer.start()
 def pump():app.processEvents();time.sleep(.02)
 def until(pred,timeout=55):
  end=time.monotonic()+timeout
  while not pred() and time.monotonic()<end:pump()
  assert pred(),w.status.text()+' | '+w.execution_result.text()
 def mode(key,scenario='Standard game',human='white'):
  if c.lifecycle not in ['stopped','failed']:
   QTest.mouseClick(w.stop,Qt.LeftButton);until(lambda:c.lifecycle=='stopped',25)
  w.mode.setCurrentIndex(next(i for i,m in enumerate(MODES) if m.key==key));w.scenario.setCurrentText(scenario);w.human.setCurrentIndex(0 if human=='white' else 1)
  QTest.mouseClick(w.start,Qt.LeftButton)
  until(lambda:c.initialized and (c.ready or bool(c.policy.paused)))
  print('READY',key,scenario,human,flush=True)
 def rejection(uci,scenario):
  before=c.data['state']['fen'];start=len(events);QTest.mouseClick(w.board,Qt.LeftButton,pos=QPoint(*map(round,w.board.square_centre(uci[:2]))));QTest.mouseClick(w.board,Qt.LeftButton,pos=QPoint(*map(round,w.board.square_centre(uci[2:4]))))
  # Promotion choice comes from the existing combo's queen default.
  until(lambda:bool(c.data['state'].get('last_rejection')) and c.ready)
  s=c.data['state'];assert s['fen']==before and s['committed_plies']==0 and not s['pending_plan_id'] and not c.policy.history
  assert w.board.fen==before and w.submit.isEnabled() and 'unreachable_' in w.execution_result.text()
  ack=[e['data'] for e in events[start:] if e['kind']=='ack'];assert len(ack)==1 and ack[0]['safe_rejection'] and not ack[0]['arm_activity'] and not ack[0]['gripper_activity']
  assert not c.policy.requested
  report={'scenario':scenario,'uci':uci,'result':'rejected_before_activity','reason':s['last_rejection']['reason'],'confirmed_fen_unchanged':True,'ready_for_another_human_move':True,'no_automatic_reply':True}
  if scenario=='After c4 e5: g1f3':
   assert chess.Board(before).piece_at(chess.H2).symbol()=='P'
   report['h2_pawn_present']=True
  reports.append(report);w.grab().save(str(out/('qt_'+uci+'_rejected.png')))
 try:
  mode('manual');assert c.graph['joint_publishers']==['dual_scara_desktop_bridge']
  for arm,value in [('robot1',.061),('robot2',.063)]:
   w.arm.setCurrentText(arm);pump();w.joint_controls[arm+'_joint1'][0].setValue(value)
   until(lambda:abs(c.data['joints'][arm+'_joint1']-value)<1e-5)
  reports.append({'scenario':'manual','result':'passed','both_arm_slider_feedback':True,'single_publisher':True})
  mode('moveit');assert c.ready
  for arm in ['robot1','robot2']:
   w.arm.setCurrentText(arm);w.square.setCurrentText('e4')
   for kind in ['plan','execute']:
    c.last_error='';QTest.mouseClick(w.square_buttons[kind],Qt.LeftButton);until(lambda:c.square_id is None and c.ready)
    assert 'unreachable_' in c.last_error;reports.append({'scenario':'moveit_'+kind,'arm':arm,'result':'unreachable','reason':c.last_error})
   c.last_error='';QTest.mouseClick(w.square_buttons['home'],Qt.LeftButton);until(lambda:c.square_id is None and c.ready)
   assert not c.last_error;reports.append({'scenario':'moveit_home','arm':arm,'result':'mock_controller_execution_passed'})
  for scenario,uci in [('After c4 e5: g1f3','g1f3'),('Capture e4d5','e4d5'),('Castling e1g1','e1g1'),('En passant e5d6','e5d6'),('Promotion a7a8q','a7a8q')]:
   mode('mock',scenario);rejection(uci,scenario)
  mode('stockfish',human='white');rejection('e2e4','stockfish_human_white')
  mode('stockfish',human='black')
  until(lambda:bool(c.data['state'].get('last_rejection')) and bool(c.policy.paused))
  assert c.data['state']['committed_plies']==0 and c.policy.requested=={0};assert not w.submit.isEnabled()
  end=time.monotonic()+3
  while time.monotonic()<end:pump()
  assert c.policy.requested=={0} and c.data['state']['committed_plies']==0
  reports.append({'scenario':'stockfish_human_black','result':'opening_robot1_rejected','one_opening_request':True,'no_retry_or_commit':True,'reason':c.data['state']['last_rejection']['reason'],'status':w.status.text()})
  mode('gazebo');assert c.ready and c.data.get('clock') and c.data.get('joints')
  reports.append({'scenario':'gazebo','result':'clock_joint_feedback_ready','limits':'Direct physics mode, not MoveIt or verified chess execution','joints':c.data['joints']})
 except Exception as exc:
  reports.append({'scenario':'probe_error','error':str(exc)});raise
 finally:
  c.close();end=time.monotonic()+25
  while (c.supervisor.alive() or c.bridge.alive()) and time.monotonic()<end:pump()
  assert not c.supervisor.alive() and not c.bridge.alive()
  (out/'qt_workflow.json').write_text(json.dumps({'cases':reports,'events':events,'clean_shutdown':True,'max_qt_timer_gap':max((b-a for a,b in zip(pulses,pulses[1:])),default=0)},indent=2)+'\n')
  print(json.dumps(reports,indent=2))


if __name__=='__main__':main()
