"""Opening-move trace from fresh Qt/ROS simulation sessions (opt-in)."""
import argparse,json,time,uuid,subprocess,os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt,QPoint,QTimer
from PySide6.QtTest import QTest
from ament_index_python.packages import get_package_share_directory
from chess_brain_handover.core import chess
from dual_scara_desktop.window import MainWindow
from dual_scara_desktop.controller import DesktopController


def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',required=True);p.add_argument('--domain',type=int,default=221)
 p.add_argument('--moves',nargs='+',default=['g1f3','g1h3','b1c3','e2e4']);p.add_argument('--transport',choices=['qt','ros'],default='qt')
 p.add_argument('--expect-rejection',action='store_true');p.add_argument('--inspect',action='store_true');p.add_argument('--geometry');p.add_argument('--recover-move');p.add_argument('--hold-vision',action='store_true');p.add_argument('--fail-gripper',action='store_true');p.add_argument('--drop-command',action='store_true')
 p.add_argument('--tag',default='before');args=p.parse_args();app=QApplication([]);app.setQuitOnLastWindowClosed(False)
 out=Path(args.workspace)/'docs/pickup_validation'/args.tag;out.mkdir(parents=True,exist_ok=True)
 for i,uci in enumerate(args.moves):
  case_move=uci
  domain=args.domain+i;profile=Path(get_package_share_directory('dual_chess_description'))/'config/chess_geometry.yaml'
  if args.geometry:profile=Path(args.geometry).resolve()
  w=MainWindow(profile,domain);c=DesktopController(w,args.workspace,domain);w.rviz.setChecked(False);w.show()
  events=[];pulses=[];start=time.monotonic();report={'move':uci,'transport':args.transport,'domain':domain,'complete':False}
  timer=QTimer();timer.setInterval(20);timer.timeout.connect(lambda:pulses.append(time.monotonic()));timer.start()
  initial_joints={};travel={}
  original_publish=c.publish
  def publish(topic,payload):
   if topic=='/chess/command':events.append({'t':time.monotonic()-start,'kind':'published_command','data':payload})
   if args.drop_command and topic=='/chess/command' and payload.get('action')=='uci':return
   original_publish(topic,payload)
  c.publish=publish
  original=c.consume
  def consume(kind,data,stamp):
   if kind not in ['joints','clock','limits','graph']:events.append({'t':round(stamp-start,4),'kind':kind,'data':data})
   if kind=='joints' and initial_joints:
    for name,value in data.items():travel[name]=max(travel.get(name,0),abs(value-initial_joints.get(name,value)))
   original(kind,data,stamp)
  c.consume=consume
  sent_reply=False
  def pump():app.processEvents();time.sleep(.015)
  def until(pred,timeout=160,check_fail=True):
   deadline=time.monotonic()+timeout;last=0
   while not pred():
    pump()
    if check_fail and c.policy.paused:raise AssertionError(c.policy.paused)
    if c.lifecycle in ['failed','stopped']:raise AssertionError(c.last_error or 'Backend stopped')
    if time.monotonic()>deadline:raise AssertionError('Timeout '+w.status.text())
    if time.monotonic()-last>15:print(uci,args.transport,w.status.text(),flush=True);last=time.monotonic()
  try:
   QTest.mouseClick(w.start,Qt.LeftButton);until(lambda:c.policy.can_human(c.ready))
   assert not c.competitors()
   c.policy.enabled=False
   for setup_move in ['c2c4','e7e5']:
    before=c.data['state']['committed_plies']
    c.publish('/chess/command',{'action':'uci','uci':setup_move,'request_id':'setup-'+uuid.uuid4().hex})
    until(lambda:(c.data['state']['committed_plies']==before+1 and c.ready) or bool(c.data['state'].get('last_rejection')))
    assert c.data['state']['committed_plies']==before+1, c.data['state'].get('last_rejection')
   assert c.data['state']['fen'].split()[0]=='rnbqkbnr/pppp1ppp/8/4p3/2P5/8/PP1PPPPP/RNBQKBNR'
   c.policy.enabled=True;c.update_controls();pump()
   report['setup_fen']=c.data['state']['fen']
   if args.inspect:
    env=dict(os.environ);env['ROS_DOMAIN_ID']=str(domain)
    pool=ThreadPoolExecutor(1)
    future=pool.submit(subprocess.run,['/usr/bin/python3',str(Path(args.workspace)/'tools/inspect_pickup_scene.py'),
       '--geometry',str(profile),'--output',str(out/(args.transport+'_'+uci+'_inspection.json'))],env=env,capture_output=True,text=True)
    until(future.done);result=future.result();pool.shutdown()
    assert result.returncode==0,result.stdout+result.stderr
   initial_joints.update(c.data.get('joints',{}))
   if args.hold_vision:
    w.hold_vision.setChecked(True);until(lambda:not c.parameter_pending and c.ready)
   if args.fail_gripper:
    QTest.mouseClick(w.fail_gripper,Qt.LeftButton);until(lambda:not c.parameter_pending and c.ready)
   if args.transport=='qt':
    for sq in [uci[:2],uci[2:4]]:
     x,y=w.board.square_centre(sq);events.append({'t':time.monotonic()-start,'kind':'click','data':sq})
     QTest.mouseClick(w.board,Qt.LeftButton,pos=QPoint(round(x),round(y)));pump()
   else:
    # One turn owner: this driver, not the desktop coordinator, owns both dispatches.
    c.policy.enabled=False
    payload={'action':'uci','uci':uci,'request_id':'opening-'+uuid.uuid4().hex}
    events.append({'t':time.monotonic()-start,'kind':'direct_ros_command','data':payload});c.publish('/chess/command',payload)
   if args.expect_rejection:
    until(lambda:bool(c.data['state'].get('last_rejection')) and c.ready and (c.policy.can_human(c.ready) or args.transport=='ros'),40,check_fail=False)
    assert c.data['state']['committed_plies']==2 and c.data['state']['fen']==report['setup_fen']
    assert not c.data['state']['pending_plan_id'] and not c.policy.inflight and not c.policy.paused
    assert not c.policy.requested and max(travel.values(),default=0)==0
    report['safe_rejection_recovered']=True;report['rejection_notice']=w.execution_result.text()
    if args.transport=='qt':assert w.submit.isEnabled() and w.board.interactive
    w.grab().save(str(out/(args.transport+'_'+uci+'_recovered.png')))
    if not args.recover_move:raise RuntimeError('EXPECTED_SAFE_REJECTION')
    uci=args.recover_move;report['recovery_move']=uci;c.policy.enabled=True
    for sq in [uci[:2],uci[2:4]]:
     x,y=w.board.square_centre(sq);QTest.mouseClick(w.board,Qt.LeftButton,pos=QPoint(round(x),round(y)));pump()
   if args.hold_vision:
    until(lambda:c.data['state'].get('robot_ack_received'))
    before=c.data['state']['fen'];until(lambda:bool(c.policy.wait_notice),45)
    assert c.data['state']['committed_plies']==2 and len(c.policy.history)==2 and w.board.fen==before
    assert not w.submit.isEnabled() and not c.policy.requested
    report['held_vision_notice']=w.status.text();report['held_vision_board_unchanged']=True
    w.grab().save(str(out/(args.transport+'_'+uci+'_held.png')))
    w.hold_vision.setChecked(False)
   deadline=time.monotonic()+160;last=0
   while c.data['state']['committed_plies']<4 or not c.ready:
    pump();s=c.data['state']
    if c.policy.paused:raise AssertionError(c.policy.paused)
    if args.transport=='ros' and s['committed_plies']==3 and c.ready and not sent_reply:
     sent_reply=True;payload={'action':'next','request_id':'reply-'+uuid.uuid4().hex}
     events.append({'t':time.monotonic()-start,'kind':'direct_ros_command','data':payload});c.publish('/chess/command',payload)
    if time.monotonic()>deadline:raise AssertionError('Timeout '+w.status.text())
    if time.monotonic()-last>15:print(uci,args.transport,w.status.text(),flush=True);last=time.monotonic()
   assert len(c.policy.history)==4 and c.policy.history[2]['uci']==uci
   assert [h['arm'] for h in c.policy.history]==['robot1','robot2','robot1','robot2']
   assert w.board.fen==c.data['state']['fen'] and not c.data['state']['pending_plan_id']
   assert (c.policy.requested=={3} if args.transport=='qt' else sent_reply)
   assert chess.Board(c.data['state']['fen']).piece_at(chess.H2).symbol()=='P'
   report['h2_pawn_present']=True
   report['complete']=True
  except Exception as exc:
   report['error']=str(exc)
   if args.drop_command:
    assert 'timeout' in str(exc) and c.data['state']['committed_plies']==0 and not w.submit.isEnabled()
    assert c.policy.timeout_queried and c.policy.inflight
    for _ in range(3):QTest.mouseClick(w.submit,Qt.LeftButton);pump()
    submitted=[e for e in events if e['kind']=='published_command' and e['data'].get('action')=='uci']
    assert len(submitted)==1
    report['expected_timeout_no_retry']=True
   if args.fail_gripper:
    assert 'gripper' in str(exc) and c.data['state']['committed_plies']==2 and len(c.policy.history)==2
    assert not w.submit.isEnabled() and not c.policy.requested
    report['expected_failure_visible_without_commit']=True
  finally:
   end=time.monotonic()+1
   while time.monotonic()<end:pump()
   report.update(history=c.policy.history,state=c.data.get('state'),motion=c.data.get('motion'),status=w.status.text(),
    pending=w.pending.text(),execution=w.execution_result.text(),selected=c.selected,inflight=c.policy.inflight,
    initial_joints=initial_joints,max_joint_travel=travel,
    automatic_reply_keys=sorted(c.policy.requested),launch_log=c.log_path,
    qt_ticks=len(pulses),max_qt_gap=max((b-a for a,b in zip(pulses,pulses[1:])),default=0),elapsed=time.monotonic()-start)
   name=args.transport+'_'+case_move
   w.grab().save(str(out/(name+'.png')));(out/(name+'_events.json')).write_text(json.dumps(events,indent=2))
   c.close();end=time.monotonic()+25
   while (c.supervisor.alive() or c.bridge.alive()) and time.monotonic()<end:pump()
   report['clean_shutdown']=not c.supervisor.alive() and not c.bridge.alive();timer.stop()
   (out/(name+'.json')).write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
 return 0

if __name__=='__main__':raise SystemExit(main())
