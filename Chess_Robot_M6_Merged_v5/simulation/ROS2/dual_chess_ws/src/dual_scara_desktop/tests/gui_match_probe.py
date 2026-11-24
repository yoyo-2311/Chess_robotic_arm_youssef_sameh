"""Explicit integration probe, not collected by default pytest. Runs REAL Qt controls + ROS mock execution."""
import argparse
import json
import os
from pathlib import Path
import time
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt,QPoint,QTimer
from PySide6.QtTest import QTest
from ament_index_python.packages import get_package_share_directory
from dual_scara_desktop.window import MainWindow
from dual_scara_desktop.controller import DesktopController


def main():
    p=argparse.ArgumentParser();p.add_argument('--workspace',required=True);p.add_argument('--domain',type=int,default=212);p.add_argument('--human',choices=['white','black'],default='white');args=p.parse_args()
    app=QApplication([]);app.setQuitOnLastWindowClosed(False)
    profile=Path(get_package_share_directory('dual_chess_description'))/'config/chess_geometry.yaml'
    w=MainWindow(profile,args.domain);c=DesktopController(w,args.workspace,args.domain);w.rviz.setChecked(False);w.human.setCurrentIndex(0 if args.human=='white' else 1);w.show()
    pulses=[];timer=QTimer();timer.setInterval(20);timer.timeout.connect(lambda:pulses.append(time.monotonic()));timer.start()
    output=Path(args.workspace)/'.desktop-state';output.mkdir(exist_ok=True)
    def until(predicate,timeout=180):
        deadline=time.monotonic()+timeout;last=''
        while not predicate() and time.monotonic()<deadline:
            app.processEvents();time.sleep(.015)
            phase=w.status.text()+' | '+w.pending.text()
            if phase!=last:print(phase,flush=True);last=phase
            if c.policy.paused:raise AssertionError(c.policy.paused+'\n'+w.log.toPlainText()[-5000:])
            if c.lifecycle=='failed':raise AssertionError(c.last_error)
        assert predicate(),w.status.text()+'\n'+w.log.toPlainText()[-5000:]
    try:
        QTest.mouseClick(w.start,Qt.LeftButton)
        until(lambda:c.policy.can_human(c.ready))
        baseline=c.data['state']['committed_plies']
        if args.human=='white':
            assert baseline==0
            for square in ['e2','e4']:
                x,y=w.board.square_centre(square);QTest.mouseClick(w.board,Qt.LeftButton,pos=QPoint(round(x),round(y)));app.processEvents()
        else:
            assert baseline==1 and c.policy.history[0]['arm']=='robot1'
            move='g8f6' if 'g8f6' in c.data['state']['legal_moves'] else c.data['state']['legal_moves'][0]
            w.uci.setText(move);QTest.mouseClick(w.submit,Qt.LeftButton)
        until(lambda:c.data.get('state',{}).get('committed_plies',0)>=baseline+2 and c.policy.can_human(c.ready))
        assert len(c.policy.history)==baseline+2
        assert c.policy.history[-2]['side']==args.human
        assert c.policy.history[-2]['arm']==('robot1' if args.human=='white' else 'robot2')
        assert c.policy.history[-1]['side']!=args.human
        assert w.board.fen==c.data['state']['fen'] and not c.data['state']['pending_plan_id']
        assert len(c.policy.requested)==(1 if args.human=='white' else 2)
        max_gap=max((b-a for a,b in zip(pulses,pulses[1:])),default=0)
        report={'human':args.human,'confirmed_plies':c.data['state']['committed_plies'],'history':c.policy.history,
                'automatic_reply_ply_keys':sorted(c.policy.requested),'board_matches_confirmed_fen':True,
                'qt_timer_ticks':len(pulses),'maximum_qt_timer_gap_seconds':max_gap,'launch_log':c.log_path}
        (output/f'qt_match_{args.human}.json').write_text(json.dumps(report,indent=2));w.grab().save(str(output/f'qt_match_{args.human}.png'))
        print(json.dumps(report,indent=2),flush=True)
    finally:
        c.close();deadline=time.monotonic()+25
        while (c.supervisor.alive() or c.bridge.alive()) and time.monotonic()<deadline:app.processEvents();time.sleep(.03)
        assert not c.supervisor.alive() and not c.bridge.alive(),'Owned processes/worker did not stop cleanly'
    return 0

if __name__=='__main__':raise SystemExit(main())
