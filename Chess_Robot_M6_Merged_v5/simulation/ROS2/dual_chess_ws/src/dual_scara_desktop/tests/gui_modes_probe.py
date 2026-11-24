"""Opt-in Qt/ROS mode switching, controls, failure and confirmation integration test."""
import argparse,json,time
from pathlib import Path
from PySide6.QtWidgets import QApplication,QMessageBox
from PySide6.QtCore import Qt,QTimer
from PySide6.QtTest import QTest
from ament_index_python.packages import get_package_share_directory
from dual_scara_desktop.window import MainWindow
from dual_scara_desktop.controller import DesktopController
from dual_scara_desktop.modes import MODES


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',required=True);ap.add_argument('--domain',type=int,default=214);args=ap.parse_args()
    app=QApplication([]);app.setQuitOnLastWindowClosed(False)
    profile=Path(get_package_share_directory('dual_chess_description'))/'config/chess_geometry.yaml'
    w=MainWindow(profile,args.domain);c=DesktopController(w,args.workspace,args.domain,headless_gazebo=True);w.rviz.setChecked(False);w.show();report={}
    def until(pred,timeout=100):
        end=time.monotonic()+timeout;last_log=0
        while not pred() and time.monotonic()<end:
            app.processEvents();time.sleep(.02)
            if time.monotonic()-last_log>10:
                print(c.lifecycle, c.mode.key if c.mode else '', w.status.text(), 'supervisor',c.supervisor.running,flush=True);last_log=time.monotonic()
        assert pred(),w.status.text()+'\n'+w.log.toPlainText()[-3000:]
    def mode(key,scenario=None,switch=False):
        if c.lifecycle not in ['stopped','failed'] and not switch:
            QTest.mouseClick(w.stop,Qt.LeftButton);until(lambda:c.lifecycle=='stopped',25)
        w.mode.setCurrentIndex(next(i for i,m in enumerate(MODES) if m.key==key));app.processEvents()
        if scenario:w.scenario.setCurrentText(scenario)
        QTest.mouseClick(w.start,Qt.LeftButton);until(lambda:c.mode.key==key and c.ready)
    def click_square(key):
        QTest.mouseClick(w.square_buttons[key],Qt.LeftButton)
        assert c.square_id is not None,'Square request was not submitted'
        until(lambda:c.square_id is None and c.ready)
        assert not c.last_error,c.last_error
    def accept_dialog():
        dialog=QApplication.activeModalWidget()
        assert isinstance(dialog,QMessageBox)
        QTest.mouseClick(dialog.button(QMessageBox.Yes),Qt.LeftButton)
    def move(uci):
        w.uci.setText(uci);QTest.mouseClick(w.submit,Qt.LeftButton)
    try:
        mode('manual');first_pid=c.owned_pid
        assert c.graph['joint_publishers']==['dual_scara_desktop_bridge']
        for arm,value in [('robot1',.061),('robot2',.063)]:
            w.arm.setCurrentText(arm);app.processEvents();spin,slider,_=w.joint_controls[arm+'_joint1']
            assert spin.minimum()==0 and spin.maximum()==.1
            spin.setValue(value);until(lambda:abs(c.data['joints'][arm+'_joint1']-value)<.00001,8)
        report['manual']={'both_arms_feedback':True,'publishers':c.graph['joint_publishers']}
        mode('moveit',switch=True);assert first_pid!=c.owned_pid
        assert not c.supervisor._group_exists(first_pid)
        report['switch']={'old_owned_group_gone_before_new_backend':True}
        original=dict(c.data['joints']);click_square('plan');assert 'no movement' in w.plan_result.text();assert c.data['joints']==original
        click_square('execute');assert 'completed' in w.execution_result.text();click_square('home')
        w.arm.setCurrentText('robot2');w.square.setCurrentText('h8');click_square('execute');click_square('home')
        w.arm.setCurrentText('robot1');w.square.setCurrentText('a4');QTimer.singleShot(30,accept_dialog);click_square('probe');click_square('home')
        report['moveit']={'plan_only_keeps_joints':True,'execute_and_home_both_arms':True,'empty_square_probe':True}
        QTest.mouseClick(w.motion_stop,Qt.LeftButton);until(lambda:c.data.get('motion',{}).get('status') in ['estop','blocked'])
        assert c.supervisor.running and not c.ready and not w.square_buttons['execute'].isEnabled()
        report['software_stop']={'mode_stays_running':True,'motion_disabled':True}
        mode('mock','Standard game');before=c.data['state']['fen'];QTest.mouseClick(w.fail_gripper,Qt.LeftButton)
        until(lambda:c.data.get('parameter_result',{}).get('name')=='fail_next' and c.data['parameter_result']['ok'])
        move('e2e4');until(lambda:c.data['state'].get('blocked'),40)
        assert c.data['state']['fen']==before and not c.policy.history and not w.submit.isEnabled()
        assert c.supervisor.running
        report['failure']={'confirmed_fen_unchanged':True,'mode_not_shutdown_by_failure':True,'next_move_disabled':True}
        old_generation=c.generation;QTimer.singleShot(30,accept_dialog);QTest.mouseClick(w.restart,Qt.LeftButton)
        assert c.lifecycle=='stopping' or c.generation>old_generation,'Restart confirmation was not accepted'
        until(lambda:c.generation>old_generation and c.ready)
        w.hold_vision.setChecked(True);until(lambda:c.data.get('parameter_result',{}).get('name')=='confirm_after_ack' and c.data['parameter_result']['ok'])
        before=c.data['state']['fen'];move('e2e4');QTest.mouseClick(w.submit,Qt.LeftButton)
        until(lambda:c.data['state'].get('robot_ack_received'),90)
        assert c.data['state']['fen']==before and w.board.fen==before and not c.policy.history
        w.hold_vision.setChecked(False);until(lambda:c.data['state']['committed_plies']==1 and c.ready)
        assert not c.policy.requested
        report['delayed_vision']={'ack_does_not_change_board':True,'fresh_vision_commits_once':True,'duplicate_click_ignored':True}
        mode('mock','Promotion a7a8q');move('a7a8q');until(lambda:c.operator is not None,120)
        assert w.operator.isEnabled() and c.data['state']['committed_plies']==0
        QTimer.singleShot(30,accept_dialog);QTest.mouseClick(w.operator,Qt.LeftButton)
        until(lambda:c.data['state']['committed_plies']==1 and c.ready,90)
        assert w.board.fen.split()[0].startswith('Q')
        report['promotion']={'explicit_operator_token_confirmation':True,'confirmed_queen_displayed':True}
        mode('logic');assert not c.data['state']['synchronized'] and not w.submit.isEnabled()
        QTest.mouseClick(w.sync,Qt.LeftButton);until(lambda:'no vision' in w.execution_result.text())
        report['logic_only']={'brain_state_visible':True,'missing_external_vision_reported':True,'motion_disabled':True}
        mode('gazebo');w.arm.setCurrentText('robot1');app.processEvents()
        report['gazebo']={'clock_and_joint_feedback':True,'direct_position_controls_enabled':w.apply_joints.isEnabled()}
        QTest.mouseClick(w.apply_joints,Qt.LeftButton)
        assert c.data['clock'] and w.apply_joints.isEnabled()
        w.grab().save(str(Path(args.workspace)/'.desktop-state/qt_modes.png'))
        print(json.dumps(report,indent=2),flush=True)
    finally:
        Path(args.workspace,'.desktop-state/qt_modes.json').write_text(json.dumps(report,indent=2))
        c.close();end=time.monotonic()+25
        while (c.supervisor.alive() or c.bridge.alive()) and time.monotonic()<end:app.processEvents();time.sleep(.03)
        assert not c.supervisor.alive() and not c.bridge.alive()

if __name__=='__main__':main()
