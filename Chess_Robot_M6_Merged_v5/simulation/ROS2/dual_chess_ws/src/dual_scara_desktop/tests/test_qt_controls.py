"""Fast offscreen Qt tests; no ROS graph or processes are started."""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import time
from types import SimpleNamespace
import pytest
pytest.importorskip('PySide6')
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt,QPoint
from PySide6.QtTest import QTest
from dual_scara_desktop.window import ChessBoard
from dual_scara_desktop.controller import DesktopController

@pytest.fixture(scope='module')
def app():return QApplication.instance() or QApplication([])


def test_board_resize_and_black_orientation_are_screen_only(app):
    board=ChessBoard();board.resize(500,500);board.set_position('8/8/8/8/8/8/4P3/8 w - - 0 1');board.interactive=True;board.show()
    clicks=[];board.square_clicked.connect(clicks.append)
    x,y=board.square_centre('e2');QTest.mouseClick(board,Qt.LeftButton,pos=QPoint(round(x),round(y)));assert clicks==['e2']
    before=board.board_rect().width();board.resize(800,700);assert board.board_rect().width()>before
    board.set_position(board.fen,flipped=True);x,y=board.square_centre('h8');QTest.mouseClick(board,Qt.LeftButton,pos=QPoint(round(x),round(y)));assert clicks[-1]=='h8'
    assert board.fen=='8/8/8/8/8/8/4P3/8 w - - 0 1'
    board.interactive=False;QTest.mouseClick(board,Qt.LeftButton,pos=QPoint(round(x),round(y)));assert len(clicks)==2
    board.close()


def test_readiness_requires_idle_stability_and_ignores_unknown_discovery_without_advancing(app):
    now=time.monotonic()
    fake=SimpleNamespace(data={'motion':{'status':'ready'},'safety':True},arrival={'motion':now,'safety':now},mode=SimpleNamespace(key='moveit'),
                         graph={'nodes':[],'joint_publishers':['joint_state_broadcaster'],'command_publishers':[]},square_id=None,
                         idle_since=None,parameter_pending=set(),software_stopped=False,component_failed=False,stage_info=None)
    assert not DesktopController.compute_readiness(fake)[0]
    fake.data['motion']['status']='idle';fake.idle_since=now
    assert not DesktopController.compute_readiness(fake)[0]
    fake.idle_since=now-1
    assert DesktopController.compute_readiness(fake)[0]
    fake.graph['command_publishers']=['_NODE_NAME_UNKNOWN_']
    assert not DesktopController.compute_readiness(fake)[0]
    fake.graph['command_publishers']=[];fake.software_stopped=True
    assert not DesktopController.compute_readiness(fake)[0]
    fake.software_stopped=False;fake.parameter_pending={'parameter-request'}
    assert not DesktopController.compute_readiness(fake)[0]


def test_competing_coordinator_or_command_publisher_is_not_ignored(app):
    fake=SimpleNamespace(graph={'nodes':['chess_brain','chess_auto_coordinator'],
                               'command_publishers':['dual_scara_desktop_bridge','mock_chess_vision','other_auto_owner','_NODE_NAME_UNKNOWN_']})
    competitors=DesktopController.competitors(fake)
    assert set(competitors)=={'chess_auto_coordinator','other_auto_owner'}


def test_motion_readiness_requires_fresh_unlocked_safety(app):
    now=time.monotonic()
    fake=SimpleNamespace(data={'motion':{'status':'idle'},'safety':False},arrival={'motion':now,'safety':now},
                         mode=SimpleNamespace(key='moveit'),graph={'nodes':[]},square_id=None,idle_since=now-1,
                         parameter_pending=set(),software_stopped=False,component_failed=False,stage_info=None)
    assert not DesktopController.compute_readiness(fake)[0]
    fake.data['safety']=True;fake.arrival['safety']=now-3
    assert not DesktopController.compute_readiness(fake)[0]
    fake.arrival['safety']=now
    assert DesktopController.compute_readiness(fake)[0]
    fake.data['motion']={'status':'initialization_failed','reason':'scene unavailable'}
    assert DesktopController.compute_readiness(fake)==(False,'scene unavailable')


def test_operator_confirmation_is_token_scoped_and_not_duplicated(app,monkeypatch):
    from PySide6.QtWidgets import QMessageBox
    sent=[]
    fake=SimpleNamespace(operator={'plan_id':'plan','token':'one','piece':'Q','square':'a8'},last_operator_token=None,
                         lifecycle='running',w=None,publish=lambda topic,payload:sent.append(payload),update_controls=lambda:None)
    monkeypatch.setattr(QMessageBox,'question',lambda *args:QMessageBox.Yes)
    DesktopController.confirm_promotion(fake);DesktopController.confirm_promotion(fake)
    assert sent==[{'plan_id':'plan','token':'one','confirmed':True}]
    fake.operator={'plan_id':'other','token':'two'};errors=[];fake.error=errors.append
    def expired(*args):fake.operator=None;return QMessageBox.Yes
    monkeypatch.setattr(QMessageBox,'question',expired)
    DesktopController.confirm_promotion(fake)
    assert len(sent)==1 and errors


def test_safe_rejection_requires_new_motion_and_safety_readiness(app):
    now=time.monotonic()
    fake=SimpleNamespace(data={'motion':{'status':'idle'},'safety':True},arrival={'motion':now,'safety':now-1},
                         mode=SimpleNamespace(key='moveit'),graph={'nodes':[]},square_id=None,idle_since=now-1,
                         parameter_pending=set(),software_stopped=False,component_failed=False,stage_info=None,
                         policy=SimpleNamespace(rejection_cleared_at=now-.5))
    assert not DesktopController.compute_readiness(fake)[0]
    fake.arrival={'motion':now-1,'safety':now}
    assert not DesktopController.compute_readiness(fake)[0]
    fake.arrival={'motion':now,'safety':now}
    assert DesktopController.compute_readiness(fake)[0]


def test_manual_controls_start_from_profile_home_until_feedback_arrives(app):
    calls=[]
    fake=SimpleNamespace(configuration={'initial_joints':{'robot1_joint1':.09,'robot1_joint2':-1.57}},
        data={'limits':{}},ready=False,w=SimpleNamespace(arm=SimpleNamespace(currentText=lambda:'robot1'),
        show_joints=lambda *args:calls.append(args)))
    DesktopController.refresh_joints(fake)
    assert calls[-1][2]=={'robot1_joint1':.09,'robot1_joint2':-1.57}
    fake.data['joints']={'robot1_joint1':.08}
    DesktopController.refresh_joints(fake)
    assert calls[-1][2]=={'robot1_joint1':.08,'robot1_joint2':-1.57}
