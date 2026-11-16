from pathlib import Path
import copy
import json
import threading
from types import SimpleNamespace
import xml.etree.ElementTree as E
import numpy as np
import pytest
import xacro
from chess_motion_interface.geometry import Geometry
from chess_motion_interface.model import Model
from chess_motion_interface.node import MotionNode
from chess_motion_interface.mocks import observation
from chess_brain_handover.core import chess, describe_move, GameController

DESC = Path(__file__).resolve().parents[2]/'dual_chess_description'


def geometry():
    return Geometry.load(DESC/'config/chess_geometry.yaml')


def test_rectangular_pitch_orientation_height_and_base_transform():
    raw = geometry().raw
    raw['board'].update(length=.28,width=.20,position=[.02,-.01,.006],rpy=[.03,0.,.4])
    g = Geometry(raw)
    assert g.raw['board']['square_size_x'] == pytest.approx(.03125)
    assert g.raw['board']['square_size_y'] == pytest.approx(.02125)
    delta = g.square('b1')-g.square('a1')
    assert np.linalg.norm(delta) == pytest.approx(.03125)
    p = np.array([.1,.2,.3])
    transform = np.eye(4);transform[:3,3]=p
    assert g.target_in_frame('h8',transform) == pytest.approx(g.square('h8')-p)


@pytest.mark.parametrize('change',[{'length':.01},{'position':[0.,float('nan'),0.]},{'grid_mode':'unknown'},{'surface_height':.1}])
def test_invalid_geometry_rejected(change):
    raw = geometry().raw;raw['board'].update(change)
    with pytest.raises(ValueError):Geometry(raw)


def test_low_board_reach_depends_on_real_mount_not_assumed_base_height():
    raw = geometry().raw
    model = Model(E.fromstring(xacro.process_file(str(DESC/'urdf/dual_chess.urdf.xacro')).toxml()))
    g = Geometry(raw)
    for square in ['a1','e2','e4','h8']:
        assert model.solutions('robot1',g.square(square,g.grasp('p')))
    # Measured CAD base origin at 0.44m would put this same low board out of Z reach.
    joint = model.by_child['robot1_mount']
    joint.find('origin').set('xyz','0 -0.19 0.44')
    assert not model.solutions('robot1',g.square('e2',g.grasp('p')))


@pytest.mark.parametrize('fen,uci,expected',[
    (chess.STARTING_FEN,'e2e4',['move']),
    ('rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2','e4d5',['remove','move']),
    ('r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1','e1g1',['move','move']),
    ('4k3/8/8/3pP3/8/8/8/4K3 w - d6 0 1','e5d6',['remove','move']),
    ('4k3/P7/8/8/8/8/8/4K3 w - - 0 1','a7a8q',['move','promote'])])
def test_existing_chess_actions_drive_adapter_without_rewriting_rules(fen,uci,expected):
    plan = describe_move(chess.Board(fen),chess.Move.from_uci(uci)).to_dict()
    assert [a['kind'] for a in plan['actions']] == expected
    calls = []
    class Backend:
        def valid(self):return True,[]
        def joints(self):return {f'{a}_joint{i}':v for a in ['robot1','robot2'] for i,v in enumerate([.09,0.,0.],1)}
        def park(self,arm):calls.append(('park',arm))
    class Confirm:
        def clear(self):pass
        def wait(self,timeout):return True
    node = SimpleNamespace(backend=Backend(),geometry=geometry(),bin_counts={'robot1':0,'robot2':0},event=lambda *a,**k:None,preflight_plan=lambda *a:None,grip=lambda *a:None,operator_event=Confirm(),operator_token=None,
                           transfer=lambda arm,source,target=None,bin_slot=None:calls.append(('transfer',source,target,bin_slot)))
    if 'promote' in expected:
        # Cover physical expansion separately below; this case confirms operator requirement in schema.
        assert plan['actions'][-1]['piece']=='q'
        return
    MotionNode.execute_plan(node,plan)
    transfers = [c for c in calls if c[0]=='transfer']
    assert len(transfers) == len(expected)
    if uci=='e5d6':assert transfers[0][1]=='d5'
    if uci=='e1g1':assert transfers[1][1:3]==('h1','f1')


def test_robot_failure_cannot_commit_confirmed_board_even_with_matching_mock_vision():
    class Player:
        def close(self):pass
    game = GameController(players={chess.WHITE:Player(),chess.BLACK:Player()})
    before = game.board.fen()
    assert game.sync_initial_board(observation(before))['ok']
    plan = game.manual_move('e2','e4')
    game.robot_ack(plan.plan_id,success=False,arm_parked=False,gripper_released=False)
    assert not game.confirm_vision(observation(plan.expected_fen))['ok']
    assert game.board.fen()==before and game.state()['committed_plies']==0


def test_ros_double_arrays_are_canonical_json_and_geometry_is_not_silently_embedded_in_support():
    import array
    raw = geometry().raw
    raw['board']['position'] = array.array('d',[0.,0.,.001])
    g = Geometry(raw)
    assert isinstance(g.raw['board']['position'],list)
    json.dumps(g.raw,allow_nan=False)
    raw['board']['position'] = [0.,0.,-.005]
    with pytest.raises(ValueError,match='board_intersects_support'):Geometry(raw)


def test_readiness_cannot_be_true_before_bootstrap_releases_scene_lock():
    import time
    published = []
    lock = threading.Lock()
    node = SimpleNamespace(get_parameter=lambda name:SimpleNamespace(value=True),ready=True,busy=False,blocked=False,lock=lock,
                           backend=SimpleNamespace(stop=threading.Event(),received_at=time.monotonic()),safety_pub=SimpleNamespace(publish=lambda msg:published.append(msg.data)),event=lambda *a:None)
    lock.acquire()
    MotionNode.readiness(node)
    assert published[-1] is False
    lock.release()
    MotionNode.readiness(node)
    assert published[-1] is True


def test_geometry_collision_rejection_rolls_back_scene_and_configuration(monkeypatch):
    import chess_motion_interface.node as module
    from rclpy.parameter import Parameter
    applied = []
    class FakeScene:
        def __init__(self,backend,g):self.geometry=g;self.attached=None;self.ids=set();self.pieces={}
        def initialize(self,fen):applied.append(self.geometry.raw['board']['length'])
    g = geometry()
    backend = SimpleNamespace(geometry=g,valid=lambda:(False,['robot/table']))
    previous = FakeScene(backend,g)
    node = SimpleNamespace(ready=True,blocked=False,awaiting_fen=None,lock=threading.Lock(),geometry=g,scene=previous,revision=0,
                           backend=backend,dynamic_names={'board.length':['board','length']},confirmed_fen='',
                           publish_geometry=lambda:None,event=lambda *a,**kw:None)
    monkeypatch.setattr(module,'Scene',FakeScene)
    result = MotionNode.on_parameters(node,[Parameter('board.length',value=.26)])
    assert not result.successful and 'collides' in result.reason
    assert applied==[.26,.24]
    assert node.geometry is g and node.scene is previous and node.revision==0 and not node.lock.locked()


def test_partial_cartesian_paths_never_reach_execution():
    from chess_motion_interface.backend import Backend,MotionError
    from moveit_msgs.msg import RobotState
    seen=[]
    fake=SimpleNamespace(geometry=geometry(),state=lambda:RobotState(),joints=lambda:{},model=SimpleNamespace(matrix=lambda *a:np.eye(4)),
                         cart_client=SimpleNamespace(call_async=lambda req:seen.append(req)),
                         wait=lambda *a,**kw:SimpleNamespace(fraction=.5,error_code=SimpleNamespace(val=1)),
                         execute=lambda trajectory:pytest.fail('partial trajectory must not execute'))
    with pytest.raises(MotionError,match='cartesian_path_incomplete'):Backend.vertical(fake,'robot1',[.01,.01,.03])
    assert seen[0].avoid_collisions


def test_promotion_expands_pawn_removal_and_waits_for_correlated_operator():
    from moveit_msgs.msg import CollisionObject
    calls=[]
    plan=describe_move(chess.Board('4k3/P7/8/8/8/8/8/4K3 w - - 0 1'),chess.Move.from_uci('a7a8n')).to_dict()
    class Confirm:
        def clear(self):calls.append('operator_clear')
        def wait(self,timeout):calls.append('operator_wait');return True
    backend=SimpleNamespace(valid=lambda:(True,[]),joints=lambda:{f'{a}_joint{i}':v for a in ['robot1','robot2'] for i,v in enumerate([.09,0.,0.],1)},
                            park=lambda arm:calls.append(('park',arm)),apply=lambda scene:calls.append('replacement_scene_applied'))
    node=SimpleNamespace(backend=backend,geometry=geometry(),bin_counts={'robot1':0,'robot2':0},event=lambda status,**data:calls.append(status),
                         preflight_plan=lambda *a:None,grip=lambda *a:None,operator_event=Confirm(),operator_token=None,
                         transfer=lambda arm,src,target=None,bin_slot=None:calls.append(('transfer',src,target,bin_slot)),
                         scene=SimpleNamespace(at={},pieces={},ids=set(),piece_object=lambda name,symbol,local:CollisionObject(id=name)))
    MotionNode.execute_plan(node,plan)
    assert [c for c in calls if isinstance(c,tuple) and c[0]=='transfer']==[('transfer','a7','a8',None),('transfer','a8',None,0)]
    assert calls.index('operator_required')<calls.index('operator_wait')<calls.index('replacement_scene_applied')
    assert list(node.scene.pieces.values())[0]['symbol']=='N'
    node.active_id='plan';node.operator_token='token';node.operator_event=threading.Event()
    MotionNode.on_operator(node,SimpleNamespace(data=json.dumps({'plan_id':'plan','token':'wrong','confirmed':True})))
    assert not node.operator_event.is_set()
    MotionNode.on_operator(node,SimpleNamespace(data=json.dumps({'plan_id':'plan','token':'token','confirmed':True})))
    assert node.operator_event.is_set()


@pytest.mark.parametrize('reason,started,attached,blocked,stopped,expected',[
    ('unreachable_target',False,False,False,False,False),
    ('planning_failed',True,False,False,False,True),
    ('gripper_operation_failed',False,False,False,False,True),
    ('scene_update_failed',False,True,False,False,True),
    ('cancelled',False,False,True,False,True),
    ('stopped',False,False,False,True,True)])
def test_worker_failure_cleanup_preserves_stop_and_gripper_uncertainty(reason,started,attached,blocked,stopped,expected):
    from chess_motion_interface.backend import requires_reconciliation
    assert requires_reconciliation(reason,started,attached,blocked,stopped) is expected


def test_configured_approach_clearance_also_changes_square_approach_and_retreat_height():
    g=geometry();raw=copy.deepcopy(g.raw)
    raw['motion']['approach_clearance']+=.01
    raised=Geometry(raw)
    assert raised.travel()==pytest.approx(g.travel()+.01)
