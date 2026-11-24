"""High approach reachability must not be mistaken for low grasp clearance."""
from types import SimpleNamespace
import copy
from pathlib import Path
import numpy as np
import pytest
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject
from chess_motion_interface.backend import Backend, MotionError
from chess_motion_interface.geometry import Geometry
from chess_motion_interface.node import MotionNode


def geometry():
    return Geometry.load(Path(__file__).parents[2]/'dual_chess_description/config/chess_geometry.yaml')


def backend(fractions):
    b=Backend.__new__(Backend);b.geometry=geometry();calls=[];events=[]
    state=RobotState();state.is_diff=True
    state.joint_state.name=['robot1_joint1','robot1_joint2','robot1_joint3','robot2_joint1']
    state.joint_state.position=[.09,0.,0.,.09]
    state.attached_collision_objects=[AttachedCollisionObject(object=CollisionObject(id='carried-piece'))]
    b.state=lambda:state
    b.model=SimpleNamespace(solutions=lambda *args:[{'q':[.08,.1,2.8]},{'q':[.08,2.7,-2.4]}],matrix=lambda *args:np.eye(4))
    b.cart_client=SimpleNamespace(call_async=lambda req:calls.append(req))
    responses=iter(fractions)
    b.wait=lambda *args,**kw:SimpleNamespace(fraction=next(responses),error_code=SimpleNamespace(val=1))
    b.valid=lambda end:(False,['robot1_link4/chess_piece_h2'])
    b.event=lambda status,**data:events.append((status,data))
    b.joint_goal=lambda arm,solutions,execute:(arm,solutions,execute)
    return b,calls,events,state


def test_only_complete_collision_checked_branch_can_be_approach_goal():
    b,calls,events,state=backend([.89,1.])
    original=copy.deepcopy(state)
    result=b.approach('robot1',[.06,-.09,.06],[.06,-.09,.0243],execute=False)
    assert result==('robot1',[[.08,2.7,-2.4]],False)
    assert state==original  # Synthetic candidates must not mutate live feedback.
    assert all(r.avoid_collisions and r.start_state.is_diff for r in calls)
    assert all(r.start_state.attached_collision_objects[0].object.id=='carried-piece' for r in calls)
    assert calls[0].start_state.joint_state.position[-1]==.09  # Preserve other arm.
    assert events[-1][1]['feasible_branches']==1


def test_all_partial_branches_rejected_before_any_approach_is_planned_or_executed():
    b,calls,events,state=backend([.89,.31])
    b.joint_goal=lambda *args:pytest.fail('An infeasible descent must not dispatch an approach')
    with pytest.raises(MotionError,match='no_collision_free_descent.*chess_piece_h2'):
        b.approach('robot1',[.06,-.09,.06],[.06,-.09,.0243])
    assert len(calls)==2 and events[-1][1]['feasible_branches']==0


def test_pickup_failure_restores_touch_permission_without_moving_or_gripping():
    g=geometry();calls=[]
    def fail(*args,**kwargs):raise MotionError('no_collision_free_descent')
    scene=SimpleNamespace(at={'g1':'knight'},pieces={'knight':{'symbol':'N','local':g.local_square('g1',g.piece_height('N')/2)}},
                          touch=lambda *args:calls.append(args))
    node=SimpleNamespace(geometry=g,scene=scene,event=lambda *args,**kwargs:None,
                         backend=SimpleNamespace(approach=fail),
                         grip=lambda *args:pytest.fail('Gripper must not operate for rejected pickup'))
    node.prepare_transfer=lambda *args,**kwargs:MotionNode.prepare_transfer(node,*args,**kwargs)
    with pytest.raises(MotionError,match='no_collision_free_descent'):
        MotionNode.transfer(node,'robot1','g1','f3')
    assert calls==[('knight','robot1',True),('knight','robot1',False)]
    assert scene.at=={'g1':'knight'}


def test_named_operation_timeout_is_explicit_and_cancels_ambiguous_execution(monkeypatch):
    import chess_motion_interface.backend as module
    events=[];cancelled=[]
    b=Backend.__new__(Backend)
    b.active_goal=SimpleNamespace(cancel_goal_async=lambda:cancelled.append(True))
    b.stop=SimpleNamespace(is_set=lambda:False)
    b.event=lambda status,**data:events.append((status,data))
    values=iter([0.,40.]);monkeypatch.setattr(module.time,'monotonic',lambda:next(values))
    with pytest.raises(MotionError,match='controller result'):
        b.wait(SimpleNamespace(done=lambda:False),operation='controller result',motion=True)
    assert cancelled and [d['phase'] for _,d in events]==['waiting','failed']
    assert events[0][1]['operation_id']==events[1][1]['operation_id']


def test_new_stage_keeps_move_context_without_stale_coordinates_or_detail(monkeypatch):
    import chess_motion_interface.node as module
    sent=[];monkeypatch.setattr(module,'send',lambda publisher,payload:sent.append(payload))
    fake=SimpleNamespace(active_context={'arm':'robot1','side':'white','uci':'g1f3','source':'g1','target':'f3',
                                        'world':[0.,0.,.0243],'detail':'before movement'},
                         active_id='plan',revision=0,status_pub=None,
                         get_logger=lambda:SimpleNamespace(info=lambda text:None))
    MotionNode.event(fake,'stage',stage='transfer',piece='knight')
    assert sent[0]['uci']=='g1f3' and sent[0]['source']=='g1' and sent[0]['stage']=='transfer'
    assert 'world' not in sent[0] and 'detail' not in sent[0]


@pytest.mark.parametrize('execution,gripper,partial,expected',[(0,0,False,True),(1,0,False,False),(0,1,False,False),(0,0,True,False)])
def test_safe_rejection_proof_requires_no_arm_or_gripper_intent(execution,gripper,partial,expected):
    from chess_motion_interface.backend import PreflightRejected,proven_safe_rejection
    assert proven_safe_rejection(PreflightRejected('blocked'),0,execution,0,gripper,partial)==expected
    assert not proven_safe_rejection(MotionError('unknown outcome'),0,0,0,0,False)


def test_validity_query_waits_for_service_before_dispatch():
    b=Backend.__new__(Backend)
    b.valid_client=SimpleNamespace(wait_for_service=lambda **kwargs:False,
                                  call_async=lambda req:pytest.fail('Undiscovered service must not be called'))
    with pytest.raises(MotionError,match='state_validity_service_unavailable'):b.valid()


@pytest.mark.parametrize('fail',[False,True])
def test_payload_preview_restores_world_object_and_collision_permissions(fail):
    from chess_motion_interface.scene import Scene
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    from contextlib import nullcontext
    g=geometry();applied=[]
    acm=AllowedCollisionMatrix(entry_names=['knight','robot1_link4'],
        entry_values=[AllowedCollisionEntry(enabled=[False,True]),AllowedCollisionEntry(enabled=[True,False])])
    b=SimpleNamespace(model=SimpleNamespace(matrix=lambda *args:np.eye(4)),
        wait=lambda future:SimpleNamespace(scene=SimpleNamespace(allowed_collision_matrix=acm)),
        apply=lambda value:applied.append(copy.deepcopy(value)))
    scene=Scene.__new__(Scene);scene.backend=b;scene.geometry=g;scene.attached=None
    scene.get_client=SimpleNamespace(call_async=lambda req:None)
    scene.pieces={'knight':{'symbol':'N','local':g.local_square('g1',.011)}}
    scene.at={'g1':'knight'};before=copy.deepcopy(scene.pieces)
    state=RobotState();state.joint_state.name=['robot1_joint1'];state.joint_state.position=[.09]
    with pytest.raises(MotionError) if fail else nullcontext():
        with scene.preview_attachment('knight','robot1',state):
            assert scene.attached is None and scene.at=={'g1':'knight'}
            if fail:raise MotionError('no_collision_free_descent')
    assert scene.pieces.keys()==before.keys()
    assert np.array_equal(scene.pieces['knight']['local'],before['knight']['local'])
    assert applied[-1].allowed_collision_matrix==acm
    assert applied[-1].world.collision_objects[0].id=='knight'
    assert applied[-1].world.collision_objects[0].operation==CollisionObject.ADD
    assert applied[-2].robot_state.attached_collision_objects[0].object.operation==CollisionObject.REMOVE


def test_adaptive_grasp_candidates_keep_payload_clearance_and_reject_bad_profiles():
    raw=copy.deepcopy(geometry().raw)
    raw['motion']['grasp_search_fractions']=[.8,.9,.8,1.]
    g=Geometry(raw)
    assert g.grasp_fractions()==[.65,.8,.9,1.]
    for fraction in g.grasp_fractions():
        assert g.travel(.022,fraction)-.022*fraction >= max(raw['pieces']['heights'].values())+raw['motion']['transfer_clearance']-1e-12
    for invalid in [[True],[0.],[-.1],[1.1],[float('nan')],[.8]*9]:
        raw['motion']['grasp_search_fractions']=invalid
        with pytest.raises(ValueError,match='grasp_search_fractions'):Geometry(raw)


@pytest.mark.parametrize('uncertain',[False,True])
def test_transfer_search_preflights_placement_before_any_activity(uncertain):
    from contextlib import contextmanager
    raw=copy.deepcopy(geometry().raw);raw['motion']['grasp_search_fractions']=[.9]
    g=Geometry(raw);calls=[]
    @contextmanager
    def preview(*args):
        calls.append('preview')
        try:yield
        finally:calls.append('restore')
    def destination(*args):
        calls.append('destination')
        if calls.count('destination')==1:
            raise MotionError('service_or_action_timeout: restore uncertain' if uncertain else 'no_collision_free_descent: destination')
    scene=SimpleNamespace(at={'g1':'N'},pieces={'N':{'symbol':'N','local':g.local_square('g1',.011)}},
        touch=lambda *args:calls.append(('touch',args[-1])),preview_attachment=preview)
    backend=SimpleNamespace(approach=lambda *args,**kwargs:calls.append('approach') or object(),
        trajectory_end_state=lambda *args:object(),plan_vertical=lambda *args:object(),descent_candidates=destination)
    fake=SimpleNamespace(geometry=g,scene=scene,backend=backend,event=lambda *args,**kwargs:None)
    if uncertain:
        with pytest.raises(MotionError,match='uncertain'):MotionNode.prepare_transfer(fake,'robot1','g1','f3')
        assert calls.count('approach')==1  # Unknown outcomes never trigger another candidate.
    else:
        plan=MotionNode.prepare_transfer(fake,'robot1','g1','f3')
        assert plan['grasp_fraction']==.9 and calls.count('destination')==2
    assert calls[-1]==('touch',False)
    assert calls.count('restore')==calls.count('preview')
