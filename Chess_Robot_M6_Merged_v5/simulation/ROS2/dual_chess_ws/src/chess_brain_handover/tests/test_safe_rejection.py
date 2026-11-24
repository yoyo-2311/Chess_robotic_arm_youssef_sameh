"""Only an explicit no-activity proof may discard an unexecuted pending plan."""
import pytest
from chess_brain_handover.core import GameController,chess
from chess_brain_handover.selfplay import fake_vision

class Player:
 def close(self):pass
 def choose_move(self,board):return next(iter(board.legal_moves)),None,0.

def game():
 g=GameController(players={chess.WHITE:Player(),chess.BLACK:Player()})
 assert g.sync_initial_board(fake_vision(g.board.board_fen()))['ok']
 return g

def reject(g,p,**changes):
 proof=dict(before_fen=p.before_fen,arm_activity=False,gripper_activity=False,scene_restored=True,reason='no_collision_free_descent')
 proof.update(changes)
 return g.reject_unexecuted(p.plan_id,**proof)

def test_proven_rejection_clears_pending_without_committing_or_requiring_vision():
 with game() as g:
  p=g.manual_move('g1','f3');before=g.state()['fen']
  assert reject(g,p)['status']=='rejected_before_activity'
  assert g.pending is None and not g.blocked and g.state()['fen']==before
  assert g.state()['committed_plies']==0 and g.state()['last_rejection']['plan_id']==p.plan_id
  next_plan=g.manual_move('e2','e4')
  assert g.state()['last_rejection'] is None
  assert not reject(g,p)['ok'] and g.pending.plan_id==next_plan.plan_id

@pytest.mark.parametrize('change',[{'arm_activity':True},{'gripper_activity':True},{'scene_restored':False},
 {'arm_activity':None},{'gripper_activity':0},{'before_fen':'wrong'}])
def test_incomplete_or_activity_proof_cannot_clear_pending(change):
 with game() as g:
  p=g.manual_move('g1','f3');assert not reject(g,p,**change)['ok']
  assert g.pending.plan_id==p.plan_id and g.state()['committed_plies']==0

def test_failed_or_successful_ack_cannot_be_reclassified_as_safe_rejection():
 for success in [False,True]:
  with game() as g:
   p=g.manual_move('e2','e4');g.robot_ack(p.plan_id,success=success,arm_parked=success,gripper_released=success)
   assert reject(g,p)['status']=='reconciliation_required'
   assert g.pending.plan_id==p.plan_id


def test_rejection_state_is_a_snapshot_not_mutable_internal_proof():
 with game() as g:
  p=g.manual_move('g1','f3');reject(g,p)
  snapshot=g.state();snapshot['last_rejection']['before_fen']='tampered'
  assert g.state()['last_rejection']['before_fen']==p.before_fen
