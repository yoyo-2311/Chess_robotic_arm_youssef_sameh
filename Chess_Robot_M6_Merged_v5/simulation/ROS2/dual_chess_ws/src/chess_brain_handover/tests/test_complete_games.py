"""Complete-game and terminal-state contracts; use public reset/ack/vision APIs."""
import time
import uuid
import pytest
from chess_brain_handover.core import GameController, GameBlocked, chess, ORIENTATION, SCHEMA_VERSION


class NoEngine:
    def choose_move(self, board):
        raise AssertionError('Scripted tests must not request engine decisions')
    def close(self):
        pass


def observation(placement):
    return {'schema_version':SCHEMA_VERSION,'capture_id':uuid.uuid4().hex,
            'source':'LOGIC_TEST_ONLY','captured_at':time.time(),'stable':True,
            'calibrated':True,'confidence':1.0,'orientation':ORIENTATION,'piece_placement':placement}


def controller(fen=None):
    game=GameController(players={chess.WHITE:NoEngine(),chess.BLACK:NoEngine()})
    if fen:game.reset(fen=fen,physical_reset_confirmed=True)
    assert game.sync_initial_board(observation(game.board.board_fen()))['ok']
    return game


@pytest.mark.parametrize('moves,result,termination',[
    ('f2f3 e7e5 g2g4 d8h4','0-1','CHECKMATE'),
    ('e2e4 e7e5 f1c4 b8c6 d1h5 g8f6 h5f7','1-0','CHECKMATE'),
    ('g1f3 g8f6 f3g1 f6g8 g1f3 g8f6 f3g1 f6g8','1/2-1/2','THREEFOLD_REPETITION'),
])
def test_complete_games_commit_only_after_ack_and_fresh_vision(moves,result,termination):
    reference=chess.Board()
    with controller() as game:
        for uci in moves.split():
            if game.result() is not None:break  # The brain automatically claims an available draw.
            before=game.state()['fen'];ply=game.state()['committed_plies']
            plan=game.manual_move(uci[:2],uci[2:4])
            frame_before_ack=observation(plan.expected_fen.split()[0])
            assert not game.confirm_vision(frame_before_ack)['ok']
            assert game.robot_ack(plan.plan_id,success=True,arm_parked=True,gripper_released=True)['ok']
            assert game.robot_ack(plan.plan_id,success=True,arm_parked=True,gripper_released=True)['status']=='ack_already_received'
            assert not game.confirm_vision(frame_before_ack)['ok']
            assert game.state()['fen']==before and game.state()['committed_plies']==ply
            frame=observation(plan.expected_fen.split()[0]);assert game.confirm_vision(frame)['status']=='committed'
            reference.push_uci(uci)
            assert game.state()['fen']==reference.fen()
            assert game.state()['committed_plies']==ply+1
            assert game.confirm_vision(frame)['status']=='no_pending_plan'
            assert game.state()['committed_plies']==ply+1
        assert game.result()==result
        assert reference.outcome(claim_draw=True).termination.name==termination
        assert game.state()['pending_plan_id'] is None
        with pytest.raises(GameBlocked):game.next_plan()
        with pytest.raises(GameBlocked):game.manual_move('a2','a3')


@pytest.mark.parametrize('fen,termination',[
    ('7k/5Q2/6K1/8/8/8/8/8 b - - 0 1','STALEMATE'),
    ('7k/8/8/8/8/8/8/K7 w - - 0 1','INSUFFICIENT_MATERIAL'),
    ('7k/8/8/8/8/8/8/K5R1 w - - 100 51','FIFTY_MOVES'),
])
def test_terminal_draw_positions_reject_further_plans(fen,termination):
    with controller(fen) as game:
        assert game.result()=='1/2-1/2'
        assert game.board.outcome(claim_draw=True).termination.name==termination
        with pytest.raises(GameBlocked):game.next_plan()
        assert game.state()['committed_plies']==0 and game.state()['pending_plan_id'] is None
