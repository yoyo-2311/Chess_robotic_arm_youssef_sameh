import pytest

from chess_robot_executor.config import build_config
from chess_robot_executor.task_planner import INITIAL_FEN_PREFIX, PlanError, plan_tasks

chess = pytest.importorskip('chess')
core = pytest.importorskip('chess_brain_handover.core')
CFG = build_config()


def plan_for(fen, uci):
    return core.describe_move(chess.Board(fen), chess.Move.from_uci(uci)).to_dict()


def travels(tp):
    return [(round(s.x, 5), round(s.y, 5)) for s in tp.steps if s.kind == 'travel']


def sq(name):
    x, y, _ = CFG.board.square_center(name)
    return (round(x, 5), round(y, 5))


def test_pawn_move_structure():
    tp = plan_tasks(plan_for(chess.STARTING_FEN, 'e2e4'), CFG)
    assert tp.arm == 'robot1' and tp.bin_count == 0
    assert travels(tp) == [sq('e2'), sq('e4')]
    assert [s.kind for s in tp.steps][0] == 'z' and tp.steps[0].z == CFG.travel_z
    assert [s.kind for s in tp.steps][-2:] == ['z', 'park']
    grasps = [s for s in tp.steps if s.kind == 'grip' and s.width == CFG.raw['gripper']['grasp_width']]
    assert len(grasps) == 1
    zs = [s.z for s in tp.steps if s.kind == 'z']
    assert CFG.grasp_z('P') in zs and CFG.place_z in zs


def test_black_uses_robot2():
    fen = 'rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1'
    assert plan_tasks(plan_for(fen, 'e7e5'), CFG).arm == 'robot2'


def test_capture_removes_first_into_bin_slot():
    fen = 'rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2'
    tp = plan_tasks(plan_for(fen, 'e4d5'), CFG)
    slot0 = CFG.bin_slot('robot1', 0)
    assert travels(tp) == [sq('d5'), (round(slot0[0], 5), round(slot0[1], 5)), sq('e4'), sq('d5')]
    assert tp.bin_count == 1
    tp2 = plan_tasks(plan_for(fen, 'e4d5'), CFG, {'robot1': 3})
    assert tp2.bin_count == 4 and travels(tp2)[1] == (round(CFG.bin_slot('robot1', 3)[0], 5), round(CFG.bin_slot('robot1', 3)[1], 5))


def test_en_passant_removes_the_pawn_beside_not_on_target():
    fen = 'rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3'
    tp = plan_tasks(plan_for(fen, 'e5f6'), CFG)
    assert travels(tp)[0] == sq('f5') and tp.bin_count == 1


@pytest.mark.parametrize('fen,uci,arm,first,second', [
    ('r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1', 'e1g1', 'robot1', ('e1', 'g1'), ('h1', 'f1')),
    ('r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1', 'e1c1', 'robot1', ('e1', 'c1'), ('a1', 'd1')),
    ('r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1', 'e8g8', 'robot2', ('e8', 'g8'), ('h8', 'f8')),
    ('r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1', 'e8c8', 'robot2', ('e8', 'c8'), ('a8', 'd8')),
])
def test_castling_is_king_then_rook(fen, uci, arm, first, second):
    tp = plan_tasks(plan_for(fen, uci), CFG)
    assert tp.arm == arm
    assert travels(tp) == [sq(first[0]), sq(first[1]), sq(second[0]), sq(second[1])]


def test_promotion_needs_operator_and_bins_the_pawn():
    tp = plan_tasks(plan_for('8/P7/8/8/8/8/k6K/8 w - - 0 1', 'a7a8q'), CFG)
    ops = [s for s in tp.steps if s.kind == 'operator']
    assert len(ops) == 1 and 'a8' in ops[0].note and 'white queen' in ops[0].note and tp.bin_count == 1
    tp2 = plan_tasks(plan_for('1n6/P7/8/8/8/8/k6K/8 w - - 0 1', 'a7b8q'), CFG)
    assert tp2.bin_count == 2                                    # captured knight + promoted pawn


def test_new_game_resets_bin_counters():
    p = plan_for(chess.STARTING_FEN, 'e2e4')
    assert p['before_fen'].startswith(INITIAL_FEN_PREFIX)
    assert plan_tasks(p, CFG, {'robot1': 9, 'robot2': 4}).bin_count == 0


def test_bin_full():
    fen = 'rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2'
    with pytest.raises(PlanError) as e:
        plan_tasks(plan_for(fen, 'e4d5'), CFG, {'robot1': 24})
    assert e.value.reason == 'bin_full'


GOOD = {'plan_id': 'a', 'side': 'white', 'piece': 'P', 'actions': [{'kind': 'move', 'source': 'e2', 'target': 'e4'}]}


@pytest.mark.parametrize('mutate,reason', [
    (lambda p: p.pop('plan_id'), 'malformed_plan'),
    (lambda p: p.update(plan_id=''), 'malformed_plan'),
    (lambda p: p.update(side='green'), 'malformed_plan'),
    (lambda p: p.update(actions=[]), 'malformed_plan'),
    (lambda p: p.update(actions='move'), 'malformed_plan'),
    (lambda p: p.update(actions=[{'kind': 'teleport', 'source': 'e2', 'target': 'e4'}]), 'unknown_action'),
    (lambda p: p.update(actions=[{'kind': 'move', 'source': 'e2'}]), 'malformed_plan'),
    (lambda p: p.update(actions=[{'kind': 'move', 'source': 'z9', 'target': 'e4'}]), 'invalid_action'),
    (lambda p: p.update(actions=[None]), 'unknown_action'),
])
def test_malformed_plans_are_rejected(mutate, reason):
    p = {**GOOD, 'actions': list(GOOD['actions'])}
    mutate(p)
    with pytest.raises(PlanError) as e:
        plan_tasks(p, CFG)
    assert e.value.reason == reason


def test_non_dict_plan():
    for bad in (None, [], 'x', 3):
        with pytest.raises(PlanError):
            plan_tasks(bad, CFG)
