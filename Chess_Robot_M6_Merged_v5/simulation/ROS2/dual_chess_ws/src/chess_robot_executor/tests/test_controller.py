import random

import pytest

from chess_robot_executor.config import build_config
from chess_robot_executor.controller import ExecutorController, State
from chess_robot_executor.sinks import RecordingSink

chess = pytest.importorskip('chess')
core = pytest.importorskip('chess_brain_handover.core')
DT = 0.02


def make(over=None):
    cfg = build_config(over)
    acks, events, sink = [], [], RecordingSink()
    return cfg, ExecutorController(cfg, sink, acks.append, events.append), sink, acks, events


def plan_for(fen, uci, pid=None):
    p = core.describe_move(chess.Board(fen), chess.Move.from_uci(uci)).to_dict()
    if pid:
        p['plan_id'] = pid
    return p


def run(ex, confirm=False, limit=3000.0):
    t = 0.0
    while t < limit:
        ex.tick(DT)
        t += DT
        if ex.state is State.IDLE:
            return t
        if ex.state is State.WAIT_OPERATOR and confirm:
            ex.on_operator_confirm()
        if ex.state is State.WAIT_OPERATOR and not confirm:
            return t
    raise AssertionError('did not finish')


def check_invariants(cfg, sink, arm):
    """Safety invariants of the commanded motion of one plan."""
    log = [c for a, c in sink.log if a == arm]
    assert log and all(a == arm for a, _ in sink.log), 'only the assigned arm may be commanded'
    vmax = cfg.vmax()
    sc = cfg.scara
    m = cfg.mounts[arm]
    prev = None
    for c in log:
        assert sc.q1_min - 1e-9 <= c['shoulder'] <= sc.q1_max + 1e-9 and sc.q2_min - 1e-9 <= c['elbow'] <= sc.q2_max + 1e-9
        assert sc.z_min - 1e-9 <= c['z'] <= sc.z_max + 1e-9
        assert 0.0 <= c['gripper'] <= cfg.raw['gripper']['open_width'] + 1e-9
        if prev is not None:
            dq = [abs(c['shoulder'] - prev['shoulder']), abs(c['elbow'] - prev['elbow'])]
            dz = abs(c['z'] - prev['z'])
            dg = abs(c['gripper'] - prev['gripper'])
            moving_xy = max(dq) > 1e-9
            assert dq[0] / DT <= vmax[0] * 1.02 and dq[1] / DT <= vmax[1] * 1.02 and dz / DT <= vmax[2] * 1.02
            if moving_xy:       # never travel horizontally below the travel height, never mix with z or gripper motion
                assert c['z'] >= cfg.travel_z - m.base_z - 1e-6 and prev['z'] >= cfg.travel_z - m.base_z - 1e-6
                assert dz < 1e-9 and dg < 1e-9
            if dg > 1e-9:       # the gripper only moves while the arm stands still
                assert max(dq) < 1e-9 and dz < 1e-9
        prev = c


def test_pawn_move_end_to_end():
    cfg, ex, sink, acks, events = make()
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'p1'))
    assert ex.state is State.EXECUTING
    t = run(ex)
    assert len(acks) == 1
    a = acks[0]
    assert a['plan_id'] == 'p1' and a['success'] and a['arm_parked'] and a['gripper_released'] and a['arm'] == 'robot1'
    check_invariants(cfg, sink, 'robot1')
    assert 5.0 < t < 120.0
    assert ex.parked == {'robot1': True, 'robot2': True} and ex.state is State.IDLE
    names = [e['event'] for e in events]
    assert names[0] == 'plan_started' and names[-1] == 'plan_done'


def test_black_move_uses_robot2_only():
    cfg, ex, sink, acks, _ = make()
    ex.on_plan(plan_for('rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1', 'e7e5', 'b1'))
    run(ex)
    assert acks[-1]['success'] and acks[-1]['arm'] == 'robot2'
    check_invariants(cfg, sink, 'robot2')


def test_capture_and_bin_counter_persist():
    cfg, ex, sink, acks, _ = make()
    ex.on_plan(plan_for('rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2', 'e4d5', 'c1'))
    run(ex)
    assert acks[-1]['success'] and ex.bin_counts == {'robot1': 1}
    ex.on_plan(plan_for('rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2', 'e4d5', 'c2'))
    run(ex)
    assert ex.bin_counts == {'robot1': 2}


@pytest.mark.parametrize('fen,uci,arm', [
    ('r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1', 'e1g1', 'robot1'),
    ('r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1', 'e8c8', 'robot2'),
    ('rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3', 'e5f6', 'robot1'),
])
def test_castling_and_en_passant(fen, uci, arm):
    cfg, ex, sink, acks, _ = make()
    ex.on_plan(plan_for(fen, uci, 'z'))
    run(ex)
    assert acks[-1]['success'] and acks[-1]['arm'] == arm
    check_invariants(cfg, sink, arm)


def test_promotion_waits_for_operator_then_completes():
    cfg, ex, sink, acks, events = make()
    ex.on_plan(plan_for('1n6/P7/8/8/8/8/k6K/8 w - - 0 1', 'a7b8q', 'pr'))
    run(ex, confirm=False)
    assert ex.state is State.WAIT_OPERATOR and acks == []
    assert any(e['event'] == 'wait_operator' and 'b8' in e['message'] for e in events)
    n = len(sink.log)
    for _ in range(100):
        ex.tick(DT)
    assert len(sink.log) == n                          # no motion while waiting
    ex.on_operator_confirm()
    run(ex, confirm=True)
    assert acks[-1]['success'] and ex.bin_counts == {'robot1': 2}


def test_operator_timeout_fails_and_requires_recovery():
    cfg, ex, sink, acks, _ = make({'timing': {'operator_timeout': 1.0}})
    ex.on_plan(plan_for('8/P7/8/8/8/8/k6K/8 w - - 0 1', 'a7a8q', 'pt'))
    run(ex, confirm=False)
    for _ in range(100):
        ex.tick(DT)
    assert acks and not acks[-1]['success'] and acks[-1]['reason'] == 'operator_timeout' and ex.needs_recovery
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'after'))
    assert acks[-1]['plan_id'] == 'after' and not acks[-1]['success'] and acks[-1]['reason'] == 'estop_or_recovery_required'
    ex.on_recovered()
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'after2'))
    run(ex)
    assert acks[-1]['plan_id'] == 'after2' and acks[-1]['success']


def test_estop_mid_motion_aborts_blocks_and_recovers():
    cfg, ex, sink, acks, _ = make()
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'e1'))
    for _ in range(200):
        ex.tick(DT)
    ex.on_estop(True)
    assert ex.state is State.ESTOP and sink.stops == ['robot1']
    assert acks[-1]['plan_id'] == 'e1' and not acks[-1]['success'] and not acks[-1]['arm_parked'] and acks[-1]['reason'] == 'estop'
    n = len(sink.log)
    for _ in range(50):
        ex.tick(DT)
    assert len(sink.log) == n                                    # frozen
    ex.on_plan(plan_for(chess.STARTING_FEN, 'd2d4', 'e2'))
    assert acks[-1]['plan_id'] == 'e2' and not acks[-1]['success'] and len(sink.log) == n
    ex.on_estop(False)
    assert ex.state is State.IDLE and ex.needs_recovery
    ex.on_plan(plan_for(chess.STARTING_FEN, 'd2d4', 'e3'))
    assert not acks[-1]['success']                                # still needs operator recovery
    ex.on_recovered()
    assert not ex.needs_recovery and all(ex.parked.values())
    ex.on_plan(plan_for(chess.STARTING_FEN, 'd2d4', 'e4'))
    run(ex)
    assert acks[-1]['plan_id'] == 'e4' and acks[-1]['success']


def test_estop_while_idle_blocks_until_released():
    cfg, ex, sink, acks, _ = make()
    ex.on_estop(True)
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'i1'))
    assert not acks[-1]['success'] and sink.log == []
    ex.on_estop(False)
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'i2'))
    run(ex)
    assert acks[-1]['plan_id'] == 'i2' and acks[-1]['success']


def test_busy_and_duplicate_plans_are_ignored():
    cfg, ex, sink, acks, events = make()
    p = plan_for(chess.STARTING_FEN, 'e2e4', 'd1')
    ex.on_plan(p)
    ex.on_plan(plan_for(chess.STARTING_FEN, 'd2d4', 'd2'))       # different plan while busy
    ex.on_plan(p)                                                # same plan again while busy
    run(ex)
    assert [a['plan_id'] for a in acks] == ['d1']
    ex.on_plan(p)                                                # same plan after completion
    assert len(acks) == 1
    reasons = [e['reason'] for e in events if e['event'] == 'plan_ignored']
    assert reasons == ['busy with another plan', 'duplicate plan_id', 'duplicate plan_id']


@pytest.mark.parametrize('plan', [
    {'plan_id': 'm1', 'side': 'white', 'actions': []},
    {'plan_id': 'm2', 'side': 'white', 'actions': [{'kind': 'move', 'source': 'e2', 'target': 'j4'}]},
    {'plan_id': 'm3', 'side': 'blue', 'actions': [{'kind': 'move', 'source': 'e2', 'target': 'e4'}]},
])
def test_malformed_plan_fails_without_motion(plan):
    cfg, ex, sink, acks, _ = make()
    ex.on_plan(plan)
    assert len(acks) == 1 and acks[0]['plan_id'] == plan['plan_id'] and not acks[0]['success']
    assert acks[0]['arm_parked'] and acks[0]['gripper_released'] and sink.log == [] and ex.state is State.IDLE


def test_unreachable_target_is_caught_before_any_motion():
    cfg, ex, sink, acks, _ = make({'scara': {'l1': 0.15, 'l2': 0.10, 'z_limits': [0.0, 0.15]}, 'park': {'q1': 1.5, 'q2': 1.0}})
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'u1'))
    assert acks and not acks[0]['success'] and acks[0]['reason'] == 'unreachable' and sink.log == []


def test_z_limit_violation_is_caught_before_any_motion():
    # stroke starts above the grasp height -> the plan cannot be executed, and nothing may move
    cfg, ex, sink, acks, _ = make({'scara': {'z_limits': [0.05, 0.15]}})
    ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 'z1'))
    assert acks and not acks[0]['success'] and acks[0]['reason'] == 'z_out_of_range' and sink.log == []


def test_travel_height_must_be_inside_the_z_stroke():
    with pytest.raises(ValueError):
        ExecutorController(build_config({'scara': {'z_limits': [0.0, 0.05]}}), RecordingSink(), lambda a: None)


def test_speed_scale_changes_duration():
    times = []
    for s in (0.25, 1.0):
        cfg, ex, sink, acks, _ = make({'limits': {'speed_scale': s}})
        ex.on_plan(plan_for(chess.STARTING_FEN, 'e2e4', 's'))
        times.append(run(ex))
        check_invariants(cfg, sink, 'robot1')
    assert times[0] > 1.5 * times[1]


def test_park_pose_must_respect_limits():
    with pytest.raises(ValueError):
        ExecutorController(build_config({'park': {'q1': 3.5}}), RecordingSink(), lambda a: None)


def test_random_full_games_are_executable():
    """Whole random games (captures, castling, en passant, promotions) go through the executor without a single failure."""
    for seed in (1, 2, 3):
        rng = random.Random(seed)
        board = chess.Board()
        cfg, ex, sink, acks, _ = make()
        kinds = set()
        for ply in range(160):
            moves = list(board.legal_moves)
            if not moves or board.is_game_over():
                break
            move = rng.choice(moves)
            if board.is_castling(move): kinds.add('castle')
            if board.is_en_passant(move): kinds.add('ep')
            if board.is_capture(move): kinds.add('capture')
            if move.promotion: kinds.add('promotion')
            plan = core.describe_move(board, move).to_dict()
            sink.log.clear()
            ex.on_plan(plan)
            run(ex, confirm=True, limit=1000.0)
            assert acks[-1]['plan_id'] == plan['plan_id'] and acks[-1]['success'], (seed, ply, move.uci(), acks[-1])
            check_invariants(cfg, sink, 'robot1' if plan['side'] == 'white' else 'robot2')
            board.push(move)
        assert 'capture' in kinds


def test_all_legal_first_moves_and_special_positions_execute():
    cfg, ex, sink, acks, _ = make()
    n = 0
    for fen in (chess.STARTING_FEN, 'r3k2r/pppppppp/8/8/8/8/PPPPPPPP/R3K2R w KQkq - 0 1', '8/PPPPPPPP/8/8/8/8/k6K/8 w - - 0 1',
                'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR b KQkq - 0 1'):
        board = chess.Board(fen)
        for move in board.legal_moves:
            sink.log.clear()
            ex.on_plan(core.describe_move(board, move).to_dict())
            run(ex, confirm=True, limit=1000.0)
            assert acks[-1]['success'], (fen, move.uci(), acks[-1])
            ex.bin_counts.clear()
            n += 1
    assert n > 100
