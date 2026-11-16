"""Closed loop WITHOUT ROS: the brain's GameController plans, the executor 'moves' (commanded state only), the brain gets
the acknowledgement and a simulated camera observation of the expected position. Proves the contract between the two packages."""
import random
import time
import uuid

import pytest

chess = pytest.importorskip('chess')
brain = pytest.importorskip('chess_brain_handover')
from chess_robot_executor.config import build_config                       # noqa: E402
from chess_robot_executor.controller import ExecutorController, State       # noqa: E402
from chess_robot_executor.sinks import RecordingSink                         # noqa: E402

DT = 0.02


class RandomPlayer:
    def __init__(self, seed):
        self.rng = random.Random(seed)

    def choose_move(self, board):
        return self.rng.choice(list(board.legal_moves)), None, 0.01

    def close(self):
        pass


def camera(placement):
    return {'schema_version': '1.0', 'capture_id': uuid.uuid4().hex, 'source': 'sim_camera', 'captured_at': time.time(),
            'stable': True, 'calibrated': True, 'confidence': 0.99, 'orientation': 'a8_top_left', 'piece_placement': placement}


def play(plies, seed):
    g = brain.GameController(players={chess.WHITE: RandomPlayer(seed), chess.BLACK: RandomPlayer(seed + 100)})
    assert g.sync_initial_board(camera(g.board.board_fen()))['ok']
    acks = []
    ex = ExecutorController(build_config(), RecordingSink(), acks.append)
    done = 0
    for _ in range(plies):
        if g.board.is_game_over():
            break
        plan = g.next_plan()
        ex.on_plan(plan.to_dict())
        t = 0.0
        while ex.state is not State.IDLE and t < 2000:
            ex.tick(DT); t += DT
            if ex.state is State.WAIT_OPERATOR:
                ex.on_operator_confirm()
        a = acks[-1]
        assert a['plan_id'] == plan.plan_id and a['success'], a
        resp = g.robot_ack(plan.plan_id, success=a['success'], arm_parked=a['arm_parked'], gripper_released=a['gripper_released'])
        assert resp['ok'], resp
        resp = g.confirm_vision(camera(plan.expected_fen.split()[0]))
        assert resp['ok'], resp
        done += 1
    return done, ex


@pytest.mark.parametrize('seed', [1, 2, 3])
def test_brain_and_executor_play_a_game_in_a_closed_loop(seed):
    done, ex = play(120, seed)
    assert done >= 60
    assert ex.state is State.IDLE and all(ex.parked.values()) and not ex.needs_recovery


def test_executor_failure_blocks_the_brain():
    g = brain.GameController(players={chess.WHITE: RandomPlayer(5), chess.BLACK: RandomPlayer(6)})
    assert g.sync_initial_board(camera(g.board.board_fen()))['ok']
    plan = g.next_plan()
    acks = []
    ex = ExecutorController(build_config(), RecordingSink(), acks.append)
    ex.on_estop(True)
    ex.on_plan(plan.to_dict())
    a = acks[-1]
    assert not a['success']
    g.robot_ack(plan.plan_id, success=a['success'], arm_parked=a['arm_parked'], gripper_released=a['gripper_released'])
    with pytest.raises(brain.GameBlocked):
        g.next_plan()
