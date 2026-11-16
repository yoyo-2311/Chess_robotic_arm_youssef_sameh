import math
import random

import pytest

from chess_robot_executor.board_geometry import BoardGeometry, RobotMount, parse_square
from chess_robot_executor.config import build_config
from chess_robot_executor.scara import ScaraParams, wrap

CFG = build_config()
SQUARES = [f + r for f in 'abcdefgh' for r in '12345678']


def test_parse_square():
    assert parse_square('a1') == (0, 0) and parse_square('h8') == (7, 7) and parse_square('E4') == (4, 3)
    for bad in ('i1', 'a9', 'a', 'a10', '', None, 11):
        with pytest.raises(ValueError):
            parse_square(bad)


def test_board_square_centres():
    b = BoardGeometry()
    assert b.square == pytest.approx(0.04375)
    assert b.square_center('a1')[:2] == pytest.approx((-0.153125, -0.153125))
    assert b.square_center('h8')[:2] == pytest.approx((0.153125, 0.153125))
    x, y, z = b.square_center('e4')
    assert (x, y) == pytest.approx((0.021875, -0.021875)) and z == b.surface_z
    xs = sorted({b.square_center(s)[0] for s in SQUARES})
    assert all(xs[i + 1] - xs[i] == pytest.approx(b.square) for i in range(7))


def test_mount_roundtrip_and_forward_direction():
    m = RobotMount('robot1', 0.0, -0.30, math.pi / 2)
    assert m.to_robot(0.0, 0.0)[:2] == pytest.approx((0.30, 0.0))       # board centre is straight ahead of robot1
    m2 = RobotMount('robot2', 0.0, 0.30, -math.pi / 2)
    assert m2.to_robot(0.0, 0.0)[:2] == pytest.approx((0.30, 0.0))
    rng = random.Random(1)
    for _ in range(50):
        p = (rng.uniform(-.3, .3), rng.uniform(-.3, .3), rng.uniform(0, .1))
        mm = RobotMount('x', rng.uniform(-.5, .5), rng.uniform(-.5, .5), rng.uniform(-3, 3), 0.02)
        assert mm.to_board(*mm.to_robot(*p)) == pytest.approx(p)


def test_wrap():
    assert wrap(3 * math.pi) == pytest.approx(math.pi) and wrap(-3.5) == pytest.approx(-3.5 + 2 * math.pi)


def test_fk_ik_roundtrip_and_elbows():
    p = ScaraParams(0.30, 0.24, -2.8, 2.8, -2.8, 2.8, 0.0, 0.15)
    rng = random.Random(2)
    n = 0
    for _ in range(400):
        q1, q2 = rng.uniform(-2.5, 2.5), rng.uniform(-2.5, 2.5)
        x, y, z = p.fk(q1, q2, 0.05)
        if math.hypot(x, y) > p.r_max or math.hypot(x, y) < p.r_min:
            continue
        sols = p.ik(x, y, 0.05)
        assert sols, (q1, q2)
        for s in sols:
            assert p.fk(s.q1, s.q2, s.z) == pytest.approx((x, y, 0.05), abs=1e-9)
        assert any(abs(s.q1 - q1) < 1e-6 and abs(s.q2 - q2) < 1e-6 for s in sols)
        n += 1
    assert n > 100


def test_ik_rejects_unreachable_and_out_of_limits():
    p = ScaraParams(0.30, 0.24, -2.8, 2.8, -2.8, 2.8, 0.0, 0.15)
    assert p.ik(0.6, 0.0, 0.05) == []          # beyond l1 + l2
    assert p.ik(0.01, 0.0, 0.05) == []         # inside |l1 - l2|
    assert p.ik(0.4, 0.0, 0.5) == []           # z above stroke
    tight = ScaraParams(0.30, 0.24, -0.1, 0.1, -0.1, 0.1, 0.0, 0.15)
    assert tight.ik(0.4, 0.2, 0.05) == []      # needs more joint range than allowed


def test_ik_best_seed_and_preference():
    p = ScaraParams(0.30, 0.24, -2.8, 2.8, -2.8, 2.8, 0.0, 0.15)
    both = p.ik(0.3, 0.1, 0.05)
    assert len(both) == 2 and both[0].elbow == 'left'
    assert p.ik_best(0.3, 0.1, 0.05, preference='right').elbow == 'right'
    s = p.ik_best(0.3, 0.1, 0.05, seed=(both[1].q1, both[1].q2))
    assert s.elbow == both[1].elbow


@pytest.mark.parametrize('arm', ['robot1', 'robot2'])
def test_all_64_squares_reachable_at_every_working_height(arm):
    m = CFG.mounts[arm]
    heights = {CFG.travel_z, CFG.place_z} | {CFG.grasp_z(p) for p in 'pnbrqk'}
    for sq in SQUARES:
        x, y, _ = CFG.board.square_center(sq)
        xr, yr, _ = m.to_robot(x, y)
        for z in heights:
            assert CFG.scara.reachable(xr, yr, z - m.base_z), (arm, sq, z)


@pytest.mark.parametrize('arm', ['robot1', 'robot2'])
def test_capture_bin_slots_reachable_and_outside_board(arm):
    m = CFG.mounts[arm]
    for i in range(CFG.raw['capture_bins'][arm]['slots']):
        x, y = CFG.bin_slot(arm, i)
        assert not CFG.board.contains_xy(x, y, margin=0.02), (arm, i, x, y)
        xr, yr, _ = m.to_robot(x, y)
        assert CFG.scara.reachable(xr, yr, CFG.place_z - m.base_z) and CFG.scara.reachable(xr, yr, CFG.travel_z - m.base_z)
    with pytest.raises(ValueError):
        CFG.bin_slot(arm, CFG.raw['capture_bins'][arm]['slots'])


def test_park_pose_is_outside_the_board_and_arms_are_apart():
    park = CFG.raw['park']
    tools = {}
    for arm, m in CFG.mounts.items():
        assert CFG.scara.within_limits(park['q1'], park['q2'], CFG.travel_z)
        elbow = m.to_board(CFG.scara.l1 * math.cos(park['q1']), CFG.scara.l1 * math.sin(park['q1']))
        x, y, _ = CFG.scara.fk(park['q1'], park['q2'], CFG.travel_z)
        tool = m.to_board(x, y)
        tools[arm] = tool
        assert not CFG.board.contains_xy(*tool[:2], margin=0.05)
        assert not CFG.board.contains_xy(*elbow[:2], margin=0.05)
    assert math.dist(tools['robot1'][:2], tools['robot2'][:2]) > 0.5
